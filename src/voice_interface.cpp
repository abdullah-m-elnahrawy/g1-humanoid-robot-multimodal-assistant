#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <cstdlib>
#include <string>
#include <signal.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <algorithm>
#include <filesystem>
#include <cctype>
#include <fstream>          // added
#include <set>              // added
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <netdb.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/common/time/time_tool.hpp>

#include <robot/audio/audio_manager.hpp>
#include <robot/realtime/realtime_openai.hpp>

#include "audio/audio_processing_pipeline.hpp"
#include "robot/branding/branding_fingerprint.hpp"
#include "project_branding.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

#define GROUP_IP "239.168.123.161"
#define PORT     5555

static std::unique_ptr<RealtimeOpenAI> rt;
static std::unique_ptr<AudioProcessingPipeline> g_apm;

static std::atomic<bool> shutdown_requested{false};

static std::queue<std::vector<int16_t>> pcm_queue; // 16k PCM full answers
static std::mutex queue_mtx;
static std::condition_variable queue_cv;
static std::thread playback_thread;
static std::thread udp_mic_thread;

static unitree::robot::g1::AudioClient* global_audio = nullptr;
static std::atomic<bool> is_playing{false};
static std::atomic<bool> cancel_playback{false};
static int sock = -1;

static bool wake_required = false;
static bool print_transcript = false;

// ---------- helpers ----------
static fs::path self_dir() {
    char buf[4096];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf)-1);
    if (n <= 0) return fs::current_path();
    buf[n] = '\0';
    return fs::path(buf).parent_path();
}

// trim helpers
static std::string trim_copy(std::string s) {
    auto not_space = [](unsigned char c){ return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

// load file to string (whole file), trimmed
static bool read_file_trimmed(const fs::path& p, std::string& out) {
    std::ifstream f(p);
    if (!f.good()) return false;
    std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    out = trim_copy(s);
    return !out.empty();
}

// set default env var if missing/empty
static void set_default_env(const char* key, const char* value) {
    const char* cur = std::getenv(key);
    if (!cur || !*cur) setenv(key, value, 1);
}

// --- NEW: load runtime environment from config/runtime.env if present ---
// Only whitelisted keys are read here (security & predictability).
static void load_runtime_env_from_config() {
    // locate config/runtime.env near the binary first, then in source tree
    fs::path bin_cfg = self_dir().parent_path() / "config" / "runtime.env";
    fs::path src_cfg = fs::path(PROJECT_SOURCE_DIR) / "config" / "runtime.env";
    fs::path cwd_cfg = fs::current_path() / "config" / "runtime.env";

    fs::path cfg;
    if (fs::exists(bin_cfg)) cfg = bin_cfg;
    else if (fs::exists(src_cfg)) cfg = src_cfg;
    else if (fs::exists(cwd_cfg)) cfg = cwd_cfg;

    if (cfg.empty()) return;

    // whitelist of keys we allow from the project config file
    const std::set<std::string> allowed = {
        "OPENAI_REALTIME_MODEL",
        "AUTO_RESPOND",
        "WAKEWORD_REQUIRED",
        "ASR_LANG",
        "VAD_THRESHOLD_RMS",
        "PRINT_TRANSCRIPT",
        "WAKEWORD"
        // NOTE: OPENAI_API_KEY is intentionally NOT allowed here
    };

    std::ifstream in(cfg);
    if (!in.good()) return;

    std::string line;
    while (std::getline(in, line)) {
        // strip comments (# or ;)
        auto hash = line.find('#'); if (hash != std::string::npos) line.erase(hash);
        auto semi = line.find(';'); if (semi != std::string::npos) line.erase(semi);
        line = trim_copy(line);
        if (line.empty()) continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = trim_copy(line.substr(0, eq));
        std::string val = trim_copy(line.substr(eq + 1));

        // remove optional surrounding quotes
        if (!val.empty() && ((val.front()=='"' && val.back()=='"') || (val.front()=='\'' && val.back()=='\''))) {
            val = val.substr(1, val.size()-2);
        }

        if (allowed.count(key) == 0) continue;        // skip non-whitelisted keys
        if (key == "OPENAI_API_KEY") continue;        // belt & suspenders

        // only set if not already set in real environment
        const char* cur = std::getenv(key.c_str());
        if (!cur || !*cur) setenv(key.c_str(), val.c_str(), 1);
    }

    std::cout << "Loaded runtime env from " << cfg.string() << "\n";
}

// Bootstrap environment so users don’t need to export anything manually.
// - loads OPENAI_API_KEY from $XDG_CONFIG_HOME/openai/api_key or ~/.config/openai/api_key if unset
// - loads additional runtime env from config/runtime.env (whitelisted keys only)
// - sets safe defaults for all runtime knobs if still unset
static void bootstrap_env_defaults() {
    // 1) read project config first (if present)
    load_runtime_env_from_config();

    // 2) defaults requested, only if still unset
    set_default_env("OPENAI_REALTIME_MODEL", "gpt-4o-realtime-preview");
    set_default_env("AUTO_RESPOND", "0");
    set_default_env("WAKEWORD_REQUIRED", "0");
    set_default_env("ASR_LANG", "auto");
    set_default_env("VAD_THRESHOLD_RMS", "900");
    set_default_env("PRINT_TRANSCRIPT", "1");

    // 3) API key from user config if needed (never from project config)
    const char* api_env = std::getenv("OPENAI_API_KEY");
    if (!api_env || !*api_env) {
        fs::path cfgHome = std::getenv("XDG_CONFIG_HOME")
                           ? fs::path(std::getenv("XDG_CONFIG_HOME"))
                           : (fs::path(std::getenv("HOME")) / ".config");
        fs::path keyPath = cfgHome / "openai" / "api_key";
        std::string key;
        if (read_file_trimmed(keyPath, key)) {
            setenv("OPENAI_API_KEY", key.c_str(), 1);
            std::cout << "Loaded OPENAI_API_KEY from " << keyPath.string() << "\n";
        }
    }
}

static void run_motion_by_phrase(const std::string& phrase) {
    std::string iface = std::getenv("ROBOT_IFACE") ? std::getenv("ROBOT_IFACE") : "eth0";
    fs::path motion_bin = self_dir() / "abdullah_elnahrawy_g1_motions";
    std::cout << "Executing motion via phrase: " << phrase << std::endl;
    std::string cmd = "\"" + motion_bin.string() + "\" " + iface + " --say \"" + phrase + "\"";
    int ret = std::system(cmd.c_str());
    if (ret != 0) std::cerr << "[WARN] runner exited with code " << ret << std::endl;
}

static std::string lowercase(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return (char)std::tolower(c); });
    return s;
}

static bool looks_arabic(const std::string& s) {
    for (size_t i=0;i<s.size();++i) {
        unsigned char c = s[i];
        if (c >= 0xD8 && c <= 0xDF) return true; // quick UTF-8 lead byte check
    }
    return false;
}

static std::string strip_robot_name(const std::string& in) {
    std::string s = in;
    auto low = lowercase(s);
    auto erase_all = [&](const std::string& needle){
        size_t pos=0;
        while((pos = low.find(needle, pos)) != std::string::npos){
            low.erase(pos, needle.size());
            s.erase(pos, needle.size());
        }
    };
    // English forms
    erase_all("hasan "); erase_all("hassan "); erase_all("ok hasan "); erase_all("hey hasan ");
    // Arabic common forms
    const char* ar_forms[] = {"يا حسن ", "ياحسن ", "حسن "};
    for (auto* f: ar_forms) {
        std::string fs = f; size_t p=0;
        while ((p = s.find(fs, p)) != std::string::npos) { s.erase(p, fs.size()); low.erase(p, fs.size()); }
    }
    while (!s.empty() && std::isspace((unsigned char)s.front())) s.erase(s.begin());
    while (!s.empty() && std::isspace((unsigned char)s.back()))  s.pop_back();
    return s;
}

struct MotionIntent {
    bool is_motion = false;
    std::string english_cmd;
};

static MotionIntent detect_motion_intent(const std::string& raw) {
    MotionIntent m;
    std::string t = strip_robot_name(raw);
    std::string tl = lowercase(t);

    // Arabic
    if (t.find("صافح") != std::string::npos || t.find("سلم علي") != std::string::npos || t.find("مصافحة") != std::string::npos) {
        m.is_motion = true; m.english_cmd = "shake hands"; return m;
    }
    if (t.find("تحية عسكرية") != std::string::npos || t.find("سلام عسكري") != std::string::npos || t.find("حيي") != std::string::npos) {
        m.is_motion = true; m.english_cmd = "perform military salute"; return m;
    }
    if (t.find("لوح") != std::string::npos || t.find("لوّح") != std::string::npos) { m.is_motion = true; m.english_cmd = "wave hand"; return m; }
    if (t.find("انحني") != std::string::npos || t.find("انحناء") != std::string::npos) { m.is_motion = true; m.english_cmd = "bow"; return m; }
    if (t.find("اجلس") != std::string::npos) { m.is_motion = true; m.english_cmd = "sit down"; return m; }
    if (t.find("قف") != std::string::npos || t.find("انهض") != std::string::npos) { m.is_motion = true; m.english_cmd = "stand up"; return m; }

    // English
    if (tl.find("shake hand") != std::string::npos || tl.find("handshake") != std::string::npos) { m.is_motion = true; m.english_cmd = "shake hands"; return m; }
    if (tl.find("salute") != std::string::npos) { m.is_motion = true; m.english_cmd = "perform military salute"; return m; }
    if (tl.find("wave") != std::string::npos) { m.is_motion = true; m.english_cmd = "wave hand"; return m; }
    if (tl.find("bow") != std::string::npos) { m.is_motion = true; m.english_cmd = "bow"; return m; }
    if (tl.find("sit") != std::string::npos) { m.is_motion = true; m.english_cmd = "sit down"; return m; }
    if (tl.find("stand") != std::string::npos) { m.is_motion = true; m.english_cmd = "stand up"; return m; }

    return m;
}

// ---------- playback ----------
static void feed_apm_aec(const std::vector<int16_t>& pcm16) {
    if (g_apm && !pcm16.empty()) g_apm->setPlaybackReference(pcm16.data(), pcm16.size());
}
static std::vector<int16_t> resample_24k_to_16k(const std::vector<int16_t>& in) {
    if (in.empty()) return {};
    double r = 16000.0/24000.0;
    size_t outN = static_cast<size_t>(std::ceil(in.size()*r));
    std::vector<int16_t> out; out.reserve(outN);
    for (size_t o=0;o<outN;++o) {
        double pos = o / r; size_t i = (size_t)std::floor(pos); double f = pos - i;
        if (i+1<in.size()) {
            double v = in[i]*(1.0-f) + in[i+1]*f;
            out.push_back((int16_t)std::clamp(v, -32768.0, 32767.0));
        } else out.push_back(in[i]);
    }
    return out;
}
static std::vector<int16_t> resample_16k_to_24k(const std::vector<int16_t>& in) {
    if (in.empty()) return {};
    double r = 24000.0/16000.0;
    size_t outN = static_cast<size_t>(std::ceil(in.size()*r));
    std::vector<int16_t> out; out.reserve(outN);
    for (size_t o=0;o<outN;++o) {
        double pos = o / r; size_t i = (size_t)std::floor(pos); double f = pos - i;
        if (i+1<in.size()) {
            double v = in[i]*(1.0-f) + in[i+1]*f;
            out.push_back((int16_t)std::clamp(v, -32768.0, 32767.0));
        } else out.push_back(in[i]);
    }
    return out;
}

static void onAssistantAudio(const std::vector<int16_t> &pcm24k_full) {
    if (pcm24k_full.empty()) return;
    if (cancel_playback.load()) return; // barge-in
    auto pcm16 = resample_24k_to_16k(pcm24k_full);
    feed_apm_aec(pcm16);
    {
        std::lock_guard<std::mutex> lk(queue_mtx);
        pcm_queue.push(std::move(pcm16));
    }
    queue_cv.notify_one();
}

static void clear_playback_queue() {
    std::lock_guard<std::mutex> lk(queue_mtx);
    std::queue<std::vector<int16_t>> empty; std::swap(pcm_queue, empty);
}

static void playback_thread_func() {
    while (!shutdown_requested) {
        std::unique_lock<std::mutex> lk(queue_mtx);
        queue_cv.wait(lk, []{ return !pcm_queue.empty() || shutdown_requested; });
        if (shutdown_requested) break;

        auto pcm16 = std::move(pcm_queue.front());
        pcm_queue.pop();
        lk.unlock();

        if (pcm16.empty() || !global_audio) continue;

        feed_apm_aec(pcm16);
        std::vector<uint8_t> bytes(reinterpret_cast<const uint8_t*>(pcm16.data()),
                                   reinterpret_cast<const uint8_t*>(pcm16.data()+pcm16.size()));
        std::string stream_id = "assistant_answer";
        std::string ts = std::to_string(unitree::common::GetCurrentTimeMillisecond());

        is_playing = true; cancel_playback.store(false);
        global_audio->LedControl(255,0,0);
        int ret = global_audio->PlayStream(stream_id, ts, bytes);
        if (ret != 0) std::cout << "[Play] Failed to start chunk, ret=" << ret << std::endl;

        double dur_s = (double)pcm16.size()/16000.0;
        auto t0 = std::chrono::steady_clock::now();
        while (!shutdown_requested && !cancel_playback.load()) {
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count();
            if (ms >= (long)(dur_s*1000.0)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
        global_audio->PlayStop(stream_id);

        if (cancel_playback.load()) { cancel_playback.store(false); std::cout << "[Playback] Cancelled by user speech.\n"; }

        is_playing = false;
        global_audio->LedControl(0,0,255);
    }
    std::cout << "[Playback] Thread stopped\n";
}

// ---------- UDP mic ----------
static std::string get_local_ip_for_multicast() {
    struct ifaddrs *ifaddr, *ifa; char host[NI_MAXHOST]; std::string result = "";
    getifaddrs(&ifaddr);
    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
        getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
        std::string ip(host);
        if (ip.find("192.168.123.") == 0) { result = ip; break; }
    }
    freeifaddrs(ifaddr);
    return result;
}

static void thread_mic() {
    std::cout << "[UDP Mic] Starting UDP multicast audio capture (continuous streaming with client APM + server VAD)...\n";
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { std::cerr << "[UDP Mic] ❌ socket\n"; return; }

    sockaddr_in local_addr{}; local_addr.sin_family = AF_INET; local_addr.sin_port = htons(PORT); local_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (sockaddr *)&local_addr, sizeof(local_addr)) < 0) { std::cerr << "[UDP Mic] ❌ bind\n"; close(sock); return; }

    ip_mreq mreq{}; inet_pton(AF_INET, GROUP_IP, &mreq.imr_multiaddr);
    std::string local_ip = get_local_ip_for_multicast();
    std::cout << "[UDP Mic] Local IP: " << local_ip << std::endl;
    mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());
    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) { std::cerr << "[UDP Mic] ❌ join group\n"; close(sock); return; }
    std::cout << "[UDP Mic] ✅ Successfully joined multicast group " << GROUP_IP << ":" << PORT << std::endl;

    const size_t input_chunk_size = 1600;  // 100ms at 16k
    std::vector<int16_t> chunk_buffer; chunk_buffer.reserve(input_chunk_size * 2);
    std::cout << "[UDP Mic] 🎤 Starting continuous audio capture and APM(HPF+gate+AEC) → 24kHz resample...\n";

    while (!shutdown_requested) {
        char buffer[2048];
        ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
        if (len > 0) {
            size_t sample_count = len / 2;
            const int16_t *samples = reinterpret_cast<const int16_t *>(buffer);
            chunk_buffer.insert(chunk_buffer.end(), samples, samples + sample_count);

            while (chunk_buffer.size() >= input_chunk_size) {
                std::vector<int16_t> chunk16(chunk_buffer.begin(), chunk_buffer.begin() + input_chunk_size);

                std::vector<int16_t> processed16;
                bool ok = true;
                if (g_apm) ok = g_apm->process(chunk16.data(), chunk16.size(), processed16);
                else processed16 = std::move(chunk16);

                if (ok && !processed16.empty() && rt) {
                    auto resampled = resample_16k_to_24k(processed16);
                    if (!resampled.empty() && !shutdown_requested) {
                        rt->sendAudioChunk(resampled.data(), resampled.size());
                    }
                }
                chunk_buffer.erase(chunk_buffer.begin(), chunk_buffer.begin() + input_chunk_size);
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    std::cout << "[UDP Mic] ✅ UDP microphone capture stopped\n";
    inet_pton(AF_INET, GROUP_IP, &mreq.imr_multiaddr);
    mreq.imr_interface.s_addr = inet_addr(get_local_ip_for_multicast().c_str());
    setsockopt(sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
    close(sock); sock = -1;
}

// ---------- utterance handler (no wakeword) ----------
static void onUtteranceCommit(const std::string& full_text) {
    if (full_text.empty()) return;

    // Simple noise filter
    std::string trimmed = full_text;
    while (!trimmed.empty() && isspace((unsigned char)trimmed.front())) trimmed.erase(trimmed.begin());
    while (!trimmed.empty() && isspace((unsigned char)trimmed.back()))  trimmed.pop_back();
    if (trimmed.size() < 2) return;

    // Barge-in: stop local playback and cancel server response if any
    if (is_playing.load()) {
        cancel_playback.store(true);
        clear_playback_queue();
    }
    if (rt && rt->responseInFlight()) {
        rt->cancelInFlight();
        std::this_thread::sleep_for(std::chrono::milliseconds(60)); // give server time to acknowledge
    }

    if (print_transcript) std::cout << "📝 [Utterance] " << trimmed << std::endl;

    // Route to motion or chat
    MotionIntent intent = detect_motion_intent(trimmed);
    if (intent.is_motion) {
        std::cout << "[Router] Motion intent → " << intent.english_cmd << std::endl;
        std::string phrase = "hasan, " + intent.english_cmd;
        run_motion_by_phrase(phrase);
        return;
    }

    // Speak back (low-latency streaming comes from realtime_openai.hpp)
    bool ar = looks_arabic(trimmed);
    std::string instr = ar
        ? std::string("أجب بإيجاز شديد (جملة أو جملتين) وبالعربية الفصحى. بدون مقدمات. نص المستخدم: ") + trimmed
        : std::string("Answer briefly (1–2 sentences) in English. No preambles. User said: ") + trimmed;

    // Guard: don’t stack responses
    if (rt && !rt->responseInFlight()) {
        rt->createSpeakResponse(instr, "alloy");
    }
}

// Optional: show user-only transcript deltas
static void onTranscriptDelta(const std::string& s) {
    if (print_transcript) std::cout << "📝 [Transcript] " << s << std::endl;
}

// ---------- main ----------
int main(int argc, char **argv) {
    std::cout << PROJECT_NAME << " — " << PROJECT_AUTHOR << " (" << PROJECT_SEMVER << ")\n";
    std::cout << "Contact: " << PROJECT_EMAIL << " | " << PROJECT_GITHUB << "\n";
    if (g_brand_fingerprint) std::cout << "Fingerprint: " << g_brand_fingerprint << "\n";
    if (argc < 2) { std::cout << "Usage: voice_interface [NetWorkInterface(eth0)]\n"; return 0; }
    setenv("ROBOT_IFACE", argv[1], 1);

    // <<< ensure defaults & load API key from user config; also load config/runtime.env
    bootstrap_env_defaults();

    wake_required = (std::getenv("WAKEWORD_REQUIRED") && std::string(std::getenv("WAKEWORD_REQUIRED"))=="1");
    print_transcript = (std::getenv("PRINT_TRANSCRIPT") && std::string(std::getenv("PRINT_TRANSCRIPT"))=="1");

    signal(SIGINT,  [](int){ shutdown_requested=true; queue_cv.notify_all(); std::cout << "\n[Signal] Received signal 2, shutting down gracefully...\n"; });
    signal(SIGTERM, [](int){ shutdown_requested=true; queue_cv.notify_all(); std::cout << "\n[Signal] Received signal 15, shutting down gracefully...\n"; });

    std::cout << "=== Enhanced Real-Time Voice Chat System (Router Mode) ===\n";
    std::cout << "Features: Multi-threaded audio processing, real-time analysis, motion routing (no wakeword by default)\n";

    try {
        std::cout << "[Init] Initializing Unitree middleware on " << argv[1] << "...\n";
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

        unitree::robot::g1::AudioClient audio;
        audio.Init(); audio.SetTimeout(10.0f); global_audio = &audio;
        std::cout << "[Init] Unitree audio client initialized\n";
        audio.SetVolume(100);
        audio.LedControl(0,0,255);

        // Local APM 16 kHz
        g_apm = std::make_unique<AudioProcessingPipeline>(16000);
        if (const char* thr = std::getenv("VAD_THRESHOLD_RMS")) { int v = std::atoi(thr); if (v>0) g_apm->setNoiseGateRms(float(v)/32768.0f); }
        if (const char* hpf = std::getenv("AUDIO_HPF_CUT_HZ")) { int hz = std::atoi(hpf); if (hz>10) g_apm->setHighpassCutHz(float(hz)); }

        // AudioManager (for LED/streams + AEC ref)
        AudioManager::AudioConfig config; config.sample_rate=24000; config.channels=1; config.chunk_size=2400; config.buffer_size=48000;
        auto am = std::make_unique<AudioManager>(&audio, config);
        if (!am->start()) { std::cerr << "[Error] Failed to start AudioManager\n"; return 1; }
        std::cout << "[Init] AudioManager started with multi-threaded processing\n";

        const char* apiKey = getenv("OPENAI_API_KEY");
        if (!apiKey || !*apiKey) { std::cerr << "[Error] OPENAI_API_KEY not set.\n"; return 1; }

        std::cout << "[Init] Connecting to voice server (OpenAI protocol)...\n";
        rt = std::make_unique<RealtimeOpenAI>(apiKey, onAssistantAudio, onTranscriptDelta, onUtteranceCommit);
        rt->setResponseGate([](){ return true; });
        if (!rt->start()) { std::cerr << "[Error] Failed to start Realtime session.\n"; return 1; }
        std::cout << "[Init] ✅ Realtime session established\n";

        playback_thread = std::thread(playback_thread_func);
        udp_mic_thread  = std::thread([](){ thread_mic(); });

        std::cout << "\n=== System Ready ===\n";
        std::cout << "🎤 Audio capture: ACTIVE (client HPF+gate+AEC → server VAD)\n";
        std::cout << "📝 Utterance routing: ACTIVE (Arabic/English, no wakeword)\n";
        std::cout << "📡 OpenAI streaming: ACTIVE (buffered TTS playback)\n";
        std::cout << "🤖 Motion routing: ACTIVE (via runner --say)\n";
        std::cout << "Wake word required: " << (wake_required ? "YES" : "NO") << "\n";
        std::cout << "Press Ctrl-C to quit\n\n";

        while (!shutdown_requested) std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (udp_mic_thread.joinable()) { if (sock >= 0) { close(sock); sock = -1; } udp_mic_thread.join(); }
        if (playback_thread.joinable()) { queue_cv.notify_all(); playback_thread.join(); }
        am->stop(); am.reset();
        if (rt) { rt->stop(); rt.reset(); }
        global_audio = nullptr;
        std::cout << "[Shutdown] ✅ Graceful shutdown complete\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "[Fatal Error] " << e.what() << "\n";
        return 1;
    }
}
