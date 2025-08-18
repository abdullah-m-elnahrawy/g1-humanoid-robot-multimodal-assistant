#include "wake_word_gate.hpp"
#include <cstdlib>

bool WakeWordGate::required_ = true;
std::vector<std::string> WakeWordGate::tokens_;
std::atomic<long long> WakeWordGate::open_until_ms_{0};
int WakeWordGate::window_ms_ = 5000;

long long WakeWordGate::now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out; std::string cur;
    for (char c : s) {
        if (c==',' ) { 
            if (!cur.empty()) { out.push_back(cur); cur.clear(); }
        } else if (c!=' ' && c!='\t' && c!='\n' && c!='\r') {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

std::string WakeWordGate::to_lower_(std::string s){
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return s;
}

bool WakeWordGate::contains_arabic_(const std::string& s) {
    for (size_t i=0;i<s.size();) {
        unsigned char c = s[i];
        if (c < 0x80) { i++; continue; }
        int extra = (c & 0xE0) == 0xC0 ? 1 : (c & 0xF0) == 0xE0 ? 2 : 3;
        unsigned cp = c & (0x1F >> (extra - 1));
        i++;
        while (extra-- && i < s.size()) { cp = (cp << 6) | (s[i++] & 0x3F); }
        if ((cp >= 0x0600 && cp <= 0x06FF) || (cp >= 0x0750 && cp <= 0x08FF) ||
            (cp >= 0xFB50 && cp <= 0xFDFF) || (cp >= 0xFE70 && cp <= 0xFEFF)) return true;
    }
    return false;
}

void WakeWordGate::init() {
    // Required?
    const char* req = std::getenv("WAKEWORD_REQUIRED");
    required_ = !(req && std::string(req)=="0");

    // Window
    if (const char* w = std::getenv("WAKE_WINDOW_MS")) {
        int v = std::atoi(w);
        if (v >= 1000 && v <= 20000) window_ms_ = v;
    }

    // Base wake word
    std::string base = "hasan";
    if (const char* ww = std::getenv("WAKEWORD")) {
        if (*ww) base = ww;
    }

    // Aliases
    tokens_.clear();
    tokens_.push_back(to_lower_(base));
    if (const char* al = std::getenv("WAKEWORD_ALIASES")) {
        for (auto& t : split_csv(al)) tokens_.push_back(to_lower_(t));
    }

    // Always add common romanizations and Arabic forms for Hasan
    const char* defaults[] = {"hassan","hasen","hasan.","hey hasan","ya hasan","ياحسن","يا حسن","حسن","ياحسن!"};
    for (auto* s : defaults) tokens_.push_back(to_lower_(s));

    // De-duplicate
    std::sort(tokens_.begin(), tokens_.end());
    tokens_.erase(std::unique(tokens_.begin(), tokens_.end()), tokens_.end());
}

void WakeWordGate::openForMs(int ms) {
    long long until = now_ms() + ms;
    long long cur = open_until_ms_.load();
    while (until > cur && !open_until_ms_.compare_exchange_weak(cur, until)) {}
}

bool WakeWordGate::allowStreaming() {
    if (!required_) return true;
    return now_ms() < open_until_ms_.load();
}

void WakeWordGate::onTranscriptDelta(const std::string& text) {
    if (!required_) return;

    std::string l = to_lower_(text);
    // quick strip punctuation we see often
    for (char& ch : l) if (std::string(".,;:!?\"'()[]{}|/\\").find(ch)!=std::string::npos) ch = ' ';

    // If the transcript *starts* with a wake token or contains “hey/ya <token>”, open
    for (const auto& t : tokens_) {
        if (t.empty()) continue;
        // direct prefix
        if (l.rfind(t, 0) == 0) { openForMs(window_ms_); return; }
        // with “hey/ya …” in EN or AR
        if (l.find("hey " + t) != std::string::npos || l.find("ya " + t) != std::string::npos ||
            l.find("يا " + t) != std::string::npos) { openForMs(window_ms_); return; }
        // Arabic exact token present (already lowercased for ascii; arabic unaffected)
        if (l.find(t) != std::string::npos && contains_arabic_(t)) { openForMs(window_ms_); return; }
    }
}
