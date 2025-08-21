#pragma once

#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>
#include <boost/asio/ssl.hpp>

#include <nlohmann/json.hpp>
#include <robot/third_party/base64.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <cstring>   // for memcpy

class RealtimeOpenAI {
public:
    using AudioCallback      = std::function<void(const std::vector<int16_t>&)>; // 24k PCM, delivered once per reply
    using TranscriptCallback = std::function<void(const std::string&)>;          // user transcript deltas (optional)
    using UtteranceCallback  = std::function<void(const std::string&)>;          // final user utterance (once per VAD)
    using ToolCallback       = std::function<void(const std::string& /*name*/, const nlohmann::json& /*args*/)>;

    RealtimeOpenAI(const std::string& api_key,
                   const AudioCallback& onAudio,
                   const TranscriptCallback& onTranscript = nullptr,
                   const UtteranceCallback& onUtterance = nullptr)
        : api_key_(api_key), onAudio_(onAudio), onTranscript_(onTranscript), onUtterance_(onUtterance) {
        const char* ar = std::getenv("AUTO_RESPOND");
        auto_respond_ = (ar && (*ar=='1' || std::string(ar)=="on"));
    }

    ~RealtimeOpenAI() { stop(); }

    void setInstructions(const std::string& s) { instructions_ = s; }
    void setResponseGate(std::function<bool()> fn) { responseGate_ = std::move(fn); }
    void setToolCallback(const ToolCallback& cb) { tool_cb_ = cb; }

    // === RESTORED PUBLIC APIS (used by voice_interface.cpp) ===
    // Provide a compact JSON summary of motions_registry.yaml so the model can match on-device definitions
    // Format suggestion: {"motions":[{"file":"...", "threshold":0.8, "triggers":["...","..."]}, ...]}
    void setMotionRegistrySummary(const std::string& json_summary) { motion_registry_summary_ = json_summary; }
    
    // Ask OpenAI to arbitrate intent (motion vs. speak) and, if motion, PICK a file from the provided registry.
    // It should call tool `run_motion` with { phrase: "...", file: "..." }.
    void requestMotionArbitration(const std::string& utterance) {
        if (!conn_.lock()) return;

        std::string reg = motion_registry_summary_.empty() ? std::string("{\"motions\":[]}") : motion_registry_summary_;

        std::string instr =
            std::string(
                "You control a humanoid robot named Hasan.\n"
                "TASK: Decide intent and act:\n"
                "  • If the user's utterance is an imperative/request for a PHYSICAL GESTURE/MOTION, "
                "call tool `run_motion`.\n"
                "    - Use the following registry (JSON) as the authoritative list:\n"
                "      ") + reg + "\n"
                "    - Pick the best match, respecting each motion's threshold notionally; include:\n"
                "        { \"phrase\": short English name of the motion, \"file\": exact file from registry }\n"
                "    - Do NOT produce any spoken/textual answer in that case.\n"
                "  • Otherwise (it's Q&A, chit-chat, info, etc.), answer briefly (1–2 sentences) and SPEAK.\n"
                "User utterance: \"" + utterance + "\"";

        nlohmann::json j = {
            {"type","response.create"},
            {"response", {
                {"instructions", instr},
                {"voice", "alloy"},
                {"modalities", {"audio","text"}}
            }}
        };
        client_.send(conn_, j.dump(), websocketpp::frame::opcode::text);
    }

    // === START/STOP the WebSocket client ==

    bool start() {
        try {
            client_.clear_access_channels(websocketpp::log::alevel::all);
            client_.init_asio();
            client_.start_perpetual();

            client_.set_open_handler([this](websocketpp::connection_hdl hdl) {
                conn_ = hdl;
                sendSessionUpdate();
            });

            client_.set_message_handler([this](websocketpp::connection_hdl,
                                              websocketpp::config::asio_tls_client::message_type::ptr msg) {
                handleMessage(msg->get_payload());
            });

            client_.set_tls_init_handler([](websocketpp::connection_hdl) {
                auto ctx = std::make_shared<boost::asio::ssl::context>(boost::asio::ssl::context::tlsv12_client);
                ctx->set_options(boost::asio::ssl::context::default_workarounds |
                                 boost::asio::ssl::context::no_sslv2 |
                                 boost::asio::ssl::context::no_sslv3 |
                                 boost::asio::ssl::context::single_dh_use);
                return ctx;
            });

            const char* env_model = std::getenv("OPENAI_REALTIME_MODEL");
            std::string model = (env_model && *env_model) ? env_model : "gpt-4o-realtime-preview";
            std::string uri = "wss://api.openai.com/v1/realtime?model=" + model;
            printf("[RealtimeOpenAI] Using model: %s\n", model.c_str());

            websocketpp::lib::error_code ec;
            auto con = client_.get_connection(uri, ec);
            if (ec) {
                printf("[RealtimeOpenAI] get_connection error: %s\n", ec.message().c_str());
                return false;
            }
            if (!api_key_.empty()) {
                con->replace_header("Authorization", std::string("Bearer ") + api_key_);
                con->replace_header("OpenAI-Beta", "realtime=v1");
            } else {
                printf("[RealtimeOpenAI] ⚠️  No API key provided\n");
            }

            client_.connect(con);
            ws_thread_ = std::thread([this]() { client_.run(); });
            return true;
        } catch (const std::exception& e) {
            printf("[RealtimeOpenAI] start() exception: %s\n", e.what());
            return false;
        }
    }

    void stop() {
        if (!stopped_.exchange(true)) {
            try { if (conn_.lock()) client_.close(conn_, websocketpp::close::status::normal, "bye"); } catch (...) {}
            client_.stop_perpetual();
            if (ws_thread_.joinable()) ws_thread_.join();
        }
    }

    // Mic → server
    void sendAudioChunk(const int16_t* data, size_t sampleCount) {
        if (!conn_.lock()) return;
        std::string b64 = base64_encode(reinterpret_cast<const unsigned char*>(data),
                                        static_cast<unsigned int>(sampleCount * sizeof(int16_t)), false);
        nlohmann::json j = { {"type", "input_audio_buffer.append"}, {"audio", b64} };
        client_.send(conn_, j.dump(), websocketpp::frame::opcode::text);
    }

    // Ask the model to SPEAK a reply (audio+text)
    void createSpeakResponse(const std::string& instructions, const std::string& voice = "alloy") {
        if (!conn_.lock() || instructions.empty()) return;
        nlohmann::json j = {
            {"type","response.create"},
            {"response", {
                {"instructions", instructions},
                {"voice", voice},
                {"modalities", {"audio","text"}}
            }}
        };
        client_.send(conn_, j.dump(), websocketpp::frame::opcode::text);
    }

    // Cancel an in-flight server response (for barge-in)
    void cancelInFlight() {
        if (!conn_.lock()) return;
        if (inflight_id_.empty()) return;
        nlohmann::json j = { {"type","response.cancel"}, {"response_id", inflight_id_} };
        client_.send(conn_, j.dump(), websocketpp::frame::opcode::text);
    }

    bool responseInFlight() const { return response_inflight_.load(); }

private:
    void sendSessionUpdate() {
        std::string asr = "auto";
        if (const char* e = std::getenv("ASR_LANG")) { if (*e) asr = e; }

        nlohmann::json transcribe = { {"model","gpt-4o-mini-transcribe"} };
        if (asr == "en" || asr == "zh" || asr == "ar") transcribe["language"] = asr;

        std::string instr = instructions_.empty()
          ? "You control a humanoid robot named Hasan.\n"
            "- If the user issues an imperative or request to perform a PHYSICAL GESTURE/MOTION (e.g., handshake, military salute, wave, bow, sit, stand, welcome visitors, etc.), call the tool `run_motion` with a short English `phrase` describing the motion (e.g., \"shake hands\", \"perform military salute\").\n"
            "- Otherwise, answer briefly and clearly (1–2 sentences) and speak.\n"
            "Do not require a wake word unless the user uses it."
          : instructions_;

        // NEW: Surface the registry at session level so the model can select an exact file
        if (!motion_registry_summary_.empty()) {
            instr += "\n\nMotion registry (read-only JSON, authoritative for matching):\n";
            instr += motion_registry_summary_;
        }

        // Expose a single function-style tool the model can call
        nlohmann::json tools = nlohmann::json::array({
            {
                {"type","function"},
                {"name","run_motion"},
                {"description","Trigger a predefined robot motion by phrase and file."},
                {"parameters", {
                    {"type","object"},
                    {"properties", {
                        {"phrase", {{"type","string"}, {"description","Concise English instruction for the motion, e.g., 'shake hands', 'perform military salute', 'welcome visitors'."}}},
                        {"file",   {{"type","string"}, {"description","Exact file name from the registry, e.g., 'military_salute.seq'."}}}
                    }},
                    {"required", {"phrase","file"}}
                }}
            }
        });

        nlohmann::json cfg = {
            {"type", "session.update"},
            {"session", {
                {"voice", "alloy"},
                {"input_audio_format", "pcm16"},
                {"output_audio_format", "pcm16"},
                {"turn_detection", {{"type", "server_vad"}}},
                {"input_audio_transcription", transcribe},
                {"instructions", instr},
                {"tools", tools}
            }}
        };
        client_.send(conn_, cfg.dump(), websocketpp::frame::opcode::text);
        printf("✅ [Session] Configuration sent to server (ASR_LANG=%s, AUTO_RESPOND=%s)\n",
               asr.c_str(), auto_respond_ ? "on" : "off");
    }

    // --------- Audio accumulation helpers (NEW) ---------
    static inline int16_t f32_to_i16(float v) {
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        return static_cast<int16_t>(v * 32767.0f);
    }

    void append_pcm16_base64_(const std::string& b64) {
        std::string raw = base64_decode(b64);
        if (raw.size() < 2) return;
        size_t n = raw.size() / 2;
        const int16_t* p = reinterpret_cast<const int16_t*>(raw.data());
        audio_accum_.insert(audio_accum_.end(), p, p + n);
    }

    void append_float_array_(const nlohmann::json& arr) {
        if (!arr.is_array() || arr.empty()) return;
        audio_accum_.reserve(audio_accum_.size() + arr.size());
        for (const auto& v : arr) {
            float f = 0.0f;
            if (v.is_number_float())        f = static_cast<float>(v.get<double>());
            else if (v.is_number_integer()) f = static_cast<float>(v.get<long long>());
            else                             continue;
            audio_accum_.push_back(f32_to_i16(f));
        }
    }

    void maybe_accumulate_audio_(const nlohmann::json& j, const char* field) {
        if (!j.contains(field)) return;
        const auto& x = j[field];
        if (x.is_string()) {
            append_pcm16_base64_(x.get<std::string>());
        } else if (x.is_array()) {
            append_float_array_(x);
        }
    }

    void flush_accumulated_audio_once_() {
        if (emitted_this_reply_) return;
        if (!audio_accum_.empty() && onAudio_) {
            onAudio_(audio_accum_); // one contiguous 24k PCM16 vector
        }
        emitted_this_reply_ = true;
        audio_accum_.clear();
        audio_accum_.shrink_to_fit(); // prevent memory growth across turns
    }

    void reset_reply_audio_state_() {
        audio_accum_.clear();
        emitted_this_reply_ = false;
    }

    void handleMessage(const std::string& payload) {
        try {
            auto j = nlohmann::json::parse(payload);
            if (!j.contains("type")) return;
            const std::string type = j["type"].get<std::string>();

            // =================== ASSISTANT RESPONSES ===================
            if (type.rfind("response.", 0) == 0) {
                if (type == "response.created") {
                    reset_reply_audio_state_(); // NEW: start fresh for this reply

                    if (j.contains("response") && j["response"].is_object() && j["response"].contains("id") && j["response"]["id"].is_string())
                        inflight_id_ = j["response"]["id"].get<std::string>();
                    else if (j.contains("id") && j["id"].is_string())
                        inflight_id_ = j["id"].get<std::string>();
                    response_inflight_.store(true);
                    return;
                }

                // ======== AUDIO CHUNKS (ACCUMULATION) ========
                if ((type == "response.output_audio.delta" || type == "response.audio.delta") && j.contains("delta")) {
                    maybe_accumulate_audio_(j, "delta");
                    return;
                }
                if (type == "response.output_audio" && j.contains("audio")) {
                    maybe_accumulate_audio_(j, "audio");
                    return;
                }
                if (type == "response.delta" && j.contains("output") && j["output"].is_array()) {
                    for (const auto& out : j["output"]) {
                        if (out.contains("content") && out["content"].is_array()) {
                            for (const auto& c : out["content"]) {
                                if (c.contains("type") && c["type"].is_string() && c["type"] == "audio") {
                                    if (c.contains("audio")) maybe_accumulate_audio_(c, "audio");
                                    if (c.contains("data"))  maybe_accumulate_audio_(c, "data");
                                }
                            }
                        }
                    }
                    return;
                }

                // ======== Tool/function call handling (unchanged) ========
                if (tool_cb_) {
                    std::string fname;
                    nlohmann::json fargs;

                    if ((type.find("response.function_call") != std::string::npos ||
                         type.find("response.tool_call") != std::string::npos ||
                         type.find("arguments") != std::string::npos)) {

                        if (j.contains("name") && j["name"].is_string()) {
                            fname = j["name"].get<std::string>();
                        } else if (j.contains("function_call") && j["function_call"].is_object() &&
                                   j["function_call"].contains("name") && j["function_call"]["name"].is_string()) {
                            fname = j["function_call"]["name"].get<std::string>();
                        }

                        if (j.contains("arguments")) {
                            if (j["arguments"].is_string()) {
                                try { fargs = nlohmann::json::parse(j["arguments"].get<std::string>()); }
                                catch (...) { /* ignore parse error */ }
                            } else if (j["arguments"].is_object()) {
                                fargs = j["arguments"];
                            }
                        } else if (j.contains("function_call") && j["function_call"].is_object() &&
                                   j["function_call"].contains("arguments")) {
                            const auto& a = j["function_call"]["arguments"];
                            if (a.is_string()) {
                                try { fargs = nlohmann::json::parse(a.get<std::string>()); } catch (...) {}
                            } else if (a.is_object()) {
                                fargs = a;
                            }
                        }

                        if (!fname.empty()) {
                            tool_cb_(fname, fargs);
                            return;
                        }
                    }
                }

                // ======== Completion / cancellation: emit once and reset ========
                if (type == "response.audio.done") {
                    flush_accumulated_audio_once_();
                    return;
                }
                if (type == "response.canceled" || type == "response.completed" ||
                    type == "response.error"    || type == "response.failed" ||
                    type == "response.done") {
                    flush_accumulated_audio_once_();
                    response_inflight_.store(false);
                    inflight_id_.clear();
                    return;
                }
                return;
            }

            // =================== USER TRANSCRIPTION ONLY ===================
            if (type == "input_audio_buffer.started") {
                user_buf_.clear();
                have_final_text_ = false;
                return;
            }

            const bool user_delta =
                (type.find("input_audio_transcription.delta") != std::string::npos) ||
                (type.find("conversation.item.input_audio_transcription.delta") != std::string::npos);

            const bool user_completed =
                (type.find("input_audio_transcription.completed") != std::string::npos) ||
                (type.find("conversation.item.input_audio_transcription.completed") != std::string::npos);

            if (user_delta) {
                if (j.contains("delta") && j["delta"].is_string()) {
                    user_buf_ += j["delta"].get<std::string>();
                    if (onTranscript_) onTranscript_(j["delta"].get<std::string>());
                } else if (j.contains("text") && j["text"].is_string()) {
                    user_buf_ += j["text"].get<std::string>();
                    if (onTranscript_) onTranscript_(j["text"].get<std::string>());
                }
                return;
            }

            if (user_completed) {
                if (j.contains("text") && j["text"].is_string()) {
                    final_text_ = j["text"].get<std::string>();
                    have_final_text_ = true;
                } else if (j.contains("transcript") && j["transcript"].is_string()) {
                    final_text_ = j["transcript"].get<std::string>();
                    have_final_text_ = true;
                }
                return;
            }

            // VAD turn end → deliver user utterance (once)
            if (type == "input_audio_buffer.committed") {
                bool allow = true;
                if (responseGate_) { try { allow = responseGate_(); } catch (...) { allow = true; } }

                if (onUtterance_ && allow) {
                    std::string text = have_final_text_ ? final_text_ : user_buf_;
                    if (!text.empty()) {
                        auto mid = text.size()/2;
                        if (text.size()%2==0 && text.substr(0,mid)==text.substr(mid)) text = text.substr(0,mid);
                    }
                    onUtterance_(text);
                }
                user_buf_.clear();
                final_text_.clear();
                have_final_text_ = false;

                if (auto_respond_ && allow) {
                    nlohmann::json r = { {"type","response.create"}, {"response", {{"modalities", {"text"}}}} };
                    client_.send(conn_, r.dump(), websocketpp::frame::opcode::text);
                }
                return;
            }

            if (type == "error") {
                std::string error_msg = j.contains("message") && j["message"].is_string() ? j["message"].get<std::string>() : "Unknown error";
                if (j.contains("error") && j["error"].is_object() && j["error"].contains("message") && j["error"]["message"].is_string())
                    error_msg = j["error"]["message"].get<std::string>();
                printf("❌ [Error] %s\n", error_msg.c_str());
                return;
            }

        } catch (const std::exception& e) {
            printf("[OpenAI] ❌ parse: %s\n", e.what());
        }
    }

private:
    using client_t = websocketpp::client<websocketpp::config::asio_tls_client>;
    client_t client_;
    websocketpp::connection_hdl conn_;
    std::thread ws_thread_;
    std::atomic<bool> stopped_{false};

    std::string api_key_;
    AudioCallback onAudio_;
    TranscriptCallback onTranscript_;
    UtteranceCallback onUtterance_;
    ToolCallback tool_cb_;
    std::function<bool()> responseGate_;
    std::string instructions_;
    bool auto_respond_ = false;

    // user buffers/state
    std::string user_buf_;
    std::string final_text_;
    bool have_final_text_ = false;

    // assistant state
    std::atomic<bool> response_inflight_{false};
    std::string inflight_id_;

    // NEW: motion registry JSON provided by app
    std::string motion_registry_summary_;

    // NEW: per-reply audio accumulator
    std::vector<int16_t> audio_accum_;
    bool emitted_this_reply_ = false;
};
