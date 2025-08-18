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

class RealtimeOpenAI {
public:
    using AudioCallback      = std::function<void(const std::vector<int16_t>&)>; // 24k PCM, may be small streaming chunks
    using TranscriptCallback = std::function<void(const std::string&)>;          // user transcript deltas (optional)
    using UtteranceCallback  = std::function<void(const std::string&)>;          // final user utterance (once per VAD)

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
          ? "Transcribe only. Do not autonomously answer; only speak when I send response.create."
          : instructions_;

        nlohmann::json cfg = {
            {"type", "session.update"},
            {"session", {
                {"voice", "alloy"},
                {"input_audio_format", "pcm16"},
                {"output_audio_format", "pcm16"},
                {"turn_detection", {{"type", "server_vad"}}},
                {"input_audio_transcription", transcribe},
                {"instructions", instr}
            }}
        };
        client_.send(conn_, cfg.dump(), websocketpp::frame::opcode::text);
        printf("✅ [Session] Configuration sent to server (ASR_LANG=%s, AUTO_RESPOND=%s)\n",
               asr.c_str(), auto_respond_ ? "on" : "off");
    }

    void handleMessage(const std::string& payload) {
        try {
            auto j = nlohmann::json::parse(payload);
            if (!j.contains("type")) return;
            const std::string type = j["type"].get<std::string>();

            // =================== ASSISTANT RESPONSES ===================
            if (type.rfind("response.", 0) == 0) {
                // Track response id state
                if (type == "response.created") {
                    if (j.contains("response") && j["response"].is_object() && j["response"].contains("id") && j["response"]["id"].is_string())
                        inflight_id_ = j["response"]["id"].get<std::string>();
                    else if (j.contains("id") && j["id"].is_string())
                        inflight_id_ = j["id"].get<std::string>();
                    response_inflight_.store(true);
                    return;
                }

                // Stream audio deltas immediately to the speaker (low latency)
                if ((type == "response.output_audio.delta" || type == "response.audio.delta") &&
                    j.contains("delta") && j["delta"].is_string()) {
                    std::string raw = base64_decode(j["delta"].get<std::string>());
                    if (raw.size() >= 2 && onAudio_) {
                        const int16_t* p = reinterpret_cast<const int16_t*>(raw.data());
                        size_t n = raw.size() / 2;
                        onAudio_(std::vector<int16_t>(p, p + n)); // 24k PCM
                    }
                    return;
                }

                // Some servers may send a non-delta lump; support it too.
                if (type == "response.output_audio" && j.contains("audio") && j["audio"].is_string()) {
                    std::string raw = base64_decode(j["audio"].get<std::string>());
                    if (raw.size() >= 2 && onAudio_) {
                        const int16_t* p = reinterpret_cast<const int16_t*>(raw.data());
                        size_t n = raw.size() / 2;
                        onAudio_(std::vector<int16_t>(p, p + n));
                    }
                    return;
                }

                if (type == "response.canceled" || type == "response.completed" ||
                    type == "response.error"    || type == "response.failed") {
                    response_inflight_.store(false);
                    inflight_id_.clear();
                    return;
                }
                return;
            }

            // =================== USER TRANSCRIPTION ONLY ===================
            // Reset buffer when a new input buffer starts
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
                // Some backends send the whole text here. Prefer it and mark final.
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
                    // de-duplicate accidental doubles like "X.X"
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
};
