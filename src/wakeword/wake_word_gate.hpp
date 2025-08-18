#pragma once
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <chrono>
#include <algorithm>

class WakeWordGate {
public:
    // Initialize from env:
    //  WAKEWORD_REQUIRED=1|0
    //  WAKEWORD="hasan"
    //  WAKEWORD_ALIASES="hasan,hassan,حسن,يا حسن,hey hasan"
    //  WAKE_WINDOW_MS=5000  (how long gate stays open after a wake)
    static void init();

    // Feed with every final transcript to detect wake words
    static void onTranscriptDelta(const std::string& text);

    // Is the gate open right now?
    static bool allowStreaming();

    // Force-open (used when we detect a clear intent like “shake hands”)
    static void openForMs(int ms);

private:
    static bool required_;
    static std::vector<std::string> tokens_;
    static std::atomic<long long> open_until_ms_;
    static int window_ms_;

    static long long now_ms();
    static bool contains_arabic_(const std::string& s);
    static std::string to_lower_(std::string s);
};
