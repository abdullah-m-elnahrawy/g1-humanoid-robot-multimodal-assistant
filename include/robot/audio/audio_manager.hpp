#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <chrono>
#include <iostream>
#include <mutex>
#include <cmath>
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

#ifdef ENABLE_UNITREE_SDK
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#endif

#include <robot/audio/audio_buffer.hpp>
#include <robot/audio/audio_analyzer.hpp>


/**
 * Main audio manager coordinating capture, processing, and playback
 * Implements the threading architecture for real-time audio processing
 */
class AudioManager {
public:
    struct AudioConfig {
        size_t sample_rate = 24000;
        size_t channels = 1;
        size_t chunk_size = 2400;  // 100ms at 24kHz
        size_t buffer_size = 48000; // 2 seconds buffer
    };

    using AudioCallback = std::function<void(const int16_t*, size_t)>;
    using AnalysisCallback = std::function<void(const AudioAnalyzer::AnalysisResult&)>;
    using LevelCallback = std::function<void(const AudioLevelMeter::LevelInfo&)>;

    AudioManager(
#ifdef ENABLE_UNITREE_SDK
        unitree::robot::g1::AudioClient* audio_client, 
        const AudioConfig& config)
        : audio_client_(audio_client)
        , config_(config)
        , running_(false)
        , input_buffer_(config.buffer_size)
        , output_buffer_(config.buffer_size)
        , chunk_queue_(100)
        , analyzer_(1024)
        , level_meter_(10.0f, 100.0f, config.sample_rate) {
#else
        const AudioConfig& config)
        : config_(config)
        , running_(false)
        , input_buffer_(config.buffer_size)
        , output_buffer_(config.buffer_size)
        , chunk_queue_(100)
        , analyzer_(1024)
        , level_meter_(10.0f, 100.0f, config.sample_rate) {
#endif
        
        // Pre-allocate buffers
        chunk_buffer_.resize(config_.chunk_size);
        temp_buffer_.resize(config_.chunk_size);
    }

    ~AudioManager() {
        stop();
    }

    bool start() {
        if (running_.exchange(true)) {
            return true; // Already running
        }

        try {
            // Start audio processing threads
            capture_thread_ = std::thread(&AudioManager::captureThread, this);
            processing_thread_ = std::thread(&AudioManager::processingThread, this);
            network_thread_ = std::thread(&AudioManager::networkThread, this);
            
            // Set thread priorities (platform specific)
            #ifdef __linux__
            // Set real-time priority for capture thread
            struct sched_param param;
            param.sched_priority = 80;
            pthread_setschedparam(capture_thread_.native_handle(), SCHED_FIFO, &param);
            #endif

            std::cout << "[AudioManager] Started with " << config_.sample_rate 
                     << "Hz, chunk size: " << config_.chunk_size << " samples\n";
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "[AudioManager] Failed to start: " << e.what() << std::endl;
            running_ = false;
            return false;
        }
    }

    void stop() {
        if (!running_.exchange(false)) {
            return; // Already stopped
        }

        // Wait for threads to finish
        if (capture_thread_.joinable()) capture_thread_.join();
        if (processing_thread_.joinable()) processing_thread_.join();
        if (network_thread_.joinable()) network_thread_.join();

        std::cout << "[AudioManager] Stopped\n";
    }

    // Set callbacks
    void setAudioCallback(const AudioCallback& callback) { audio_callback_ = callback; }
    void setAnalysisCallback(const AnalysisCallback& callback) { analysis_callback_ = callback; }
    void setLevelCallback(const LevelCallback& callback) { level_callback_ = callback; }

    // Play incoming audio from assistant
    void playAudio(const int16_t* data, size_t samples) {
        size_t written = output_buffer_.write(data, samples);
        if (written < samples) {
            std::cout << "[AudioManager] Output buffer overflow, dropped " 
                     << (samples - written) << " samples\n";
        }
    }

    // Get current audio levels
    AudioLevelMeter::LevelInfo getCurrentLevels() const {
        return current_levels_;
    }

    // Get latest analysis result
    AudioAnalyzer::AnalysisResult getLatestAnalysis() const {
        std::lock_guard<std::mutex> lock(analysis_mutex_);
        return latest_analysis_;
    }

private:
    void captureThread() {
        std::cout << "[AudioManager] Capture thread started\n";
        
        auto last_time = std::chrono::steady_clock::now();
        const auto chunk_duration = std::chrono::microseconds(
            (config_.chunk_size * 1000000) / config_.sample_rate);

        while (running_) {
            try {
                // Simulate microphone capture - replace with actual Unitree capture
                // For now, we'll capture from a simple source or generate test signal
                captureAudioChunk();

                // Maintain timing
                last_time += chunk_duration;
                std::this_thread::sleep_until(last_time);
                
            } catch (const std::exception& e) {
                std::cerr << "[AudioManager] Capture error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        std::cout << "[AudioManager] Capture thread stopped\n";
    }

    void processingThread() {
        std::cout << "[AudioManager] Processing thread started\n";
        
        while (running_) {
            try {
                // Process audio analysis and levels
                if (input_buffer_.available() >= config_.chunk_size) {
                    size_t read = input_buffer_.read(temp_buffer_.data(), config_.chunk_size);
                    
                    if (read > 0) {
                        // Calculate audio levels
                        current_levels_ = level_meter_.process(temp_buffer_.data(), read);
                        
                        // Perform FFT analysis periodically
                        if (++analysis_counter_ % 5 == 0) { // Every 500ms
                            auto analysis = analyzer_.analyze(temp_buffer_.data(), read, config_.sample_rate);
                            
                            {
                                std::lock_guard<std::mutex> lock(analysis_mutex_);
                                latest_analysis_ = analysis;
                            }
                            
                            if (analysis_callback_) {
                                analysis_callback_(analysis);
                            }
                        }
                        
                        // Call level callback
                        if (level_callback_) {
                            level_callback_(current_levels_);
                        }
                        
                        // Queue for network transmission
                        chunk_queue_.push(temp_buffer_.data(), read);
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                
            } catch (const std::exception& e) {
                std::cerr << "[AudioManager] Processing error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        std::cout << "[AudioManager] Processing thread stopped\n";
    }

    void networkThread() {
        std::cout << "[AudioManager] Network thread started\n";
        
        while (running_) {
            try {
                AudioChunkQueue::AudioChunk chunk;
                if (chunk_queue_.pop(chunk, 100)) {
                    if (audio_callback_) {
                        audio_callback_(chunk.data.data(), chunk.data.size());
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "[AudioManager] Network error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        std::cout << "[AudioManager] Network thread stopped\n";
    }

    void captureAudioChunk() {
        // TODO: Replace with actual Unitree microphone capture
        // For now, generate test signal or read from system microphone
        
        // Example: Generate a low-level test tone
        static float phase = 0.0f;
        const float frequency = 440.0f; // A4 note
        const float amplitude = 0.1f;   // Low volume
        
        for (size_t i = 0; i < config_.chunk_size; ++i) {
            float sample = amplitude * std::sin(2.0f * M_PI * frequency * phase);
            chunk_buffer_[i] = static_cast<int16_t>(sample * 32767.0f);
            phase += 1.0f / config_.sample_rate;
            if (phase >= 1.0f) phase -= 1.0f;
        }
        
        // Write to input buffer
        size_t written = input_buffer_.write(chunk_buffer_.data(), config_.chunk_size);
        if (written < config_.chunk_size) {
            std::cout << "[AudioManager] Input buffer overflow, dropped " 
                     << (config_.chunk_size - written) << " samples\n";
        }
    }

    // Configuration and state
#ifdef ENABLE_UNITREE_SDK
    unitree::robot::g1::AudioClient* audio_client_;
#endif
    AudioConfig config_;
    std::atomic<bool> running_;

    // Threading
    std::thread capture_thread_;
    std::thread processing_thread_;
    std::thread network_thread_;

    // Buffers
    CircularBuffer<int16_t> input_buffer_;
    CircularBuffer<int16_t> output_buffer_;
    AudioChunkQueue chunk_queue_;
    std::vector<int16_t> chunk_buffer_;
    std::vector<int16_t> temp_buffer_;

    // Analysis
    AudioAnalyzer analyzer_;
    AudioLevelMeter level_meter_;
    mutable std::mutex analysis_mutex_;
    AudioAnalyzer::AnalysisResult latest_analysis_;
    AudioLevelMeter::LevelInfo current_levels_;
    int analysis_counter_ = 0;

    // Callbacks
    AudioCallback audio_callback_;
    AnalysisCallback analysis_callback_;
    LevelCallback level_callback_;
}; 
