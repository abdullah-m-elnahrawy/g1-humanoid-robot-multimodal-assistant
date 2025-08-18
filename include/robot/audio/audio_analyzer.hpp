#pragma once

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include <numeric>

/**
 * Simple FFT implementation for audio analysis
 * Based on Cooley-Tukey algorithm
 */
class AudioAnalyzer {
public:
    AudioAnalyzer(size_t fft_size = 1024) 
        : fft_size_(fft_size), window_(fft_size), fft_buffer_(fft_size) {
        // Generate Hann window
        for (size_t i = 0; i < fft_size_; ++i) {
            window_[i] = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (fft_size_ - 1)));
        }
    }

    struct AnalysisResult {
        std::vector<float> frequencies;    // Frequency bins (Hz)
        std::vector<float> magnitudes;     // Magnitude spectrum (dB)
        float rms_level;                   // RMS level (0-1)
        float peak_level;                  // Peak level (0-1)
        bool voice_detected;               // Voice activity detection
        float voice_confidence;            // VAD confidence (0-1)
    };

    AnalysisResult analyze(const int16_t* samples, size_t count, float sample_rate = 24000.0f) {
        AnalysisResult result;
        
        if (count < fft_size_) {
            // Not enough samples for analysis
            result.rms_level = 0.0f;
            result.peak_level = 0.0f;
            result.voice_detected = false;
            result.voice_confidence = 0.0f;
            return result;
        }

        // Convert to float and apply window
        for (size_t i = 0; i < fft_size_; ++i) {
            float sample = static_cast<float>(samples[i]) / 32768.0f;
            fft_buffer_[i] = std::complex<float>(sample * window_[i], 0.0f);
        }

        // Perform FFT
        fft(fft_buffer_);

        // Calculate magnitude spectrum
        result.frequencies.resize(fft_size_ / 2);
        result.magnitudes.resize(fft_size_ / 2);
        
        float max_magnitude = 0.0f;
        float sum_squared = 0.0f;
        
        for (size_t i = 0; i < fft_size_ / 2; ++i) {
            result.frequencies[i] = (i * sample_rate) / fft_size_;
            float magnitude = std::abs(fft_buffer_[i]);
            result.magnitudes[i] = 20.0f * std::log10(std::max(magnitude, 1e-10f));
            max_magnitude = std::max(max_magnitude, magnitude);
            sum_squared += magnitude * magnitude;
        }

        // Calculate levels
        result.peak_level = std::min(max_magnitude, 1.0f);
        result.rms_level = std::sqrt(sum_squared / (fft_size_ / 2));

        // Voice Activity Detection
        detectVoiceActivity(result);

        return result;
    }

private:
    void fft(std::vector<std::complex<float>>& buffer) {
        size_t n = buffer.size();
        if (n <= 1) return;

        // Bit-reversal permutation
        for (size_t i = 1, j = 0; i < n; ++i) {
            size_t bit = n >> 1;
            for (; j & bit; bit >>= 1) {
                j ^= bit;
            }
            j ^= bit;
            if (i < j) {
                std::swap(buffer[i], buffer[j]);
            }
        }

        // Cooley-Tukey FFT
        for (size_t len = 2; len <= n; len <<= 1) {
            float angle = -2.0f * M_PI / len;
            std::complex<float> wlen(std::cos(angle), std::sin(angle));
            
            for (size_t i = 0; i < n; i += len) {
                std::complex<float> w(1);
                for (size_t j = 0; j < len / 2; ++j) {
                    std::complex<float> u = buffer[i + j];
                    std::complex<float> v = buffer[i + j + len / 2] * w;
                    buffer[i + j] = u + v;
                    buffer[i + j + len / 2] = u - v;
                    w *= wlen;
                }
            }
        }
    }

    void detectVoiceActivity(AnalysisResult& result) {
        // Simple VAD based on energy in voice frequency range (300Hz - 3400Hz)
        const float voice_freq_min = 300.0f;
        const float voice_freq_max = 3400.0f;
        
        float voice_energy = 0.0f;
        float total_energy = 0.0f;
        int voice_bins = 0;
        
        for (size_t i = 0; i < result.frequencies.size(); ++i) {
            float freq = result.frequencies[i];
            float magnitude = std::pow(10.0f, result.magnitudes[i] / 20.0f);
            
            total_energy += magnitude;
            
            if (freq >= voice_freq_min && freq <= voice_freq_max) {
                voice_energy += magnitude;
                voice_bins++;
            }
        }
        
        // Calculate voice confidence
        if (total_energy > 0.0f && voice_bins > 0) {
            float voice_ratio = voice_energy / total_energy;
            float energy_threshold = 0.01f; // Minimum energy threshold
            
            result.voice_confidence = std::min(1.0f, voice_ratio * (result.rms_level / energy_threshold));
            result.voice_detected = result.voice_confidence > 0.3f && result.rms_level > energy_threshold;
        } else {
            result.voice_confidence = 0.0f;
            result.voice_detected = false;
        }
    }

    size_t fft_size_;
    std::vector<float> window_;
    std::vector<std::complex<float>> fft_buffer_;
};

/**
 * Audio level meter for real-time monitoring
 */
class AudioLevelMeter {
public:
    AudioLevelMeter(float attack_time_ms = 10.0f, float release_time_ms = 100.0f, float sample_rate = 24000.0f)
        : attack_coeff_(std::exp(-1.0f / (attack_time_ms * sample_rate / 1000.0f)))
        , release_coeff_(std::exp(-1.0f / (release_time_ms * sample_rate / 1000.0f)))
        , peak_level_(0.0f)
        , rms_level_(0.0f) {}

    struct LevelInfo {
        float peak;      // Peak level (0-1)
        float rms;       // RMS level (0-1)  
        float db_peak;   // Peak in dB
        float db_rms;    // RMS in dB
    };

    LevelInfo process(const int16_t* samples, size_t count) {
        float sum_squares = 0.0f;
        float max_sample = 0.0f;

        // Calculate RMS and peak for this block
        for (size_t i = 0; i < count; ++i) {
            float sample = std::abs(static_cast<float>(samples[i]) / 32768.0f);
            sum_squares += sample * sample;
            max_sample = std::max(max_sample, sample);
        }

        float block_rms = std::sqrt(sum_squares / count);

        // Apply envelope detection
        float coeff = (max_sample > peak_level_) ? attack_coeff_ : release_coeff_;
        peak_level_ = max_sample + coeff * (peak_level_ - max_sample);

        coeff = (block_rms > rms_level_) ? attack_coeff_ : release_coeff_;
        rms_level_ = block_rms + coeff * (rms_level_ - block_rms);

        LevelInfo info;
        info.peak = std::min(peak_level_, 1.0f);
        info.rms = std::min(rms_level_, 1.0f);
        info.db_peak = 20.0f * std::log10(std::max(info.peak, 1e-6f));
        info.db_rms = 20.0f * std::log10(std::max(info.rms, 1e-6f));

        return info;
    }

    void reset() {
        peak_level_ = 0.0f;
        rms_level_ = 0.0f;
    }

private:
    float attack_coeff_;
    float release_coeff_;
    float peak_level_;
    float rms_level_;
}; 