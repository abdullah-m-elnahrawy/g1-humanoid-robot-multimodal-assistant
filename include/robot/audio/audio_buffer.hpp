#pragma once

#include <atomic>
#include <vector>
#include <cstring>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>

/**
 * Lock-free circular buffer for real-time audio processing
 * Uses atomic operations for thread-safe read/write
 */
template<typename T>
class CircularBuffer {
public:
    explicit CircularBuffer(size_t capacity) 
        : buffer_(capacity + 1), capacity_(capacity + 1) {
        read_pos_.store(0);
        write_pos_.store(0);
    }

    // Write data to buffer, returns number of samples actually written
    size_t write(const T* data, size_t count) {
        size_t write_pos = write_pos_.load(std::memory_order_relaxed);
        size_t read_pos = read_pos_.load(std::memory_order_acquire);
        
        size_t available = (read_pos + capacity_ - write_pos - 1) % capacity_;
        size_t to_write = std::min(count, available);
        
        if (to_write == 0) return 0;
        
        // Handle wrap-around
        size_t first_chunk = std::min(to_write, capacity_ - write_pos);
        std::memcpy(&buffer_[write_pos], data, first_chunk * sizeof(T));
        
        if (to_write > first_chunk) {
            std::memcpy(&buffer_[0], data + first_chunk, (to_write - first_chunk) * sizeof(T));
        }
        
        write_pos_.store((write_pos + to_write) % capacity_, std::memory_order_release);
        return to_write;
    }

    // Read data from buffer, returns number of samples actually read
    size_t read(T* data, size_t count) {
        size_t read_pos = read_pos_.load(std::memory_order_relaxed);
        size_t write_pos = write_pos_.load(std::memory_order_acquire);
        
        size_t available = (write_pos + capacity_ - read_pos) % capacity_;
        size_t to_read = std::min(count, available);
        
        if (to_read == 0) return 0;
        
        // Handle wrap-around
        size_t first_chunk = std::min(to_read, capacity_ - read_pos);
        std::memcpy(data, &buffer_[read_pos], first_chunk * sizeof(T));
        
        if (to_read > first_chunk) {
            std::memcpy(data + first_chunk, &buffer_[0], (to_read - first_chunk) * sizeof(T));
        }
        
        read_pos_.store((read_pos + to_read) % capacity_, std::memory_order_release);
        return to_read;
    }

    // Get number of samples available for reading
    size_t available() const {
        size_t read_pos = read_pos_.load(std::memory_order_relaxed);
        size_t write_pos = write_pos_.load(std::memory_order_relaxed);
        return (write_pos + capacity_ - read_pos) % capacity_;
    }

    // Get free space available for writing
    size_t free_space() const {
        size_t read_pos = read_pos_.load(std::memory_order_relaxed);
        size_t write_pos = write_pos_.load(std::memory_order_relaxed);
        return (read_pos + capacity_ - write_pos - 1) % capacity_;
    }

    // Clear buffer
    void clear() {
        read_pos_.store(0);
        write_pos_.store(0);
    }

private:
    std::vector<T> buffer_;
    size_t capacity_;
    std::atomic<size_t> read_pos_;
    std::atomic<size_t> write_pos_;
};

/**
 * Audio chunk queue for network transmission
 */
class AudioChunkQueue {
public:
    struct AudioChunk {
        std::vector<int16_t> data;
        uint64_t timestamp_ms;
    };

    AudioChunkQueue(size_t max_chunks = 100) : max_chunks_(max_chunks) {}

    void push(const int16_t* data, size_t samples) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Remove old chunks if queue is full
        while (chunks_.size() >= max_chunks_) {
            chunks_.pop();
        }
        
        AudioChunk chunk;
        chunk.data.assign(data, data + samples);
        chunk.timestamp_ms = getCurrentTimeMs();
        chunks_.push(std::move(chunk));
        
        cv_.notify_one();
    }

    bool pop(AudioChunk& chunk, int timeout_ms = 100) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (!cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                         [this] { return !chunks_.empty(); })) {
            return false;
        }
        
        chunk = std::move(chunks_.front());
        chunks_.pop();
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return chunks_.size();
    }

private:
    uint64_t getCurrentTimeMs() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<AudioChunk> chunks_;
    size_t max_chunks_;
}; 