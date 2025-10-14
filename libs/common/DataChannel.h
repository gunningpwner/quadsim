#pragma once

#include <vector>
#include <algorithm> // For std::min
#include <type_traits> // For SFINAE/if constexpr
#include <stdexcept>   // For std::invalid_argument
#include <string>      // For std::string
#include <typeinfo>    // For typeid

// Use a more robust check for an embedded ARM GCC environment.
#ifdef FIRMWARE_BUILD
#include <array>
#include <atomic>
#else
#include <deque>
#include <mutex>
#endif

namespace detail {
    // Type trait to check if a type T has a member function `bool containsNaN() const`.
    // This uses SFINAE (Substitution Failure Is Not An Error).
    template<typename T, typename = void>
    struct is_nan_checkable : std::false_type {};

    template<typename T>
    struct is_nan_checkable<T, std::void_t<decltype(std::declval<const T&>().containsNaN())>> : std::true_type {};

    // Helper variable template for convenience
    template<typename T>
    inline constexpr bool is_nan_checkable_v = is_nan_checkable<T>::value;
}

template <typename T>
class DataChannel {
public:
#ifdef FIRMWARE_BUILD
    // --- Embedded-Safe, Lock-Free Ring Buffer Implementation ---
    // The buffer_size argument is ignored here, as the size is fixed at compile time for SPMC.
    explicit DataChannel(size_t /*buffer_size*/) : m_head(0), m_update_count(0) {}

    // Throws std::invalid_argument if data contains NaN and T is NaN-checkable.
    void post(const T& data) {
        // On embedded, we cannot throw exceptions from an ISR.
        // The check is performed, but we simply don't post the data if it's NaN.
        if constexpr (detail::is_nan_checkable_v<T>) {
            if (data.containsNaN()) {
                return; // Silently drop NaN data in an embedded context.
            }
        }

        size_t current_head = m_head.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) % COMPILE_TIME_BUFFER_SIZE;

        // In this SPSC queue, if the head catches the tail, we overwrite old data.
        // This is a common strategy for sensor data where the latest is most important.
        m_buffer[current_head] = data;
        m_head.store(next_head, std::memory_order_release);
        m_update_count.fetch_add(1, std::memory_order_relaxed);
    }

    bool consume(std::vector<T>& samples, unsigned int& last_seen_count) {
        unsigned int current_update_count = m_update_count.load(std::memory_order_acquire);
        if (last_seen_count == current_update_count) {
            return false;
        }

        samples.clear();
        // For SPMC, each consumer must track its own tail. This method is now invalid.
        // This is a placeholder to indicate it should not be used.
        // A new consume method is needed that takes the consumer's state.
        return false;
    }

    // New consume method for SPMC. Returns number of samples read.
    size_t consume(T* sample_buffer, size_t max_samples, unsigned int& last_seen_count, size_t& last_read_index) {
        unsigned int current_update_count = m_update_count.load(std::memory_order_acquire);
        if (last_seen_count == current_update_count) {
            return 0; // No new data
        }

        // Calculate how many samples have been written since we last checked.
        // This is the key to detecting a buffer overrun.
        unsigned int samples_missed = current_update_count - last_seen_count;
        if (samples_missed > COMPILE_TIME_BUFFER_SIZE) {
            // We've been lapped! The producer has overwritten all the data we were
            // supposed to read. The last_read_index is now invalid.
            // The safest thing to do is to reset our read index to the oldest available data.
            last_read_index = (m_head.load(std::memory_order_acquire) + 1) % COMPILE_TIME_BUFFER_SIZE;
        }

        size_t current_head = m_head.load(std::memory_order_acquire);
        size_t samples_consumed = 0;
        while(last_read_index != current_head && samples_consumed < max_samples) {
            sample_buffer[samples_consumed++] = m_buffer[last_read_index];
            last_read_index = (last_read_index + 1) % COMPILE_TIME_BUFFER_SIZE;
        }

        last_seen_count = current_update_count;
        return samples_consumed;
    }

#else
    // --- Desktop/SITL, Mutex-based, Dynamic Implementation ---
    explicit DataChannel(size_t buffer_size) : m_update_count(0), m_buffer_size(buffer_size) {}

    void post(const T& data) {
        // On desktop, we can safely throw an exception for NaN data.
        if constexpr (detail::is_nan_checkable_v<T>) {
            if (data.containsNaN()) {
                throw std::invalid_argument("NaN detected in posted data of type: " + std::string(typeid(T).name()));
            }
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        m_buffer.push_back(data);
        if (m_buffer.size() > m_buffer_size) {
            m_buffer.pop_front();
        }
        m_update_count++;
    }

    bool consume(std::vector<T>& samples, unsigned int& last_seen_count) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (last_seen_count >= m_update_count) {
            return false;
        }

        size_t new_samples_count = m_update_count - last_seen_count;
        size_t num_to_copy = std::min(new_samples_count, m_buffer.size());
        
        samples.assign(m_buffer.end() - num_to_copy, m_buffer.end());

        last_seen_count = m_update_count;
        return true;
    }

    void getLatest(T& latest_data) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_buffer.empty()) {
            latest_data = m_buffer.back();
        }
    }
#endif


private:
#ifdef FIRMWARE_BUILD
    // For embedded, we use a std::array to prevent dynamic allocation.
    // The size must be known at compile time. We'll use a generous default.
    // NOTE: This requires all DataChannels to have the same size.
    static constexpr size_t COMPILE_TIME_BUFFER_SIZE = 50;
    std::array<T, COMPILE_TIME_BUFFER_SIZE> m_buffer;
    std::atomic<size_t> m_head;
    std::atomic<unsigned int> m_update_count;
#else
    std::deque<T> m_buffer;
    unsigned int m_update_count;
    size_t m_buffer_size;
    std::mutex m_mutex; // Each channel now has its own mutex
#endif
};