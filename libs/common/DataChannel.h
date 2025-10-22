#pragma once

#include <algorithm> // For std::min
#include <type_traits> // For SFINAE/if constexpr
#include <stdexcept>   // For std::invalid_argument
#include <string>      // For std::string
#include <typeinfo>    // For typeid

#include <array>
#include <atomic>

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

/**
 * @brief A non-templated base interface for DataChannel.
 * This allows Consumers to hold a reference to a DataChannel without needing to
 * know its specific buffer size, enabling decoupling.
 */
template <typename T>
class IDataChannel {
public:
    virtual ~IDataChannel() = default;
    virtual size_t consume(T* sample_buffer, size_t max_samples, unsigned int& last_seen_count, size_t& last_read_index) = 0;
};

template <typename T, size_t BufferSize>
class DataChannel : public IDataChannel<T> {
public:
    // --- Embedded-Safe, Lock-Free Ring Buffer Implementation ---
    // The buffer_size is now a template argument, so no runtime argument is needed.
    explicit DataChannel() : m_head(0), m_update_count(0) {}

    // Throws std::invalid_argument if data contains NaN and T is NaN-checkable.
    void post(const T& data) {
        // In a unified build, we decide how to handle NaNs.
        // The check is performed, but we simply don't post the data if it's NaN.
        if constexpr (detail::is_nan_checkable_v<T>) {
            if (data.containsNaN()) {
                return; // Silently drop NaN data in an embedded context.
            }
        }

        size_t current_head = m_head.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) % BufferSize;

        // In this SPSC queue, if the head catches the tail, we overwrite old data.
        // This is a common strategy for sensor data where the latest is most important.
        m_buffer[current_head] = data;
        m_head.store(next_head, std::memory_order_release);
        m_update_count.fetch_add(1, std::memory_order_release); // Use release to sync with consumers
    }

    // New consume method for SPMC. Returns number of samples read.
    size_t consume(T* sample_buffer, size_t max_samples, unsigned int& last_seen_count, size_t& last_read_index) override {
        unsigned int current_update_count = m_update_count.load(std::memory_order_acquire);
        if (last_seen_count == current_update_count) {
            return 0; // No new data
        }

        // Calculate how many samples have been written since we last checked.
        // This is the key to detecting a buffer overrun.
        unsigned int samples_missed = current_update_count - last_seen_count;
        if (samples_missed > BufferSize) {
            // We've been lapped! The producer has overwritten all the data we were
            // supposed to read. The last_read_index is now invalid.
            // The safest thing to do is to reset our read index to the oldest available data.
            last_read_index = (m_head.load(std::memory_order_acquire) + 1) % BufferSize;
        }

        size_t current_head = m_head.load(std::memory_order_acquire);
        size_t samples_consumed = 0;
        while(last_read_index != current_head && samples_consumed < max_samples) {
            sample_buffer[samples_consumed++] = m_buffer[last_read_index];
            last_read_index = (last_read_index + 1) % BufferSize;
        }

        last_seen_count = current_update_count;
        return samples_consumed;
    }

    bool getLatest(T& latest_data) {
        // If no data has ever been posted, do nothing.
        if (m_update_count.load(std::memory_order_acquire) == 0) {
            return false;
        }

        // The latest data is at the index just before the current head.
        // We use memory_order_acquire to ensure we see the data written by the producer.
        size_t current_head = m_head.load(std::memory_order_acquire);
        size_t latest_index = (current_head == 0) ? BufferSize - 1 : current_head - 1;
        latest_data = m_buffer[latest_index];
        return true;
    }

private:
    // For embedded, we use a std::array to prevent dynamic allocation.
    // The size must be known at compile time. We'll use a generous default.
    // NOTE: This requires all DataChannels to have the same size.
    std::array<T, BufferSize> m_buffer;
    std::atomic<size_t> m_head;
    std::atomic<unsigned int> m_update_count;
};