#pragma once
#include "DataManager.h" // One-way dependency

// Use a more robust check for an embedded ARM GCC environment.
#ifdef FIRMWARE_BUILD
#include <array>
#endif

template <typename T>
class Consumer {
public:
    // Constructor takes the DataManager it will wrap
    explicit Consumer(DataManager& data_manager)
        : m_data_manager(data_manager), m_last_seen_count(0)
        , m_last_read_index(0)
    {}

    bool consume(std::vector<T>& samples) {
#ifdef FIRMWARE_BUILD
        // For firmware, we consume into a static internal buffer to avoid allocations
        // within the lock-free DataChannel, then copy to the user's vector.
        size_t num_consumed = consume_into_internal_buffer();
        if (num_consumed > 0) {
            // This assignment may allocate, but it happens after the critical
            // data consumption step.
            samples.assign(m_internal_buffer.begin(), m_internal_buffer.begin() + num_consumed);
            return true;
        }
        return false;
#else
        // For desktop/SITL, we can call the std::vector-based consume method directly.
        return m_data_manager.consume(samples, m_last_seen_count);
#endif
    }

private:
#ifdef FIRMWARE_BUILD
    // This helper calls the firmware-specific overload of DataManager::consume.
    size_t consume_into_internal_buffer() {
        return m_data_manager.consume(m_internal_buffer.data(), m_internal_buffer.size(), m_last_seen_count, m_last_read_index);
    }
#endif

    DataManager& m_data_manager;
    unsigned int m_last_seen_count;
    size_t m_last_read_index; // Each consumer now tracks its own read index.
#ifdef FIRMWARE_BUILD
    std::array<T, 50> m_internal_buffer; // Pre-allocated buffer to avoid dynamic memory.
#endif
};