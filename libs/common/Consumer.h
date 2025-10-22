#pragma once
#include "DataChannel.h" // Dependency on DataChannel
#include <array>         // For std::array
// #include <span>          // For std::span (C++20), or custom Span for C++17
#include <utility>       // For std::pair

// If C++20 std::span is not available, a simple custom Span can be used.
// For this example, we'll use std::pair<const T*, size_t> for C++17 compatibility.
template<typename T>
using DataSpan = std::pair<const T*, size_t>;

template<typename T>
class IDataChannel; // Forward declaration of the base interface

template <typename T, size_t InternalBufferSize>
class Consumer {
public:
    /**
     * @brief Constructs a Consumer for a specific DataChannel.
     * @param channel A reference to an object that implements the IDataChannel interface.
     */
    explicit Consumer(IDataChannel<T>& channel)
        : m_channel(channel), m_last_seen_count(0), m_last_read_index(0), m_valid_sample_count(0) {}

    /**
     * @brief Consumes all new samples from the DataChannel, placing them into the internal buffer.
     *        The internal buffer will contain up to InternalBufferSize samples.
     * @return The number of new samples that were consumed. Returns 0 if no new data.
     */
    size_t consumeAll() {
        m_valid_sample_count = m_channel.consume(m_internal_buffer.data(), InternalBufferSize, m_last_seen_count, m_last_read_index);
        return m_valid_sample_count;
    }

    /**
     * @brief Consumes only the single latest sample from the DataChannel, placing it
     *        at the start of the internal buffer. Marks all new data as seen.
     * @return True if a new sample was consumed, false otherwise.
     */
    bool consumeLatest() {
        // First, try to get the latest data. This will update m_last_seen_count and m_last_read_index
        // within the DataChannel's consume method, effectively marking all prior data as seen.
        size_t num_consumed = m_channel.consume(m_internal_buffer.data(), InternalBufferSize, m_last_seen_count, m_last_read_index);

        if (num_consumed > 0) {
            // If new data was consumed, the *latest* sample will be the last one in the buffer.
            // We move it to the first position and set valid count to 1.
            m_internal_buffer[0] = m_internal_buffer[num_consumed - 1];
            m_valid_sample_count = 1;
            return true;
        }
        m_valid_sample_count = 0;
        return false;
    }

    /**
     * @brief Provides a non-owning view of the valid data in the internal buffer.
     *        Call this after a successful consumeAll() or consumeLatest().
     * @return A DataSpan (pair of pointer and size) viewing the consumed data.
     */
    DataSpan<const T> get_span() const {
        return {m_internal_buffer.data(), m_valid_sample_count};
    }

private:
    IDataChannel<T>& m_channel; // Reference to the DataChannel interface
    unsigned int m_last_seen_count;
    size_t m_last_read_index; // Each consumer now tracks its own read index.
    std::array<T, InternalBufferSize> m_internal_buffer; // Pre-allocated buffer for consumed data.
    size_t m_valid_sample_count; // Number of valid samples currently in m_internal_buffer.
};