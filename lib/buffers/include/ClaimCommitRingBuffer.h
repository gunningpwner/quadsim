#pragma once
#include <cstddef>
#include <array>

template <typename T, size_t BufferSize>
class CCRBNonCopyingConsumer;

template <typename T, size_t BufferSize>
class ClaimCommitRingBuffer
{
    // A ring buffer that uses a claim - commit sequence to write to the buffer in place
public:
    friend class CCRBNonCopyingConsumer<T, BufferSize>;

    ClaimCommitRingBuffer() : m_read_head(BufferSize), m_read_tail(0), m_claim_head(0), m_wrap_count(0) {}
    ClaimCommitRingBuffer(const ClaimCommitRingBuffer &) = delete;
    ClaimCommitRingBuffer &operator=(const ClaimCommitRingBuffer &) = delete;

    T *claim()
    {

        if (m_claim_head == m_read_tail && m_read_head != BufferSize)
            m_read_tail = (m_read_tail + 1) % BufferSize;

        T *ret = &m_buffer[m_claim_head];
        m_claim_head = (m_claim_head + 1) % BufferSize;

        return ret;
    };

    void commit(const T *data)
    {
        // Since all reads operate in a non-interrupt context and all writes operate in an interrupt context,
        // there is no way that a read can occur with a hole in the data (if we ignore a process erroring which in that case we have bigger problems)
        // Therefore, we immediately move the read head to the latest committed data under the assumption that all holes will be filled
        // by the time any reads can occur
        // Proof/Example: Consider the case where the IMU and GPS are both triggered from ISR contexts.
        // In the event that the IMU claims a chunk then gets interrupted by the GPS, the GPS will claim and commit its chunk first, leaving a hole.
        // Then the IMU will commit its chunk before finally releasing control.

        size_t idx = data - m_buffer.data();
        if (m_read_head == BufferSize)
            m_read_head = idx;

        else if (m_read_head < m_claim_head)
        {
            // Standard case
            if (idx > m_read_head)
                m_read_head = idx;
        }
        else
        {
            // Claim head has looped but read has not
            if (idx > m_read_head || idx < m_claim_head)
            {
                if (m_read_head > idx)
                    m_wrap_count++;
                m_read_head = idx;
            }
        }
    };

private:
    std::array<T, BufferSize> m_buffer;
    size_t m_read_head;
    size_t m_read_tail; // Marks the end (oldest) valid data in buffer
    size_t m_claim_head;
    size_t m_wrap_count; // Denotes the number of times m_read_head has wrapped around
};

template <typename T, size_t BufferSize>
class CCRBNonCopyingConsumer
{
public:
    CCRBNonCopyingConsumer(ClaimCommitRingBuffer<T, BufferSize> &m_buffer)
        : m_buffer(m_buffer), m_read_idx(0), last_wrap(0), is_worried(false)
    {
        m_read_idx = m_buffer.m_read_tail;
    }

    T *readNext()
    {
        // Meant to be used in conjunction with reset.

        if (m_buffer.m_read_head == BufferSize)
            return nullptr;
        size_t abs_buf_head = m_buffer.m_read_head + BufferSize * m_buffer.m_wrap_count;
        size_t abs_read_idx = m_read_idx + BufferSize * last_wrap;

        if (abs_read_idx > abs_buf_head)
            return nullptr;

        // Check if our read index is behind the current claim index
        // This would indicate that we've been overtaken
        size_t abs_claim_head;
        if (m_buffer.m_claim_head > m_buffer.m_read_head)
        {
            abs_claim_head = m_buffer.m_claim_head + BufferSize * m_buffer.m_wrap_count;
        }
        else
        {
            abs_claim_head = m_buffer.m_claim_head + BufferSize * (m_buffer.m_wrap_count + 1);
        }
        if (abs_read_idx + BufferSize < abs_claim_head)
        {
            // We've been overtaken
            is_worried = true;
            m_read_idx = m_buffer.m_read_tail;
            last_wrap = m_buffer.m_wrap_count;
            if (m_buffer.m_read_tail > m_buffer.m_read_head)
            {
                last_wrap--;
            }
            abs_read_idx=m_read_idx+last_wrap*BufferSize;
        }
        T *ret = &m_buffer.m_buffer[m_read_idx];
        abs_read_idx++;
        m_read_idx = abs_read_idx % BufferSize;
        last_wrap = abs_read_idx / BufferSize;
        return ret;
    };
    void reset()
    {
        m_read_idx = m_buffer.m_read_tail;
        is_worried = false;
        last_wrap = m_buffer.m_wrap_count;
    };

    T *readLatest()
    {
        size_t current_buf_head = m_buffer.m_read_head;
        if (current_buf_head == BufferSize)
            return nullptr;
        size_t abs_buf_head = current_buf_head + BufferSize * m_buffer.m_wrap_count;
        size_t abs_read_idx = m_read_idx + BufferSize * last_wrap;

        if (abs_read_idx > abs_buf_head)
            return nullptr;

        T *ret = &m_buffer.m_buffer[current_buf_head];
        abs_buf_head++;
        m_read_idx = abs_buf_head % BufferSize;
        last_wrap = abs_buf_head / BufferSize;
        return ret;
    };

    size_t available() const
    {
        // Returns the number of unread items in the buffer
        size_t current_buf_head = m_buffer.m_read_head;
        size_t abs_buf_head = current_buf_head + BufferSize * m_buffer.m_wrap_count;
        size_t abs_read_idx = m_read_idx + BufferSize * last_wrap;

        if (current_buf_head == BufferSize)
        { // Buffer is empty
            return 0;
        }
        return abs_buf_head - abs_read_idx+1;
    }

private:
    ClaimCommitRingBuffer<T, BufferSize> &m_buffer;
    size_t m_read_idx;
    size_t last_wrap;
    bool is_worried;
};