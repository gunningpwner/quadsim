#pragma once
#include <cstddef>
#include <array>
template <typename T, size_t BufferSize>
class ClaimCommitDataChannel{
// A ring buffer that uses a claim - commit sequence to write to the buffer in place
public:
    ClaimCommitDataChannel() : m_read_head(0), m_claim_head(0) {}

    *T claim(){
        *T ret = m_buffer[m_claim_head];
        m_claim_head = (m_claim_head + 1) % BufferSize;
        return ret;
    }

    void commit(const T* data){
        // Since all reads operate in a non-interrupt context and all writes operate in an interrupt context,
        // there is no way that a read can occur with a hole in the data (if we ignore a process erroring which in that case we have bigger problems)
        // Therefore, we immediately move the read head to the latest committed data under the assumption that all holes will be filled
        // by the time any reads can occur
        // Proof/Example: Consider the case where the IMU and GPS are both triggered from ISR contexts.
        // In the event that the IMU claims a chunk then gets interrupted by the GPS, the GPS will claim and commit its chunk first, leaving a hole.
        // Then the IMU will commit its chunk before finally releasing control.
        
        size_t idx = data-m_buffer;
        if (m_read_head == m_claim_head)
            // if the read head is already at the claim head, nothing to do
            return;
        else if (m_read_head < m_claim_head)
        {
            // Standard case
            if (idx > m_read_head)
                m_read_head=idx;
        } else if (m_read_head > m_claim_head)
        {
            // Claim head has looped but read has not
            if (idx< m_read_head)
                m_read_head=idx;
        }
        
    }

private:

std::array<T, BufferSize> m_buffer;
size_t m_read_head;
size_t m_claim_head;


};