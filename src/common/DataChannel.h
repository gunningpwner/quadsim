#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <algorithm> // For std::min

template <typename T>
class DataChannel {
public:
    explicit DataChannel(size_t buffer_size) : m_update_count(0),m_buffer_size(buffer_size) {}

    void post(const T& data) {
        

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


private:
    std::deque<T> m_buffer;
    unsigned int m_update_count;
    size_t m_buffer_size;
    std::mutex m_mutex; // Each channel now has its own mutex
};