#pragma once
#include "DataManager.h" // One-way dependency

template <typename T>
class Consumer {
public:
    // Constructor takes the DataManager it will wrap
    explicit Consumer(DataManager& data_manager)
        : m_data_manager(data_manager), m_last_seen_count(0) {}

    bool consume(std::vector<T>& samples) {
        return m_data_manager.consume(samples, m_last_seen_count);
    }

private:
    DataManager& m_data_manager;
    unsigned int m_last_seen_count;
};