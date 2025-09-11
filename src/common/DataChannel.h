#pragma once

#include <deque>
#include <vector>
#include <mutex>
#include <algorithm> // For std::min
#include <type_traits> // For SFINAE/if constexpr
#include <stdexcept>   // For std::invalid_argument
#include <string>      // For std::string
#include <typeinfo>    // For typeid

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
    explicit DataChannel(size_t buffer_size) : m_update_count(0),m_buffer_size(buffer_size) {}

    // Throws std::invalid_argument if data contains NaN and T is NaN-checkable.
    void post(const T& data) {
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


private:
    std::deque<T> m_buffer;
    unsigned int m_update_count;
    size_t m_buffer_size;
    std::mutex m_mutex; // Each channel now has its own mutex
};