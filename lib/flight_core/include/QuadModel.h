#include <array>
#include <Eigen/Dense>
#include <cstdint>
#include "BasicFilters.h"

using Vector4f = Eigen::Vector4f;
using Matrix4f = Eigen::Matrix4f;

template <typename T>
class FilteredSignal{
public:

    T raw;
    T val;
    T prev_val;
    T diff; 
    T dot; 
    T dot_diff;

    BiquadFilter<T> filter; 
    FilteredSignal() {
        reset();
    }
    void update(const T& new_raw, uint64_t timestamp_us) {
        raw = new_raw;

        T new_filtered = filter.apply(raw);

        if (sample_count== 0){
            val = new_filtered;
            prev_val = val;
            last_timestamp_us = timestamp_us;
            sample_count++;
            return;
        }
        
        float dt = (float)(timestamp_us - last_timestamp_us) * 1e-6f;
        if (dt <= 1e-6f) return;

        val = new_filtered;
        diff = val - prev_val;
        dot = diff / dt;
        if (sample_count>1){
            dot_diff = dot - prev_dot;
        }
        prev_val = val;
        prev_dot = dot;
        last_timestamp_us = timestamp_us;
        sample_count++;
    }
    
    void reset() { 
        sample_count = 0; 
        filter.reset(); 
        setZero(raw); setZero(val); setZero(diff); 
        setZero(dot); setZero(dot_diff); setZero(prev_val); setZero(prev_dot);
    }

private:
    
    T prev_dot;
    uint64_t last_timestamp_us;
    int sample_count = 0;

    void setZero(T& v) {
        if constexpr (std::is_class<T>::value) v.setZero();
        else v = 0; 
    }
};
struct QuadcopterModel {
    Matrix4f rls_motor_covariances[4];
    Vector4f rls_motor_estimates[4];
    FilteredSignal omega_sig;
    FilteredSignal imu_sig;
    FilteredSignal control_sig;

    
};