#pragma once
#include <array>
#include <Eigen/Dense>
#include <cstdint>
#include "BasicFilters.h"

using Vector4f = Eigen::Vector4f;
using Matrix4f = Eigen::Matrix4f;

// Wrapper to track raw, filtered, diff, and derivative of a signal
// Mimics the pre-calculation done in RLS.py
template <typename T>
class FilteredSignal{
public:
    T raw;
    T val;      // Filtered Value
    T prev_val; 
    T diff;     // val - prev_val
    T dot;      // diff / dt
    T prev_dot;
    T dot_diff; // dot - prev_dot

    BiquadFilter<T> filter; 
    
    FilteredSignal() {
        reset();
    }

    void update(const T& new_raw, uint64_t timestamp_us) {
        raw = new_raw;
        T new_filtered = filter.apply(raw);

        if (sample_count == 0){
            val = new_filtered;
            prev_val = val;
            last_timestamp_us = timestamp_us;
            sample_count++;
            return;
        }
        
        float dt = (float)(timestamp_us - last_timestamp_us) * 1e-6f;
        if (dt <= 1e-6f) return; 

        // Shift history
        prev_val = val;
        prev_dot = dot;

        // Update
        val = new_filtered;
        diff = val - prev_val;
        dot = diff / dt;
        
        if (sample_count > 1){
            dot_diff = dot - prev_dot;
        }
        
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

    Eigen::Matrix<float, 4, 3> B1;

    Eigen::Matrix<float, 8, 3> B2;

    FilteredSignal<Vector4f> omega_sig;
    
    FilteredSignal<Eigen::Matrix<float, 6, 1>> imu_sig;

    FilteredSignal<Vector4f> control_sig;
};