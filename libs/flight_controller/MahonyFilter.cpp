#include "MahonyFilter.h"
#include "gravity.h" // For GRAVITY_MAGNITUDE
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

MahonyFilter::MahonyFilter(DataManager& data_manager) :
    FilterBase(data_manager),
    m_gyro_consumer(data_manager),
    m_accel_consumer(data_manager)
{
    init_filter();
}

void MahonyFilter::init_filter() {
    // Initialize orientation to identity (level)
    m_q = Eigen::Quaternionf::Identity();
    // Initialize gyro bias to zero
    m_gyro_bias.setZero();

    // Set tuning gains
    m_kp = 1.0f; // Proportional gain
    m_ki = 0.2f; // Integral gain

    m_last_update_time_us = 0;
}

void MahonyFilter::run() {
    // --- 1. GATHER ALL NEW DATA ---
    std::vector<std::unique_ptr<TimestampedData>> data_log;

    std::vector<GyroData> gyro_samples;
    if (m_gyro_consumer.consume(gyro_samples)) {
        for (const auto& sample : gyro_samples) {
            data_log.push_back(std::make_unique<GyroData>(sample));
        }
    }

    std::vector<AccelData> accel_samples;
    if (m_accel_consumer.consume(accel_samples)) {
        for (const auto& sample : accel_samples) {
            data_log.push_back(std::make_unique<AccelData>(sample));
        }
    }

    // --- 2. SORT DATA BY TIMESTAMP ---
    std::sort(data_log.begin(), data_log.end(), [](const auto& a, const auto& b) {
        return a->Timestamp < b->Timestamp;
    });

    // --- 3. PROCESS DATA IN CHRONOLOGICAL ORDER ---
    for (const auto& data_point_ptr : data_log) {
        const uint64_t timestamp = data_point_ptr->Timestamp;

        // Update last known sensor values
        if (auto gyro = dynamic_cast<GyroData*>(data_point_ptr.get())) {
            m_last_gyro = *gyro;
        } else if (auto accel = dynamic_cast<AccelData*>(data_point_ptr.get())) {
            m_last_accel = *accel;
        }

        // Calculate dt and run the update step
        if (m_last_update_time_us == 0) {
            m_last_update_time_us = timestamp;
        } else if (timestamp > m_last_update_time_us) {
            const float dt = (timestamp - m_last_update_time_us) * 1.0e-6f;
            update(dt);
            m_last_update_time_us = timestamp;
        }
    }

    // --- 4. PUBLISH THE ESTIMATED STATE ---
    // This filter primarily estimates orientation and gyro bias.
    // For a complete state, it would need to be combined with a position/velocity estimator.
    // For now, we can create a partial StateData for debugging or other consumers.
    StateData estimated_state;
    estimated_state.orientation = {m_q.w(), m_q.x(), m_q.y(), m_q.z()};

    Eigen::Vector3f corrected_gyro = m_last_gyro.AngularVelocity - m_gyro_bias;
    estimated_state.angular_velocity_body = {corrected_gyro.x(), corrected_gyro.y(), corrected_gyro.z()};

    // Post the state if the filter has been run at least once
    if (m_last_update_time_us > 0) {
        m_data_manager.post(estimated_state);
    }
}

void MahonyFilter::update(float dt) {
    if (dt <= 0.0f) return;

    // --- 1. Calculate Orientation Error from Accelerometer ---
    Eigen::Vector3f error(0, 0, 0);
    if (m_last_accel.Acceleration.norm() > 0.1f) { // Only use accelerometer if it's not in freefall
        // Normalize accelerometer measurement
        Eigen::Vector3f accel_unit = m_last_accel.Acceleration.normalized();

        // Estimate gravity direction in the body frame from the current orientation
        // This is done by rotating the world gravity vector [0, 0, -1] into the body frame.
        Eigen::Vector3f g_body = m_q.conjugate() * Eigen::Vector3f(0, 0, -1);

        // Calculate the error as the cross product between measured and estimated gravity.
        error = accel_unit.cross(g_body);
    }

    // --- 2. Update Gyroscope Bias ---
    // Integrate the orientation error to update the bias estimate.
    m_gyro_bias += -m_ki * error * dt;

    // --- 3. Correct Gyroscope Measurement ---
    // Correct the raw measurement with the estimated bias and the proportional error term.
    Eigen::Vector3f gyro_corrected = m_last_gyro.AngularVelocity - m_gyro_bias + m_kp * error;

    // --- 4. Integrate Orientation ---
    // Update the orientation quaternion by integrating the corrected angular velocity.
    Eigen::Quaternionf dq;
    dq.w() = 1.0;
    dq.vec() = 0.5f * gyro_corrected * dt;
    m_q = (m_q * dq).normalized();
}