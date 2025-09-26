#pragma once

#include "FilterBase.h"
#include "Consumer.h"
#include "SensorData.h"
#include <Eigen/Dense>

/**
 * @class MahonyFilter
 * @brief Implements a Mahony complementary filter for orientation estimation.
 *
 * This filter uses accelerometer and gyroscope data to provide a computationally
 * efficient and robust estimate of the vehicle's orientation (as a quaternion)
 * and the gyroscope's bias.
 */
class MahonyFilter : public FilterBase {
public:
    explicit MahonyFilter(DataManager& data_manager);

    void run() override;

private:
    // --- Data Consumers ---
    Consumer<GyroData> m_gyro_consumer;
    Consumer<AccelData> m_accel_consumer;

    // --- Filter State ---
    Eigen::Quaternionf m_q;      // Estimated orientation
    Eigen::Vector3f m_gyro_bias; // Estimated gyroscope bias

    // --- Last Sensor Measurements ---
    AccelData m_last_accel;
    GyroData m_last_gyro;

    // --- Tuning Gains ---
    float m_kp; // Proportional gain for accelerometer correction
    float m_ki; // Integral gain for gyroscope bias correction

    // --- Timekeeping ---
    uint64_t m_last_update_time_us;

    // --- Private Methods ---
    void init_filter();
    void update(float dt);
};