#pragma once

#include "DataManager.h" // For IMU_BUFFER_SIZE
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
class MahonyFilter  {
public:
    MahonyFilter(DataManager& data_manager);

    void run();

private:
    // --- Data Consumers ---
    Consumer<IMUData, IMU_BUFFER_SIZE> m_imu_consumer;

    // --- Filter State ---
    Eigen::Quaternionf m_q;      // Estimated orientation
    Eigen::Vector3f m_gyro_bias; // Estimated gyroscope bias

    // --- Last Sensor Measurements ---
    IMUData m_last_imu;

    // --- Tuning Gains ---
    float m_kp; // Proportional gain for accelerometer correction
    float m_ki; // Integral gain for gyroscope bias correction

    // --- Timekeeping ---
    uint64_t m_last_update_time_us;

    // --- Private Methods ---
    void init_filter();
    void update(float dt);
};