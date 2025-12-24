#pragma once

#include "DataManager.h" // For IMU_BUFFER_SIZE
#include "DataTypes.h"
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
    MahonyFilter(DataManager::SensorConsumer& sensor_consumer);

    void run();

private:
    // --- Data Consumers ---
    DataManager::SensorConsumer& m_sensor_consumer;

    // --- Filter State ---
    Eigen::Quaternionf m_q;      // Estimated orientation
    Eigen::Vector3f m_gyro_bias; // Estimated gyroscope bias


    // --- Tuning Gains ---
    float m_kp; // Proportional gain for accelerometer correction
    float m_ki; // Integral gain for gyroscope bias correction

    // --- Timekeeping ---
    uint64_t m_last_timestamp;

    // --- Private Methods ---
    void init_filter();
    void update(const SensorData &imu_data);
};