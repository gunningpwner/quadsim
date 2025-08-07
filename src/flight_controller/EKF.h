// EKF.h
#pragma once

#include "sensor_data.h"
#include "DataManager.h"

// Extended Kalman Filter class for state estimation
class EKF {
public:
    EKF(DataManager& data_manager):
        m_data_manager(data_manager),
        m_last_gyro_count(0),
        m_last_accel_count(0),
        m_last_mag_count(0),
        m_last_gps_count(0),
        locked_in(false){};

    void run();

private:
    DataManager& m_data_manager;

    // Internal state variables for the EKF
    // For example:
    // Eigen::VectorXd m_state; // State vector (e.g., position, velocity, orientation)
    // Eigen::MatrixXd m_covariance; // Covariance matrix

    // Last seen update counts for sensor data
    unsigned int m_last_gyro_count;
    unsigned int m_last_accel_count;
    unsigned int m_last_mag_count;
    unsigned int m_last_gps_count;
    bool locked_in;
    Vector3 ref_lla;

    // Private methods for prediction and correction steps
    void predict();
    void correct();
    void init_filter();
    void reset_filter();

    void bootstrap_position();
    // Steps for EKF
    //1. Lock in navigation frame with GPS, accelerometer and magnetometer measurements.
    // error out if a high enough velocity or acceleration is detected
    // 
};
