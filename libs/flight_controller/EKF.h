// EKF.h
#pragma once

#include "SensorData.h"
#include "FilterBase.h"
#include "Consumer.h"

// It is assumed that the Eigen library has been added to the project's include path.
#include <Eigen/Dense>

constexpr int STATE_SIZE = 16; // p(3), v(3), q(4), ba(3), bg(3)
constexpr int ERROR_STATE_SIZE = 15; // dp(3), dv(3), dtheta(3), dba(3), dbg(3)


// Extended Kalman Filter class for state estimation
class EKF : public FilterBase {
public:
    EKF(DataManager& data_manager);

    void run() override;
    void processSensorMeasurements();

private:
    Consumer<GyroData> m_gyro_consumer;
    Consumer<AccelData> m_accel_consumer;
    Consumer<MagData> m_mag_consumer;
    Consumer<GPSPositionData> m_gps_consumer;
    
    // --- EKF State and Covariance ---
    // State vector: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, q_w, q_x, q_y, q_z, b_ax, b_ay, b_az, b_gx, b_gy, b_gz]
    Eigen::Matrix<float, STATE_SIZE, 1> m_x;

    // Error-state covariance matrix (15x15)
    Eigen::Matrix<float, ERROR_STATE_SIZE, ERROR_STATE_SIZE> m_P;

    // Process noise covariance matrix
    Eigen::Matrix<float, ERROR_STATE_SIZE, ERROR_STATE_SIZE> m_Q;

    // --- Sensor Specific Data ---
    // Last used sensor measurements for prediction
    AccelData m_last_accel;
    GyroData m_last_gyro;

    // Magnetometer measurement noise covariance matrix
    Eigen::Matrix3f m_R_mag;
    // Reference magnetic field vector in ECEF frame (normalized)
    Eigen::Vector3f m_mag_ref_ecef;

    // GPS measurement noise covariance matrix
    Eigen::Matrix3f m_R_gps;

    // --- Filter State ---
    bool m_is_initialized;

    uint64_t m_current_time_us;
    uint64_t m_last_predict_time_us;

    // Private methods for prediction and correction steps
    void predict(float dt);
    void correctWithMag(const MagData& mag_data);
    void correctWithGps(const GPSPositionData& gps_data);
    void init_filter();
    void reset_filter();

    // Steps for EKF
    //1. Lock in navigation frame with GPS, accelerometer and magnetometer measurements.
    // error out if a high enough velocity or acceleration is detected
    // 
};
