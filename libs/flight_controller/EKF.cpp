#include "EKF.h"
#include "coord_trans.h"
#include "gravity.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <cstdio>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- DEBUGGING ---
// Set to true to enable verbose logging of EKF internal states.
constexpr bool EKF_DEBUG_LOGGING = true;

// The constructor now calls the base class constructor
EKF::EKF(DataManager& data_manager) :
    FilterBase(data_manager), // Pass data_manager to the base class
    m_gyro_consumer(data_manager),
    m_accel_consumer(data_manager),
    m_mag_consumer(data_manager),
    m_gps_consumer(data_manager)
{   
    // Initialize state and covariance matrices
    init_filter();
}

void EKF::run(){
    m_current_time_us = m_data_manager.getCurrentTimeUs();

    processSensorMeasurements();
}

void EKF::predict(float dt) {
    if (dt <= 0.0f) {
        return;
    }

    if (EKF_DEBUG_LOGGING) {
        std::cout << "\n--- EKF PREDICT (dt: " << dt << "s) ---" << std::endl;
        std::cout << "  State (Before): pos(" << m_x.segment<3>(0).transpose() << "), vel(" << m_x.segment<3>(3).transpose() << "), q(" << m_x(6) << "," << m_x.segment<3>(7).transpose() << ")" << std::endl;
        std::cout << "  Biases (Before): acc(" << m_x.segment<3>(10).transpose() << "), gyr(" << m_x.segment<3>(13).transpose() << ")" << std::endl;        
        std::cout << "  IMU Input: Accel(" << m_last_accel.Acceleration.x() << "," << m_last_accel.Acceleration.y() << "," << m_last_accel.Acceleration.z()
                  << "), Gyro(" << m_last_gyro.AngularVelocity.x() << "," << m_last_gyro.AngularVelocity.y() << "," << m_last_gyro.AngularVelocity.z() << ")" << std::endl;
    }

    // --- 1. Get current state from the state vector m_x ---
    Eigen::Vector3f position = m_x.segment<3>(0);
    Eigen::Vector3f velocity = m_x.segment<3>(3);
    Eigen::Quaternionf q(m_x(6), m_x(7), m_x(8), m_x(9));
    Eigen::Vector3f accel_bias = m_x.segment<3>(10);
    Eigen::Vector3f gyro_bias = m_x.segment<3>(13);

    // --- 2. Get latest sensor measurements (control inputs) ---
    const Eigen::Vector3f& accel_meas = m_last_accel.Acceleration;
    const Eigen::Vector3f& gyro_meas = m_last_gyro.AngularVelocity;

    // --- 3. Correct measurements with current bias estimates ---
    Eigen::Vector3f corrected_gyro = gyro_meas - gyro_bias;
    Eigen::Vector3f corrected_accel = accel_meas - accel_bias;

    // --- 4. Predict next state: x_k = f(x_{k-1}, u_k) ---

    // a) Predict position: p_k = p_{k-1} + v_{k-1} * dt
    position += velocity * dt;

    if (EKF_DEBUG_LOGGING && position.hasNaN()) {
        std::cout << "  [NaN DETECTED] in position prediction. Prev Vel: (" << velocity.transpose() << "), dt: " << dt << std::endl;
    }

    // b) Predict orientation: q_k = q_{k-1} * delta_q(gyro)
    // Small angle approximation for quaternion update
    Eigen::Quaternionf dq;
    dq.w() = 1.0;
    dq.vec() = 0.5f * corrected_gyro * dt;
    q = (q * dq).normalized();

    if (EKF_DEBUG_LOGGING && (q.w() != q.w() || q.vec().hasNaN())) { // Check all parts of quaternion for NaN
        std::cout << "  [NaN DETECTED] in orientation prediction. Corrected Gyro: (" << corrected_gyro.transpose() << "), dt: " << dt << std::endl;
    }

    // c) Predict velocity: v_k = v_{k-1} + (C(q_k) * a_k - g) * dt
    Eigen::Matrix3f C_b_e = q.toRotationMatrix(); // Rotation from body to ECEF
    Eigen::Vector3f accel_ecef = C_b_e * corrected_accel;
    Eigen::Vector3f gravity_ecef = calculate_gravity_ecef(position);
    velocity += (accel_ecef + gravity_ecef) * dt; // Note: gravity is added because it's negative

    if (EKF_DEBUG_LOGGING) {
        std::cout << "  Velocity Update: accel_ecef(" << accel_ecef.transpose() << "), gravity(" << gravity_ecef.transpose() << ")" << std::endl;
        if (velocity.hasNaN()) {
            std::cout << "  [NaN DETECTED] in velocity prediction. accel_ecef: (" << accel_ecef.transpose() << "), gravity: (" << gravity_ecef.transpose() << "), dt: " << dt << std::endl;
        }
    }

    // d) Biases are assumed to be random walks, so their prediction is just their previous value.

    // --- 5. Update the state vector m_x with predicted values ---
    m_x.segment<3>(0) = position;
    m_x.segment<3>(3) = velocity;
    m_x.segment<4>(6) << q.w(), q.x(), q.y(), q.z();
    // Biases m_x.segment<3>(10) and m_x.segment<3>(13) remain unchanged.

    if (EKF_DEBUG_LOGGING) {
        std::cout << "  State (After):  pos(" << m_x.segment<3>(0).transpose() << "), vel(" << m_x.segment<3>(3).transpose() << "), q(" << m_x(6) << "," << m_x.segment<3>(7).transpose() << ")" << std::endl;
    }


    // --- 6. Predict error covariance: P_k = F * P_{k-1} * F^T + Q ---

    // a) Calculate the state transition matrix (Jacobian) F
    Eigen::Matrix<float, ERROR_STATE_SIZE, ERROR_STATE_SIZE> F = Eigen::Matrix<float, ERROR_STATE_SIZE, ERROR_STATE_SIZE>::Identity();
    
    // dp/dv
    F.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;

    // dv/dtheta (effect of orientation error on velocity)
    Eigen::Matrix3f accel_skew;
    accel_skew << 0, -accel_ecef.z(), accel_ecef.y(),
                  accel_ecef.z(), 0, -accel_ecef.x(),
                  -accel_ecef.y(), accel_ecef.x(), 0;
    F.block<3, 3>(3, 6) = -accel_skew * dt;

    // dv/dba (effect of accel bias error on velocity)
    F.block<3, 3>(3, 9) = -C_b_e * dt;

    // dtheta/dbg (effect of gyro bias error on orientation)
    F.block<3, 3>(6, 12) = -Eigen::Matrix3f::Identity() * dt;

    // b) Propagate the covariance matrix
    m_P = F * m_P * F.transpose() + m_Q;

    if (EKF_DEBUG_LOGGING && m_P.hasNaN()) {
        std::cout << "  [NaN DETECTED] in covariance matrix P after prediction." << std::endl;
    }
}

void EKF::correctWithMag(const MagData& mag_data) {
    if (EKF_DEBUG_LOGGING) {
        std::cout << "\n--- EKF CORRECT (Mag) ---" << std::endl;
        std::cout << "  Mag Measurement (Raw): (" << mag_data.MagneticField.x() << ", " << mag_data.MagneticField.y() << ", " << mag_data.MagneticField.z() << ")" << std::endl;
        std::cout << "  State (Before): pos(" << m_x.segment<3>(0).transpose() << "), vel(" << m_x.segment<3>(3).transpose() << "), q(" << m_x(6) << "," << m_x.segment<3>(7).transpose() << ")" << std::endl;
    }

    // --- 1. Get current orientation from the state vector ---
    Eigen::Quaternionf q(m_x(6), m_x(7), m_x(8), m_x(9));

    // --- 2. Define the measurement model h(x) ---
    // The measurement model predicts what the magnetometer should read given the current state.
    // It rotates the reference magnetic field from the ECEF frame to the body frame.
    Eigen::Matrix3f C_b_e = q.toRotationMatrix(); // Rotation from body to ECEF
    Eigen::Matrix3f C_e_b = C_b_e.transpose();    // Rotation from ECEF to body
    Eigen::Vector3f z_predicted = C_e_b * m_mag_ref_ecef;

    if (EKF_DEBUG_LOGGING) {
        std::cout << "  Mag Predicted: (" << z_predicted.transpose() << "), Mag Ref ECEF: (" << m_mag_ref_ecef.transpose() << ")" << std::endl;
    }

    // --- 3. Get the actual measurement ---
    // The measurement should be a unit vector
    Eigen::Vector3f z_actual = mag_data.MagneticField.normalized();

    // --- 4. Calculate the measurement Jacobian H ---
    // H relates the error-state to the measurement. Mag is only affected by orientation error.
    Eigen::Matrix<float, 3, ERROR_STATE_SIZE> H = Eigen::Matrix<float, 3, ERROR_STATE_SIZE>::Zero();

    // The Jacobian of the measurement with respect to the orientation error (dtheta)
    // is the skew-symmetric matrix of the predicted measurement.
    Eigen::Matrix3f z_predicted_skew;
    z_predicted_skew << 0, -z_predicted.z(), z_predicted.y(),
                        z_predicted.z(), 0, -z_predicted.x(),
                        -z_predicted.y(), z_predicted.x(), 0;
    H.block<3, 3>(0, 6) = z_predicted_skew;

    // --- 5. Perform the Kalman Update ---

    // a) Calculate innovation (measurement residual)
    Eigen::Vector3f y = z_actual - z_predicted;

    if (EKF_DEBUG_LOGGING) {
        std::cout << "  Mag Innovation 'y': (" << y.transpose() << ")" << std::endl;
    }

    // b) Calculate innovation covariance
    Eigen::Matrix3f S = H * m_P * H.transpose() + m_R_mag;

    // c) Calculate Kalman Gain
    Eigen::Matrix3f S_inv = S.inverse();
    if (EKF_DEBUG_LOGGING && S_inv.hasNaN()) {
        std::cout << "  [NaN DETECTED] in Mag S.inverse(). Matrix S might be singular." << std::endl;
        std::cout << "  S = \n" << S << std::endl;
        return; // Abort correction if S is not invertible
    }
    Eigen::Matrix<float, ERROR_STATE_SIZE, 3> K = m_P * H.transpose() * S_inv;

    if (EKF_DEBUG_LOGGING && K.hasNaN()) {
        std::cout << "  [NaN DETECTED] in Mag Kalman Gain K." << std::endl;
    }
    
    // d) Calculate the error state correction
    Eigen::Matrix<float, ERROR_STATE_SIZE, 1> dx = K * y;

    // e) Update the state with the correction
    // The error state dx is [dp, dv, dtheta, dba, dbg]
    m_x.segment<3>(0) += dx.segment<3>(0); // Update position
    m_x.segment<3>(3) += dx.segment<3>(3); // Update velocity

    // Update orientation using the orientation error dtheta
    Eigen::Vector3f dtheta = dx.segment<3>(6);
    Eigen::Quaternionf dq_error(1.0, 0.5f * dtheta.x(), 0.5f * dtheta.y(), 0.5f * dtheta.z());
    q = (q * dq_error).normalized();
    m_x.segment<4>(6) << q.w(), q.x(), q.y(), q.z();

    m_x.segment<3>(10) += dx.segment<3>(9);  // Update accel bias
    m_x.segment<3>(13) += dx.segment<3>(12); // Update gyro bias

    // f) Update the covariance matrix
    m_P = (Eigen::Matrix<float, ERROR_STATE_SIZE, ERROR_STATE_SIZE>::Identity() - K * H) * m_P;

    if (EKF_DEBUG_LOGGING) {
        std::cout << "  State Correction 'dx': dPos(" << dx.segment<3>(0).transpose() << "), dVel(" << dx.segment<3>(3).transpose() << "), dTheta(" << dx.segment<3>(6).transpose() << ")" << std::endl;
        std::cout << "  State (After):  pos(" << m_x.segment<3>(0).transpose() << "), vel(" << m_x.segment<3>(3).transpose() << "), q(" << m_x(6) << "," << m_x.segment<3>(7).transpose() << ")" << std::endl;
        if (m_P.hasNaN()) {
            std::cout << "  [NaN DETECTED] in covariance matrix P after Mag correction." << std::endl;
        }
    }
}

void EKF::correctWithGps(const GPSPositionData& gps_data) {
    // Convert LLA measurement to ECEF
    Eigen::Vector3f z_actual = lla_to_ecef(gps_data.lla);


    // If the filter is not yet initialized, use this GPS measurement to set the initial state.
    if (!m_is_initialized) {
        std::cout << "EKF: Initializing with first GPS fix." << std::endl;

        // Set initial position
        m_x.segment<3>(0) = z_actual;

        // Set initial position uncertainty from GPS data
        m_P(0, 0) = gps_data.position_covariances.x();
        m_P(1, 1) = gps_data.position_covariances.y();
        m_P(2, 2) = gps_data.position_covariances.z();

        m_is_initialized = true;
        return;
    }

    if (EKF_DEBUG_LOGGING) {
        std::cout << "\n--- EKF CORRECT (GPS) ---" << std::endl;
        std::cout << "  GPS Measurement (ECEF): (" << z_actual.transpose() << ")" << std::endl;
        std::cout << "  State (Before): pos(" << m_x.segment<3>(0).transpose() << "), vel(" << m_x.segment<3>(3).transpose() << "), q(" << m_x(6) << "," << m_x.segment<3>(7).transpose() << ")" << std::endl;
    }

    // --- 1. Define the measurement model h(x) ---
    // The measurement is a direct observation of the position state.
    Eigen::Vector3f z_predicted = m_x.segment<3>(0);

    // --- 2. Define the measurement Jacobian H ---
    // H maps the error-state to the measurement. GPS position only observes position error.
    Eigen::Matrix<float, 3, ERROR_STATE_SIZE> H = Eigen::Matrix<float, 3, ERROR_STATE_SIZE>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

    // --- 3. Define and rotate the measurement noise R ---
    // The GPS provides covariance in the local NED frame. It must be rotated to the ECEF frame
    // for the Kalman update, since the filter state and innovation are in ECEF.
    Eigen::Matrix3f R_ned;
    R_ned.setZero();
    R_ned.diagonal() << gps_data.position_covariances.x(), gps_data.position_covariances.y(), gps_data.position_covariances.z();

    // Get the rotation matrix from ECEF to NED
    Eigen::Matrix3f C_e_n = ecef_to_ned_matrix(gps_data.lla.x(), gps_data.lla.y());
    // The rotation from NED to ECEF is the transpose
    Eigen::Matrix3f C_n_e = C_e_n.transpose();
    // Rotate the covariance: R_ecef = C_n_e * R_ned * C_n_e^T
    Eigen::Matrix3f R_gps = C_n_e * R_ned * C_n_e;
    // --- 4. Perform the Kalman Update ---

    // a) Calculate innovation (measurement residual)
    Eigen::Vector3f y = z_actual - z_predicted;

    if (EKF_DEBUG_LOGGING) {
        std::cout << "  GPS Innovation 'y': (" << y.transpose() << ")" << std::endl;
    }

    // b) Calculate innovation covariance
    Eigen::Matrix3f S = H * m_P * H.transpose() + R_gps;

    // c) Calculate Kalman Gain
    Eigen::Matrix3f S_inv = S.inverse();
    if (EKF_DEBUG_LOGGING && S_inv.hasNaN()) {
        std::cout << "  [NaN DETECTED] in GPS S.inverse(). Matrix S might be singular." << std::endl;
        std::cout << "  S = \n" << S << std::endl;
        return; // Abort correction if S is not invertible
    }
    Eigen::Matrix<float, ERROR_STATE_SIZE, 3> K = m_P * H.transpose() * S_inv;

    if (EKF_DEBUG_LOGGING && K.hasNaN()) {
        std::cout << "  [NaN DETECTED] in GPS Kalman Gain K." << std::endl;
    }
    
    // d) Calculate the error state correction
    Eigen::Matrix<float, ERROR_STATE_SIZE, 1> dx = K * y;

    // e) Update the state with the correction
    m_x.segment<3>(0) += dx.segment<3>(0); // Update position
    m_x.segment<3>(3) += dx.segment<3>(3); // Update velocity

    Eigen::Quaternionf q(m_x(6), m_x(7), m_x(8), m_x(9));
    Eigen::Vector3f dtheta = dx.segment<3>(6);
    Eigen::Quaternionf dq_error(1.0, 0.5f * dtheta.x(), 0.5f * dtheta.y(), 0.5f * dtheta.z());
    q = (q * dq_error).normalized();
    m_x.segment<4>(6) << q.w(), q.x(), q.y(), q.z();

    m_x.segment<3>(10) += dx.segment<3>(9);  // Update accel bias
    m_x.segment<3>(13) += dx.segment<3>(12); // Update gyro bias

    // f) Update the covariance matrix using the Joseph form for numerical stability
    Eigen::Matrix<float, ERROR_STATE_SIZE, ERROR_STATE_SIZE> I_KH = Eigen::Matrix<float, ERROR_STATE_SIZE, ERROR_STATE_SIZE>::Identity() - K * H;
    m_P = I_KH * m_P * I_KH.transpose() + K * R_gps * K.transpose();

    if (EKF_DEBUG_LOGGING) {
        std::cout << "  State Correction 'dx': dPos(" << dx.segment<3>(0).transpose() << "), dVel(" << dx.segment<3>(3).transpose() << "), dTheta(" << dx.segment<3>(6).transpose() << ")" << std::endl;
        std::cout << "  State (After):  pos(" << m_x.segment<3>(0).transpose() << "), vel(" << m_x.segment<3>(3).transpose() << "), q(" << m_x(6) << "," << m_x.segment<3>(7).transpose() << ")" << std::endl;
        if (m_P.hasNaN()) {
            std::cout << "  [NaN DETECTED] in covariance matrix P after GPS correction." << std::endl;
        }
    }
}

void EKF::processSensorMeasurements() {
    // === 1. GATHER ALL NEW DATA FROM EVERY CHANNEL ===
    // This vector will hold all measurements from all sensors for this update cycle.
    std::vector<std::unique_ptr<TimestampedData>> data_log;
    // Consume gyroscope data
    std::vector<GyroData> gyro_samples;
    if (m_gyro_consumer.consume(gyro_samples)) {
        for (auto& sample : gyro_samples) {
            data_log.push_back(std::make_unique<GyroData>(sample));
            // Store the most recent gyro data for the predict step
            if (sample.Timestamp > m_last_gyro.Timestamp) {
                m_last_gyro = sample;
                if (EKF_DEBUG_LOGGING){
                    std::cout << "New Gyro sample with timestamp " << sample.Timestamp << std::endl;
                }
            }
        }
    }

    // Consume accelerometer data
    std::vector<AccelData> accel_samples;
    if (m_accel_consumer.consume(accel_samples)) {
        for (auto& sample : accel_samples) {
            data_log.push_back(std::make_unique<AccelData>(sample));
            // Store the most recent accel data for the predict step
            if (sample.Timestamp > m_last_accel.Timestamp) {
                m_last_accel = sample;
                if (EKF_DEBUG_LOGGING){
                    std::cout << "New accelerometer sample with timestamp " << sample.Timestamp << std::endl;
                }
            }
        }
    }

    // Consume magnetometer data
    std::vector<MagData> mag_samples;
    if (m_mag_consumer.consume(mag_samples)) {
        for (auto& sample : mag_samples) {
            data_log.push_back(std::make_unique<MagData>(sample));
            if (EKF_DEBUG_LOGGING){
                std::cout << "New magnetometer sample with timestamp " << sample.Timestamp << std::endl;
            }
        }
    }

    // Consume GPS data
    std::vector<GPSPositionData> gps_samples;
    if (m_gps_consumer.consume(gps_samples)) {
        for (auto& sample : gps_samples) {
            data_log.push_back(std::make_unique<GPSPositionData>(sample));
            if (EKF_DEBUG_LOGGING){
                std::cout << "New GPS sample with timestamp " << sample.Timestamp << std::endl;
            }
        }
    }


    // === 2. SORT THE COMBINED DATA BY TIMESTAMP ===
    // Now, data_log contains a mix of sensor data. We sort it into a
    // single, chronologically correct timeline.
    std::sort(data_log.begin(), data_log.end(), [](const auto& a, const auto& b) {
        return a->Timestamp < b->Timestamp;
    });


    // === 3. PROCESS ALL DATA IN CHRONOLOGICAL ORDER ===
    // Loop through the sorted timeline and apply each measurement to the filter.
    
    for (const auto& data_point_ptr : data_log) {
        // Predict the state forward to the current measurement's timestamp
        if (EKF_DEBUG_LOGGING){
            std::cout << "Processing data point with timestamp " << data_point_ptr->Timestamp << std::endl;
        }
        const uint64_t timestamp = data_point_ptr->Timestamp;

        // Handle the very first measurement to initialize the time
        if (m_last_predict_time_us == 0) {
            m_last_predict_time_us = timestamp;
        } else if (timestamp > m_last_predict_time_us) {
            // Predict forward to the current measurement's timestamp
            const float dt = (timestamp - m_last_predict_time_us) * 1.0e-6f;
            if (EKF_DEBUG_LOGGING) {
                std::cout << "Measurement prediction timestamp " << timestamp << " last predict " << m_last_predict_time_us << std::endl;
            }
            predict(dt);
            m_last_predict_time_us = timestamp;
        }

        // Use the type enum to determine the data type and call the correct update function.
        // This avoids RTTI and is safe for embedded systems.
        switch (data_point_ptr->type) {
            case TimestampedData::Type::Gyro:
                m_last_gyro = *static_cast<GyroData*>(data_point_ptr.get());
                break;
            case TimestampedData::Type::Accel:
                m_last_accel = *static_cast<AccelData*>(data_point_ptr.get());
                break;
            case TimestampedData::Type::GPSPosition:
                correctWithGps(*static_cast<GPSPositionData*>(data_point_ptr.get()));
                break;
            case TimestampedData::Type::Mag:
                correctWithMag(*static_cast<MagData*>(data_point_ptr.get()));
                break;
            default:
                break; // Ignore other data types
        }
    }

    // === 4. PUBLISH THE UPDATED STATE ===
    // Only publish if the filter has been initialized.
    if (m_is_initialized) {
        StateData estimated_state;

        // Copy position, velocity, and orientation from the EKF state vector
        estimated_state.position_ecef = m_x.segment<3>(0);
        estimated_state.velocity_ecef = m_x.segment<3>(3);
        estimated_state.orientation = Eigen::Quaternionf(m_x(6), m_x(7), m_x(8), m_x(9));

        // Calculate and add the bias-corrected angular velocity to the state
        Eigen::Vector3f gyro_bias = m_x.segment<3>(13);
        estimated_state.angular_velocity_body = m_last_gyro.AngularVelocity - gyro_bias;

        if (EKF_DEBUG_LOGGING && estimated_state.containsNaN()) {
            std::cout << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            std::cout << "!!! EKF: NaN DETECTED IN FINAL STATE BEFORE POSTING !!!" << std::endl;
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            std::cout << "Final State: pos(" << estimated_state.position_ecef.transpose()
                      << "), vel(" << estimated_state.velocity_ecef.transpose() << ")" << std::endl;
        }

        m_data_manager.post(estimated_state);
    }
}

void EKF::init_filter() {
    m_is_initialized = false;

    m_x.setZero();
    m_P.setZero();
    m_Q.setZero();

    // --- Initialize State Vector (m_x) ---
    // Position and velocity will be initialized by the first GPS message.
    // For now, set to zero.
    m_x.segment<3>(0).setZero(); // Position
    m_x.segment<3>(3).setZero(); // Velocity
    // Set initial orientation to identity quaternion (no rotation)
    m_x(6) = 1.0; // q_w
    m_x.segment<3>(7).setZero(); // q_x, q_y, q_z
    // Initialize biases to zero
    m_x.segment<3>(10).setZero(); // Accel biases
    m_x.segment<3>(13).setZero(); // Gyro biases

    // --- Initialize Covariance Matrix (m_P) ---
    // Set high initial uncertainty for all states
    m_P.diagonal() << 100, 100, 100, // Position (m^2)
                      10, 10, 10,    // Velocity ((m/s)^2)
                      1, 1, 1,       // Orientation (rad^2)
                      0.1, 0.1, 0.1, // Accel bias ((m/s^2)^2)
                      0.1, 0.1, 0.1; // Gyro bias ((rad/s)^2)

    // --- Initialize Sensor-Specific Parameters ---
    // Define a plausible normalized magnetic field vector for the northern hemisphere (points North and Down).
    // This is a simplification. A real implementation would use a magnetic field model (e.g., WMM).
    // This vector is defined in the ECEF frame.
    m_mag_ref_ecef = Eigen::Vector3f(0.5f, 0.0f, 0.866f).normalized();

    // Set magnetometer measurement noise. This value depends on the sensor quality.
    // Assuming a variance of 0.01 for each axis (stdev of 0.1).
    float mag_noise = 0.01f;
    m_R_mag = Eigen::Matrix3f::Identity() * mag_noise;

    // Set default GPS measurement noise. This will be overridden by the
    // actual covariance from the GPS message during the update step.
    float gps_pos_noise = 10.0f;
    m_R_gps = Eigen::Matrix3f::Identity() * gps_pos_noise;


    m_last_predict_time_us = 0;
}
