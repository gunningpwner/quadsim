#include "KinematicAdjudicator.h"

KinematicAdjudicator::KinematicAdjudicator() : 
    current_mode(FlightMode::LEARNING),
    m_estimator(model),
    m_indicator(model),
    last_timestamp_us(0)
{
    // Initialize model if needed (reset filters, etc)
}

Eigen::Vector4f KinematicAdjudicator::update(uint64_t timestamp_us, 
                                             const Eigen::Vector4f& raw_omega, 
                                             const Eigen::Vector3f& raw_acc, 
                                             const Eigen::Vector3f& raw_gyro)
{
    float dt = (timestamp_us - last_timestamp_us) * 1e-6f;
    if (last_timestamp_us == 0) dt = 0.0005f; // Initial safe dt
    
    // ---------------------------------------------------------
    // 1. SIGNAL CONDITIONING (The "Single Source of Truth")
    // ---------------------------------------------------------
    
    // Pack IMU data [Acc, Gyro]
    Eigen::Matrix<float, 6, 1> imu_vec;
    imu_vec << raw_acc, raw_gyro;

    // Update the QuadModel state
    // NOTE: Estimator should NO LONGER call .update() internally!
    model.omega_sig.update(raw_omega, timestamp_us);
    model.imu_sig.update(imu_vec, timestamp_us);
    
    // We update control_sig later, after we decide what the control is.

    Eigen::Vector4f motor_command = Eigen::Vector4f::Zero();

    // ---------------------------------------------------------
    // 2. STATE MACHINE ADJUDICATION
    // ---------------------------------------------------------

    switch (current_mode) {
        case FlightMode::LEARNING: {
            // A. Get Excitation Command
            motor_command = m_exciter.getCommand(dt, raw_gyro);

            // B. Update Control Signal in Model 
            // (RLS needs to know what we *just* commanded vs the *previous* reaction)
            model.control_sig.update(motor_command, timestamp_us);

            // C. Run Estimator
            // The Estimator reads from model.omega_sig, model.imu_sig, model.control_sig
            m_estimator.run(timestamp_us, motor_command, raw_omega, imu_vec);

            // D. Check for Completion
            if (m_exciter.isComplete()) {
                // Learning finished. 
                // Initialize INDI Controller with learned parameters
                // m_indicator.initialize(model.B1, model.B2, ...); 
                current_mode = FlightMode::FLIGHT;
            }
            break;
        }

        case FlightMode::FLIGHT: {
            // A. Run Control Law
            // INDIController reads state from 'model'
            m_indicator.run(); 
            
            // Assuming INDIController stores its output somewhere, 
            // or returns it. For now let's assume it updates a variable:
            // motor_command = m_indicator.getOutput();
            
            // B. Update Control History (useful if we keep adaptive running)
            model.control_sig.update(motor_command, timestamp_us);
            break;
        }
    }

    last_timestamp_us = timestamp_us;
    return motor_command;
}