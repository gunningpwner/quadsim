#include "KinematicAdjudicator.h"
#include "Logger.h"

KinematicAdjudicator::KinematicAdjudicator() : 

    m_estimator(model),
    m_indicator(model),
    last_timestamp_us(0)
{
    model.current_mode = FlightMode::LEARNING;
    model.current_motor = 0;
    model.B1 <<0,0,0,0,0,0,0,0,0,0,0,0,-2.03083e-07,2.06884e-07,2.10073e-07,-2.53886e-08,-1.90439e-07,-2.11691e-07,2.518e-07,-1.44087e-06,5e-08,4.99999e-08,4.99993e-08,4.84542e-08;
    model.B2 <<0,0,0,0,0,0,0,0,0,0,0,0,-1.94739e-07,-7.23204e-07,2.18145e-06,0.000510169,1.78738e-07,-6.4241e-07,-1.41923e-06,-0.00265609,-2.27419e-11,8.82825e-11,-1.58726e-09,-2.64969e-06;
    model.motor_omega_max << 29975.2,29975.3,29975.3,29975.3;
    model.motor_kappa << 0.498454,0.498458,0.498458,0.498458;
    model.motor_tau << 0.0247397,0.0247397,0.0247397,0.0247397;

    // m_exciter.startTimer();


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
    Logger::getInstance().log("omega_filt", model.omega_sig.val, timestamp_us);
    model.imu_sig.update(imu_vec, timestamp_us);
    Logger::getInstance().log("imu_filt", model.imu_sig.val, timestamp_us);

    Logger::getInstance().log("command_filt", model.control_sig.val, timestamp_us);
    // We update control_sig later, after we decide what the control is.

    Eigen::Vector4f motor_command = Eigen::Vector4f::Zero();

    // ---------------------------------------------------------
    // 2. STATE MACHINE ADJUDICATION
    // ---------------------------------------------------------

    switch (model.current_mode) {
        case FlightMode::LEARNING: {
            m_estimator.run();
                // 1. Safety Check (Paper Sec II.B.d):
                // Abort this motor if gyro exceeds safety margin
                // Simple check: if any axis > 1500 deg/s (approx 26 rad/s)
                // float max_rate = gyro_meas.cwiseAbs().maxCoeff();
                // if (max_rate > 26.0f)
                // {
                //     advanceMotor();
                //     return Eigen::Vector4f::Zero();
                // }
            motor_command = m_exciter.getCommand();
            model.current_motor = m_exciter.getCurrentMotor();
            // B. Update Control Signal in Model 
            // (RLS needs to know what we *just* commanded vs the *previous* reaction)
            model.control_sig.update(motor_command, timestamp_us);

            // C. Run Estimator
            // The Estimator reads from model.omega_sig, model.imu_sig, model.control_sig
            

            // D. Check for Completion
            if (m_exciter.isComplete()) {
                // Learning finished. 
                // Initialize INDI Controller with learned parameters
                // m_indicator.initialize(model.B1, model.B2, ...); 
                model.current_mode = FlightMode::FLIGHT;
                model.current_motor=-1;
                m_estimator.estimates_to_model();
                Logger::getInstance().log("omega_max", model.motor_omega_max, timestamp_us);
                Logger::getInstance().log("kappa", model.motor_kappa, timestamp_us);
                Logger::getInstance().log("tau", model.motor_tau, timestamp_us);
                Logger::getInstance().log("B1", model.B1, timestamp_us);
                Logger::getInstance().log("B2", model.B2, timestamp_us);
            }
            break;
        }

        case FlightMode::FLIGHT: {
            // A. Run Control Law
            // INDIController reads state from 'model'
            motor_command = m_indicator.run(); 
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