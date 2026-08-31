#include "KinematicAdjudicator.h"

// #include "Logger.h"

using Vec6Map = Eigen::Map<const Eigen::Matrix<float, 6, 1>>;
using Vec4Map = Eigen::Map<const Eigen::Vector4f>;



KinematicAdjudicator::KinematicAdjudicator() : 

    m_estimator(model),
    m_indicator(model),
    last_timestamp_us(0)
{
    model.current_mode = FlightMode::IDLE;
    model.current_motor = 0;
    // model.B1 <<0,0,0,0,0,0,0,0,0,0,0,0,-2.03083e-07,2.06884e-07,2.10073e-07,-2.53886e-08,-1.90439e-07,-2.11691e-07,2.518e-07,-1.44087e-06,5e-08,4.99999e-08,4.99993e-08,4.84542e-08;
    // model.B2 <<0,0,0,0,0,0,0,0,0,0,0,0,-1.94739e-07,-7.23204e-07,2.18145e-06,0.000510169,1.78738e-07,-6.4241e-07,-1.41923e-06,-0.00265609,-2.27419e-11,8.82825e-11,-1.58726e-09,-2.64969e-06;
    // model.motor_omega_max << 29975.2,29975.3,29975.3,29975.3;
    // model.motor_kappa << 0.498454,0.498458,0.498458,0.498458;
    // model.motor_tau << 0.0247397,0.0247397,0.0247397,0.0247397;
}

void KinematicAdjudicator::launch(){
   model.current_mode = FlightMode::LEARNING;
   m_exciter.startTimer(); 
}

void KinematicAdjudicator::update(FastIMUData *imu_data, FastRPMData *rpm_data, float *command)
{
    uint64_t timestamp_us = getCurrentTimeUs();
    // Pack IMU data [Acc, Gyro]
    Vec6Map imu_vec(imu_data->data.data());
    model.imu_sig.update(imu_vec, imu_data->timestamp);
    Eigen::Vector4f omega;
    omega[0] = rpm_data->rpm[0];
    omega[1] = rpm_data->rpm[1];
    omega[2] = rpm_data->rpm[2];
    omega[3] = rpm_data->rpm[3];
    model.omega_sig.update(omega, rpm_data->timestamp);

    Eigen::Vector4f motor_command = Eigen::Vector4f::Zero();

    switch (model.current_mode) {
        case FlightMode::LEARNING: {
            m_estimator.run();

            motor_command = m_exciter.getCommand();
            model.current_motor = m_exciter.getCurrentMotor();

            model.control_sig.update(motor_command, timestamp_us);

            if (m_exciter.isComplete()) {
                model.current_mode = FlightMode::FLIGHT;
                model.current_motor=-1;
                m_estimator.estimates_to_model();
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
        case FlightMode::IDLE:  {
            break;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        if(std::isnan(motor_command[i])){
            command[i]=0;
        } else{
            command[i] = motor_command[i];
        }
    }

}