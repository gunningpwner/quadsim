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

    Vec4Map omega(rpm_data->rpm.data());
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
        default: {
            break;
        }
    }
    for (int i = 0; i < 4; i++)
    {
        command[i] = motor_command[i];
    }

}