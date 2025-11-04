#include "GeometricController.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
GeometricController::GeometricController(DataManager &data_manager) : 
m_rc_channels_consumer(data_manager.getRCChannelsChannel()),              
m_state_consumer(data_manager.getStateChannel(),
m_flight_mode(GeomFlightMode::ATTITUDE)) {}

inline Eigen::Matrix3f hat(const Eigen::Vector3f& v) {
    Eigen::Matrix3f m;
    m << 0.0f, -v.z(), v.y(),
         v.z(), 0.0f, -v.x(),
         -v.y(), v.x(), 0.0f;
    return m;
}

inline Eigen::Vector3f vee(const Eigen::Matrix3f& m) {
    return Eigen::Vector3f(m(2, 1), m(0, 2), m(1, 0));
}

void GeometricController::run()
{

    if (m_rc_channels_consumer.consumeLatest())
    {
        m_rc_data = m_rc_channels_consumer.get_span().first[0];
    }
    if (m_state_consumer.consumeLatest())
    {
        m_state_data = m_state_consumer.get_span().first[0];
    }
    // Implementation of Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3)
    // https://arxiv.org/pdf/1003.2005

    // little variable to hold temporary or re-used steps
    Eigen::Vector3f tmp;
    
    Eigen::Matrix3f R=m_state_data.orientation.toRotationMatrix();

    switch (m_flight_mode){
        case GeomFlightMode::ATTITUDE:
            // Interpret rc data as desired attitude?
            float pitch = 0;
            float roll = 0;
            float yaw = 0;

            float sin_pitch = sin(pitch);
            float cos_pitch = cos(pitch);
            float sin_roll = sin(roll);
            float cos_roll = cos(roll);
            float sin_yaw = sin(yaw);
            float cos_yaw = cos(yaw);

            Eigen::Matrix3f Rd = {cos_yaw*cos_pitch, cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll, cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll,
                                   sin_yaw*cos_pitch, sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll, sin_yaw*sin_pitch*cos_roll - cos_yaw*sin_roll,
                                   -sin_pitch, cos_pitch*sin_roll, cos_pitch*cos_roll};
            
            tmp = Rd.transpose() * R - R.transpose() * Rd;
            Eigen::Vector3f rot_err = .5 * vee(tmp);
            
            break;
        case GeomFlightMode::POSITION:
            // Interpret rc data as desired position?

            break;
        case GeomFlightMode::VELOCITY:
            // Interpret rc data as desired velocity
            // Treat throttle, roll and pitch as desired translational velocity


            break;
        default:
            break;
    }

    
    
}