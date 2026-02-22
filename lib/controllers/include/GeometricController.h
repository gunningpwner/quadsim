#pragma once
#include "DataManager.h"
#include "DataTypes.h"
#include <Eigen/Dense>
#include "timing.h"

using Vector3f = Eigen::Vector3f;
using Matrix3f = Eigen::Matrix3f;

class GeometricController
{
public:
    GeometricController(DataManager::StateConsumer m_state_consumer);
    void run();
    void updatePositionControl();
    void updateVelocityControl();
    void updateRotationControl();
    
    Vector3f ang_acc_desired;
    Matrix3f rot_desired;
    Vector3f ang_acc_cmd;
    Vector3f vel_desired;
    float linear_z_accel_cmd; // Output for INDI

    // Independent cascade commands
    Vector3f pos_desired;
    Vector3f vel_cmd;
    Vector3f acc_cmd;
    Vector3f jerk_cmd;
    Vector3f snap_cmd;

private:
    DataManager::StateConsumer m_state_consumer;
    StateEstimate* state_data;

    
    Vector3f acc_desired;
    Vector3f jerk_desired;
    Vector3f snap_desired;
    
    Vector3f ang_vel_desired;

    Vector3f front_dir_desired; // b1d, vector to align the front axis to. essentially just defines yaw

    float kx; // Position Gain
    float kv; // Velocity Gain
    float kr; // Rotation Gain
    float komega; // Angular Velocity Gain
};