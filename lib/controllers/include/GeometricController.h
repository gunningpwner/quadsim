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
    void updateRotationControl();


private:
    DataManager::StateConsumer m_state_consumer;
    StateEstimate* state_data;

    Vector3f pos_desired;
    Vector3f vel_desired;
    Vector3f acc_desired;
    Vector3f jerk_desired;
    Vector3f snap_desired;

    Matrix3f rot_desired;
    Vector3f ang_vel_desired;
    Vector3f ang_acc_desired;

    Vector3f front_dir_desired; // b1d, vector to align the front axis to. essentially just defines yaw

    float kx; // Position Gains
    float kv; // Velocity Gains
    float kr; // Rotation Gains
    float komega; // Angular Velocity Gains

    float mass;
    Matrix3f inertia_mat;
};