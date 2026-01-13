#pragma once
#include "DataManager.h"
#include "DataTypes.h"
#include <Eigen/Dense>

using Vector3f = Eigen::Vector3f;
using Matrix3f = Eigen::Matrix3f;
using Matrix4f = Eigen::Matrix<float, 4, 4>;
using Vector4f = Eigen::Matrix<float, 4, 1>;

class INDIController
{
public:
    INDIController(DataManager::SensorConsumer m_sensor_consumer);
    void run();
private:
    DataManager::SensorConsumer m_sensor_consumer;
    Vector3f predictAngularAcceleration();
    Vector3f filterAngularAcceleration(const Vector3f &gyro_data, Vector3f &pred, float dt);
    Vector3f ang_acc_cmd;
    float acc_cmd;

    Vector3f est_ang_vel;
    Vector3f gyro_integrator;
    float gyro_kp;
    float gyro_ki;
};