#include "DataManager.h"
#include "DataTypes.h"
#include <Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::VectorXf;

class ESKF
{
public:
    ESKF(DataManager::SensorConsumer m_sensor_consumer, DataManager::StateBuffer &state_buffer);
    void run();

private:
    DataManager::SensorConsumer m_sensor_consumer;
    DataManager::StateBuffer &m_state_buffer;

    void updateIMU(const SensorData &imu_data);
    void updateMag(const SensorData &mag_data);
    void updateGPS(const SensorData &gps_data);
    void correctionStep(MatrixXf H, MatrixXf V, VectorXf measurement, VectorXf prediction);

    Vector3f nominalPos;
    Vector3f nominalVel;
    Quaternionf nominalQuat;
    Vector3f nominalAccBias;
    Vector3f nominalGyroBias;
    Vector3f nominalGrav;

    Vector3f refLLA;

    MatrixXf errorStateCovariance;

    uint64_t last_timestamp;

    float accVar = .01;
    float accBiasVar = .0001;
    float gyroVar = .01;
    float gyroBiasVar = .0001;
    float gpsPosVar = .25;
    float gpsVelVar = .01;
    float magVar = .0025;
};