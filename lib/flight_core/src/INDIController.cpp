#include "INDIController.h"

using Vec3Map = Eigen::Map<const Eigen::Vector3f>;

INDIController::INDIController(DataManager::SensorConsumer m_sensor_consumer)
    : m_sensor_consumer(m_sensor_consumer)
{
}

void INDIController::run(){
    SensorData* sensor_data = m_sensor_consumer.readLatest();
    // Need to figure out how to get the latest imu data.
    float dt;
    if (sensor_data->sensor == SensorData::Type::IMU){
        Vector3f pred;
        Vector3f ang_acc = filterAngularAcceleration(Vec3Map(sensor_data->data.imu.gyro.data()), pred, dt);

        float thrust; // get this somehow

        

    }
}

Vector3f INDIController::filterAngularAcceleration(const Vector3f &gyro_data, Vector3f &pred, float dt){
    Vector3f rate_err = gyro_data-est_ang_vel;
    gyro_integrator+=gyro_ki*rate_err*dt;

    Vector3f est_acc = pred+(gyro_kp*rate_err)+gyro_integrator;
    est_ang_vel+=est_acc*dt;
    return est_acc;
}