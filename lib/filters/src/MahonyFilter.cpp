#include "MahonyFilter.h"

#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

using Vec3Map = Eigen::Map<const Eigen::Vector3f>;

MahonyFilter::MahonyFilter(DataManager::SensorConsumer &sensor_consumer)
    : m_sensor_consumer(sensor_consumer)
{
    init_filter();
}

void MahonyFilter::init_filter()
{

    m_q = Eigen::Quaternionf::Identity();
    m_gyro_bias.setZero();

    m_kp = 1.0f;
    m_ki = 0.2f;

    m_last_timestamp = 0;
}

void MahonyFilter::run()
{
    SensorData *sensor_data = m_sensor_consumer.readNext();
    while (sensor_data != nullptr)
    {
        switch (sensor_data->sensor)
        {
        case SensorData::Type::IMU:
            update(*sensor_data);
            break;

        default:
            break;
        }
        sensor_data = m_sensor_consumer.readNext();
    }

}

void MahonyFilter::update(const SensorData &imu_data)
{
    float dt = (imu_data.timestamp - m_last_timestamp) / 1e6;
    if (dt <= 0.0f)
        return;

    Eigen::Vector3f error(0, 0, 0);
    Eigen::Vector3f acc = Vec3Map(imu_data.data.imu.accel.data());
    Eigen::Vector3f gyro = Vec3Map(imu_data.data.imu.gyro.data());
    if (acc.norm() > 0.1f)
    {

        Eigen::Vector3f accel_unit = acc.normalized();
        // I think normally that should be -1, but +1 makes everything work
        // Make it work, make it right, make it fast
        // (I am here)
        Eigen::Vector3f g_body = m_q.conjugate() * Eigen::Vector3f(0, 0, 1);

        error = accel_unit.cross(g_body);
    }
    m_gyro_bias += -m_ki * error * dt;

    Eigen::Vector3f gyro_corrected = gyro - m_gyro_bias + m_kp * error;

    Eigen::Quaternionf dq;
    dq.w() = 1.0;
    dq.vec() = 0.5f * gyro_corrected * dt;
    m_q = (m_q * dq).normalized();
}