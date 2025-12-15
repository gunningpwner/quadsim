#include "MahonyFilter.h"

#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

MahonyFilter::MahonyFilter(DataManager &data_manager) : FilterBase(data_manager),
                                                        m_imu_consumer(data_manager.getIMUChannel())
{
    init_filter();
}

void MahonyFilter::init_filter()
{

    m_q = Eigen::Quaternionf::Identity();
    m_gyro_bias.setZero();

    m_kp = 1.0f;
    m_ki = 0.2f;

    m_last_update_time_us = 0;
}

void MahonyFilter::run()
{

    std::vector<std::unique_ptr<TimestampedData>> data_log;

    if (m_imu_consumer.consumeAll() > 0)
    {
        auto span = m_imu_consumer.get_span();
        for (size_t i = 0; i < span.second; ++i)
        {
            const auto &sample = span.first[i];
            data_log.push_back(std::make_unique<IMUData>(sample));
        }
    }


    std::sort(data_log.begin(), data_log.end(), [](const auto &a, const auto &b)
              { return a->Timestamp < b->Timestamp; });

    for (const auto &data_point_ptr : data_log)
    {
        const uint64_t timestamp = data_point_ptr->Timestamp;

        switch (data_point_ptr->type)
        {
        case TimestampedData::Type::IMU:
            m_last_imu = *static_cast<IMUData *>(data_point_ptr.get());
            break;
        default:
            break;
        }

        if (m_last_update_time_us == 0)
        {
            m_last_update_time_us = timestamp;
        }
        else if (timestamp > m_last_update_time_us)
        {
            const float dt = (timestamp - m_last_update_time_us) * 1.0e-6f;
            update(dt);
            m_last_update_time_us = timestamp;
        }
    }

    StateData estimated_state;
    estimated_state.Timestamp = m_last_update_time_us;
    estimated_state.orientation = {m_q.w(), m_q.x(), m_q.y(), m_q.z()};

    Eigen::Vector3f corrected_gyro = m_last_imu.AngularVelocity - m_gyro_bias;
    estimated_state.angular_velocity_body = {corrected_gyro.x(), corrected_gyro.y(), corrected_gyro.z()};

    if (m_last_update_time_us > 0)
    {
        m_data_manager.post(estimated_state);
    }
}

void MahonyFilter::update(float dt)
{
    if (dt <= 0.0f)
        return;

    Eigen::Vector3f error(0, 0, 0);
    if (m_last_imu.Acceleration.norm() > 0.1f)
    {

        Eigen::Vector3f accel_unit = m_last_imu.Acceleration.normalized();
        // I think normally that should be -1, but +1 makes everything work
        // Make it work, make it right, make it fast
        // (I am here)
        Eigen::Vector3f g_body = m_q.conjugate() * Eigen::Vector3f(0, 0, 1);

        error = accel_unit.cross(g_body);
    }
    m_gyro_bias += -m_ki * error * dt;

    Eigen::Vector3f gyro_corrected = m_last_imu.AngularVelocity - m_gyro_bias + m_kp * error;

    Eigen::Quaternionf dq;
    dq.w() = 1.0;
    dq.vec() = 0.5f * gyro_corrected * dt;
    m_q = (m_q * dq).normalized();
}