#include "EKF.h"
#include "coord_trans.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <cstdio>
#include <iostream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// The constructor now calls the base class constructor
EKF::EKF(DataManager& data_manager) :
    FilterBase(data_manager), // Pass data_manager to the base class
    m_gyro_consumer(data_manager),
    m_accel_consumer(data_manager),
    m_mag_consumer(data_manager),
    m_gps_consumer(data_manager)
{
}

void EKF::run(){
    m_current_time_us = m_data_manager.getCurrentTimeUs();

}




void EKF::processSensorMeasurements() {
    // === 1. GATHER ALL NEW DATA FROM EVERY CHANNEL ===
    // This vector will hold all measurements from all sensors for this update cycle.
    std::vector<std::unique_ptr<TimestampedData>> data_log;

    // Consume gyroscope data
    std::vector<GyroData> gyro_samples;
    if (m_gyro_consumer.consume(gyro_samples)) {
        for (auto& sample : gyro_samples) {
            data_log.push_back(std::make_unique<GyroData>(sample));
        }
    }

    // Consume accelerometer data
    std::vector<AccelData> accel_samples;
    if (m_accel_consumer.consume(accel_samples)) {
        for (auto& sample : accel_samples) {
            data_log.push_back(std::make_unique<AccelData>(sample));
        }
    }

    // Consume GPS data
    std::vector<GPSPositionData> gps_samples;
    if (m_gps_consumer.consume(gps_samples)) {
        for (auto& sample : gps_samples) {
            data_log.push_back(std::make_unique<GPSPositionData>(sample));
        }
    }


    // === 2. SORT THE COMBINED DATA BY TIMESTAMP ===
    // Now, data_log contains a mix of sensor data. We sort it into a
    // single, chronologically correct timeline.
    std::sort(data_log.begin(), data_log.end(), [](const auto& a, const auto& b) {
        return a->Timestamp < b->Timestamp;
    });


    // === 3. PROCESS ALL DATA IN CHRONOLOGICAL ORDER ===
    // Loop through the sorted timeline and apply each measurement to the filter.
    // for (const auto& data_point : data_log) {
    //     // Predict the state forward to the current measurement's timestamp
    //     float dt = (data_point->Timestamp - m_last_predict_time_us) * 1.0e-6f;
    //     predict(dt);
    //     m_last_predict_time_us = data_point->Timestamp;

    //     // Use dynamic_cast to safely figure out what type of data this is
    //     // and call the correct update function.
    //     if (auto gyro = dynamic_cast<GyroData*>(data_point.get())) {
    //         updateWithGyro(*gyro);
    //     }
    //     else if (auto accel = dynamic_cast<AccelData*>(data_point.get())) {
    //         updateWithAccel(*accel);
    //     }
    //     else if (auto gps = dynamic_cast<GPSPositionData*>(data_point.get())) {
    //         updateWithGps(*gps);
    //     }
    // }
}
