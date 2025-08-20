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
    m_gps_consumer(data_manager),
    locked_in(false),
    pos_lock(false),
    grav_lock(false),
    mag_lock(false)
{
}

void EKF::run(){
    m_current_time_us = m_data_manager.getCurrentTimeUs();
    if (!locked_in){
        bootstrap_position();
        bootstrap_grav();
        bootstrap_mag();
        if (pos_lock && grav_lock && mag_lock){
            lock_in();
        }
    } else {
        
    }

}

void EKF::bootstrap_position(){
    if (!pos_lock){
        std::vector<GPSPositionData> gps_samples;
        if (m_gps_consumer.consume(gps_samples)) {
            for (auto& sample : gps_samples) {
                
                if (ref_lla.magnitude()==0){
                    std::cout << "initializing" << std::endl;
                    ref_lla = sample.lla;
                    pos_var = sample.position_covariances;
                } else {
                    Vector3 K = pos_var / (pos_var + sample.position_covariances);
                    ref_lla+=K*(sample.lla-ref_lla);
                    pos_var=(1-K)*pos_var;
                    // std::cout << ref_lla.ToString() << std::endl;
                    // std::cout << pos_var.ToString() << std::endl;
                }
                if (pos_var.magnitude()<1e-3){
                    pos_lock=true;
                    break;
                }
            }
        }
    }
}
// fix covariances for these
void EKF::bootstrap_grav(){
    if (!grav_lock){
        std::vector<AccelData> accel_samples;
        if(m_accel_consumer.consume(accel_samples)){
            for (auto& sample : accel_samples) {
                if (grav.magnitude()==0){
                    grav = sample.Acceleration;
                    grav_var = sample.Acceleration;
                } else {
                    Vector3 K = grav_var / (grav_var + sample.Acceleration);
                    grav+=K*(sample.Acceleration-grav);
                    grav_var=1-K*grav_var;
                }   
                if (grav_var.magnitude()<1e-3){
                    grav_lock=true;
                    break;
                }
            }
        }
    }
}

void EKF::bootstrap_mag(){
    if (!mag_lock){
        std::vector<MagData> mag_samples;
        if(m_mag_consumer.consume(mag_samples)){
            for (auto& sample : mag_samples) {
                if (mag.magnitude()==0){
                    mag = sample.MagneticField;

                    mag_var = sample.MagneticField; //to do
                } else {
                    Vector3 K = mag_var / (mag_var + sample.MagneticField);
                    mag+=K*(sample.MagneticField-mag);
                    mag_var=1-K*mag_var;
                }   
                if (mag_var.magnitude()<1e-3){
                    mag_lock=true;
                    break;
                }
            }
        }
    }
}

void EKF::lock_in(){
    locked_in=true;
    grav.normalize();
    mag.normalize();

    float roll = atan2(grav.y, grav.z);
    float pitch = asin(-grav.x);

    float mag_x_leveled = mag.x * cos(pitch) + mag.y * sin(roll) * sin(pitch) + mag.z * cos(roll) * sin(pitch);
    float mag_y_leveled = mag.y * cos(roll) - mag.z * sin(roll);

    float yaw_magnetic = atan2(-mag_y_leveled, mag_x_leveled);

    //placeholder
    const float magnetic_declination = -4.2 * (M_PI / 180.0f); 

    float yaw_true = yaw_magnetic + magnetic_declination;

    // Assign the calculated Euler angles (in radians) to the orientation vector.
    
    Quaternion ori;
    ori.fromEuler(roll, pitch, yaw_true);
    // Vector3 ecef = lla_to_ecef(ref_lla);
    StateData state = {ref_lla, Vector3(), ori, Vector3()};
    std::cout << "locked in" + ref_lla.ToString() << std::endl;
    m_data_manager.post(state);
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
