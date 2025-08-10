// EKF.h
#pragma once

#include "SensorData.h"
#include "FilterBase.h"
#include "Consumer.h"

// Extended Kalman Filter class for state estimation
class EKF : public FilterBase {
public:
    EKF(DataManager& data_manager);

    void run() override;
    void processSensorMeasurements();

private:
    Consumer<GyroData> m_gyro_consumer;
    Consumer<AccelData> m_accel_consumer;
    Consumer<MagData> m_mag_consumer;
    Consumer<GPSPositionData> m_gps_consumer;

    bool locked_in;

    uint64_t m_current_time_us;
    uint64_t m_last_predict_time_us;
    bool pos_lock;
    Vector3 ref_lla;
    Vector3 pos_var;
    bool grav_lock;
    Vector3 grav;
    Vector3 grav_var;
    bool mag_lock;
    Vector3 mag;
    Vector3 mag_var;

    // Private methods for prediction and correction steps
    void predict();
    void correct();
    void init_filter();
    void reset_filter();

    void bootstrap_position();
    void bootstrap_mag();
    void bootstrap_grav();
    void lock_in();
    // Steps for EKF
    //1. Lock in navigation frame with GPS, accelerometer and magnetometer measurements.
    // error out if a high enough velocity or acceleration is detected
    // 
};
