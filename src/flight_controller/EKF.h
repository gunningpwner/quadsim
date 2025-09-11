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

    uint64_t m_current_time_us;
    uint64_t m_last_predict_time_us;

    // Private methods for prediction and correction steps
    void predict();
    void correct();
    void init_filter();
    void reset_filter();

    // Steps for EKF
    //1. Lock in navigation frame with GPS, accelerometer and magnetometer measurements.
    // error out if a high enough velocity or acceleration is detected
    // 
};
