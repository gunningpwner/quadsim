// DataManager.h
#pragma once

#include "SensorData.h"
#include "DataChannel.h"
#include <deque>
#include <mutex>
#include <vector>

#include <functional> // Required for std::function

// Configurable buffer sizes for sensor data.
constexpr size_t IMU_BUFFER_SIZE = 50;
constexpr size_t GPS_BUFFER_SIZE = 10; // GPS updates less frequently

using TimeSource = std::function<uint64_t()>;

// This class is the central "blackboard" for the entire flight software.
// It holds buffers of recent  sensor data, the latest calculated state
// from the flight controller, and other shared system state.
class DataManager {
public:
    // --- Struct for Calculated State ---


    DataManager(TimeSource time_source_func):
    m_time_source(time_source_func),
    m_gyro_channel(IMU_BUFFER_SIZE),
    m_accel_channel(IMU_BUFFER_SIZE),
    m_mag_channel(IMU_BUFFER_SIZE),
    m_gps_channel(GPS_BUFFER_SIZE){}

    // --- WRITE Methods (Called by Producers like HAL and FlightController) ---

    void post(const GyroData& data){m_gyro_channel.post(data);};
    void post(const AccelData& data){m_accel_channel.post(data);};
    void post(const MagData& data){m_mag_channel.post(data);};
    void post(const GPSPositionData& data){m_gps_channel.post(data);};


    // --- READ Methods (For simple consumers wanting only the latest value) ---

    void getLatest(GyroData& latest_data){m_gyro_channel.getLatest(latest_data);};
    void getLatest(AccelData& latest_data){m_accel_channel.getLatest(latest_data);};
    void getLatest(MagData& latest_data){m_mag_channel.getLatest(latest_data);};
    void getLatest(GPSPositionData& latest_data){m_gps_channel.getLatest(latest_data);};

    uint64_t getCurrentTimeUs() const;
    // --- CONSUME Methods (For stateful consumers like the EKF) ---

    // This pattern allows a consumer to get all new data since it last checked.
    bool consume(std::vector<GyroData>& samples, unsigned int& last_seen_count){
        return m_gyro_channel.consume(samples, last_seen_count);
    };
    bool consume(std::vector<AccelData>& samples, unsigned int& last_seen_count){
        return m_accel_channel.consume(samples, last_seen_count);
    };
    bool consume(std::vector<MagData>& samples, unsigned int& last_seen_count){
        return m_mag_channel.consume(samples, last_seen_count);
    };
    bool consume(std::vector<GPSPositionData>& samples, unsigned int& last_seen_count){
        return m_gps_channel.consume(samples, last_seen_count);
    };

private:

    TimeSource m_time_source;

    DataChannel<GyroData>  m_gyro_channel;
    DataChannel<AccelData> m_accel_channel;
    DataChannel<MagData>   m_mag_channel;
    DataChannel<GPSPositionData>   m_gps_channel;

};