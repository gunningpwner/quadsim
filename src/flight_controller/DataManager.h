// DataManager.h
#pragma once

#include "sensor_data.h"
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
    struct FlightState {
        Vector3 estimated_velocity_enu;
        Vector3 estimated_orientation_body;
        // ... add other states like position, altitude, etc.
    };

    DataManager(TimeSource time_source_func);

    // --- WRITE Methods (Called by Producers like HAL and FlightController) ---

    void postGyro(const GyroData& gyro);
    void postAccel(const AccelData& accel);
    void postMag(const MagData& mag);
    void postGPS(const GPSData& gps);
    void postFlightState(const FlightState& state);

    // --- READ Methods (For simple consumers wanting only the latest value) ---

    GyroData getLatestGyro();
    AccelData getLatestAccel();
    MagData getLatestMag();
    GPSData getLatestGPS();
    FlightState getFlightState();
    uint64_t getCurrentTimeUs() const;
    // --- CONSUME Methods (For stateful consumers like the EKF) ---

    // This pattern allows a consumer to get all new data since it last checked.
    bool consumeGyro(std::vector<GyroData>& gyro_samples, unsigned int& last_seen_count);
    bool consumeAccel(std::vector<AccelData>& accel_samples, unsigned int& last_seen_count);
    bool consumeMag(std::vector<MagData>& mag_samples, unsigned int& last_seen_count);
    bool consumeGPS(std::vector<GPSData>& gps_samples, unsigned int& last_seen_count);

private:
    std::mutex m_sensor_mutex; // Guards Gyro, Accel, Mag
    std::mutex m_gps_mutex;
    std::mutex m_state_mutex;

    // ---  Sensor Data Buffers ---
    std::deque<GyroData> m_gyro_buffer;
    std::deque<AccelData> m_accel_buffer;
    std::deque<MagData> m_mag_buffer;
    std::deque<GPSData> m_gps_buffer;
    
    TimeSource m_time_source;

    // --- Update counters to track "newness" ---
    unsigned int m_gyro_update_count;
    unsigned int m_accel_update_count;
    unsigned int m_mag_update_count;
    unsigned int m_gps_update_count;

    

    // --- Calculated State Data ---
    FlightState m_flight_state;
};