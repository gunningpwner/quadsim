// DataManager.h
#pragma once

#include "sensor_data.h" // Using the types from your provided header
#include <deque>
#include <mutex>

// This class is the central "blackboard" for the entire flight software.
// It holds the latest raw sensor data, the latest calculated state from the
// flight controller, and any other shared system state.
class DataManager {
public:
    // --- Struct for Calculated State ---
    struct FlightState {
        Vector3 estimated_velocity_enu;
        Vector3 estimated_orientation_body;
        // ... add other states like position, altitude, etc.
    };

    DataManager() : 
        m_new_gps_data_available(false),
        m_gyro_update_count(0),
        m_accel_update_count(0),
        m_mag_update_count(0)
    {}

    // --- WRITE Methods (Called by Producers like HAL and FlightController) ---

    void postRawGyro(const GyroData& gyro) { 
        std::lock_guard<std::mutex> lock(m_sensor_mutex); 
        m_raw_gyro = gyro; 
        m_gyro_update_count++; // Increment counter on new data
    }
    void postRawAccel(const AccelData& accel) { 
        std::lock_guard<std::mutex> lock(m_sensor_mutex); 
        m_raw_accel = accel; 
        m_accel_update_count++; // Increment counter on new data
    }
    void postRawMag(const MagData& mag) { 
        std::lock_guard<std::mutex> lock(m_sensor_mutex); 
        m_raw_mag = mag;
        m_mag_update_count++; // Increment counter on new data
    }
    
    void postRawGPS(const GPSData& gps) {
        std::lock_guard<std::mutex> lock(m_gps_mutex);
        m_gps_history.push_back(gps);
        if (m_gps_history.size() > 2) m_gps_history.pop_front();
        if (m_gps_history.size() >= 2) m_new_gps_data_available = true;
    }

    void postFlightState(const FlightState& state) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        m_flight_state = state;
    }

    // --- READ Methods (For simple consumers like OSD/Loggers) ---

    GyroData getRawGyro() { std::lock_guard<std::mutex> lock(m_sensor_mutex); return m_raw_gyro; }
    AccelData getRawAccel() { std::lock_guard<std::mutex> lock(m_sensor_mutex); return m_raw_accel; }
    MagData getRawMag() { std::lock_guard<std::mutex> lock(m_sensor_mutex); return m_raw_mag; }
    
    FlightState getFlightState() {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        return m_flight_state;
    }

    // --- CONSUME Methods (For stateful consumers like the EKF) ---

    // This pattern allows a consumer to check for new data since it last checked.
    bool consumeRawGyro(GyroData& gyro, unsigned int& last_seen_count) {
        std::lock_guard<std::mutex> lock(m_sensor_mutex);
        if (last_seen_count < m_gyro_update_count) {
            gyro = m_raw_gyro;
            last_seen_count = m_gyro_update_count;
            return true; // There was new data
        }
        return false; // No new data since last check
    }

    bool consumeRawAccel(AccelData& accel, unsigned int& last_seen_count) {
        std::lock_guard<std::mutex> lock(m_sensor_mutex);
        if (last_seen_count < m_accel_update_count) {
            accel = m_raw_accel;
            last_seen_count = m_accel_update_count;
            return true;
        }
        return false;
    }

    bool consumeRawMag(MagData& mag, unsigned int& last_seen_count) {
        std::lock_guard<std::mutex> lock(m_sensor_mutex);
        if (last_seen_count < m_mag_update_count) {
            mag = m_raw_mag;
            last_seen_count = m_mag_update_count;
            return true;
        }
        return false;
    }
    
    bool consumeGPSHistory(GPSData& new_data, GPSData& old_data) {
        std::lock_guard<std::mutex> lock(m_gps_mutex);
        if (!m_new_gps_data_available) return false;
        
        new_data = m_gps_history[1];
        old_data = m_gps_history[0];
        m_new_gps_data_available = false; // Flag is consumed
        return true;
    }


private:
    std::mutex m_sensor_mutex;
    std::mutex m_gps_mutex;
    std::mutex m_state_mutex;

    // Raw Sensor Data (now using your specific struct types)
    GyroData m_raw_gyro;
    AccelData m_raw_accel;
    MagData m_raw_mag;
    std::deque<GPSData> m_gps_history;
    bool m_new_gps_data_available;

    // Update counters to track "newness"
    unsigned int m_gyro_update_count;
    unsigned int m_accel_update_count;
    unsigned int m_mag_update_count;

    // Calculated State Data
    FlightState m_flight_state;
};
