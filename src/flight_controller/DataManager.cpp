// DataManager.cpp
#include "DataManager.h"
#include <algorithm> // For std::min

DataManager::DataManager(TimeSource time_source_func):
    m_time_source(time_source_func),
    m_gyro_update_count(0),
    m_accel_update_count(0),
    m_mag_update_count(0),
    m_gps_update_count(0)
{}

// --- WRITE Method Implementations ---

void DataManager::postGyro(const GyroData& gyro) {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    m_gyro_buffer.push_back(gyro);
    if (m_gyro_buffer.size() > IMU_BUFFER_SIZE) {
        m_gyro_buffer.pop_front();
    }
    m_gyro_update_count++;
}

void DataManager::postAccel(const AccelData& accel) {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    m_accel_buffer.push_back(accel);
    if (m_accel_buffer.size() > IMU_BUFFER_SIZE) {
        m_accel_buffer.pop_front();
    }
    m_accel_update_count++;
}

void DataManager::postMag(const MagData& mag) {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    m_mag_buffer.push_back(mag);
    if (m_mag_buffer.size() > IMU_BUFFER_SIZE) {
        m_mag_buffer.pop_front();
    }
    m_mag_update_count++;
}

void DataManager::postGPS(const GPSData& gps) {
    std::lock_guard<std::mutex> lock(m_gps_mutex);
    m_gps_buffer.push_back(gps);
    if (m_gps_buffer.size() > GPS_BUFFER_SIZE) {
        m_gps_buffer.pop_front();
    }
    m_gps_update_count++;
}

void DataManager::postFlightState(const FlightState& state) {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_flight_state = state;
}

// --- READ Method Implementations ---

GyroData DataManager::getLatestGyro() {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    return m_gyro_buffer.empty() ? GyroData() : m_gyro_buffer.back();
}

AccelData DataManager::getLatestAccel() {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    return m_accel_buffer.empty() ? AccelData() : m_accel_buffer.back();
}

MagData DataManager::getLatestMag() {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    return m_mag_buffer.empty() ? MagData() : m_mag_buffer.back();
}

GPSData DataManager::getLatestGPS() {
    std::lock_guard<std::mutex> lock(m_gps_mutex);
    return m_gps_buffer.empty() ? GPSData() : m_gps_buffer.back();
}

DataManager::FlightState DataManager::getFlightState() {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_flight_state;
}

// --- CONSUME Method Implementations ---

bool DataManager::consumeGyro(std::vector<GyroData>& gyro_samples, unsigned int& last_seen_count) {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    if (last_seen_count >= m_gyro_update_count) return false;

    size_t new_samples_count = m_gyro_update_count - last_seen_count;
    size_t num_to_copy = std::min(new_samples_count, m_gyro_buffer.size());

    gyro_samples.assign(m_gyro_buffer.end() - num_to_copy, m_gyro_buffer.end());

    last_seen_count = m_gyro_update_count;
    return true;
}

bool DataManager::consumeAccel(std::vector<AccelData>& accel_samples, unsigned int& last_seen_count) {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    if (last_seen_count >= m_accel_update_count) return false;

    size_t new_samples_count = m_accel_update_count - last_seen_count;
    size_t num_to_copy = std::min(new_samples_count, m_accel_buffer.size());

    accel_samples.assign(m_accel_buffer.end() - num_to_copy, m_accel_buffer.end());

    last_seen_count = m_accel_update_count;
    return true;
}

bool DataManager::consumeMag(std::vector<MagData>& mag_samples, unsigned int& last_seen_count) {
    std::lock_guard<std::mutex> lock(m_sensor_mutex);
    if (last_seen_count >= m_mag_update_count) return false;

    size_t new_samples_count = m_mag_update_count - last_seen_count;
    size_t num_to_copy = std::min(new_samples_count, m_mag_buffer.size());
    
    mag_samples.assign(m_mag_buffer.end() - num_to_copy, m_mag_buffer.end());
    
    last_seen_count = m_mag_update_count;
    return true;
}

bool DataManager::consumeGPS(std::vector<GPSData>& gps_samples, unsigned int& last_seen_count) {
    std::lock_guard<std::mutex> lock(m_gps_mutex);
    if (last_seen_count >= m_gps_update_count) return false;

    size_t new_samples_count = m_gps_update_count - last_seen_count;
    size_t num_to_copy = std::min(new_samples_count, m_gps_buffer.size());

    gps_samples.assign(m_gps_buffer.end() - num_to_copy, m_gps_buffer.end());

    last_seen_count = m_gps_update_count;
    return true;
}