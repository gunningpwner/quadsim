// DataManager.h
#pragma once

#include "SensorData.h"
#include "DataChannel.h"
#include "OtherData.h"
#include <vector>
#include <stdexcept>
#include <functional> // Required for std::function

#ifndef FIRMWARE_BUILD
#include <mutex>
#endif



// Configurable buffer sizes for sensor data.
constexpr size_t IMU_BUFFER_SIZE = 50;
constexpr size_t GPS_BUFFER_SIZE = 10; // GPS updates less frequently
constexpr size_t INPUT_BUFFER_SIZE = 2;
constexpr size_t STATE_BUFFER_SIZE = 2;
constexpr size_t MOTOR_COMMAND_BUFFER_SIZE = 2;
constexpr size_t RC_CHANNELS_BUFFER_SIZE = 5; // RC frames can come in bursts, a slightly larger buffer is safer.

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
    m_gps_channel(GPS_BUFFER_SIZE),
    m_input_channel(INPUT_BUFFER_SIZE),
    m_state_channel(STATE_BUFFER_SIZE),
    m_motor_command_channel(MOTOR_COMMAND_BUFFER_SIZE),
    m_rc_channels_channel(RC_CHANNELS_BUFFER_SIZE){}

    // --- WRITE Methods (Called by Producers like HAL and FlightController) ---

    void post(const GyroData& data){m_gyro_channel.post(data);};
    void post(const AccelData& data){m_accel_channel.post(data);};
    void post(const MagData& data){m_mag_channel.post(data);};
    void post(const GPSPositionData& data){m_gps_channel.post(data);};
    void post(const InputData& data){m_input_channel.post(data);};
    void post(const StateData& data){m_state_channel.post(data);};
    void post(const MotorCommands& data){m_motor_command_channel.post(data);};
    void post(const RCChannelsData& data){m_rc_channels_channel.post(data);};


    // --- READ Methods (For simple consumers wanting only the latest value) ---

    void getLatest(GyroData& latest_data){m_gyro_channel.getLatest(latest_data);};
    void getLatest(AccelData& latest_data){m_accel_channel.getLatest(latest_data);};
    void getLatest(MagData& latest_data){m_mag_channel.getLatest(latest_data);};
    void getLatest(GPSPositionData& latest_data){m_gps_channel.getLatest(latest_data);};
    void getLatest(InputData& latest_data){m_input_channel.getLatest(latest_data);};
    void getLatest(StateData& latest_data){m_state_channel.getLatest(latest_data);};
    void getLatest(MotorCommands& latest_data){m_motor_command_channel.getLatest(latest_data);};
    void getLatest(RCChannelsData& latest_data){m_rc_channels_channel.getLatest(latest_data);};

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
    bool consume(std::vector<InputData>& samples, unsigned int& last_seen_count){
        return m_input_channel.consume(samples, last_seen_count);
    };
    bool consume(std::vector<StateData>& samples, unsigned int& last_seen_count){
        return m_state_channel.consume(samples,last_seen_count);
    };
    bool consume(std::vector<MotorCommands>& samples, unsigned int& last_seen_count){
        return m_motor_command_channel.consume(samples,last_seen_count);
    };
    bool consume(std::vector<RCChannelsData>& samples, unsigned int& last_seen_count){
        return m_rc_channels_channel.consume(samples, last_seen_count);
    };


    uint64_t getCurrentTimeUs() const {
        // Check if the time source function is valid before calling it.
        if (!m_time_source) {
#ifdef FIRMWARE_BUILD
            // On embedded, we can't throw. Halt the system on critical error.
            while(1);
#else
            // Handle the error appropriately. Throwing an exception is a common choice.
            throw std::runtime_error("Time source not initialized in DataManager.");
#endif
        }
        
        // Execute the stored function and return its value.
        return m_time_source();
    }
    
    void setTimeSource(TimeSource time_source_func) {
#ifndef FIRMWARE_BUILD
        std::lock_guard<std::mutex> lock(m_time_mutex);
#endif
        m_time_source = time_source_func;
    }


private:

    TimeSource m_time_source;

    DataChannel<GyroData>  m_gyro_channel;
    DataChannel<AccelData> m_accel_channel;
    DataChannel<MagData>   m_mag_channel;
    DataChannel<GPSPositionData>   m_gps_channel;
    DataChannel<InputData> m_input_channel;
    DataChannel<StateData> m_state_channel;
    DataChannel<MotorCommands> m_motor_command_channel;
    DataChannel<RCChannelsData> m_rc_channels_channel;

#ifndef FIRMWARE_BUILD
    std::mutex m_time_mutex;
#endif
};