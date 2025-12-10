// DataManager.h
#pragma once

#include "SensorData.h"
#include "DataChannel.h"
#include "OtherData.h"
#include <stdexcept>
#include <functional> // Required for std::function



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
    m_gyro_channel(), // DataChannel now takes BufferSize as template arg, not constructor arg
    m_accel_channel(),
    m_mag_channel(),
    m_gps_channel(),
    m_input_channel(),
    m_state_channel(),
    m_motor_commands_channel(),
    m_rc_channels_channel(){}

    // --- WRITE Methods (Called by Producers like HAL and FlightController to post data) ---

    void post(const GyroData& data){m_gyro_channel.post(data);};
    void post(const AccelData& data){m_accel_channel.post(data);};
    void post(const MagData& data){m_mag_channel.post(data);};
    void post(const GPSData& data){m_gps_channel.post(data);};
    void post(const InputData& data){m_input_channel.post(data);};
    void post(const StateData& data){m_state_channel.post(data);}; 
    void post(const MotorCommands& data){m_motor_commands_channel.post(data);};
    void post(const RCChannelsData& data){m_rc_channels_channel.post(data);};

    // --- READ Access (For Consumers to get a reference to the DataChannel) ---
    // These methods provide access to the underlying DataChannels for Consumer objects.
    DataChannel<GyroData, IMU_BUFFER_SIZE>& getGyroChannel() { return m_gyro_channel; }
    DataChannel<AccelData, IMU_BUFFER_SIZE>& getAccelChannel() { return m_accel_channel; }
    DataChannel<MagData, IMU_BUFFER_SIZE>& getMagChannel() { return m_mag_channel; }
    DataChannel<GPSData, GPS_BUFFER_SIZE>& getGpsChannel() { return m_gps_channel; }
    DataChannel<InputData, INPUT_BUFFER_SIZE>& getInputChannel() { return m_input_channel; }
    DataChannel<StateData, STATE_BUFFER_SIZE>& getStateChannel() { return m_state_channel; }
    DataChannel<MotorCommands, MOTOR_COMMAND_BUFFER_SIZE>& getMotorCommandsChannel() { return m_motor_commands_channel; }
    DataChannel<RCChannelsData, RC_CHANNELS_BUFFER_SIZE>& getRCChannelsChannel() { return m_rc_channels_channel; }


    uint64_t getCurrentTimeUs() const {
        // Check if the time source function is valid before calling it.
        if (!m_time_source) {
            // In a unified build, we need a consistent error handling strategy.
            // For critical errors like this, a system halt is often appropriate for flight control.
            // For SITL, this could be configured to throw an exception or log and exit.
            // For now, we'll use a simple while(1) as a placeholder for a system halt.
            // A more sophisticated system might have a global error handler.
            while(1); 
        }
        
        // Execute the stored function and return its value.
        return m_time_source();
    }
    
    void setTimeSource(TimeSource time_source_func) {
        m_time_source = time_source_func;
    }




private:

    TimeSource m_time_source; // Function pointer for getting current time

    // DataChannels for various sensor and system data types
    DataChannel<GyroData, IMU_BUFFER_SIZE>           m_gyro_channel;
    DataChannel<AccelData, IMU_BUFFER_SIZE>          m_accel_channel;
    DataChannel<MagData, IMU_BUFFER_SIZE>            m_mag_channel;
    DataChannel<GPSData, GPS_BUFFER_SIZE>    m_gps_channel;
    DataChannel<InputData, INPUT_BUFFER_SIZE>          m_input_channel;
    DataChannel<StateData, STATE_BUFFER_SIZE>          m_state_channel;
    DataChannel<MotorCommands, MOTOR_COMMAND_BUFFER_SIZE>      m_motor_commands_channel;
    DataChannel<RCChannelsData, RC_CHANNELS_BUFFER_SIZE>     m_rc_channels_channel;


};