// DataManager.h
#pragma once

#include "DataTypes.h"
#include "buffers/include/ClaimCommitRingBuffer.h"

#include <stdexcept>
#include <functional> // Required for std::function



// Configurable buffer sizes for sensor data.
constexpr size_t SENSOR_BUFFER_SIZE = 50;
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


    DataManager():
    m_imu_channel(), 
    m_mag_channel(),
    m_gps_channel(),
    m_input_channel(),
    m_state_channel(),
    m_motor_commands_channel(),
    m_rc_channels_channel(){}

    // --- WRITE Methods (Called by Producers like HAL and FlightController to post data) ---

    void post(const IMUData& data){m_imu_channel.post(data);};
    void post(const MagData& data){m_mag_channel.post(data);};
    void post(const GPSData& data){m_gps_channel.post(data);};
    void post(const InputData& data){m_input_channel.post(data);};
    void post(const StateData& data){m_state_channel.post(data);}; 
    void post(const MotorCommands& data){m_motor_commands_channel.post(data);};
    void post(const RCChannelsData& data){m_rc_channels_channel.post(data);};

    // --- READ Access (For Consumers to get a reference to the DataChannel) ---
    // These methods provide access to the underlying DataChannels for Consumer objects.
    DataChannel<IMUData, IMU_BUFFER_SIZE>& getIMUChannel() { return m_imu_channel; }
    DataChannel<MagData, MAG_BUFFER_SIZE>& getMagChannel() { return m_mag_channel; }
    DataChannel<GPSData, GPS_BUFFER_SIZE>& getGPSChannel() { return m_gps_channel; }
    DataChannel<InputData, INPUT_BUFFER_SIZE>& getInputChannel() { return m_input_channel; }
    DataChannel<StateData, STATE_BUFFER_SIZE>& getStateChannel() { return m_state_channel; }
    DataChannel<MotorCommands, MOTOR_COMMAND_BUFFER_SIZE>& getMotorCommandsChannel() { return m_motor_commands_channel; }
    DataChannel<RCChannelsData, RC_CHANNELS_BUFFER_SIZE>& getRCChannelsChannel() { return m_rc_channels_channel; }





private:
    // DataChannels for various sensor and system data types
    ClaimCommitRingBuffer<SensorData, SENSOR_BUFFER_SIZE> m_sensor_buffer;
    
    DataChannel<InputData, INPUT_BUFFER_SIZE>          m_input_channel;
    DataChannel<StateData, STATE_BUFFER_SIZE>          m_state_channel;
    DataChannel<MotorCommands, MOTOR_COMMAND_BUFFER_SIZE>      m_motor_commands_channel;
    DataChannel<RCChannelsData, RC_CHANNELS_BUFFER_SIZE>     m_rc_channels_channel;


};