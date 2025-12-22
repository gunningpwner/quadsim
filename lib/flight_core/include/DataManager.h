// DataManager.h
#pragma once

#include "DataTypes.h"
#include "ClaimCommitRingBuffer.h"

// Configurable buffer sizes for sensor data.
constexpr size_t SENSOR_BUFFER_SIZE = 50;
constexpr size_t STATE_BUFFER_SIZE = 2;
constexpr size_t MOTOR_COMMAND_BUFFER_SIZE = 2;
constexpr size_t RC_CHANNELS_BUFFER_SIZE = 5;

class DataManager
{
public:
    DataManager() : m_sensor_buffer(),
                    m_state_buffer(),
                    m_motor_commands_buffer(),
                    m_rc_channels_buffer() {};

    using SensorBuffer = ClaimCommitRingBuffer<SensorData, SENSOR_BUFFER_SIZE>;
    using StateBuffer = ClaimCommitRingBuffer<StateEstimate, STATE_BUFFER_SIZE>;
    using MotorCommandsBuffer = ClaimCommitRingBuffer<MotorCommands, MOTOR_COMMAND_BUFFER_SIZE>;
    using RCChannelsBuffer = ClaimCommitRingBuffer<RCChannelsData, RC_CHANNELS_BUFFER_SIZE>;

    SensorBuffer &getSensorBuffer() { return m_sensor_buffer; };
    StateBuffer &getStateBuffer() { return m_state_buffer; };
    MotorCommandsBuffer &getMotorCommandsBuffer() { return m_motor_commands_buffer; };
    RCChannelsBuffer &getRCChannelsBuffer() { return m_rc_channels_buffer; };

    using SensorConsumer = CCRBNonCopyingConsumer<SensorData, SENSOR_BUFFER_SIZE>;
    using StateConsumer = CCRBNonCopyingConsumer<StateEstimate, STATE_BUFFER_SIZE>;
    using MotorCommandsConsumer = CCRBNonCopyingConsumer<MotorCommands, MOTOR_COMMAND_BUFFER_SIZE>;
    using RCChannelsConsumer = CCRBNonCopyingConsumer<RCChannelsData, RC_CHANNELS_BUFFER_SIZE>;

    SensorConsumer makeSensorConsumer() { return SensorConsumer(m_sensor_buffer); };
    StateConsumer makeStateConsumer() { return StateConsumer(m_state_buffer); };
    MotorCommandsConsumer makeMotorCommandsConsumer() { return MotorCommandsConsumer(m_motor_commands_buffer); };
    RCChannelsConsumer makeRCChannelsConsumer() { return RCChannelsConsumer(m_rc_channels_buffer); };

private:
    SensorBuffer m_sensor_buffer;
    StateBuffer m_state_buffer;
    MotorCommandsBuffer m_motor_commands_buffer;
    RCChannelsBuffer m_rc_channels_buffer;
};