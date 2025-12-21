#pragma once
#include <cstdint>

struct SensorData
{
    uint64_t timestamp;
    enum class Type : uint8_t
    {
        IMU,
        GPS,
        MAG
    } sensor;
    union Data {
        struct 
        {
            float accel[3];
            float gyro[3];
        } imu;
        
        struct
        {
            float lla[3];
            float vel[3];
        } gps;

        struct
        {
            float mag[3];
        } mag;
    } data;
};

struct StateEstimate{
    uint64_t timestamp;
    float position_ecef[3];
    float velocity_ecef[3];
    float orientation[4];
    float angular_velocity_body[3];

};

struct MotorCommands
{
    uint64_t timestamp;
    bool is_throttle_command;
    uint16_t throttle[4];
    uint16_t command;
};

struct CRSFPackedChannels {
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__((__packed__));

/// <summary>
/// Holds a timestamped set of RC channel values.
/// </summary>
struct RCChannelsData {
    uint64_t timestamp;
    CRSFPackedChannels channels;
};