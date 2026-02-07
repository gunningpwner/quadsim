#pragma once
#include <cstdint>
#include <array>

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
            std::array<float, 3> accel;
            std::array<float, 3> gyro;
        } imu;
        
        struct
        {
            std::array<float, 3> lla;
            std::array<float, 3> vel;
        } gps;

        struct
        {
            std::array<float, 3> mag;
        } mag;
    } data;
};

struct FastIMUData
{
    uint64_t timestamp;
    std::array<float, 6> data;
};

struct FastRPMData
{
    uint64_t timestamp;
    std::array<float, 4> rpm;
};


struct StateEstimate{
    uint64_t timestamp;
    std::array<float, 3> position_enu;
    std::array<float, 3> velocity_enu;
    std::array<float, 4> orientation;
    std::array<float, 3> ref_lla;
    std::array<float, 3> angular_vel;

};

struct MotorCommands
{
    uint64_t timestamp;
    bool is_throttle_command;
    std::array<uint16_t, 4> throttle;
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

#define CRSF_CHANNEL_MIN 191
#define CRSF_CHANNEL_MAX 1792