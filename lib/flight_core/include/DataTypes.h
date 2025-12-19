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