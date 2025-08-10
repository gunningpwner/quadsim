#ifndef SENSORDATA_H
#define SENSORDATA_H

#include "Vector3.h"
#include <cstdint>
#include <array>
struct TimestampedData {
    int64_t Timestamp = 0;

    // A virtual destructor is crucial for polymorphism
    virtual ~TimestampedData() = default;
};

// most likely from NMEA GPGGA, and GPGSA messages.
// UBX-NAV-COV will give full 3x3 ned covariance
struct GPSPositionData:TimestampedData
{   
    Vector3 lla;
    Vector3 position_covariances;
    int FixStatus;
    int Satellites;
};

// NMEA GPVTG
// UBX-NAV-VELNED will give covariances
struct GPSVelocityData:TimestampedData
{
    Vector3 velocity; //technically just en since gps only gives you groundspeed
    Vector3 velocity_covariances;
};



struct AccelData:TimestampedData
{
    /// <summary>
    /// Linear acceleration in the body frame (m/s^2).
    /// </summary>
    Vector3 Acceleration;
};

/// <summary>
/// Holds a single, timestamped reading from a gyroscope.
/// </summary>
struct GyroData:TimestampedData
{
    /// <summary>
    /// Angular velocity in the body frame (radians/s).
    /// </summary>
    Vector3 AngularVelocity;
};

/// <summary>
/// Holds a single, timestamped reading from a magnetometer.
/// </summary>
struct MagData:TimestampedData
{
    int64_t Timestamp;

    /// <summary>
    /// The magnetic field vector in the body frame (normalized).
    /// </summary>
    Vector3 MagneticField;
};

struct MotorRPMs:TimestampedData
{
    std::array<float, 4> rpms;
};
#endif // SENSORDATA_H