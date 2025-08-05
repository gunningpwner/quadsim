#ifndef SENSORDATA_H
#define SENSORDATA_H

#include "Vector3.h"
#include <cstdint>

struct GPSData
{
    int64_t Timestamp;
    int FixStatus;
    double Latitude;
    double Longitude;
    float Altitude;
    float GroundSpeed;
    Vector3 GroundVelocity;
    int Satellites;
};

struct AccelData
{
    /// <summary>
    /// The timestamp when the measurement was generated (milliseconds).
    /// </summary>
    int64_t Timestamp;

    /// <summary>
    /// Linear acceleration in the body frame (m/s^2).
    /// </summary>
    Vector3 Acceleration;
};

/// <summary>
/// Holds a single, timestamped reading from a gyroscope.
/// </summary>
struct GyroData
{
    /// <summary>
    /// The timestamp when the measurement was generated (milliseconds).
    /// </summary>
    int64_t Timestamp;

    /// <summary>
    /// Angular velocity in the body frame (radians/s).
    /// </summary>
    Vector3 AngularVelocity;
};

/// <summary>
/// Holds a single, timestamped reading from a magnetometer.
/// </summary>
struct MagData
{
    /// <summary>
    /// The timestamp when the measurement was generated (milliseconds).
    /// </summary>
    int64_t Timestamp;

    /// <summary>
    /// The magnetic field vector in the body frame (normalized).
    /// </summary>
    Vector3 MagneticField;
};

struct MotorRPMs
{
    int64_t Timestamp;
    std::array<float, 4> rpms;
};
#endif // SENSORDATA_H