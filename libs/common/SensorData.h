#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <cstdint>
#include <Eigen/Dense>
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
    Eigen::Vector3f lla;
    // Covariance of position in the local NED frame (North, East, Down).
    // Units are meters-squared (m^2).
    Eigen::Vector3f position_covariances;
    int FixStatus;
    int Satellites;
};

// NMEA GPVTG
// UBX-NAV-VELNED will give covariances
struct GPSVelocityData:TimestampedData
{
    Eigen::Vector3f velocity; //technically just en since gps only gives you groundspeed
    Eigen::Vector3f velocity_covariances;
};



struct AccelData:TimestampedData
{
    /// <summary>
    /// Linear acceleration in the body frame (m/s^2).
    /// </summary>
    Eigen::Vector3f Acceleration;
};

/// <summary>
/// Holds a single, timestamped reading from a gyroscope.
/// </summary>
struct GyroData:TimestampedData
{
    /// <summary>
    /// Angular velocity in the body frame (radians/s).
    /// </summary>
    Eigen::Vector3f AngularVelocity;
};

/// <summary>
/// Holds a single, timestamped reading from a magnetometer.
/// </summary>
struct MagData:TimestampedData
{
    /// <summary>
    /// The magnetic field vector in the body frame (normalized).
    /// </summary>
    Eigen::Vector3f MagneticField;
};

struct MotorRPMs:TimestampedData
{
    std::array<float, 4> rpms;
};
#endif // SENSORDATA_H