#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <cstdint>
#include <Eigen/Dense>
#include <array>
struct TimestampedData {
    enum class Type {
        Generic,
        GPSPosition,
        GPSVelocity,
        Accel,
        Gyro,
        Mag,
        MotorRPMs,
        RCChannels
    };

    int64_t Timestamp = 0;
    Type type = Type::Generic;

    // A virtual destructor is crucial for polymorphism
    virtual ~TimestampedData() = default;
};

// most likely from NMEA GPGGA, and GPGSA messages.
// UBX-NAV-COV will give full 3x3 ned covariance
struct GPSPositionData:TimestampedData
{   
    GPSPositionData() { type = Type::GPSPosition; }
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
    GPSVelocityData() { type = Type::GPSVelocity; }
    Eigen::Vector3f velocity; //technically just en since gps only gives you groundspeed
    Eigen::Vector3f velocity_covariances;
};



struct AccelData:TimestampedData
{
    AccelData() { type = Type::Accel; }
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
    GyroData() { type = Type::Gyro; }
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
    MagData() { type = Type::Mag; }
    /// <summary>
    /// The magnetic field vector in the body frame (normalized).
    /// </summary>
    Eigen::Vector3f MagneticField;
};

struct MotorRPMs:TimestampedData
{
    MotorRPMs() { type = Type::MotorRPMs; }
    std::array<float, 4> rpms;
};


// This struct directly maps to the 22-byte payload of a CRSF "Packed RC Channels" frame.
// It should not be modified without understanding the CRSF protocol.
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
struct RCChannelsData : TimestampedData {
    RCChannelsData() { type = Type::RCChannels; }
    CRSFPackedChannels channels;
};

#endif // SENSORDATA_H