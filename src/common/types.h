#ifndef TYPES_H
#define TYPES_H
#include <cstdint>
struct Vector3 {
    float x, y, z;
};

struct Quaternion {
    float w, x, y, z;
};

struct RigidbodyState {
    Vector3 position;
    Vector3 rotation;
    Vector3 velocity;
    Vector3 angular_velocity;
};

struct UserInput {
    float roll;
    float pitch;
    float yaw;
    float throttle;
};

struct SensorData {
    Vector3 gyroscope;
    Vector3 accelerometer;
    Vector3 magnetometer;
};

struct MotorCommands {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
};

struct ForcesAndTorques {
    Vector3 force;
    Vector3 torque;
};

struct GPSData
{
    int64_t  Timestamp; // UTC time or similar
    int FixStatus; // 0 = No Fix, 2 = 2D, 3 = 3D
    double Latitude;
    double Longitude;
    float Altitude;
    float GroundSpeed;
    Vector3 GroundVelocity; // In Unity's world space
    int Satellites;
};

#endif // TYPES_H
