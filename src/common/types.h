#ifndef TYPES_H
#define TYPES_H

struct Vector3 {
    float x, y, z;
};

struct Quaternion {
    float w, x, y, z;
};

struct RigidbodyState {
    Vector3 position;
    Quaternion rotation;
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

#endif // TYPES_H
