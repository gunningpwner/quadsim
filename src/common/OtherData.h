#ifndef OTHERDATA_H
#define OTHERDATA_H

#include "Quaternion.h"
#include "Vector3.h"



struct InputData
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
};

struct StateData
{
    // Position in ECEF (Earth-Centered, Earth-Fixed) coordinates
    Vector3 position_ecef; 
    // Velocity in ECEF coordinates
    Vector3 velocity_ecef;
    // Orientation as a quaternion (w, x, y, z)
    Quaternion orientation;
    // Angular velocity in body frame
    Vector3 angular_velocity_body;
};

struct MotorCommands
{
    float motor_speed[4]; // Array to hold speeds for 4 motors
};



#endif // OTHERDATA_H