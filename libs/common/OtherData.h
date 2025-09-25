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
    // Angular velocity in the body frame (rad/s)
    Vector3 angular_velocity_body;

    /**
     * @brief Checks if any of the struct's floating-point members contain NaN.
     * 
     * This relies on the underlying Vector3 and Quaternion types also
     * implementing a containsNaN() method.
     * @return true if a NaN value is found, false otherwise.
     */
    bool containsNaN() const
    {
        return position_ecef.containsNaN() || velocity_ecef.containsNaN() || orientation.containsNaN() || angular_velocity_body.containsNaN();
    }
};

struct MotorCommands
{
    float motor_speed[4]; // Array to hold speeds for 4 motors
};



#endif // OTHERDATA_H