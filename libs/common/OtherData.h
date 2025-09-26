#ifndef OTHERDATA_H
#define OTHERDATA_H

#include <Eigen/Dense>


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
    Eigen::Vector3f position_ecef; 
    // Velocity in ECEF coordinates
    Eigen::Vector3f velocity_ecef;
    // Orientation as a quaternion (w, x, y, z)
    Eigen::Quaternionf orientation;
    // Angular velocity in the body frame (rad/s)
    Eigen::Vector3f angular_velocity_body;

    /**
     * @brief Checks if any of the struct's floating-point members contain NaN.
     * 
     * This relies on the underlying Vector3 and Quaternion types also
     * implementing a containsNaN() method.
     * @return true if a NaN value is found, false otherwise.
     */
    bool containsNaN() const
    {
        return position_ecef.hasNaN() || 
               velocity_ecef.hasNaN() || 
               angular_velocity_body.hasNaN() ||
               std::isnan(orientation.w()) || std::isnan(orientation.x()) || std::isnan(orientation.y()) || std::isnan(orientation.z());
    }
};

struct MotorCommands
{
    float motor_speed[4]; // Array to hold speeds for 4 motors
};



#endif // OTHERDATA_H