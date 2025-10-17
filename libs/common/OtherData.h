#ifndef OTHERDATA_H
#define OTHERDATA_H

#include <cstdint>
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
    int64_t Timestamp;
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

enum class DShot_Command : uint16_t {
    MOTOR_STOP = 0,
    BEEP1 = 1,
    BEEP2 = 2,
    BEEP3 = 3,
    BEEP4 = 4,
    BEEP5 = 5,
    ESC_INFO = 6, 
    SPIN_DIRECTION_1 = 7,
    SPIN_DIRECTION_2 = 8,
    MODE_3D_OFF = 9,
    MODE_3D_ON = 10,
    SETTINGS_REQUEST = 11, // V2
    SAVE_SETTINGS = 12,
    SPIN_DIRECTION_NORMAL = 20,
    SPIN_DIRECTION_REVERSED = 21,
    LED0_ON = 22,
    LED1_ON = 23,
    LED2_ON = 24,
    LED3_ON = 25,
};

struct MotorCommands
{
    uint64_t Timestamp;
    bool is_throttle_command; // true for throttle values, false for a command
    uint16_t throttle[4];     // Throttle values (0-2047)
    DShot_Command command;    // Command to be sent to all motors
};



#endif // OTHERDATA_H