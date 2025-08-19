// Quaternion.h
#pragma once

#include "Vector3.h" // Assumes you have your Vector3 class
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Quaternion {
    // --- Member Variables ---
    float w, x, y, z;

    // --- Constructors ---
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {} // Identity quaternion
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    // --- Operator Overloads ---

    // Quaternion multiplication
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }

    // --- Methods ---

    void normalize() {
        float mag = std::sqrt(w * w + x * x + y * y + z * z);
        if (mag > 1e-6f) {
            w /= mag; x /= mag; y /= mag; z /= mag;
        }
    }
    
    // Returns the conjugate of the quaternion (represents the opposite rotation)
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Rotates a vector by this quaternion
    Vector3 rotate(const Vector3& v) const {
        Quaternion v_quat(0, v.x, v.y, v.z);
        Quaternion result = *this * v_quat * this->conjugate();
        return Vector3(result.x, result.y, result.z);
    }

    // --- NEW: Convert this quaternion to Euler angles ---
    // Returns a Vector3 containing roll (x), pitch (y), yaw (z) in radians.
    Vector3 toEuler() const {
        Vector3 euler;

        // Roll (x-axis rotation)
        float sinr_cosp = 2 * (w * x + y * z);
        float cosr_cosp = 1 - 2 * (x * x + y * y);
        euler.x = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            euler.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            euler.y = std::asin(sinp);

        // Yaw (z-axis rotation)
        float siny_cosp = 2 * (w * z + x * y);
        float cosy_cosp = 1 - 2 * (y * y + z * z);
        euler.z = std::atan2(siny_cosp, cosy_cosp);

        return euler;
    }


    // --- Static Methods ---

    // This function handles the conversion from a body rate (gyro) to a quaternion.
    static Quaternion integrate(const Quaternion& q_old, const Vector3& gyro, float dt) {
        float w = q_old.w, x = q_old.x, y = q_old.y, z = q_old.z;
        float gx = gyro.x, gy = gyro.y, gz = gyro.z;

        float w_dot = -0.5f * (x * gx + y * gy + z * gz);
        float x_dot =  0.5f * (w * gx + y * gz - z * gy);
        float y_dot =  0.5f * (w * gy - x * gz + z * gx);
        float z_dot =  0.5f * (w * gz + x * gy - y * gx);

        Quaternion q_new(
            w + w_dot * dt,
            x + x_dot * dt,
            y + y_dot * dt,
            z + z_dot * dt
        );
        q_new.normalize();
        return q_new;
    }

    // --- NEW: Create a quaternion from Euler angles ---
    // Angles should be in radians: roll (x), pitch (y), yaw (z).
    static Quaternion fromEuler(float roll, float pitch, float yaw) {
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        
        return q;
    }

    static Quaternion fromEuler(const Vector3& euler) {
        return fromEuler(euler.x, euler.y, euler.z);
    }
};
