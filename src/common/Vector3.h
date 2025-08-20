#pragma once
#include <cmath> // For std::sqrt
#include <string> // For std::to_string

// Using a struct for a simple data container.
// This also makes it "blittable" and easy to pass to C# if needed.
struct Vector3 {
    // --- Member Variables ---
    float x, y, z;

    // --- Constructors ---

    // Default constructor (initializes to zero vector)
    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}

    // Constructor with x, y, z components
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    // --- Vector-Scalar Operations ---

    // Multiplication by a scalar
    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    // Division by a scalar
    Vector3 operator/(float scalar) const {
        // Add a check for division by zero for safety
        if (scalar != 0.0f) {
            return Vector3(x / scalar, y / scalar, z / scalar);
        }
        // Return a zero vector or handle error as appropriate
        return Vector3();
    }

    // Compound multiplication assignment
    Vector3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    // Compound division assignment
    Vector3& operator/=(float scalar) {
        if (scalar != 0.0f) {
            x /= scalar;
            y /= scalar;
            z /= scalar;
        }
        return *this;
    }

    // --- Vector-Vector Operations ---

    // Addition of two vectors
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    // Subtraction of two vectors
    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }
    
    // Compound addition assignment
    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    // Compound subtraction assignment
    Vector3& operator-=(const Vector3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    // --- Utility Methods ---

    // Calculate the magnitude (length) of the vector
    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Calculate the squared magnitude (faster than magnitude as it avoids sqrt)
    float sqrMagnitude() const {
        return x * x + y * y + z * z;
    }

    // Returns a normalized version of the vector (unit vector with length 1)
    Vector3 normalized() const {
        float mag = magnitude();
        if (mag > 0.0f) {
            return *this / mag;
        }
        return Vector3();
    }

    // Normalizes this vector in-place
    void normalize() {
        float mag = magnitude();
        if (mag > 0.0f) {
            *this /= mag;
        }
    }

    std::string ToString() const {
        return std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z);
    }
    // --- Static Methods ---
    
    // Dot product of two vectors
    static float dot(const Vector3& a, const Vector3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    // Cross product of two vectors
    static Vector3 cross(const Vector3& a, const Vector3& b) {
        return Vector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    
};

inline Vector3  operator-(float scalar, const Vector3& vec){
    return Vector3(scalar-vec.x,scalar-vec.y,scalar-vec.z);
}
// Allow for scalar * vector multiplication as well
inline Vector3 operator*(float scalar, const Vector3& vec) {
    return vec * scalar;
}

inline Vector3 operator*(const Vector3& a, const Vector3& b) {
    return Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline Vector3 operator/(const Vector3& a, const Vector3& b) {
    return Vector3(a.x / b.x, a.y / b.y, a.z / b.z);
}