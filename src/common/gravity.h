#pragma once

#include <Eigen/Dense>

/**
 * @brief Calculates the gravity vector in the ECEF frame using the WGS-84 model.
 *
 * This function provides a simplified model of Earth's gravity. It does not account
 * for the Earth's non-spherical shape (oblateness) beyond the main gravitational
 * parameter, nor does it include centrifugal effects from Earth's rotation.
 *
 * @param position_ecef The position vector (x, y, z) in the ECEF frame (in meters).
 * @return The gravity vector (gx, gy, gz) in the ECEF frame (in m/s^2).
 */
inline Eigen::Vector3f calculate_gravity_ecef(const Eigen::Vector3f& position_ecef) {
    // WGS-84 gravitational constant (mu = G * M)
    constexpr double GRAVITATIONAL_CONSTANT = 3.986004418e14; // m^3/s^2

    // Calculate the distance from the center of the Earth
    double r = position_ecef.norm();

    // If the position is very close to the center of the Earth, return zero gravity
    // to avoid division by zero and numerical instability.
    if (r < 1.0) {
        return Eigen::Vector3f::Zero();
    }

    // Calculate the magnitude of the gravitational acceleration using Newton's law of universal gravitation
    // g = mu / r^2
    double g_magnitude = GRAVITATIONAL_CONSTANT / (r * r);

    // The gravity vector points from the position towards the origin of the ECEF frame.
    // Therefore, it is in the opposite direction of the position vector.
    Eigen::Vector3f gravity_vector = -position_ecef.normalized() * static_cast<float>(g_magnitude);

    return gravity_vector;
}
