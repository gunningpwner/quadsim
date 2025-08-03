#include "flight_controller.h"
#include <cmath>

FlightController::FlightController(IHAL* hal) : hal(hal) {
    bodyOrientation = {0,0,0};

    // Initialize PID gains
    Kp_roll_rate = 700;
    Ki_roll_rate = 0;
    Kd_roll_rate = 20;

    Kp_pitch_rate = 700;
    Ki_pitch_rate = 0;
    Kd_pitch_rate = 20;

    Kp_yaw_rate = 500;
    Ki_yaw_rate = 0;
    Kd_yaw_rate = 20;

    // Initialize PID state
    roll_rate_error_sum = 0;
    pitch_rate_error_sum = 0;
    yaw_rate_error_sum = 0;
    last_roll_rate_error = 0;
    last_pitch_rate_error = 0;
    last_yaw_rate_error = 0;
}

void FlightController::runFlightLoop(){
    // Get user input
    
    UserInput user_input = hal->read_user_input();

    // Get sensor data
    Vector3 gyro_data = hal->read_gyros();

    // Calculate rate errors
    float roll_rate_error = user_input.roll - gyro_data.x;
    float pitch_rate_error = user_input.pitch - gyro_data.y;
    float yaw_rate_error = user_input.yaw - gyro_data.z;

    // Update integral terms
    roll_rate_error_sum += roll_rate_error;
    pitch_rate_error_sum += pitch_rate_error;
    yaw_rate_error_sum += yaw_rate_error;

    // Calculate derivative terms
    float roll_rate_error_derivative = roll_rate_error - last_roll_rate_error;
    float pitch_rate_error_derivative = pitch_rate_error - last_pitch_rate_error;
    float yaw_rate_error_derivative = yaw_rate_error - last_yaw_rate_error;

    // Update last error terms
    last_roll_rate_error = roll_rate_error;
    last_pitch_rate_error = pitch_rate_error;
    last_yaw_rate_error = yaw_rate_error;

    // Calculate PID outputs
    float roll_output = Kp_roll_rate * roll_rate_error + Ki_roll_rate * roll_rate_error_sum + Kd_roll_rate * roll_rate_error_derivative;
    float pitch_output = Kp_pitch_rate * pitch_rate_error + Ki_pitch_rate * pitch_rate_error_sum + Kd_pitch_rate * pitch_rate_error_derivative;
    float yaw_output = Kp_yaw_rate * yaw_rate_error + Ki_yaw_rate * yaw_rate_error_sum + Kd_yaw_rate * yaw_rate_error_derivative;

    // Mixer
    float throttle = user_input.throttle;
    int motor1 = throttle + roll_output - pitch_output + yaw_output;
    int motor2 = throttle - roll_output - pitch_output - yaw_output;
    int motor3 = throttle - roll_output + pitch_output + yaw_output;
    int motor4 = throttle + roll_output + pitch_output - yaw_output;

    // Clamp motor outputs
    motor1 = std::max(0, std::min(2047, motor1));
    motor2 = std::max(0, std::min(2047, motor2));
    motor3 = std::max(0, std::min(2047, motor3));
    motor4 = std::max(0, std::min(2047, motor4));

    hal->write_motor_commands({motor1, motor2, motor3, motor4});
    
}

void FlightController::complementaryFilter(){
    // Get sensor data
    Vector3 gyro_data = hal->read_gyros();
    Vector3 accel_data = hal->read_accelerometer();

    // Calculate pitch and roll from accelerometer
    float pitch_accel = atan2(accel_data.y, accel_data.z) * 180 / M_PI;
    float roll_accel = atan2(-accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z)) * 180 / M_PI;

    // Integrate gyroscope data
    float dt = 0.004; // Assuming a 250Hz loop rate
    bodyOrientation.x += gyro_data.x * dt;
    bodyOrientation.y += gyro_data.y * dt;

    // Apply complementary filter
    float alpha = 0.98;
    bodyOrientation.x = alpha * (bodyOrientation.x) + (1 - alpha) * roll_accel;
    bodyOrientation.y = alpha * (bodyOrientation.y) + (1 - alpha) * pitch_accel;
}