#include "BodyRateController.h"
#include <iostream>
#include <algorithm> // For std::clamp

// Define PID gains (these would typically be tuned)
// Placeholder values - adjust as needed
constexpr float ROLL_KP = 0.5f;
constexpr float ROLL_KI = 0.0f;
constexpr float ROLL_KD = 0.05f;
constexpr float ROLL_INTEGRAL_SAT_LIMIT = 0.1f;

constexpr float PITCH_KP = 0.5f;
constexpr float PITCH_KI = 0.0f;
constexpr float PITCH_KD = 0.05f;
constexpr float PITCH_INTEGRAL_SAT_LIMIT = 0.1f;

constexpr float YAW_KP = 0.2f;
constexpr float YAW_KI = 0.0f;
constexpr float YAW_KD = 0.02f;
constexpr float YAW_INTEGRAL_SAT_LIMIT = 0.1f;

BodyRateController::BodyRateController(DataManager& data_manager)
    : Controller(data_manager),
      m_roll_pid(ROLL_KP, ROLL_KI, ROLL_KD, ROLL_INTEGRAL_SAT_LIMIT),
      m_pitch_pid(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_INTEGRAL_SAT_LIMIT),
      m_yaw_pid(YAW_KP, YAW_KI, YAW_KD, YAW_INTEGRAL_SAT_LIMIT) {
    // Constructor implementation
}

void BodyRateController::run() {
    m_current_time_us = m_data_manager.getCurrentTimeUs();
    float dt = (m_current_time_us - m_last_run_time_us) / 1000000.0f; // Convert to seconds
    if (m_last_run_time_us == 0) { // First run
        dt = 0.0f;
    }

    InputData input_data;
    m_data_manager.getLatest(input_data);

    StateData state_data;
    m_data_manager.getLatest(state_data);

    // Calculate errors for body rates
    // Assuming input_data.roll/pitch/yaw are desired body rates
    float roll_rate_error = input_data.roll - state_data.angular_velocity_body.x;
    float pitch_rate_error = input_data.pitch - state_data.angular_velocity_body.y;
    float yaw_rate_error = input_data.yaw - state_data.angular_velocity_body.z;

    // Calculate PID outputs
    float roll_control = m_roll_pid.calculate(roll_rate_error, dt);
    float pitch_control = m_pitch_pid.calculate(pitch_rate_error, dt);
    float yaw_control = m_yaw_pid.calculate(yaw_rate_error, dt);

    // Motor Mixing (assuming X-quad configuration)
    // Motor 0: Front-Right
    // Motor 1: Rear-Left
    // Motor 2: Front-Left
    // Motor 3: Rear-Right

    MotorCommands motor_commands;
    float base_throttle = input_data.throttle;

    // Apply mixing. These coefficients might need tuning based on physical setup.
    motor_commands.motor_speed[0] = base_throttle - pitch_control + roll_control + yaw_control; // Front-Right
    motor_commands.motor_speed[1] = base_throttle + pitch_control - roll_control + yaw_control; // Rear-Left
    motor_commands.motor_speed[2] = base_throttle - pitch_control - roll_control - yaw_control; // Front-Left
    motor_commands.motor_speed[3] = base_throttle + pitch_control + roll_control - yaw_control; // Rear-Right

    // Normalize and clamp motor speeds to 0-1 range
    for (int i = 0; i < 4; ++i) {
        motor_commands.motor_speed[i] = std::clamp(motor_commands.motor_speed[i], 0.0f, 1.0f);
    }

    m_data_manager.post(motor_commands);

    m_last_run_time_us = m_current_time_us;
}