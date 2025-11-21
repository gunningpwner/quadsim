#include "AutoLevelController.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr float DEGREES_TO_RADIANS = M_PI / 180.0f;

AutoLevelController::AutoLevelController(DataManager& data_manager)
    : Controller(data_manager),
      m_rc_channels_consumer(data_manager.getRCChannelsChannel()),
      m_state_consumer(data_manager.getStateChannel()),
      m_roll_angle_pid(4.0f, 0.0f, 0.0f),
      m_pitch_angle_pid(4.0f, 0.0f, 0.0f),

      m_roll_rate_pid(0.2f, 0.1f, 0.005f),
      m_pitch_rate_pid(0.2f, 0.1f, 0.005f),
      m_yaw_rate_pid(0.3f, 0.0f, 0.0f),

      m_latest_rc_data{},
      m_latest_state_data{}
{}

void AutoLevelController::run() {

    if (m_rc_channels_consumer.consumeLatest())
    {
        m_latest_rc_data = m_rc_channels_consumer.get_span().first[0];
    }
    if (m_state_consumer.consumeLatest())
    {
        m_latest_state_data = m_state_consumer.get_span().first[0];
    }


    uint64_t current_time_us = m_data_manager.getCurrentTimeUs();
    if (m_last_run_time_us == 0) {
        m_last_run_time_us = current_time_us;
        return;
    }
    float dt = (current_time_us - m_last_run_time_us) * 1.0e-6f;
    m_last_run_time_us = current_time_us;
    if (dt <= 0.0f) {
        return;
    }


    float desired_roll_angle_rad = map_rc_to_norm(m_latest_rc_data.channels.chan0) * m_max_angle_deg * DEGREES_TO_RADIANS;

    float desired_pitch_angle_rad = -map_rc_to_norm(m_latest_rc_data.channels.chan1) * m_max_angle_deg * DEGREES_TO_RADIANS;
    float desired_yaw_rate_rps = map_rc_to_norm(m_latest_rc_data.channels.chan3) * m_max_yaw_rate_dps * DEGREES_TO_RADIANS;

    float throttle_norm = (map_rc_to_norm(m_latest_rc_data.channels.chan2) + 1.0f) / 2.0f;


    const Eigen::Quaternionf& q = m_latest_state_data.orientation;
    
    // Please don't mess with these. I have no idea what frame anything is, but i turned the quad and looked at the output and this looks right
    float current_roll_rad =asin(2.0f * (q.w() * q.y() - q.z() * q.x()));
    float current_pitch_rad = atan2(2.0f * (q.w() * q.x() + q.y() * q.z()), 1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y()));
    // printf("%f %f\n", current_roll_rad*180/M_PI, current_pitch_rad*180/M_PI);

    float roll_error = desired_roll_angle_rad - current_roll_rad;
    float pitch_error = desired_pitch_angle_rad - current_pitch_rad;
    float desired_roll_rate_rps = m_roll_angle_pid.calculate(roll_error, dt);
    float desired_pitch_rate_rps = m_pitch_angle_pid.calculate(pitch_error, dt);

    const Eigen::Vector3f& current_rates_rps = m_latest_state_data.angular_velocity_body;


    float roll_rate_error = desired_roll_rate_rps - current_rates_rps.y();
    float pitch_rate_error = desired_pitch_rate_rps - current_rates_rps.x();
    float yaw_rate_error = desired_yaw_rate_rps - current_rates_rps.z();


    float roll_cmd = m_roll_rate_pid.calculate(roll_rate_error, dt);
    float pitch_cmd = m_pitch_rate_pid.calculate(pitch_rate_error, dt);
    float yaw_cmd = m_yaw_rate_pid.calculate(yaw_rate_error, dt);


    mix_motor_commands(throttle_norm, roll_cmd, pitch_cmd, yaw_cmd);
}

/**
 * @brief Maps a raw CRSF channel value [172, 1811] to a normalized float [-1.0, 1.0].
 * This is a crucial step for interpreting pilot commands.
 */
float AutoLevelController::map_rc_to_norm(uint16_t channel_value) {
    const float CRSF_RANGE = static_cast<float>(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN);
    const float CRSF_MID = CRSF_CHANNEL_MIN + CRSF_RANGE / 2.0f;

    float clamped_value = std::max((float)CRSF_CHANNEL_MIN, std::min((float)channel_value, (float)CRSF_CHANNEL_MAX));

    return (clamped_value - CRSF_MID) / (CRSF_RANGE / 2.0f);
}

void AutoLevelController::mix_motor_commands(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd) {
    MotorCommands motor_commands = {};
    motor_commands.Timestamp = m_data_manager.getCurrentTimeUs();
    motor_commands.is_throttle_command = true;

    // --- Standard 'X' mixer configuration (like Betaflight) ---
    // Motor numbers are: 0BR, 1FR, 2BL, 3FL
    float motor_fr = throttle*(1 - roll_cmd + pitch_cmd - yaw_cmd);
    float motor_bl = throttle*(1 + roll_cmd - pitch_cmd - yaw_cmd);
    float motor_fl = throttle*(1 + roll_cmd + pitch_cmd + yaw_cmd);
    float motor_br = throttle*(1 - roll_cmd - pitch_cmd + yaw_cmd);


    const float max_throttle = 1999.0f;

    // If throttle is very low, just send motor stop command.
    // Otherwise, scale and clamp the motor values.
    motor_commands.throttle[0] = static_cast<uint16_t>(std::max(0.0f, std::min(max_throttle, motor_br * max_throttle)));
    motor_commands.throttle[1] = static_cast<uint16_t>(std::max(0.0f, std::min(max_throttle, motor_fr * max_throttle)));
    motor_commands.throttle[2] = static_cast<uint16_t>(std::max(0.0f, std::min(max_throttle, motor_bl * max_throttle)));
    motor_commands.throttle[3] = static_cast<uint16_t>(std::max(0.0f, std::min(max_throttle, motor_fl * max_throttle)));

    m_data_manager.post(motor_commands);
}