#pragma once

#include "Controller.h"
#include "DataManager.h"
#include "Consumer.h"
#include "PID.h"
#include <Eigen/Dense>

/**
 * @class AutoLevelController
 * @brief Implements an angle-stabilizing (auto-level) flight controller.
 *
 * This controller represents the outer loop in a cascaded control system.
 * It translates user stick inputs (interpreted as desired roll and pitch angles)
 * into desired angular rates. These rates are then posted to the DataManager
 * for an inner-loop rate controller to handle.
 */
class AutoLevelController : public Controller {
public:
    explicit AutoLevelController(DataManager& data_manager);

    void run() override;

private:
    /**
     * @brief Maps a raw CRSF channel value [172, 1811] to a normalized float [-1.0, 1.0].
     * @param channel_value The raw CRSF value.
     * @return The normalized value.
     */
    float map_rc_to_norm(uint16_t channel_value);

    // --- Data Consumers ---
    Consumer<RCChannelsData, RC_CHANNELS_BUFFER_SIZE> m_rc_channels_consumer;
    Consumer<StateData, STATE_BUFFER_SIZE> m_state_consumer;

    // --- PID Controllers ---
    // Angle PIDs (Outer Loop): Calculate desired angular rate from angle error.
    PID m_roll_angle_pid;
    PID m_pitch_angle_pid;

    // Rate PIDs (Inner Loop): Calculate motor command adjustments from angular rate error.
    PID m_roll_rate_pid;
    PID m_pitch_rate_pid;
    PID m_yaw_rate_pid;

    // --- Controller Parameters ---
    const float m_max_angle_deg = 30.0f; // Max tilt angle in degrees
    const float m_max_yaw_rate_dps = 180.0f; // Max yaw rate in degrees per second

    /**
     * @brief Mixes throttle and PID outputs into individual motor commands.
     */
    void mix_motor_commands(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd);

    // --- Timekeeping ---
    uint64_t m_last_run_time_us = 0;

    // --- Cached Data ---
    // It's good practice to cache the latest data to ensure consistency within a run loop.
    RCChannelsData m_latest_rc_data;
    StateData m_latest_state_data;
};