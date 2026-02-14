#pragma once
#include <Eigen/Dense>
#include <cmath>

class ExcitationGenerator {
public:
    ExcitationGenerator() {
        reset();
    }

    void reset() {
        current_motor = 0;
        state_timer = 0.0f;
        is_complete = false;
        // Constants from Python/Paper
        zero_dur = 0.030f;
        step_dur = 0.050f;
        ramp_dur = 0.150f;
        wait_dur = 0.100f;
    }

    // Returns true if excitation is finished
    bool isComplete() const { return is_complete; }

    // Generates the motor command for the current timestep
    Eigen::Vector4f getCommand(float dt, const Eigen::Vector3f& gyro_meas) {
        if (is_complete) return Eigen::Vector4f::Zero();

        state_timer += dt;
        Eigen::Vector4f cmd = Eigen::Vector4f::Zero();

        // 1. Safety Check (Paper Sec II.B.d): 
        // Abort this motor if gyro exceeds safety margin
        // Simple check: if any axis > 1500 deg/s (approx 26 rad/s)
        float max_rate = gyro_meas.cwiseAbs().maxCoeff();
        if (max_rate > 26.0f) { 
             advanceMotor(); 
             return Eigen::Vector4f::Zero();
        }

        // 2. Generate Waveform
        float val = 0.0f;
        float total_dur = zero_dur + step_dur + ramp_dur + wait_dur;

        if (state_timer < zero_dur) {
            val = 0.0f;
        } else if (state_timer < (zero_dur + step_dur)) {
            val = 0.4f; // Step level from Python
        } else if (state_timer < (zero_dur + step_dur + ramp_dur)) {
            // Ramp down logic
            float ramp_progress = state_timer - (zero_dur + step_dur);
            float remaining = ramp_dur - ramp_progress;
            val = 0.7f * (remaining / ramp_dur); // Ramp max .7
        } else if (state_timer < total_dur) {
            val = 0.0f; // Wait time
        } else {
            advanceMotor();
        }

        cmd[current_motor] = val;
        return cmd;
    }

private:
    void advanceMotor() {
        current_motor++;
        state_timer = 0.0f;
        if (current_motor >= 4) {
            is_complete = true;
        }
    }

    int current_motor;
    float state_timer;
    bool is_complete;
    
    // Timing parameters
    float zero_dur, step_dur, ramp_dur, wait_dur;
};