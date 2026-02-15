#pragma once
#include <Eigen/Dense>
#include <cmath>

class ExcitationGenerator {
public:
    ExcitationGenerator() {
        reset();
    }

    void reset();

    // Returns true if excitation is finished
    bool isComplete() const { return is_complete; }

    // Generates the motor command for the current timestep
    Eigen::Vector4f getCommand(float dt, const Eigen::Vector3f& gyro_meas);

private:
    void advanceMotor(); 

    int current_motor;
    float state_timer;
    bool is_complete;
    
    // Timing parameters
    float zero_dur, step_dur, ramp_dur, wait_dur;
};