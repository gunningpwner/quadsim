#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "timing.h"
class ExcitationGenerator
{
public:
    ExcitationGenerator() : current_motor(0),
                            state_timer(0.0f),
                            is_complete(false), 
                            zero_dur(0.030f), 
                            step_dur(0.050f), 
                            ramp_dur(0.150f), 
                            wait_dur(0.100f) {}

    void startTimer();

    // Returns true if excitation is finished
    bool isComplete() const { return is_complete; }
    int getCurrentMotor() const { return current_motor; }
    void advanceMotor();
    // Generates the motor command for the current timestep
    Eigen::Vector4f getCommand();

private:
    

    int current_motor;
    float state_timer;
    bool is_complete;

    // Timing parameters
    float zero_dur, step_dur, ramp_dur, wait_dur;
    uint64_t timer_start_timestamp;
    uint64_t motor_start_timestamp;

};