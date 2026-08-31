#include "ExcitationGenerator.h"

void ExcitationGenerator::startTimer()
{
    timer_start_timestamp = getCurrentTimeUs();
    motor_start_timestamp = getCurrentTimeUs();
    current_motor = -1;
    is_complete = false;
}


Eigen::Vector4f ExcitationGenerator::getCommand()
{

    Eigen::Vector4f cmd = Eigen::Vector4f::Zero();

    if (is_complete)
        return cmd;
    
    state_timer= (getCurrentTimeUs() - motor_start_timestamp) * 1e-6f;

    if(current_motor==-1){
        if(state_timer<=launch_dur){
            cmd<< .7,.7,.7,.7;
            return cmd;
        } else {
            advanceMotor();
            state_timer= (getCurrentTimeUs() - motor_start_timestamp) * 1e-6f;
        }
        
    }
    cmd = Eigen::Vector4f::Zero();
    // 2. Generate Waveform
    float val = 0.0f;
    float total_dur = zero_dur + step_dur + ramp_dur + wait_dur;

    if (state_timer < zero_dur)
    {
        val = 0.0f;
    }
    else if (state_timer < (zero_dur + step_dur))
    {
        val = 0.4f; // Step level from Python
    }
    else if (state_timer <= (zero_dur + step_dur + ramp_dur))
    {
        // Ramp down logic
        float ramp_progress = state_timer - (zero_dur + step_dur);
        float remaining = ramp_dur - ramp_progress;
        val = 0.7f * (remaining / ramp_dur); // Ramp max .7
    }
    else if (state_timer < total_dur)
    {
        val = 0.0f; // Wait time
    }
    else
    {
        advanceMotor();
        // check if that was the last motor
        if (is_complete) return cmd;
    }

    cmd[current_motor] = val;
    return cmd;
}

void ExcitationGenerator::advanceMotor()
{
    current_motor++;
    motor_start_timestamp = getCurrentTimeUs();
    if (current_motor >= 4)
    {
        is_complete = true;
    }
}