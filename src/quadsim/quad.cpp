#include "quad.h"
#include <cmath>

void Quadcopter::simulateQuad(float dT, std::array<float, 4>& forces, std::array<float, 4>& torques){
    calculateMotorResponse(dT);
    calculateRotorForces(forces, torques);
}

void Quadcopter::calculateMotorResponse(float dT){
   for (int i = 0; i < 4; i++) {
        float throttle_percentage = state_store->motor_commands[i] / 2047.0f;
        float target_rpm = throttle_percentage * maxRPM;
        float alpha = dT/(motorTimeConstant+dT);
        // Correct low-pass filter implementation: y_new = (1-a)*y_old + a*x_new
        state_store->motor_rpms[i] = (1-alpha) * state_store->motor_rpms[i] + alpha * target_rpm;
   } 
}

void Quadcopter::calculateRotorForces(std::array<float, 4>& forces, std::array<float, 4>& torques){
    for (int i = 0; i < 4; i++){
        float rads = state_store->motor_rpms[i] * 2 * M_PI / 60.0f;
        forces[i] = thrustCoefficient*rads*rads;
        torques[i] = momentCoefficient*rads*rads;
    }
}