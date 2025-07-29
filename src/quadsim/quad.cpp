#include "quad.h"

ForcesAndTorques Quadcopter::simulateQuad(float dT, std::array<int, 4>& forces, std::array<int, 4>& torques){
    calculateMotorResponse(dT);
    calculateForces(std::array<int, 4>& forces, std::array<int, 4>& torques);
}

void Quadcopter::calculateMotorResponse(float dT){
   for (int i = 0; i < 4; i++) {
        float throttle_percentage = state_store->motor_commands[i]/2047;
        float target_rpm = throttle_percentage * maxRPM;
        float alpha = dT/(motorTimeConstant+dT);
        state_store->motor_rpms[i] = alpha * state_store->motor_rpms[i] + (1-alpha) * target_rpm;
   } 
}

ForcesAndTorques Quadcopter::calculateRotorForces(std::array<int, 4>& forces, std::array<int, 4>& torques){
    for (int i = 0; i < 4; i++){
        float rads = state_store->motor_rpms[i]*2*3.14159/60;
        forces[i] = thrustCoefficient*rads*rads;
        torques[i] = momentCoefficient*rads;
    
    }

}