#include "../common/types.h"
#include "state_store.h"

class Quadcopter {
    public: 
        Quadcopter(StateStore* state_store);
        void simulateQuad(float dT,std::array<float, 4>& forces, std::array<float, 4>& torques);
        void calculateMotorResponse(float dT);
        void calculateRotorForces(std::array<float, 4>& forces, std::array<float, 4>& torques);

        
    private:
        float motorTimeConstant=.1;
        float voltage = 22.2;
        float motorKVC = 1750;
        float maxRPM = voltage*motorKVC;
        float thrustCoefficient =1.7e-5;
        float dragCoefficient = 7.5e-7;
        float momentCoefficient = 3e-6;
        StateStore* state_store;

};