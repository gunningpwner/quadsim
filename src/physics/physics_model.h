#ifndef PHYSICS_MODEL_H
#define PHYSICS_MODEL_H

#include "../common/types.h"

class PhysicsModel {
public:
    PhysicsModel();
    ForcesAndTorques run_motor_simulation(MotorCommands commands);
};

#endif // PHYSICS_MODEL_H
