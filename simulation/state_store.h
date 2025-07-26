#ifndef STATE_STORE_H
#define STATE_STORE_H

#include "../src/common/types.h"

// A simple class to hold the simulation's ground truth state.
class StateStore {
public:
    StateStore() : battery_voltage(12.6f) {}

    RigidbodyState ground_truth;
    float battery_voltage;
};

#endif // STATE_STORE_H
