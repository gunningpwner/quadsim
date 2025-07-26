#ifndef UNITY_HAL_H
#define UNITY_HAL_H

#include "../src/abstraction_layer/hal_interface.h"
#include "state_store.h"

class UnityHAL : public IHAL {
public:
    UnityHAL(StateStore* state_store) : state_store(state_store) {}

    SensorData read_sensors() override {
        SensorData sensors;
        sensors.gyroscope = state_store->ground_truth.angular_velocity;
        sensors.accelerometer = {0, 0, 0}; // Placeholder
        return sensors;
    }

    UserInput read_user_input() override {
        // In a real simulation, this would read from a joystick or other input device.
        // For now, we'll just return a default value.
        return UserInput();
    }

private:
    StateStore* state_store;
};

#endif // UNITY_HAL_H
