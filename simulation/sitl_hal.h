#ifndef SITL_HAL_H
#define SITL_HAL_H

#include "../src/abstraction_layer/hal_interface.h"

class SITL_HAL : public IHAL {
public:
    SITL_HAL(const UserInput& userInput) : userInput(userInput) {}

    SensorData read_sensors() override {
        // In a real SITL, this would read from a simulated sensor, not implemented yet
        return SensorData();
    }

    UserInput read_user_input() override {
        return userInput;
    }

private:
    UserInput userInput;
};

#endif // SITL_HAL_H
