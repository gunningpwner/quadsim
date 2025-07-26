#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "../abstraction_layer/hal_interface.h"
#include "../common/types.h"

class FlightController {
public:
    FlightController(IHAL* hal);
    MotorCommands calculate_motor_commands(UserInput input);

private:
    IHAL* hal;
};

#endif // FLIGHT_CONTROLLER_H
