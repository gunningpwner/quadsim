#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "../abstraction_layer/hal_interface.h"
#include "../common/types.h"

class FlightController {
public:
    FlightController(IHAL* hal);
    void runFlightLoop();
    void complementaryFilter();
    

private:
    IHAL* hal;

    Vector3 bodyOrientation;

};

#endif // FLIGHT_CONTROLLER_H
