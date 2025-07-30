#include "flight_controller.h"

FlightController::FlightController(IHAL* hal) : hal(hal) {}

void FlightController::runFlightLoop(){
    hal->write_motor_commands({2047/2,2047/2,2047/2,2047/2});
    complementaryFilter();
}

void FlightController::complementaryFilter(){

    
}
