#include "flight_controller.h"

FlightController::FlightController(IHAL* hal) : hal(hal) {}

MotorCommands FlightController::calculate_motor_commands(UserInput input) {
    SensorData sensor_data = hal->read_sensors();
    // TODO: Implement PID controller logic here
    return MotorCommands();
}
