#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "../abstraction_layer/hal_interface.h"
#include "../common/types.h"

class FlightController {
public:
    FlightController(IHAL* hal);
    void runFlightLoop();
    void garboFilter();
    Vector3 processGPSData();

private:
    IHAL* hal;

    Vector3 bodyOrientation;
    Vector3 enuVelocity;


    // PID gains for rate control
    float Kp_roll_rate, Ki_roll_rate, Kd_roll_rate;
    float Kp_pitch_rate, Ki_pitch_rate, Kd_pitch_rate;
    float Kp_yaw_rate, Ki_yaw_rate, Kd_yaw_rate;

    // PID state for rate control
    float roll_rate_error_sum, pitch_rate_error_sum, yaw_rate_error_sum;
    float last_roll_rate_error, last_pitch_rate_error, last_yaw_rate_error;
};

#endif // FLIGHT_CONTROLLER_H
