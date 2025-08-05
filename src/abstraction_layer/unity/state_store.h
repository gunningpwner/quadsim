#ifndef STATE_STORE_H
#define STATE_STORE_H
#include <array>
#include "../../common/types.h"

// A simple class to hold the simulation's ground truth state.
class StateStore {
public:
    StateStore() : battery_voltage(12.6f) {}
    Vector3 gyro_data;
    Vector3 accelerometer_data;
    Vector3 magnetometer_data;
    RigidbodyState ground_truth;
    UserInput user_input;
    float battery_voltage;
    std::array<float, 4> motor_commands;
    std::array<float, 4> motor_rpms;
    std::array<GPSData, 2> gps_data{};
};

#endif // STATE_STORE_H