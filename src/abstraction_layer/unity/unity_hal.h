#ifndef UNITY_HAL_H
#define UNITY_HAL_H

#include "../hal_interface.h"
#include "state_store.h"

class UnityHAL : public IHAL {
public:
    UnityHAL(StateStore* state_store) : state_store(state_store) {}

    Vector3 read_gyros() override {
        return state_store->gyro_data;
    }

    Vector3 read_accelerometer() override {
        return state_store->accelerometer_data;
    }

    Vector3 read_magnetometer() override {
        return state_store->magnetometer_data;
    }

    std::array<float, 4> read_motor_rpms() override {
        return state_store->motor_rpms;
    }

    void write_motor_commands(const std::array<int, 4>& motor_commands) override {
        // This assumes a simple conversion from int to float for the motor commands.
        for (std::size_t i = 0; i < motor_commands.size(); ++i) {
            state_store->motor_commands[i] = static_cast<float>(motor_commands[i]);
        }
    }

    UserInput read_user_input() override {
        return state_store->user_input;
    }
    void output_debug_data(Vector3 enuVelocity, Vector3 bodyOrientation) override {
        state_store->ground_truth.velocity = enuVelocity;
        state_store->ground_truth.rotation = bodyOrientation;
    }

    virtual std::array<GPSData, 2> read_gps() override {
        return state_store->gps_data;
    }

    bool newGPSData = false;

private:
    StateStore* state_store;
};

#endif // UNITY_HAL_H
