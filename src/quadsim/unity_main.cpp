#include "../common/types.h"
#include "../control_logic/flight_controller.h"
#include <algorithm> // For std::copy
#include "../abstraction_layer/unity/unity_hal.h"
#include "../abstraction_layer/unity/state_store.h"
#include "quad.h"

static StateStore* state_store = nullptr;
static UnityHAL* hal = nullptr;
static FlightController* controller = nullptr;
static Quadcopter* quad = nullptr;

extern "C" {
    __declspec(dllexport) void InitializeSimulation() {
        if (!state_store) {
            state_store = new StateStore();
            hal = new UnityHAL(state_store);
            controller = new FlightController(hal);
            quad = new Quadcopter(state_store);
        }
    }
     __declspec(dllexport) void UpdateSensorData(SensorData sensor_data) {
        state_store->gyro_data = sensor_data.gyroscope;
        state_store->accelerometer_data = sensor_data.accelerometer;

    }
    __declspec(dllexport) void UpdateBodyState(RigidbodyState body_state) {
        state_store->ground_truth = body_state;
    }
    
    __declspec(dllexport) void UpdateUserInput(UserInput input) {
        state_store->user_input = input;
    }

    __declspec(dllexport) void RunSimulationStep( float dt, float* out_forces, float* out_torques) {

        // Create local std::arrays to be used by the internal C++ functions.
        std::array<float, 4> forces;
        std::array<float, 4> torques;

        controller->runFlightLoop();
        quad->simulateQuad(dt,forces,torques);
        
        // Copy the results from the std::arrays to the output pointers.
        // This is a safer and more idiomatic way to copy the contents.
        std::copy(forces.begin(), forces.end(), out_forces);
        std::copy(torques.begin(), torques.end(), out_torques);
    }

    __declspec(dllexport) void TeardownSimulation() {
        delete controller;
        controller = nullptr;
        delete hal;
        hal = nullptr;
        delete quad;
        quad = nullptr;
        delete state_store;
        state_store = nullptr;
    }
}