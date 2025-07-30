#include "../common/types.h"
#include <cmath> // For std::isfinite
#include "../control_logic/flight_controller.h"

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
        std::array<float, 4> forces{}; // Use brace-initialization to zero-initialize
        std::array<float, 4> torques{}; // Use brace-initialization to zero-initialize

        controller->runFlightLoop();
        quad->simulateQuad(dt,forces,torques);
        
        // IMPORTANT: Validate the calculated forces and torques before sending them to Unity.
        // This prevents crashes if the simulation produces NaN (Not a Number) or infinite values.
        for (int i = 0; i < 4; i++) {
            // If the value is finite (not NaN/inf), use it. Otherwise, use 0.0f as a safe fallback.
            out_forces[i] = std::isfinite(forces[i]) ? forces[i] : 0.0f;
            out_torques[i] = std::isfinite(torques[i]) ? torques[i] : 0.0f;
        }
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