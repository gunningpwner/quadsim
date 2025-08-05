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
        state_store->magnetometer_data = sensor_data.magnetometer;

    }
    
    __declspec(dllexport) void UpdateUserInput(UserInput input) {
        state_store->user_input = input;
    }

    __declspec(dllexport) void AddGPSData(GPSData gps_data) {
        GPSData temp = GPSData();
        temp.Timestamp = gps_data.Timestamp;
        temp.FixStatus = gps_data.FixStatus;
        temp.Latitude = gps_data.Latitude;
        temp.Longitude = gps_data.Longitude;
        temp.Altitude = gps_data.Altitude;
        temp.GroundSpeed = gps_data.GroundSpeed;
        temp.GroundVelocity = gps_data.GroundVelocity;
        temp.Satellites = gps_data.Satellites;

        state_store->gps_data[1] = state_store->gps_data[0];
        state_store->gps_data[0] = temp;
        hal->newGPSData=true;
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

    __declspec(dllexport) void OutputAnimationElements(float* motor_rpms) {
        for (int i = 0; i < 4; i++) {

            motor_rpms[i] = state_store->motor_rpms[i];
        }
    }

    __declspec(dllexport) void OutputDebugData(RigidbodyState* body_state) {
        body_state->position = state_store->ground_truth.position;
        body_state->rotation = state_store->ground_truth.rotation;
        body_state->velocity = state_store->ground_truth.velocity;
        body_state->angular_velocity = state_store->ground_truth.angular_velocity;
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