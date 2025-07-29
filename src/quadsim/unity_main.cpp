#include "../src/common/types.h"
#include "../src/control_logic/flight_controller.h"
#include "unity_hal.h"
#include "state_store.h"
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

    __declspec(dllexport) void RunSimulationStep( UserInput input, float dt, std::array<float, 4>& forces, std::array<float, 4>& torques) {


        controller->runFlightLoop();
        quad->simulateQuad(dt,forces,torques);
        
        // The physics model can now use the state store to simulate battery drain
        // ForcesAndTorques forces_and_torques = physics->run_motor_simulation(motor_commands, dt, state_store);
        
    }

    __declspec(dllexport) void TeardownSimulation() {
        delete controller;
        controller = nullptr;
        delete hal;
        hal = nullptr;
        delete state_store;
        state_store = nullptr;
        delete quad;
        quad = nullptr;
    }
}
