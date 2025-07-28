#include "../src/common/types.h"
#include "../src/control_logic/flight_controller.h"
#include "unity_hal.h"
#include "state_store.h"

static StateStore* state_store = nullptr;
static UnityHAL* hal = nullptr;
static FlightController* controller = nullptr;


extern "C" {
    __declspec(dllexport) void InitializeSimulation() {
        if (!state_store) {
            state_store = new StateStore();
            hal = new UnityHAL(state_store);
            controller = new FlightController(hal);
        }
    }
     __declspec(dllexport) void UpdateSensorData(SensorData sensor_data) {
        state_store->sensor_data = sensor_data;

    }
    __declspec(dllexport) void UpdateBodyState(RigidbodyState body_state) {
        state_store->ground_truth = body_state;
    }

    __declspec(dllexport) ForcesAndTorques RunSimulationStep( UserInput input, float dt) {
        if (!state_store) {
            // Or handle error appropriately
            return ForcesAndTorques(); 
        }

        // The flight controller will now get the updated state when it calls read_sensors()
        MotorCommands motor_commands = controller->calculate_motor_commands(input);
        
        // The physics model can now use the state store to simulate battery drain
        // ForcesAndTorques forces_and_torques = physics->run_motor_simulation(motor_commands, dt, state_store);
        
        return forces_and_torques;
    }

    __declspec(dllexport) void TeardownSimulation() {
        // delete physics;
        physics = nullptr;
        delete controller;
        controller = nullptr;
        delete hal;
        hal = nullptr;
        delete state_store;
        state_store = nullptr;
    }
}
