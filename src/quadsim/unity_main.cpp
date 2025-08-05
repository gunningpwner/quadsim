// unity_main.cpp
#include "DataManager.h"
#include "../control_logic/flight_controller.h"
#include "MotorModel.h"
#include "quad.h" // Your quad physics model
#include <vector>
#include <cmath> // For std::isfinite

// --- Global Objects ---
// The HAL object is removed. The C functions below now fulfill the HAL's role in the SITL.
static DataManager* data_manager = nullptr;
static FlightController* controller = nullptr;
static std::vector<MotorModel> motor_models;

extern "C" {
    __declspec(dllexport) void InitializeSimulation() {
        if (!data_manager) {
            data_manager = new DataManager();
            controller = new FlightController(data_manager); 
            
            // Create 4 motor models for our simulation
            motor_models.assign(4, MotorModel());
        }
    }

    __declspec(dllexport) void TeardownSimulation() {
        delete controller; controller = nullptr;
        delete quad; quad = nullptr;
        delete data_manager; data_manager = nullptr;
        motor_models.clear();
    }

    // --- Data Input Functions (The SITL "Drivers") ---
    // These C-exported functions are the entry points from Unity.
    // They now post data directly to the DataManager.

    __declspec(dllexport) void AddAccelData(AccelData accel_data) {
        if (data_manager) data_manager->postRawAccel(accel_data);
    }
    __declspec(dllexport) void AddGyroData(GyroData gyro_data) {
        if (data_manager) data_manager->postRawGyro(gyro_data);
    }
    __declspec(dllexport) void AddMagData(MagData mag_data) {
        if (data_manager) data_manager->postRawMag(mag_data);
    }
    __declspec(dllexport) void AddGPSData(GPSData gps_data) {
        if (data_manager) data_manager->postRawGPS(gps_data);
    }


    // --- Main Simulation Loop (The "Orchestrator") ---

 __declspec(dllexport) void RunSimulationStep(float dt, float* out_forces, float* out_torques) {
        if (!controller || !data_manager || motor_models.empty()) return;

        // 1. FLIGHT LOGIC: Run the flight control loop. It reads sensor data
        //    from the DataManager and posts motor commands back to it.
        controller->runFlightLoop();

        // 2. MOTOR SIMULATION: Read the commands from the DataManager.
        std::array<int, 4> motor_commands = data_manager->getMotorCommands();
        MotorRPMs current_rpms;
        current_rpms.Timestamp = 0; // Or get a proper timestamp

        // 3. FORCE/TORQUE CALCULATION: Update each motor model and get its
        //    individual thrust and torque. This replaces the Quadcopter class.
        for (int i = 0; i < 4; ++i) {
            motor_models[i].update(motor_commands[i], dt);
            
            // Get the calculated values from the motor model itself
            out_forces[i] = motor_models[i].getThrust();
            out_torques[i] = motor_models[i].getTorque();
            current_rpms.rpms[i] = motor_models[i].getRPM();
        }

        // 4. TELEMETRY: Post the simulated RPMs back to the DataManager.
        data_manager->postMotorRPMs(current_rpms);

        // 5. SAFETY CHECK: Ensure outputs to Unity are valid numbers.
        for (int i = 0; i < 4; i++) {
            if (!std::isfinite(out_forces[i])) out_forces[i] = 0.0f;
            if (!std::isfinite(out_torques[i])) out_torques[i] = 0.0f;
        }
    }
}
