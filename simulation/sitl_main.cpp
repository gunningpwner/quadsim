#include "../src/common/types.h"
#include "../src/control_logic/flight_controller.h"
#include "../src/physics/physics_model.h"
#include "sitl_hal.h"

extern "C" {
    __declspec(dllexport) ForcesAndTorques RunSimulationStep(RigidbodyState state, UserInput input) {
        SITL_HAL hal(input);
        FlightController controller(&hal);
        PhysicsModel physics;

        MotorCommands motor_commands = controller.calculate_motor_commands(input);
        ForcesAndTorques forces_and_torques = physics.run_motor_simulation(motor_commands);

        return forces_and_torques;
    }
}
