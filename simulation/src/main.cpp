#include "TestHarness.h"
#include <webots/Robot.hpp>
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "--- WEBOTS FCS CONTROLLER STARTING ---" << std::endl;

    // 1. Create the Robot (This connects to the simulator)
    webots::Robot* robot = new webots::Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    // 2. Init Harness
    TestHarness harness(robot);

    std::cout << "Connected. Starting control loop..." << std::endl;

    // 3. The Webots Loop
    // robot->step() advances physics by 'timeStep' ms.
    // It returns -1 if Webots is closed or the run is stopped.
    while (robot->step(timeStep) != -1) {
        
        // This runs EXACTLY once per physics tick.
        // No threading or sleep needed.
        harness.update(timeStep);
    }

    std::cout << "Webots simulation ended." << std::endl;
    delete robot;
    return 0;
}