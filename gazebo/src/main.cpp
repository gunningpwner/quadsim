#include "DataManager.h"
#include "GazeboInterface.h"
#include "ControllerInterface.h"
#include "BodyRateController.h"
#include "EKF.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal> // For signal handling

// Global flag to signal exit
std::atomic<bool> g_run_application(true);

// Signal handler for Ctrl+C
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down." << std::endl;
    g_run_application = false;
}

int main(int argc, char** argv) {
    // Register signal handler for clean shutdown on Ctrl+C
    signal(SIGINT, signalHandler);

    std::cout << "Starting flight software..." << std::endl;

    // 1. Initialize components, resolving the time source dependency.
    // a. Create DataManager with a placeholder time source.
    DataManager dataManager([](){ return 0; });

    // b. Create GazeboInterface, which needs the DataManager.
    GazeboInterface gazeboInterface(dataManager);
    ControllerInterface controllerInterface(dataManager);

    // c. Now, create the real time source from Gazebo and update the DataManager.
    TimeSource simTimeSource = [&gazeboInterface]() { return gazeboInterface.getSimTimeUs(); };
    dataManager.setTimeSource(simTimeSource);

    EKF filter(dataManager);
    BodyRateController bodyRateController(dataManager);

    // 2. Start all threaded interface components
    if (controllerInterface.initialize()) {
        controllerInterface.startPolling();
    } else {
        std::cerr << "Could not initialize controller. Running without joystick input." << std::endl;
    }
    gazeboInterface.startSubscribers();
    gazeboInterface.startPublisherLoop();

    std::cout << "Main loop running. Press Ctrl+C to exit." << std::endl;

    
    while (g_run_application.load()) {
        // Run one iteration of the flight controller logic
        filter.run();
        bodyRateController.run();

        // Control the loop rate (e.g., 200 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Exiting main loop. Application will now close." << std::endl;

    return 0;
}
