#include "GazeboInterface.h"
#include "ControllerInterface.h"
#include "MCE.h"
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

    // 1. Initialize the Monolithic Control Entity (MCE), which manages the DataManager.
    MonolithicControlEntity mce;
    DataManager& dataManager = mce.getDataManager();

    // 2. Create Gazebo and Controller interfaces, passing them the DataManager.
    GazeboInterface gazeboInterface(dataManager);
    ControllerInterface controllerInterface(dataManager);

    // 3. Create the time source from Gazebo and initialize the MCE with it.
    TimeSource simTimeSource = [&gazeboInterface]() { return gazeboInterface.getSimTimeUs(); };
    mce.initialize(simTimeSource);

    // 4. Start all threaded interface components.
    if (controllerInterface.initialize()) {
        controllerInterface.startPolling();
    } else {
        std::cerr << "Could not initialize controller. Running without joystick input." << std::endl;
    }
    gazeboInterface.startSubscribers();

    std::cout << "Main loop running. Press Ctrl+C to exit." << std::endl;

    
    while (g_run_application.load()) {
        // Run one iteration of the Monolithic Control Entity's logic.
        mce.run();
        // Control the loop rate (e.g., 200 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Exiting main loop. Application will now close." << std::endl;

    return 0;
}
