#include "DataManager.h"
#include "GazeboInterface.h"
#include "ControllerInterface.h"
#include "BodyRateController.h"
#include "TruthFilter.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal> // For signal handling

// A simple time source function
uint64_t getTimeMicroseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()
    ).count();
}

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

    // 1. Initialize all components
    DataManager dataManager(getTimeMicroseconds);
    GazeboInterface gazeboInterface(dataManager);
    ControllerInterface controllerInterface(dataManager);
    TruthFilter truthFilter(dataManager);
    BodyRateController bodyRateController(dataManager);
    // 2. Start all threaded components
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
        truthFilter.run();
        bodyRateController.run();

        // Control the loop rate (e.g., 200 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Exiting main loop. Application will now close." << std::endl;

    return 0;
}
