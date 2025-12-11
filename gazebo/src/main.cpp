#include "TestHarness.h"
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

    TestHarness harness;

    std::cout << "Main loop running. Press Ctrl+C to exit." << std::endl;

    
    harness.run();

    std::cout << "Exiting main loop. Application will now close." << std::endl;

    return 0;
}
