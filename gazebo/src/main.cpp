#include "DataManager.h"
#include "GazeboInterface.h"
#include <iostream>
#include <chrono>
#include <thread>

// A simple time source function
uint64_t getTimeMicroseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()
    ).count();
}

int main(int argc, char** argv) {
    std::cout << "Starting Gazebo data interface..." << std::endl;

    // 1. Initialize the DataManager
    DataManager dataManager(getTimeMicroseconds);

    // 2. Initialize the Gazebo Interface
    GazeboInterface gazeboInterface(dataManager);

    // 3. Start the subscribers and the publisher loop
    gazeboInterface.startSubscribers();
    gazeboInterface.startPublisherLoop();

    std::cout << "Interface running. Sending takeoff command." << std::endl;

    // 4. Post a command to the DataManager to make the quadcopter take off.
    // The publisher loop in GazeboInterface will pick this up and send it.
    // These values (rad/s) might need tuning for stable flight.
    MotorCommands takeoff_command;
    takeoff_command.rpms = {500.0f, 500.0f, 500.0f, 500.0f};
    dataManager.post(takeoff_command);

    // 5. Wait for shutdown
    gz::transport::waitForShutdown();

    return 0;
}
