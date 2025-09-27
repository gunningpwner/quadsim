#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

// Include the MAVLink C headers
#include "common/mavlink.h"

// Serial port library (a simple header-only library for this example)
#include "serial/serial.h"

// Global flag to signal exit
std::atomic<bool> g_run_application(true);

// Signal handler for Ctrl+C
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down." << std::endl;
    g_run_application = false;
}

int main(int argc, char** argv) {
    // --- Argument Parsing ---
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <baud_rate>" << std::endl;
        std::cerr << "Example: " << argv[0] << " COM4 9600" << std::endl;
        return 1;
    }
    std::string port_name = argv[1];
    int baud_rate = std::stoi(argv[2]);

    // Register signal handler for clean shutdown on Ctrl+C
    signal(SIGINT, signalHandler);

    // --- Serial Port Setup ---
    serial::Serial serial_port;
    try {
        serial_port.setPort(port_name);
        serial_port.setBaudrate(baud_rate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    } catch (const serial::IOException& e) {
        std::cerr << "Failed to open serial port " << port_name << ": " << e.what() << std::endl;
        return 1;
    }

    if (serial_port.isOpen()) {
        std::cout << "Successfully opened serial port " << port_name << " at " << baud_rate << " baud." << std::endl;
    } else {
        std::cerr << "Could not open serial port." << std::endl;
        return 1;
    }

    // --- Gazebo Transport Setup ---
    gz::transport::Node node;
    const std::string poseTopic = "/model/quadcopter/pose";
    auto pose_pub = node.Advertise<gz::msgs::Pose>(poseTopic);
    if (!pose_pub) {
        std::cerr << "Error advertising topic [" << poseTopic << "]" << std::endl;
        return 1;
    }
    std::cout << "Publishing pose updates to Gazebo topic: " << poseTopic << std::endl;

    // --- MAVLink Parsing Loop ---
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buffer[1024];
    uint64_t attitude_msg_count = 0; // Counter for received attitude messages

    std::cout << "Listening for MAVLink messages. Press Ctrl+C to exit." << std::endl;

    while (g_run_application.load()) {
        size_t bytes_read = serial_port.read(buffer, sizeof(buffer));
        for (size_t i = 0; i < bytes_read; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                // A complete MAVLink message was received.
                if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
                    attitude_msg_count++;
                    mavlink_attitude_quaternion_t attitude;
                    mavlink_msg_attitude_quaternion_decode(&msg, &attitude);

                    // Create Gazebo Pose message
                    gz::msgs::Pose pose_msg;
                    pose_msg.mutable_orientation()->set_w(attitude.q1);
                    pose_msg.mutable_orientation()->set_x(attitude.q2);
                    pose_msg.mutable_orientation()->set_y(attitude.q3);
                    pose_msg.mutable_orientation()->set_z(attitude.q4);

                    // We only care about orientation for this visualization
                    // Set position to a fixed point so it doesn't fall through the world
                    pose_msg.mutable_position()->set_x(0);
                    pose_msg.mutable_position()->set_y(0);
                    pose_msg.mutable_position()->set_z(0.5);

                    // Publish to Gazebo
                    pose_pub.Publish(pose_msg);

                    // Print a debug message every 20th attitude packet to avoid spam
                    if (attitude_msg_count % 20 == 0) {
                        std::cout << "Received ATTITUDE_QUATERNION, publishing to Gazebo. "
                                  << "w=" << attitude.q1 << ", x=" << attitude.q2
                                  << std::endl;
                    }
                }
            }
        }
    }

    serial_port.close();
    std::cout << "Serial port closed. Exiting." << std::endl;
    return 0;
}