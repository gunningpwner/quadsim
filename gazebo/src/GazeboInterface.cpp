#include "GazeboInterface.h"
#include <iostream>
#include <chrono>

GazeboInterface::GazeboInterface(DataManager& dataManager)
    : m_dataManager(dataManager), m_run_publisher(false) {
    // Advertise the motor command topic
    const std::string motorTopic = "/quadcopter/cmd_vel";
    m_motor_pub = m_node.Advertise<gz::msgs::Actuators>(motorTopic);
    if (!m_motor_pub) {
        std::cerr << "Error advertising topic [" << motorTopic << "]" << std::endl;
    }
}

GazeboInterface::~GazeboInterface() {
    // Safely stop the publisher thread
    if (m_run_publisher.load()) {
        m_run_publisher.store(false);
        if (m_publisherThread.joinable()) {
            m_publisherThread.join();
        }
    }
}

void GazeboInterface::startSubscribers() {
    // Note: You may need to change these topic names depending on your Gazebo world.
    const std::string imuTopic = "/imu";
    const std::string gpsTopic = "/gps"; // You may need to add a GPS sensor to your SDF
    const std::string magTopic = "/magnetometer"; // You may need to add a magnetometer to your SDF

    // Subscribe to topics
    if (!m_node.Subscribe(imuTopic, &GazeboInterface::imuCallback, this)) {
        std::cerr << "Error subscribing to topic [" << imuTopic << "]" << std::endl;
    }
    // Add subscriptions for GPS and Magnetometer if they exist in your model
}

void GazeboInterface::startPublisherLoop() {
    m_run_publisher.store(true);
    m_publisherThread = std::thread(&GazeboInterface::runPublisherLoop, this);
}

void GazeboInterface::runPublisherLoop() {
    std::vector<MotorCommands> commands;

    while (m_run_publisher.load()) {
        // Check for new motor commands from the DataManager
        if (m_dataManager.consume(commands, m_last_seen_motor_command_count)) {
            if (!commands.empty()) {
                // Use the most recent command
                const auto& latest_command = commands.back();

                // Create the Gazebo message
                gz::msgs::Actuators motor_msg;
                for (float rpm : latest_command.rpms) {
                    motor_msg.add_velocity(rpm);
                }

                // Publish the message
                m_motor_pub.Publish(motor_msg);
            }
        }

        // Run the loop at a reasonable rate (e.g., 100 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void GazeboInterface::imuCallback(const gz::msgs::IMU& msg) {
    int64_t timestamp_us = msg.header().stamp().sec() * 1000000LL + msg.header().stamp().nsec() / 1000LL;

    AccelData accelData;
    accelData.Timestamp = timestamp_us;
    accelData.Acceleration.x = msg.linear_acceleration().x();
    accelData.Acceleration.y = msg.linear_acceleration().y();
    accelData.Acceleration.z = msg.linear_acceleration().z();
    m_dataManager.post(accelData);

    GyroData gyroData;
    gyroData.Timestamp = timestamp_us;
    gyroData.AngularVelocity.x = msg.angular_velocity().x();
    gyroData.AngularVelocity.y = msg.angular_velocity().y();
    gyroData.AngularVelocity.z = msg.angular_velocity().z();
    m_dataManager.post(gyroData);
}

void GazeboInterface::gpsCallback(const gz::msgs::NavSat& msg) {
    // Implementation for GPS callback
}

void GazeboInterface::magnetometerCallback(const gz::msgs::Magnetometer& msg) {
    // Implementation for Magnetometer callback
}
