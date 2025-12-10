#include "GazeboInterface.h"
#include <iostream>
#include <chrono>


GazeboInterface::GazeboInterface(DataManager& dataManager) : 
    m_dataManager(dataManager),
    m_motor_commands_consumer(dataManager.getMotorCommandsChannel())
{
    // Advertise the motor command topic
    const std::string motorTopic = "/quadcopter/cmd_vel";
    m_motor_pub = m_node.Advertise<gz::msgs::Actuators>(motorTopic);
    if (!m_motor_pub) {
        std::cerr << "Error advertising topic [" << motorTopic << "]" << std::endl;
    }
}

GazeboInterface::~GazeboInterface() {}

void GazeboInterface::startSubscribers() {
    // Note: You may need to change these topic names depending on your Gazebo world.
    const std::string imuTopic = "/imu";
    const std::string gpsTopic = "/gps"; // You may need to add a GPS sensor to your SDF
    const std::string magTopic = "/magnetometer"; // You may need to add a magnetometer to your SDF
    const std::string clockTopic = "/world/default/clock";

    // Subscribe to topics
    if (!m_node.Subscribe(imuTopic, &GazeboInterface::imuCallback, this)) {
        std::cerr << "Error subscribing to topic [" << imuTopic << "]" << std::endl;
    }
    if (!m_node.Subscribe(magTopic, &GazeboInterface::magnetometerCallback, this)) {
        std::cerr << "Error subscribing to topic [" << magTopic << "]" << std::endl;
    }
    if (!m_node.Subscribe(gpsTopic, &GazeboInterface::gpsCallback, this)) {
        std::cerr << "Error subscribing to topic [" << gpsTopic << "]" << std::endl;
    }
    if (!m_node.Subscribe(clockTopic, &GazeboInterface::clockCallback, this)) {
        std::cerr << "Error subscribing to topic [" << clockTopic << "]" << std::endl;
    }
    
}

void GazeboInterface::onMotorCommandPosted() {
    // This function is now called directly by the DataManager when a new command is available.

    // Consume the latest command. If there's nothing new, do nothing.
    if (!m_motor_commands_consumer.consumeLatest()) {
        return;
    }

    const MotorCommands& latest_command = m_motor_commands_consumer.get_span().first[0];

    if (latest_command.is_throttle_command){
        // Create the Gazebo message
        gz::msgs::Actuators motor_msg;
        // Iterate through the C-style array
        for (int i = 0; i < 4; ++i) {
            motor_msg.add_velocity(latest_command.throttle[i]);
        }

        // Publish the message
        m_motor_pub.Publish(motor_msg);
    }
    
}

void GazeboInterface::imuCallback(const gz::msgs::IMU& msg) {
    int64_t timestamp_us = msg.header().stamp().sec() * 1000000LL + msg.header().stamp().nsec() / 1000LL;

    IMUData imu_data;
    imu_data.Timestamp = timestamp_us;
    imu_data.Acceleration << msg.linear_acceleration().x(),
                              msg.linear_acceleration().y(),
                              msg.linear_acceleration().z();

    imu_data.AngularVelocity << msg.angular_velocity().x(),
                                msg.angular_velocity().y(),
                                msg.angular_velocity().z();
    m_dataManager.post(imu_data);
}

void GazeboInterface::gpsCallback(const gz::msgs::NavSat& msg) {
    int64_t timestamp_us = msg.header().stamp().sec() * 1000000LL + msg.header().stamp().nsec() / 1000LL;
    GPSData gpsData;
    gpsData.Timestamp = timestamp_us;
    gpsData.lla={msg.latitude_deg(),msg.longitude_deg(),msg.altitude()};
    m_dataManager.post(gpsData);
}

void GazeboInterface::magnetometerCallback(const gz::msgs::Magnetometer& msg) {
    int64_t timestamp_us = msg.header().stamp().sec() * 1000000LL + msg.header().stamp().nsec() / 1000LL;
    MagData magData;
    magData.Timestamp = timestamp_us;
    magData.MagneticField << msg.field_tesla().x(),
                             msg.field_tesla().y(),
                             msg.field_tesla().z();
    m_dataManager.post(magData);
}

void GazeboInterface::clockCallback(const gz::msgs::Clock& msg) {
    uint64_t time_us = msg.sim().sec() * 1000000LL + msg.sim().nsec() / 1000LL;
    m_sim_time_us.store(time_us);
}

uint64_t GazeboInterface::getSimTimeUs() const {
    return m_sim_time_us.load();
}
