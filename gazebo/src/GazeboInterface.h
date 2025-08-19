#ifndef GAZEBO_INTERFACE_H
#define GAZEBO_INTERFACE_H

#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include "DataManager.h"
#include <thread>
#include <atomic>

/**
 * @class GazeboInterface
 * @brief Manages communication with the Gazebo simulator.
 *
 * This class sets up a Gazebo transport node, subscribes to sensor topics,
 * forwards the received data to a DataManager instance, and publishes
 * motor commands from the DataManager to the simulator.
 */
class GazeboInterface {
public:
    /**
     * @brief Constructs a GazeboInterface.
     * @param dataManager A reference to the flight software's central DataManager.
     */
    GazeboInterface(DataManager& dataManager);

    /**
     * @brief Destructor to clean up resources.
     */
    ~GazeboInterface();

    /**
     * @brief Starts the subscribers to listen for Gazebo topics.
     */
    void startSubscribers();

    /**
     * @brief Starts the motor command publisher loop in a separate thread.
     */
    void startPublisherLoop();

private:
    /**
     * @brief The main loop for publishing motor commands.
     *
     * This function runs in a separate thread, periodically checking the
     * DataManager for new motor commands and sending them to Gazebo.
     */
    void runPublisherLoop();

    // --- Subscriber Callbacks ---
    void imuCallback(const gz::msgs::IMU& msg);
    void gpsCallback(const gz::msgs::NavSat& msg);
    void magnetometerCallback(const gz::msgs::Magnetometer& msg);

    DataManager& m_dataManager;
    gz::transport::Node m_node;
    gz::transport::Node::Publisher m_motor_pub;

    // --- Threading for Publisher ---
    std::thread m_publisherThread;
    std::atomic<bool> m_run_publisher;
    unsigned int m_last_seen_motor_command_count = 0;
};

#endif // GAZEBO_INTERFACE_H
