#ifndef GAZEBO_INTERFACE_H
#define GAZEBO_INTERFACE_H

#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include "DataManager.h"
#include "Consumer.h"
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
     * @brief Gets the latest simulation time received from Gazebo.
     * @return The simulation time in microseconds.
     */
    uint64_t getSimTimeUs() const;

private:
    /**
     * @brief Callback function triggered when a new motor command is posted.
     * This function consumes the latest command and publishes it to Gazebo.
     */
    void onMotorCommandPosted();
    // --- Subscriber Callbacks ---
    void imuCallback(const gz::msgs::IMU& msg);
    void gpsCallback(const gz::msgs::NavSat& msg);
    void magnetometerCallback(const gz::msgs::Magnetometer& msg);
    void clockCallback(const gz::msgs::Clock& msg);

    DataManager& m_dataManager;
    gz::transport::Node m_node;
    gz::transport::Node::Publisher m_motor_pub;

    Consumer<MotorCommands, 1> m_motor_commands_consumer;
    std::atomic<uint64_t> m_sim_time_us{0};
};

#endif // GAZEBO_INTERFACE_H
