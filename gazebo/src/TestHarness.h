#pragma once
#include "MCE.h"
#include "DataManager.h"
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <thread>
#include <atomic>

class TestHarness {
public:
    TestHarness();
    ~TestHarness();
    void startSubscribers();
    void update();
    void run();
    uint64_t getSimTimeUs() const;

private:
    MonolithicControlEntity* m_mce;
    DataManager* m_data_manager;
    std::atomic<uint64_t> m_sim_time_us{0};
    std::atomic<bool> m_should_restart = false;
    std::atomic<bool> m_is_paused = true;
    gz::transport::Node m_node;
    
    void restart();

    // Callbacks
    void imuCallback(const gz::msgs::IMU& msg);
    void gpsCallback(const gz::msgs::NavSat& msg);
    void magnetometerCallback(const gz::msgs::Magnetometer& msg);
    void clockCallback(const gz::msgs::Clock& msg);
    void statsCallback(const gz::msgs::WorldStatistics& msg);

};