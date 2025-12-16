#pragma once
#include "MCE.h"
#include "DataManager.h"
#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <atomic>

class TestHarness {
public:
    // Pass the Robot pointer in so we don't create multiple instances
    TestHarness(webots::Robot* robot);
    ~TestHarness();

    void update(int dt_ms); // Call this every Webots step
    uint64_t getSimTimeUs() const;

private:
    MonolithicControlEntity* m_mce;
    DataManager* m_data_manager;
    
    // Webots Hardware Handles
    webots::Robot* m_robot;
    webots::Accelerometer* m_acc;
    webots::Gyro* m_gyro;
    webots::GPS* m_gps;
    webots::Compass* m_mag;
    
    // Motors (assuming 4 for quad)
    webots::Motor* m_motors[4];

    void readSensors();
    void writeMotors();
};