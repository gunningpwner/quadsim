#include "TestHarness.h"
#include "Logger.h"
#include "timing.h"
#include <iostream>
static TestHarness* g_test_harness = nullptr;

uint64_t getCurrentTimeUs() {
    if (g_test_harness)
        return g_test_harness->getSimTimeUs();
    return 0;
}
// Use the passed robot instance
TestHarness::TestHarness(webots::Robot* robot) : m_robot(robot) {

    // 1. Initialize Webots Sensors (Must match names in your PROTO)
    m_acc = m_robot->getAccelerometer("accelerometer"); // Accelerometer usually
    m_gyro = m_robot->getGyro("gyro");       // Webots separates Accel/Gyro
    m_gps = m_robot->getGPS("gps");
    m_mag = m_robot->getCompass("compass");

    int timeStep = (int)m_robot->getBasicTimeStep();

    // 2. Enable Sensors (Crucial step!)
    m_acc->enable(timeStep);
    m_gyro->enable(timeStep);
    m_gps->enable(timeStep);
    m_mag->enable(timeStep);

    // 3. Initialize Motors
    std::string motorNames[4] = {"m1", "m2", "m3", "m4"};
    for(int i=0; i<4; i++){
        m_motors[i] = m_robot->getMotor(motorNames[i]);
        m_motors[i]->setPosition(INFINITY); // Velocity mode
        m_motors[i]->setVelocity(0.0);
    }

    // 4. Initialize Core Logic
    m_mce = new MonolithicControlEntity();
    m_data_manager = &m_mce->getDataManager();
    
    // Lambda to fetch time from Webots
    TimeSource simTimeSource = [this](){ return getSimTimeUs(); };
    
    // NOTE: You need to ensure 'DShot' or your MotorInterface 
    // is compatible with writing to these webots motors, or 
    // you handle the writing in writeMotors() below.
    DShot motor_interface = DShot(*m_data_manager);
    motor_interface.init();
    
    m_mce->initialize(simTimeSource, &motor_interface);
}

TestHarness::~TestHarness() {
    delete m_mce;
}

uint64_t TestHarness::getSimTimeUs() const {
    // Webots getTime() returns seconds as double
    return (uint64_t)(m_robot->getTime() * 1000000.0);
}

void TestHarness::update(int dt_ms) {
    // 1. Read all Webots sensors and push to DataManager
    readSensors();

    // 2. Run Flight Core
    m_mce->run();

    // 3. Write outputs back to Webots
    writeMotors();
}

void TestHarness::readSensors() {
    int64_t timestamp_us = getSimTimeUs();

    // --- IMU (Accel + Gyro) ---
    // Note: Verify frame! Webots defaults often differ from Gazebo.
    const double* acc = m_acc->getValues();  // [X, Y, Z]
    const double* gyr = m_gyro->getValues(); // [X, Y, Z]

    IMUData imu_data;
    imu_data.Timestamp = timestamp_us;
    
    // Direct mapping (Adjust signs if your drone flips!)
    imu_data.Acceleration << acc[0], acc[1], acc[2]; 
    imu_data.AngularVelocity << gyr[0], gyr[1], gyr[2];
    m_data_manager->post(imu_data);

    // --- GPS ---
    const double* gps_vals = m_gps->getValues(); // [Lat, Lon, Alt] if WGS84
    const double speed = m_gps->getSpeed();      // scalar speed m/s
    // Note: Webots GPS doesn't give NED velocity vector easily without math.
    // You might need to estimate velocity or use 'gps->getSpeedVector()' if available in newer API.
    
    GPSData gpsData;
    gpsData.Timestamp = timestamp_us;
    gpsData.lla = {gps_vals[0], gps_vals[1], gps_vals[2]};
    // Placeholder for velocity if your core needs it:
    gpsData.vel = {0, 0, 0}; 
    m_data_manager->post(gpsData);

    // --- MAG ---
    const double* mag = m_mag->getValues();
    MagData magData;
    magData.Timestamp = timestamp_us;
    magData.MagneticField << mag[0], mag[1], mag[2];
    m_data_manager->post(magData);
}

void TestHarness::writeMotors() {
    // Retrieve motor commands from your DataManager or MCE
    // This part depends on how MCE exposes outputs. 
    // Example:
    // double m1_cmd = m_data_manager->getMotorCommand(0); 
    // m_motors[0]->setVelocity(m1_cmd);
}