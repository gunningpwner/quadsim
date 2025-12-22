#include "TestHarness.h"
#include "Logger.h"
#include "timing.h"
#include <iostream>
static TestHarness *g_test_harness = nullptr;

uint64_t getCurrentTimeUs()
{
    if (g_test_harness)
        return g_test_harness->getSimTimeUs();
    return 0;
}
// Use the passed robot instance
TestHarness::TestHarness(webots::Robot *robot) : m_robot(robot)
{

    // 1. Initialize Webots Sensors (Must match names in your PROTO)
    m_acc = m_robot->getAccelerometer("accelerometer"); // Accelerometer usually
    m_gyro = m_robot->getGyro("gyro");                  // Webots separates Accel/Gyro
    m_gps = m_robot->getGPS("gps");
    m_mag = m_robot->getCompass("compass");

    int timeStep = (int)m_robot->getBasicTimeStep();

    // 2. Enable Sensors (Crucial step!)
    m_acc->enable(timeStep);
    m_gyro->enable(timeStep);
    m_gps->enable(timeStep*10);
    m_mag->enable(timeStep*5);

    // 3. Initialize Motors
    std::string motorNames[4] = {"m1", "m2", "m3", "m4"};
    for (int i = 0; i < 4; i++)
    {
        m_motors[i] = m_robot->getMotor(motorNames[i]);
        m_motors[i]->setPosition(INFINITY); // Velocity mode
        m_motors[i]->setVelocity(0.0);
    }

    // 4. Initialize Core Logic
    m_mce = new MonolithicControlEntity();
    m_data_manager = &m_mce->getDataManager();

    m_sensor_buffer = &m_data_manager->getSensorBuffer();

    // NOTE: You need to ensure 'DShot' or your MotorInterface
    // is compatible with writing to these webots motors, or
    // you handle the writing in writeMotors() below.
    DShot motor_interface = DShot(m_data_manager->makeMotorCommandsConsumer());
    motor_interface.init();

    m_mce->initialize(&motor_interface);
    g_test_harness = this;
}

TestHarness::~TestHarness()
{
    delete m_mce;
}

uint64_t TestHarness::getSimTimeUs() const
{
    // Webots getTime() returns seconds as double
    return (uint64_t)(m_robot->getTime() * 1000000.0);
}

void TestHarness::update(int dt_ms)
{
    // 1. Read all Webots sensors and push to DataManager
    readSensors();

    // 2. Run Flight Core
    m_mce->run();

    // 3. Write outputs back to Webots
    writeMotors();
}

void TestHarness::readSensors()
{
    if (m_sensor_buffer==nullptr)
        return;

    int sim_time_ms = (int)(m_robot->getTime() * 1000.0 + 0.5);

    uint64_t timestamp_us = getSimTimeUs();

    int acc_period = m_acc->getSamplingPeriod();
    if (sim_time_ms % acc_period == 0) {
        const double *acc = m_acc->getValues();  // [X, Y, Z]
        const double *gyr = m_gyro->getValues(); // [X, Y, Z]

        SensorData *imu_data = m_sensor_buffer->claim();
        imu_data->sensor = SensorData::Type::IMU;
        imu_data->timestamp = timestamp_us;

        imu_data->data.imu.accel = {acc[0], -acc[1], -acc[2]};
        imu_data->data.imu.gyro = {gyr[0], -gyr[1], -gyr[2]};
        m_sensor_buffer->commit(imu_data);
    }

    // --- GPS ---
    int gps_period = m_gps->getSamplingPeriod();
    if (sim_time_ms % gps_period == 0) {
        const double *gps_vals = m_gps->getValues(); 
        const double *speed = m_gps->getSpeedVector();      


        SensorData *gpsData = m_sensor_buffer->claim();
        gpsData->sensor = SensorData::Type::GPS;
        
        gpsData->timestamp = timestamp_us;
        gpsData->data.gps.lla = {gps_vals[0], gps_vals[1], gps_vals[2]};
        // Placeholder for velocity if your core needs it:
        gpsData->data.gps.vel = {speed[1], speed[0], -speed[2]};
        m_sensor_buffer->commit(gpsData);
    }
    // --- MAG ---
    int mag_period = m_mag->getSamplingPeriod();
    if (sim_time_ms % mag_period == 0) {
        const double *mag = m_mag->getValues();
        SensorData *magData= m_sensor_buffer->claim();
        magData->sensor = SensorData::Type::MAG;
        magData->timestamp = timestamp_us;
        magData->data.mag.mag = {mag[1], mag[0], -mag[2]};
        m_sensor_buffer->commit(magData);
    }
}

void TestHarness::writeMotors()
{
    // Retrieve motor commands from your DataManager or MCE
    // This part depends on how MCE exposes outputs.
    // Example:
    // double m1_cmd = m_data_manager->getMotorCommand(0);
    // m_motors[0]->setVelocity(m1_cmd);
}