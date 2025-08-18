#pragma once

#include "Controller.h"
#include "OtherData.h"
#include "SensorData.h"
#include "PID.h" // Include the PID class

class BodyRateController : public Controller {
public:
    BodyRateController(DataManager& data_manager);
    void run() override;

private:
    // PID controllers for roll, pitch, and yaw rates
    PID m_roll_pid;
    PID m_pitch_pid;
    PID m_yaw_pid;
};