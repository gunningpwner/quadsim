#pragma once
#include "DataManager.h"

class DShot
{
public:
    DShot(DataManager::MotorCommandsConsumer m_motor_commands_consumer);

    int init();
    void update();
    void disarm();
    void arm();
    void sendMotorCommand(MotorCommands &cmd);

private:
    DataManager::MotorCommandsConsumer m_motor_commands_consumer;
};
