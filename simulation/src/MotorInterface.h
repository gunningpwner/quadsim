#pragma once
#include "Consumer.h"
#include "OtherData.h"
#include "DataManager.h"

class DShot {
public:
    DShot(DataManager& data_manager);

    int init();
    void update();
    void disarm();
    void arm();
    void sendMotorCommand(MotorCommands& cmd);
private:
    Consumer<MotorCommands, 1> m_motor_commands_consumer;

};
