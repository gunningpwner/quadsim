#include "MotorInterface.h"

DShot::DShot(DataManager& data_manager): m_motor_commands_consumer(data_manager.getMotorCommandsChannel()){
    
}

int DShot::init(){
    return 0;
}
void DShot::arm(){
    
}

void DShot::disarm()
{
    
}

void DShot::update()
{
    
}


void DShot::sendMotorCommand(MotorCommands& cmd){
    
};

