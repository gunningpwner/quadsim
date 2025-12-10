#include "ESKF.h"


ESKF::ESKF(DataManager& data_manager):
    m_imu_consumer(data_manager.getIMUChannel()),
    m_mag_consumer(data_manager.getMagChannel()),
    m_gps_consumer(data_manager.getGPSChannel())
{

}

void ESKF::run(){
    
}