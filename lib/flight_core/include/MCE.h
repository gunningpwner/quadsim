#include "DataManager.h"
#include "DataTypes.h"
#include "ESKF.h"
#include "MCEStates.h"


#ifndef SIM
    #include "drivers/DShot.h"
#else
    #include "MotorInterface.h"
#endif



class MonolithicControlEntity
{
    // Responsible for managing hardware independent processing.
    // Will setup and expose DataManager for hardware to communicate through.
    // Essentially just a main for the software but containerized
public:
    MonolithicControlEntity();

    void init();
    void transition_to(State *new_state);
    void run();
    void runFastLoop();
    const State *getCurrentState() const { return m_current_state; }

    uint64_t last_rc_frame_time;
    RCChannelsData last_rc_data;
    const uint64_t rc_timeout_us = 500000;

private:
    ESKF *m_filter;
    State *m_current_state;
    DataManager::RCChannelsConsumer m_rc_consumer;
    DataManager::SensorBuffer &m_sensor_buffer;
};