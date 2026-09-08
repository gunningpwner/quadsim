#include "DataManager.h"
#include "DataTypes.h"
#include "ESKF.h"
#include "MCEStates.h"
#include "KinematicAdjudicator.h"
#include "GeometricController.h"

struct UserCommands{
    bool armSwitch=false;
    bool excitationTriggered=false;
    float throttle=0.0f;
    
};

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
    DataManager::StateConsumer &getStateConsumer() { return m_state_consumer; }
    KinematicAdjudicator *getKinematicAdjudicator() { return &m_adjudicator; }
    GeometricController *getGeometricController() { return m_geom_control;}
    uint64_t last_rc_frame_time;
    UserCommands last_user_commands;
    const uint64_t rc_timeout_us = 500000;
    

private:
    ESKF *m_filter;
    State *m_current_state;
    DataManager::RCChannelsConsumer m_rc_consumer;
    DataManager::SensorBuffer &m_sensor_buffer;
    DataManager::StateConsumer m_state_consumer;
    KinematicAdjudicator m_adjudicator;
    GeometricController *m_geom_control;
    
};


