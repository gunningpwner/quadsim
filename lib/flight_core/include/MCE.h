#include "DataManager.h"
#include "Controller.h"
#include "DataTypes.h"
#include "ESKF.h"

#ifndef SIM
    #include "drivers/DShot.h"
#else
    #include "MotorInterface.h"
#endif

using TimeSource = std::function<uint64_t()>;

class MonolithicControlEntity;

/**
 * @brief Abstract base class for all flight states.
 *
 * This defines the interface that all concrete states must implement.
 */
class State
{
public:
    virtual ~State() = default;

    // Called once when the state is entered.
    virtual void on_enter(MonolithicControlEntity *mce) { (void)mce; }

    // Called continuously while the state is active.
    virtual State *on_run(MonolithicControlEntity *mce) = 0;

    // Called once when the state is exited.
    virtual void on_exit(MonolithicControlEntity *mce) { (void)mce; }

    virtual const char *getName() const = 0;
};

class UnitializedState : public State
{
    // First state on startup, remain in this state until we receive positive control confirmation (aka our first crsf message)
    // Then transition into Disarmed
public:
    static State *instance();
    virtual State *on_run(MonolithicControlEntity *mce) override;
    virtual const char *getName() const override { return "UNINITIALIZED"; }
};

class DisarmedState : public State
{
    // Motors should not be running. Should stay in this state until throttle is at 0 and the arm switch is engaged
public:
    static State *instance();
    virtual void on_enter(MonolithicControlEntity *mce) override;
    virtual State *on_run(MonolithicControlEntity *mce) override;
    virtual const char *getName() const override { return "DISARMED"; }
};

class FailsafeState : public State
{
    // Transitioned to when we lose positive control for set time
    // Right now, let's just cut motors and yeet to the ground, but could do a return home in the future.
public:
    static State *instance();
    virtual void on_enter(MonolithicControlEntity *mce) override;
    virtual State *on_run(MonolithicControlEntity *mce) override;
    virtual const char *getName() const override { return "FAILSAFE"; }
};

class ArmedLevelState : public State
{
    // Runs the auto level controller
public:
    static State *instance();
    virtual void on_enter(MonolithicControlEntity *mce) override;
    virtual State *on_run(MonolithicControlEntity *mce) override;
    virtual const char *getName() const override { return "ARMED_LEVEL"; }
};

class MonolithicControlEntity
{
    // Responsible for managing hardware independent processing.
    // Will setup and expose DataManager for hardware to communicate through.
    // Essentially just a main for the software but containerized
public:
    MonolithicControlEntity() : m_data_manager(),
                                m_filter(nullptr),
                                m_dshot(nullptr),
                                m_rc_consumer(m_data_manager.makeRCChannelsConsumer()),
                                last_rc_frame_time(0),
                                last_rc_data{},
                                m_current_state(nullptr) {}

    void initialize(TimeSource time_source_func, DShot *dshot_driver);
    void transition_to(State *new_state);
    void run();
    DataManager &getDataManager() { return m_data_manager; }
    DShot *getDShotDriver() { return m_dshot; }
    const State *getCurrentState() const { return m_current_state; }
    Controller *m_auto_level_controller;

    uint64_t last_rc_frame_time;
    RCChannelsData last_rc_data;
    const uint64_t rc_timeout_us = 500000;

private:
    DataManager m_data_manager;
    ESKF *m_filter;
    DShot *m_dshot;
    State *m_current_state;
    DataManager::RCChannelsConsumer m_rc_consumer;
};