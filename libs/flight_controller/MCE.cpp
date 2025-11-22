#include "MCE.h"
#include "MahonyFilter.h"
#include "AutoLevelController.h"
#include "SensorData.h"
#include <stdio.h>

void MonolithicControlEntity::initialize(TimeSource time_source_func, DShot *dshot_driver)
{
    m_data_manager.setTimeSource(time_source_func);
    m_dshot = dshot_driver;
    m_filter = new MahonyFilter(m_data_manager);
    m_auto_level_controller = new AutoLevelController(m_data_manager);
    transition_to(UnitializedState::instance());
}

void MonolithicControlEntity::run()
{

    if (m_filter)
        m_filter->run();

    uint64_t now_time = m_data_manager.getCurrentTimeUs();

    if (m_rc_consumer.consumeLatest())
    {
        last_rc_data = m_rc_consumer.get_span().first[0];
        last_rc_frame_time = last_rc_data.Timestamp;
    }
    const uint64_t current_time = m_data_manager.getCurrentTimeUs();

    if (m_current_state != FailsafeState::instance() && last_rc_frame_time > 0 && (current_time - last_rc_frame_time > rc_timeout_us))
        transition_to(FailsafeState::instance());

    if (m_current_state != DisarmedState::instance() && last_rc_data.channels.chan4 < CRSF_CHANNEL_MAX)
        transition_to(DisarmedState::instance());

    State *next_state = m_current_state->on_run(this);

    if (next_state != m_current_state)
        transition_to(next_state);
    
}

void MonolithicControlEntity::transition_to(State *new_state)
{
    if (m_current_state != nullptr)
    {
        printf("Exiting %s state.\n", m_current_state->getName());
        m_current_state->on_exit(this);
    }
    m_current_state = new_state;
    if (m_current_state != nullptr)
    {
        printf("Entering %s state.\n", m_current_state->getName());
        m_current_state->on_enter(this);
    }
}

State *UnitializedState::instance()
{
    static UnitializedState instance;
    return &instance;
}

State *UnitializedState::on_run(MonolithicControlEntity *mce)
{

    if (mce->last_rc_frame_time > 0)
    {
        return DisarmedState::instance();
    }
    return this;
}

State *DisarmedState::instance()
{
    static DisarmedState instance;
    return &instance;
}

void DisarmedState::on_enter(MonolithicControlEntity *mce)
{

    mce->getDShotDriver()->disarm();
}

State *DisarmedState::on_run(MonolithicControlEntity *mce)
{
    RCChannelsData &rc_data = mce->last_rc_data;
    if (rc_data.channels.chan4 >= CRSF_CHANNEL_MAX && rc_data.channels.chan2 <= CRSF_CHANNEL_MIN)
    {
        return ArmedLevelState::instance();
    }
    return this;
}

State *FailsafeState::instance()
{
    static FailsafeState instance;
    return &instance;
}

void FailsafeState::on_enter(MonolithicControlEntity *mce)
{

    mce->getDShotDriver()->disarm();
}

State *FailsafeState::on_run(MonolithicControlEntity *mce)
{
    if (mce->getDataManager().getCurrentTimeUs() - mce->last_rc_frame_time < mce->rc_timeout_us)
    {
        return DisarmedState::instance();
    }
    return this;
}

State *ArmedLevelState::instance()
{
    static ArmedLevelState instance;
    return &instance;
}
void ArmedLevelState::on_enter(MonolithicControlEntity *mce)
{
    mce->getDShotDriver()->arm();
}
State *ArmedLevelState::on_run(MonolithicControlEntity *mce)
{
    mce->m_auto_level_controller->run();
    mce->getDShotDriver()->update();
    return this;
}