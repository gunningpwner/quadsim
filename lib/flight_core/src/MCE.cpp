#include "MCE.h"
#include <stdio.h>
#include "timing.h"
void MonolithicControlEntity::initialize(DShot *dshot_driver)
{
    m_dshot = dshot_driver;
    m_filter = new ESKF(m_data_manager.makeSensorConsumer(), m_data_manager.getStateBuffer());
    // m_auto_level_controller = new AutoLevelController(m_data_manager);
    transition_to(UnitializedState::instance());
}

void MonolithicControlEntity::run()
{

    if (m_filter)
        m_filter->run();

    RCChannelsData *rc_data = m_rc_consumer.readLatest();
    if (rc_data != nullptr)
    {
        last_rc_data = *rc_data;
        last_rc_frame_time = rc_data->timestamp;
    }

    const uint64_t current_time = getCurrentTimeUs();

    if (m_current_state != FailsafeState::instance() && last_rc_frame_time > 0 && (current_time - last_rc_frame_time > rc_timeout_us))
        transition_to(FailsafeState::instance());

    if (m_current_state != DisarmedState::instance() && m_current_state != FailsafeState::instance() && last_rc_frame_time > 0 && last_rc_data.channels.chan4 < CRSF_CHANNEL_MAX)
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
    if (getCurrentTimeUs() - mce->last_rc_frame_time < mce->rc_timeout_us)
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
    // mce->m_auto_level_controller->run();
    // mce->getDShotDriver()->update();
    return this;
}