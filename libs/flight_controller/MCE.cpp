#include "MCE.h"
#include "MahonyFilter.h"
#include "SensorData.h" // For RCChannelsData
#include <stdio.h>

void MonolithicControlEntity::initialize(TimeSource time_source_func){
    m_data_manager.setTimeSource(time_source_func);
    // In a real scenario, you might choose the filter type based on configuration
    m_filter = new MahonyFilter(m_data_manager);
    transition_to(UnitializedState::instance());
}

void MonolithicControlEntity::run() {
    // 1. Run the orientation filter.
    if (m_filter) {
        m_filter->run();
    }

    // 2. Check for and process new RC commands.
    RCChannelsData last_rc_frame;
    if (m_data_manager.getLatest(last_rc_frame)) {
        last_rc_frame_time = last_rc_frame.Timestamp;
    }

    // 3. Check for RC link failure (Failsafe)
    if (m_current_state != FailsafeState::instance() && (m_data_manager.getCurrentTimeUs() - last_rc_frame_time > rc_timeout_us)) {
        transition_to(FailsafeState::instance());
    }

    // 4. Run the current state's logic
    State* next_state = m_current_state->on_run(this);
    // If the state returned a different instance, transition
    if (next_state != m_current_state) {
        transition_to(next_state);
    }
}

void MonolithicControlEntity::transition_to(State* new_state) {
    if (m_current_state != nullptr) {
        printf("Exiting %s state.\n", m_current_state->getName());
        m_current_state->on_exit(this);
    }
    m_current_state = new_state;
    if (m_current_state != nullptr) {
        printf("Entering %s state.\n", m_current_state->getName());
        m_current_state->on_enter(this);
    }
}

State* UnitializedState::instance() {
    static UnitializedState instance;
    return &instance;
}

State* UnitializedState::on_run(MonolithicControlEntity* mce){
    // If we have received an RC frame, we can move to the disarmed state.
    if (mce->last_rc_frame_time > 0) {
        return DisarmedState::instance();
    }
    return this; // Remain in this state
}

State* DisarmedState::instance() {
    static DisarmedState instance;
    return &instance;
}

State* DisarmedState::on_enter(MonolithicControlEntity* mce){
    //TODO: Figure out how to pipe the motor stop command in a way that it is hardware agnostic
    // Maybe a new channel idk
};
