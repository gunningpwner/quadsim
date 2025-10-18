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
    uint64_t current_time = m_data_manager.getCurrentTimeUs();
    // Print 64-bit time as two 32-bit parts to avoid issues with printf not supporting %llu
    // printf("Time H:%lu L:%lu\n", (unsigned long)(current_time >> 32), (unsigned long)(current_time & 0xFFFFFFFF));
    // printf("Last RC Frame TimeH:%lu L:%lu\n", (unsigned long)(last_rc_frame_time >> 32), (unsigned long)(last_rc_frame_time & 0xFFFFFFFF));
    // 3. Check for RC link failure (Failsafe)
    // Only check for failsafe if we have received at least one frame (i.e., last_rc_frame_time is not 0)
    
    if (m_current_state != FailsafeState::instance() && last_rc_frame_time > 0 &&
        (m_data_manager.getCurrentTimeUs() - last_rc_frame_time > rc_timeout_us)) {
        printf("Time H:%lu L:%lu\n", (unsigned long)(current_time >> 32), (unsigned long)(current_time & 0xFFFFFFFF));
        printf("Last RC Frame TimeH:%lu L:%lu\n", (unsigned long)(last_rc_frame_time >> 32), (unsigned long)(last_rc_frame_time & 0xFFFFFFFF));
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

void DisarmedState::on_enter(MonolithicControlEntity* mce){
    printf("Entering DISARMED state.\n");
    // Post a motor command to stop all motors.
    MotorCommands motor_command = {}; // Zero-initialize
    motor_command.Timestamp = mce->getDataManager().getCurrentTimeUs();
    motor_command.is_throttle_command = false;
    motor_command.command = DShot_Command::MOTOR_STOP;
    mce->getDataManager().post(motor_command);
}

State* DisarmedState::on_run(MonolithicControlEntity* mce) {
    // TODO: Transition to ARMED_RATE state based on RC channels
    return this;
}

State* FailsafeState::instance() {
    static FailsafeState instance;
    return &instance;
}

void FailsafeState::on_enter(MonolithicControlEntity* mce) {
    printf("ENTERING FAILSAFE: RC link lost!\n");
    // Post a motor command to stop all motors.
    MotorCommands motor_command = {}; // Zero-initialize
    motor_command.Timestamp = mce->getDataManager().getCurrentTimeUs();
    motor_command.is_throttle_command = false;
    motor_command.command = DShot_Command::MOTOR_STOP;
    mce->getDataManager().post(motor_command);
}

State* FailsafeState::on_run(MonolithicControlEntity* mce) {
    // Transition: Check if RC link is restored.
    if (mce->getDataManager().getCurrentTimeUs() - mce->last_rc_frame_time < mce->rc_timeout_us) {
        printf("RC link restored. Returning to DISARMED state.\n");
        return DisarmedState::instance(); // Go to a safe state
    }
    return this;
}