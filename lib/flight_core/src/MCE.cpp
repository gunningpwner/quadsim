#include "MCE.h"
#include <stdio.h>
#include <algorithm>
#include "timing.h"
#include <math.h>
#include <inttypes.h> // Required for PRIu64
#ifdef PROFILING
    #include "SEGGER_SYSVIEW.h"
#endif
#ifdef SIM
    #include "driver_emulators/DShot.h"
    #include "driver_emulators/bmi270.h"
#else
    #include "drivers/DShot.h"
    #include "drivers/bmi270.h"
#endif

extern DShot *g_dshot_ptr;
extern BMI270 *g_imu_ptr;
extern DataManager *g_data_manager_ptr;



UserCommands interpretRCData(const RCChannelsData &rc_data){
    UserCommands user_commands;
    user_commands.armSwitch = rc_data.channels.chan5 >= CRSF_CHANNEL_MAX;
    user_commands.excitationTriggered = rc_data.channels.chan12 >= CRSF_CHANNEL_MAX;
    user_commands.throttle = (std::clamp(rc_data.channels.chan3,CRSF_CHANNEL_MIN,CRSF_CHANNEL_MAX) - CRSF_CHANNEL_MIN) / (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN);
    

    return user_commands;
}


MonolithicControlEntity::MonolithicControlEntity()
    : last_rc_frame_time(0),
      last_user_commands{},
      m_filter(nullptr),
    m_current_state(nullptr),
    m_rc_consumer(g_data_manager_ptr->makeRCChannelsConsumer()),
    m_sensor_buffer(g_data_manager_ptr->getSensorBuffer()),
    m_state_consumer(g_data_manager_ptr->makeStateConsumer()),
    m_adjudicator()
{

}
void MonolithicControlEntity::init()
{

    m_filter = new ESKF(g_data_manager_ptr->makeSensorConsumer(), g_data_manager_ptr->getStateBuffer());
    // m_auto_level_controller = new AutoLevelController(m_data_manager);
    transition_to(DisarmedState::instance());
}

void MonolithicControlEntity::run()
{

    if (m_filter){
        
        m_filter->run();
        
    }
    RCChannelsData *rc_data = m_rc_consumer.readLatest();
    if (rc_data != nullptr)
    {
        last_user_commands = interpretRCData(*rc_data);
        last_rc_frame_time = rc_data->timestamp;
    }

    const uint64_t current_time = getCurrentTimeUs();

    if (m_current_state != FailsafeState::instance() && last_rc_frame_time > 0 && (current_time - last_rc_frame_time > rc_timeout_us)){
        transition_to(FailsafeState::instance());
        return;
    }
    if (m_current_state != DisarmedState::instance() && m_current_state != FailsafeState::instance() && last_rc_frame_time > 0 && !last_user_commands.armSwitch){
        transition_to(DisarmedState::instance());
        return;
    }
    State *next_state = m_current_state->on_run(this);

    if (next_state != m_current_state)
        transition_to(next_state);
}

void MonolithicControlEntity::runFastLoop()
{

    static FastIMUData imu_data;
    static FastRPMData rpm_data;
    static float motor_cmd[4] = {0.00f, 0.00f, 0.00f, 0.00f};
    // Use this to run the kinematic adjudicator without using the commands that come out
    // static float test_buffer[4] = {1.00f, 1.00f, 1.00f, 1.00f};
    static float imu_buffer[6] = {0.0f};
    static size_t samples = 0;

    bool valid_imu = g_imu_ptr->processRawDataFAST(&imu_data);
    if (valid_imu)
    {
        imu_buffer[0] += imu_data.data[0];
        imu_buffer[1] += imu_data.data[1];
        imu_buffer[2] += imu_data.data[2];
        imu_buffer[3] += imu_data.data[3];
        imu_buffer[4] += imu_data.data[4];
        imu_buffer[5] += imu_data.data[5];
        samples++;
    }
    // TODO: Have this check if a certain time has passed rather than number of samples
    if (samples == 16)
    {
        uint64_t timestamp = getCurrentTimeUs();
        SensorData *dwnsmpled_imu_data = m_sensor_buffer.claim();
        dwnsmpled_imu_data->timestamp = timestamp;
        dwnsmpled_imu_data->sensor = SensorData::Type::IMU;
        dwnsmpled_imu_data->data.imu.accel[0] = imu_buffer[0] / samples;
        dwnsmpled_imu_data->data.imu.accel[1] = imu_buffer[1] / samples;
        dwnsmpled_imu_data->data.imu.accel[2] = imu_buffer[2] / samples;

        dwnsmpled_imu_data->data.imu.gyro[0] = imu_buffer[3] / samples;
        dwnsmpled_imu_data->data.imu.gyro[1] = imu_buffer[4] / samples;
        dwnsmpled_imu_data->data.imu.gyro[2] = imu_buffer[5] / samples;
        m_sensor_buffer.commit(dwnsmpled_imu_data);
        samples = 0;
        std::memset(imu_buffer, 0, sizeof(imu_buffer));
    }
    // TODO: This is eRPM. I need RPM. It doesn't really matter actually. 
    // The estimator will just learn the eRPM. 
    unsigned long valid_rpm = g_dshot_ptr->processTelemetry(&rpm_data);
    #ifdef PROFILING
        SEGGER_SYSVIEW_DATA_SAMPLE motor_data;
        for (int i=0; i<4;i++){
            motor_data.ID = MOTOR_1_DATA_ID+i;
            motor_data.pValue.pU32 = &rpm_data.rpm[i];
            SEGGER_SYSVIEW_SampleData(&motor_data);
        }
        motor_data.ID=VALID_RPM_DATA_ID;
        motor_data.pValue.pU32 = &valid_rpm;
        SEGGER_SYSVIEW_SampleData(&motor_data);
    #endif

    if (valid_rpm == 4 && valid_imu)
    {

        m_adjudicator.update(&imu_data, &rpm_data, motor_cmd);
    }

    
    g_dshot_ptr->sendMotorThrottle(motor_cmd);
    
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

State *DisarmedState::instance()
{
    static DisarmedState instance;
    return &instance;
}

void DisarmedState::on_enter(MonolithicControlEntity *mce)
{
    g_dshot_ptr->disarm();
}

State *DisarmedState::on_run(MonolithicControlEntity *mce)
{
    UserCommands &user_commands = mce->last_user_commands;
    if (user_commands.armSwitch && user_commands.throttle == 0.0f)
    {
        return ArmedState::instance();
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

    g_dshot_ptr->disarm();
}

State *FailsafeState::on_run(MonolithicControlEntity *mce)
{
    if (getCurrentTimeUs() - mce->last_rc_frame_time < mce->rc_timeout_us)
    {
        return DisarmedState::instance();
    }
    return this;
}

State *ArmedState::instance()
{
    static ArmedState instance;
    return &instance;
}
void ArmedState::on_enter(MonolithicControlEntity *mce)
{
    g_dshot_ptr->arm();
}

#define MAG3(v) std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

State *ArmedState::on_run(MonolithicControlEntity *mce)
{
    UserCommands &user_commands = mce->last_user_commands;
    // StateEstimate *state_data = mce->getStateConsumer().readLatest();
    // if (state_data == nullptr){
    //     return this;
    // }

    KinematicAdjudicator *adjudicator = mce->getKinematicAdjudicator();
    // Not sure if I want this mf to launch itself into the sky first
    if(user_commands.excitationTriggered && adjudicator->getCurrentMode()==FlightMode::IDLE){
        // Make sure we're not moving, rotating or upside down ( or at least make sure we dont THINK we are)
        // if ( MAG3(state_data->velocity_ned)<5 && MAG3(state_data->angular_vel)<.03f){
            printf("Trigger warning libs\n");
            adjudicator->launch();
        // }
    }


    return this;
}