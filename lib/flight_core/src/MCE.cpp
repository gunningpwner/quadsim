#include "MCE.h"
#include <stdio.h>
#include "timing.h"
#include "drivers/bmi270.h"

extern DShot *g_dshot_ptr;
extern BMI270 *g_imu_ptr;
extern DataManager *g_data_manager_ptr;

MonolithicControlEntity::MonolithicControlEntity()
    : last_rc_frame_time(0),
      last_rc_data{},
      m_filter(nullptr),
    m_current_state(nullptr),
    m_rc_consumer(g_data_manager_ptr->makeRCChannelsConsumer()),
    m_sensor_buffer(g_data_manager_ptr->getSensorBuffer())
{

}
void MonolithicControlEntity::init()
{

    m_filter = new ESKF(g_data_manager_ptr->makeSensorConsumer(), g_data_manager_ptr->getStateBuffer());
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

void MonolithicControlEntity::runFastLoop()
{

    static FastIMUData imu_data;
    static FastRPMData rpm_data;
    static float motor_cmd[4] = {0.00f, 0.00f, 0.00f, 0.00f};
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
    // TODO: This is eRPM. I need RPM
    uint8_t valid_rpm = g_dshot_ptr->processTelemetry(&rpm_data);
    if (valid_rpm == 4 && valid_imu)
    {
    }
    else
    {
        // Send last command
        g_dshot_ptr->sendMotorThrottle(motor_cmd);
    }
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
    g_dshot_ptr->disarm();
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

State *ArmedLevelState::instance()
{
    static ArmedLevelState instance;
    return &instance;
}
void ArmedLevelState::on_enter(MonolithicControlEntity *mce)
{
    g_dshot_ptr->arm();
}
State *ArmedLevelState::on_run(MonolithicControlEntity *mce)
{
    // mce->m_auto_level_controller->run();
    // mce->getDShotDriver()->update();
    return this;
}