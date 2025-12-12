#include "TestHarness.h"
#include "Logger.h"
#include "timing.h"

extern std::atomic<bool> g_run_application;

static TestHarness* g_test_harness = nullptr;

uint64_t getCurrentTimeUs() {
    if (g_test_harness)
        return g_test_harness->getSimTimeUs();
    return 0;
}

TestHarness::TestHarness(){
    g_test_harness = this;

    m_mce = new MonolithicControlEntity();
    m_data_manager = &m_mce->getDataManager();
    TimeSource simTimeSource = [this](){ return getSimTimeUs(); };
    DShot motor_interface = DShot(*m_data_manager);
    motor_interface.init();
    m_mce->initialize(simTimeSource, &motor_interface);
    
    // m_mce->transition_to(DisarmedState::instance());
    startSubscribers();

    
}

TestHarness::~TestHarness(){
    if (g_test_harness == this)
        g_test_harness = nullptr;
    delete m_mce;
    m_data_manager=nullptr;
}
void TestHarness::run(){
    while (g_run_application.load()) {
        if (m_should_restart.load()) {
            std::cout << "Gazebo Reset Detected... Restarting..." << std::endl;
            restart();
            m_should_restart.store(false);
        }
        if (!m_is_paused.load()){
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}
void TestHarness::update(){
    m_mce->run();
}
void TestHarness::restart(){
    delete m_mce;
    m_data_manager = nullptr;

    Logger::getInstance().startNewSession();

    m_mce = new MonolithicControlEntity();
    m_data_manager = &m_mce->getDataManager();
    TimeSource simTimeSource = [this](){ return getSimTimeUs(); };
    DShot motor_interface = DShot(*m_data_manager);
    motor_interface.init();
    m_mce->initialize(simTimeSource, &motor_interface);
    // m_mce->transition_to(DisarmedState::instance());

    
}

void TestHarness::startSubscribers() {
    // Note: You may need to change these topic names depending on your Gazebo world.
    const std::string imuTopic = "/imu";
    const std::string gpsTopic = "/gps"; // You may need to add a GPS sensor to your SDF
    const std::string magTopic = "/magnetometer"; // You may need to add a magnetometer to your SDF
    const std::string clockTopic = "/world/default/clock";
    const std::string statsTopic = "/stats";
    
    // Subscribe to topics
    if (!m_node.Subscribe(imuTopic, &TestHarness::imuCallback, this)) {
        std::cerr << "Error subscribing to topic [" << imuTopic << "]" << std::endl;
    }
    if (!m_node.Subscribe(magTopic, &TestHarness::magnetometerCallback, this)) {
        std::cerr << "Error subscribing to topic [" << magTopic << "]" << std::endl;
    }
    if (!m_node.Subscribe(gpsTopic, &TestHarness::gpsCallback, this)) {
        std::cerr << "Error subscribing to topic [" << gpsTopic << "]" << std::endl;
    }
    if (!m_node.Subscribe(clockTopic, &TestHarness::clockCallback, this)) {
        std::cerr << "Error subscribing to topic [" << clockTopic << "]" << std::endl;
    }
    if (!m_node.Subscribe(statsTopic, &TestHarness::statsCallback, this)) {
        std::cerr << "Error subscribing to topic [" << statsTopic << "]" << std::endl;
    }

    
}

void TestHarness::imuCallback(const gz::msgs::IMU& msg) {
    if (m_data_manager == nullptr)
        return;
    int64_t timestamp_us = msg.header().stamp().sec() * 1000000LL + msg.header().stamp().nsec() / 1000LL;

    IMUData imu_data;
    imu_data.Timestamp = timestamp_us;
    // Gazebo's body reference frame defines +x forward, +y out the left and +z upwards
    // We need to convert this to internal frame
    imu_data.Acceleration << msg.linear_acceleration().x(),
                              -msg.linear_acceleration().y(),
                              -msg.linear_acceleration().z();

    imu_data.AngularVelocity << msg.angular_velocity().x(),
                                -msg.angular_velocity().y(),
                                -msg.angular_velocity().z();
    m_data_manager->post(imu_data);
}

void TestHarness::gpsCallback(const gz::msgs::NavSat& msg) {
    if (m_data_manager == nullptr)
        return;
    int64_t timestamp_us = msg.header().stamp().sec() * 1000000LL + msg.header().stamp().nsec() / 1000LL;
    GPSData gpsData;
    gpsData.Timestamp = timestamp_us;
    gpsData.lla={msg.latitude_deg(),msg.longitude_deg(),msg.altitude()};
    gpsData.vel = {msg.velocity_north(), msg.velocity_east(),-msg.velocity_up()};
    m_data_manager->post(gpsData);
}

void TestHarness::magnetometerCallback(const gz::msgs::Magnetometer& msg) {
    if (m_data_manager == nullptr)
        return;
    int64_t timestamp_us = msg.header().stamp().sec() * 1000000LL + msg.header().stamp().nsec() / 1000LL;
    MagData magData;
    magData.Timestamp = timestamp_us;
    magData.MagneticField << msg.field_tesla().x(),
                             msg.field_tesla().y(),
                             msg.field_tesla().z();
    m_data_manager->post(magData);
}

void TestHarness::clockCallback(const gz::msgs::Clock& msg) {
    uint64_t time_us = msg.sim().sec() * 1000000LL + msg.sim().nsec() / 1000LL;
    if (time_us < m_sim_time_us.load()) {
        m_should_restart.store(true);
    }
    m_sim_time_us.store(time_us);
}

void TestHarness::statsCallback(const gz::msgs::WorldStatistics& msg) {
    m_is_paused.store(msg.paused());
}

uint64_t TestHarness::getSimTimeUs() const {
    return m_sim_time_us.load();
}