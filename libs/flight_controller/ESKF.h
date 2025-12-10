#include "DataManager.h"
#include "SensorData.h"
#include "Consumer.h"
#include <Eigen/Dense>

class ESKF{
public:
    ESKF(DataManager& data_manager)
    void run();
private:

    Consumer<IMUData, IMU_BUFFER_SIZE> m_gyro_consumer;
    Consumer<MagData, MAG_BUFFER_SIZE> m_mag_consumer;
    Consumer<GPSData, GPS_BUFFER_SIZE> m_gps_consumer;

    void updateIMU(const IMUData& imu_data);
    void updateMag(const MagData& mag_data);
    void updateGPS(const GPSData& gps_data);
    void correctionStep();


};