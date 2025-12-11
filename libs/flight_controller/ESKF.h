#include "DataManager.h"
#include "SensorData.h"
#include "Consumer.h"
#include <Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Vector3f;
using Eigen::Quaternionf;

class ESKF{
public:
    ESKF(DataManager& data_manager);
    void run();
private:

    Consumer<IMUData, IMU_BUFFER_SIZE> m_imu_consumer;
    Consumer<MagData, MAG_BUFFER_SIZE> m_mag_consumer;
    Consumer<GPSData, GPS_BUFFER_SIZE> m_gps_consumer;

    void updateIMU(const IMUData& imu_data);
    void updateMag(const MagData& mag_data);
    void updateGPS(const GPSData& gps_data);
    void correctionStep(MatrixXf H, MatrixXf V, VectorXf measurement, VectorXf prediction);

    Vector3f nominalPos;
    Vector3f nominalVel;
    Quaternionf nominalQuat;
    Vector3f nominalAccBias;
    Vector3f nominalGyroBias;
    Vector3f nominalGrav;


    MatrixXf errorStateCovariance;

    uint64_t last_timestamp;
    
    float accVar = .01;
    float accBiasVar = .0001;
    float gyroVar = .01;
    float gyroBiasVar=.0001;
    float gpsPosVar=.25;
    float gpsVelVar=.01;
    float magVar=.0025;
};