#include "ESKF.h"
#include "WMM.h"
#include <cmath>
#include <Eigen/Geometry>
#include <cstdint>
#include <iostream>
#define DEG2RAD (M_PI / 180.0f)


#define R_M 6335439.0f
#define R_N 6378137.0f

using Eigen::seq;
using Vec3Map = Eigen::Map<const Eigen::Vector3f>;


template <typename Derived>
Eigen::Matrix3f skew(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix3f m;
    m << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return m;
}

Quaternionf axisAngleToQuaternion(const Eigen::Vector3f &angularVel, float dt)
{
    float theta = angularVel.norm() * dt;
    if (theta < 1e-6f)
        return Quaternionf::Identity();

    float k = sinf(theta / 2.0f) / theta;
    Quaternionf ret;
    ret.w() = cosf(theta / 2.0f);
    ret.vec() = k * angularVel;
    return ret;
}

ESKF::ESKF(DataManager::SensorConsumer m_sensor_consumer, DataManager::StateBuffer &state_buffer)
    : m_sensor_consumer(m_sensor_consumer),
      m_state_buffer(state_buffer),
      nominalPos(0, 0, 0),
      nominalVel(0, 0, 0),
      nominalQuat(1, 0, 0, 0),
      nominalAccBias(0, 0, 0),
      nominalGyroBias(0, 0, 0),
      nominalGrav(0, 0, 9.81f),
      refLLA(0, 0, 0),
      last_timestamp(0)
{
    // Constructor initialization of matrices is implicit via default constructors
    // setZero is faster than initialization lists for arrays
    errorStateCovariance.setZero();
    Fx.setZero();

    // Initialize diagonals
    errorStateCovariance.diagonal().head<3>().setConstant(1.0f);       // Pos
    errorStateCovariance.diagonal().segment<3>(3).setConstant(1.0f);   // Vel
    errorStateCovariance.diagonal().segment<3>(6).setConstant(10.0f);   // Angle
    errorStateCovariance.diagonal().segment<3>(9).setConstant(0.1f);   // Acc Bias
    errorStateCovariance.diagonal().segment<3>(12).setConstant(0.1f);  // Gyro Bias
    errorStateCovariance.diagonal().segment<3>(15).setConstant(0.0f); // Gravity

    // Initialize Fx identity parts
    Fx.diagonal().head<6>().setConstant(1.0f); // Pos, Vel
    Fx.block<9, 9>(9, 9).setIdentity();        // Biases, Grav
}

void ESKF::run()
{
    SensorData *sensor_data = m_sensor_consumer.readNext();
    while (sensor_data != nullptr)
    {
        switch (sensor_data->sensor)
        {
        case SensorData::Type::IMU:
            updateIMU(*sensor_data);
            break;
        case SensorData::Type::GPS:
            updateGPS(*sensor_data);
            break;
        case SensorData::Type::MAG:
            updateMag(*sensor_data);
            break;
        default:
            break;
        }
        #ifdef SIM
            // Logging code remains mostly the same, casting to VectorXf only where needed for the logger interface
            Logger::getInstance().log("LLA", refLLA, getCurrentTimeUs());
            Logger::getInstance().log("Pos", nominalPos, getCurrentTimeUs());
            Logger::getInstance().log("Vel", nominalVel, getCurrentTimeUs());
            Eigen::Vector4f quatMeas;
            quatMeas << nominalQuat.x(), nominalQuat.y(), nominalQuat.z(), nominalQuat.w();
            Logger::getInstance().log("Quat", quatMeas, getCurrentTimeUs());
            Logger::getInstance().log("AccBias", nominalAccBias, getCurrentTimeUs());
            Logger::getInstance().log("GyroBias", nominalGyroBias, getCurrentTimeUs());
            Logger::getInstance().log("Grav", nominalGrav, getCurrentTimeUs());
            Logger::getInstance().log("Cov", errorStateCovariance, getCurrentTimeUs());
        #endif
        
        sensor_data = m_sensor_consumer.readNext();
    }


}

void ESKF::updateIMU(const SensorData &imu_data)
{

    float dt = (imu_data.timestamp - last_timestamp) / 1e6f;
    if (dt < 2e-6f)
        return;
    if (last_timestamp == 0){
        last_timestamp = imu_data.timestamp;
        return;
    }

    last_timestamp = imu_data.timestamp;
    Eigen::Vector3f gyro(imu_data.data.imu.gyro.data());
    gyro*=DEG2RAD;
    Vec3Map accel(imu_data.data.imu.accel.data());

    Eigen::Vector3f accCorr = accel - nominalAccBias;
    Eigen::Vector3f gyroCorr = gyro - nominalGyroBias;
    Quaternionf delta_quat = axisAngleToQuaternion(gyroCorr, dt);

    Eigen::Matrix3f rot_mat = nominalQuat.toRotationMatrix();

    nominalQuat *= delta_quat;
    nominalQuat.normalize();
    nominalPos += nominalVel * dt + (rot_mat * accCorr + nominalGrav) * (0.5f * dt * dt);
    nominalVel += (rot_mat * accCorr + nominalGrav) * dt;

    Fx.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;

    Fx.block<3, 3>(3, 6) = -rot_mat * skew(accCorr) * dt;
    Fx.block<3, 3>(3, 9) = -rot_mat * dt;

    Fx.block<3, 3>(3, 15) = Eigen::Matrix3f::Identity() * dt;

    Eigen::Matrix3f deltaRot = Eigen::AngleAxisf(gyroCorr.norm() * dt, gyroCorr.normalized()).toRotationMatrix();
    Fx.block<3, 3>(6, 6) = deltaRot.transpose();
    Fx.block<3, 3>(6, 12) = -Eigen::Matrix3f::Identity() * dt;

    errorStateCovariance = Fx * errorStateCovariance * Fx.transpose();

    float dt2 = dt * dt;

    // Velocity Noise (from Accelerometer White Noise)
    errorStateCovariance.diagonal().segment<3>(3).array() += accVar * dt2;

    // Attitude Noise (from Gyro White Noise)
    errorStateCovariance.diagonal().segment<3>(6).array() += gyroVar * dt2;

    // Accel Bias Noise (Random Walk)
    errorStateCovariance.diagonal().segment<3>(9).array() += accBiasVar * dt;

    // Gyro Bias Noise (Random Walk)
    errorStateCovariance.diagonal().segment<3>(12).array() += gyroBiasVar * dt;

#ifdef SIM
    Eigen::VectorXf meas(6);
    meas << accel, gyro;
    Logger::getInstance().log("IMU", meas, imu_data.timestamp);
    Logger::getInstance().log("acc_rot", rot_mat * accCorr, imu_data.timestamp);
#endif
}

void ESKF::updateMag(const SensorData &mag_data)
{
    Eigen::Matrix3f rot_mat = nominalQuat.toRotationMatrix();
    // We haven't gotten a gps measurement yet
    // So we have no idea what the magnetometer measurement should look like
    if (refLLA.isZero())
        return;
    float inc, dec;
    calcIncAndDec(refLLA.x(), refLLA.y(), inc, dec);
    Eigen::Vector3f ref_mag = {cosf(inc) * cosf(dec), cosf(inc) * sinf(dec), sinf(inc)};

    // Eigen::Vector3f ref_mag = {1.0f, 0.0f, 0.0f};


    Eigen::Vector3f pred_mag = rot_mat.transpose() * ref_mag;

    Eigen::Matrix<float, 3, 18> H;
    H.setZero();
    H.block<3, 3>(0, 6) = skew(pred_mag);

    Eigen::Matrix3f V = Eigen::Matrix3f::Identity() * magVar;

    // Compass is mounted with a tilt.
    float raw_x = mag_data.data.mag.mag[0];
    float raw_y = mag_data.data.mag.mag[1];
    float raw_z = mag_data.data.mag.mag[2];
    // Approximation for now
    float tilt_deg = -0.0f;  
    float theta = tilt_deg * (3.14159265f / 180.0f);
    float corr_x = raw_x * cosf(theta) + raw_z * sinf(theta);
    float corr_y = raw_y; 
    float corr_z = -raw_x * sinf(theta) + raw_z * cosf(theta);

    Eigen::Vector3f meas;
    meas << corr_x, corr_y, corr_z;
    meas.normalize();


    correctionStep<3>(H, V, meas, pred_mag);

#ifdef SIM
    Logger::getInstance().log("MAG", meas, mag_data.timestamp);
    Logger::getInstance().log("MAG_Ref", ref_mag, getCurrentTimeUs());
    Logger::getInstance().log("MAG_Pred", pred_mag, getCurrentTimeUs() );
#endif
}

void ESKF::updateGPS(const SensorData &gps_data)
{

    Eigen::Matrix<float, 6, 18> H;
    H.setZero();
    H.block<6, 6>(0, 0).setIdentity();

    Eigen::Matrix<float, 6, 6> V = Eigen::Matrix<float, 6, 6>::Identity();
    V.block<3, 3>(0, 0) *= gpsPosVar;
    V.block<3, 3>(3, 3) *= gpsVelVar;

    if (refLLA.isZero())
    {
        refLLA = Vec3Map(gps_data.data.gps.lla.data());
    }
    else
    {
        Eigen::Vector3f diff = Vec3Map(gps_data.data.gps.lla.data()) - refLLA;
        float e_coef = (R_N + refLLA.z()) * cosf(refLLA.y()* DEG2RAD);
        float n_coef = (R_M + refLLA.z());
        // NED conversion
        Eigen::Vector3f ned(n_coef * diff.x()* DEG2RAD,
                            e_coef * diff.y()* DEG2RAD,
                            -diff.z());

        Eigen::Matrix<float, 6, 1> meas;
        meas << ned, Vec3Map(gps_data.data.gps.vel.data());

        Eigen::Matrix<float, 6, 1> pred;
        pred << nominalPos, nominalVel;

        correctionStep<6>(H, V, meas, pred);

        // Re-inject position error into Reference LLA to keep nominalPos small
        // (Standard strategy for LLA navigation)
        // refLLA.x() += nominalPos.x() / n_coef; // Lat (x) from North (y)
        // refLLA.y() += nominalPos.y() / e_coef; // Lon (y) from East (x)
        // refLLA.z() += -nominalPos.z();
        // nominalPos.setZero();
        #ifdef SIM
            Eigen::VectorXf logMeas(6);
            logMeas << ned, Vec3Map(gps_data.data.gps.vel.data());
            Logger::getInstance().log("GPS", logMeas, gps_data.timestamp);
        #endif
    }

}