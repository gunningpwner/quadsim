#include "ESKF.h"
#include "WMM.h"
#include <cmath>
#include <Eigen/Geometry>
#ifdef SIM
#include "Logger.h"
#endif
#include "timing.h"

#define R_M 6335439.0
#define R_N 6378137.0

using Eigen::seq;

Quaternionf axisAngleToQuaternion(const Vector3f &angularVel, float dt)
{
    float theta = angularVel.norm() * dt;
    if (theta < 1e-6)
        return Quaternionf::Identity();

    float k = sin(theta / 2) / theta;
    Quaternionf ret = Quaternionf();
    ret.w() = cos(theta / 2);
    ret.vec() = k * angularVel;
    return ret;
}

template <typename Derived>
Eigen::Matrix3f skew(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix3f m;
    m << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return m;
}

ESKF::ESKF(DataManager::SensorConsumer m_sensor_consumer, DataManager::StateBuffer &state_buffer) : m_sensor_consumer(m_sensor_consumer),
                                                                                                    m_state_buffer(state_buffer),
                                                                                                    nominalPos(0, 0, 0),
                                                                                                    nominalVel(0, 0, 0),
                                                                                                    nominalQuat(1, 0, 0, 0),
                                                                                                    nominalAccBias(0, 0, 0),
                                                                                                    nominalGyroBias(0, 0, 0),
                                                                                                    nominalGrav(0, 0, -9.81),
                                                                                                    refLLA(0, 0, 0),
                                                                                                    last_timestamp(0),
                                                                                                    errorStateCovariance(18, 18)
{
    errorStateCovariance.setZero();
    errorStateCovariance(0, 0) = 1;
    errorStateCovariance(1, 1) = 1;
    errorStateCovariance(2, 2) = 1;

    errorStateCovariance(3, 3) = 5;
    errorStateCovariance(4, 4) = 5;
    errorStateCovariance(5, 5) = 5;

    errorStateCovariance(6, 6) = 1;
    errorStateCovariance(7, 7) = 1;
    errorStateCovariance(8, 8) = 1;

    errorStateCovariance(9, 9) = .1;
    errorStateCovariance(10, 10) = .1;
    errorStateCovariance(11, 11) = .1;

    errorStateCovariance(12, 12) = .1;
    errorStateCovariance(13, 13) = .1;
    errorStateCovariance(14, 14) = .1;

    errorStateCovariance(15, 15) = .05;
    errorStateCovariance(16, 16) = .05;
    errorStateCovariance(17, 17) = .05;
}

void ESKF::run()
{
    m_sensor_consumer.reset();
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
        sensor_data = m_sensor_consumer.readNext();
    }


// printf("LLA %f %f %f\n", refLLA.x(), refLLA.y(), refLLA.z());
// printf("Pos %f %f %f\n", nominalPos.x(), nominalPos.y(), nominalPos.z());
// printf("Vel %f %f %f\n", nominalVel.x(), nominalVel.y(), nominalVel.z());
#ifdef SIM
    Logger::getInstance().log("LLA", refLLA, getCurrentTimeUs());
    Logger::getInstance().log("Pos", nominalPos, getCurrentTimeUs());
    Logger::getInstance().log("Vel", nominalVel, getCurrentTimeUs());
    VectorXf meas(4);
    meas << nominalQuat.x(), nominalQuat.y(), nominalQuat.z(), nominalQuat.w();
    Logger::getInstance().log("Quat", meas, getCurrentTimeUs());
    Logger::getInstance().log("AccBias", nominalAccBias, getCurrentTimeUs());
    Logger::getInstance().log("GyroBias", nominalGyroBias, getCurrentTimeUs());
    Logger::getInstance().log("Grav", nominalGrav, getCurrentTimeUs());
#endif
}

void ESKF::updateIMU(const SensorData &imu_data)
{
    float dt = (imu_data.timestamp - last_timestamp) / 1e6;
    if (dt < 2e-6)
        return;
    // If dt is quite large between IMU updates, we probably missed some
    // we'll cap dt to avoid accidentally blowing up the state
    if (dt > .01)
        dt = .01;
    last_timestamp = imu_data.timestamp;
    Eigen::Map<const Eigen::Vector3f> gyro(imu_data.data.imu.gyro);
    Eigen::Map<const Eigen::Vector3f> accel(imu_data.data.imu.accel);

    Quaternionf delta_quat = axisAngleToQuaternion(gyro, dt);
    nominalQuat *= delta_quat;
    
    Vector3f accCorr = accel - nominalAccBias;
    Vector3f gyroCorr = gyro - nominalGyroBias;
    Eigen::Matrix3f rot_mat = nominalQuat.toRotationMatrix();
    nominalPos += nominalVel * dt + (rot_mat * accCorr + nominalGrav) * dt * dt / 2;
    nominalVel += rot_mat * accCorr * dt + nominalGrav * dt;

    Eigen::Matrix3f deltaRot = Eigen::AngleAxisf(gyroCorr.norm() * dt, gyroCorr.normalized()).toRotationMatrix();

    MatrixXf Fx(18, 18);
    Fx.setZero();
    Fx(0, 0) = 1;
    Fx(1, 1) = 1;
    Fx(2, 2) = 1;
    Fx(3, 3) = 1;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(0, 3) = dt;
    Fx(1, 4) = dt;
    Fx(2, 5) = dt;

    Fx(seq(3, 5), seq(6, 8)) = -dt * rot_mat * skew(accCorr);
    Fx(seq(3, 5), seq(9, 11)) = -rot_mat * dt;

    Fx(3, 15) = dt;
    Fx(4, 16) = dt;
    Fx(5, 17) = dt;

    Fx(seq(6, 8), seq(6, 8)) = deltaRot.transpose();
    Fx(6, 12) = -dt;
    Fx(7, 13) = -dt;
    Fx(8, 14) = -dt;

    Fx(seq(9, 17), seq(9, 17)) = MatrixXf::Identity(9, 9);

    MatrixXf Fi(18, 12);
    Fi.setZero();
    Fi(seq(3, 5), seq(0, 2)) = MatrixXf::Identity(3, 3);
    Fi(seq(6, 8), seq(3, 5)) = MatrixXf::Identity(3, 3);
    Fi(seq(9, 11), seq(6, 8)) = MatrixXf::Identity(3, 3);
    Fi(seq(12, 14), seq(9, 11)) = MatrixXf::Identity(3, 3);

    MatrixXf Q(12, 12);
    Q.setZero();
    Q(0, 0) = accVar * dt * dt;
    Q(1, 1) = accVar * dt * dt;
    Q(2, 2) = accVar * dt * dt;
    Q(3, 3) = gyroVar * dt * dt;
    Q(4, 4) = gyroVar * dt * dt;
    Q(5, 5) = gyroVar * dt * dt;
    Q(6, 6) = accBiasVar * dt * dt;
    Q(7, 7) = accBiasVar * dt * dt;
    Q(8, 8) = accBiasVar * dt * dt;
    Q(9, 9) = gyroBiasVar * dt * dt;
    Q(10, 10) = gyroBiasVar * dt * dt;
    Q(11, 11) = gyroBiasVar * dt * dt;

    errorStateCovariance = Fx * errorStateCovariance * Fx.transpose() + Fi * Q * Fi.transpose();
#ifdef SIM
    VectorXf meas(6);
    meas << imu_data.Acceleration, imu_data.AngularVelocity;
    Logger::getInstance().log("IMU", meas, imu_data.Timestamp);
#endif
}

void ESKF::updateMag(const SensorData &mag_data)
{
    Eigen::Matrix3f rot_mat = nominalQuat.toRotationMatrix();
    float inc, dec;
    calcIncAndDec(refLLA.x(), refLLA.y(), inc, dec);
    Vector3f ref_mag = {cos(inc) * cos(dec), cos(inc) * sin(dec), sin(inc)};
    Vector3f pred_mag = rot_mat.transpose() * ref_mag;
    MatrixXf H(3, 18);
    H.setZero();
    H(seq(0, 2), seq(6, 8)) = skew(pred_mag);
    MatrixXf V(3, 3);
    V.setZero();
    V(0, 0) = magVar;
    V(1, 1) = magVar;
    V(2, 2) = magVar;
    Eigen::Map<const Eigen::Vector3f> meas(mag_data.data.mag.mag);
    meas.normalize();
    correctionStep(H, V, meas, pred_mag);
#ifdef SIM
    Logger::getInstance().log("MAG", mag_data.MagneticField, mag_data.Timestamp);
    Logger::getInstance().log("MAG_Pred", pred_mag, getCurrentTimeUs());
    Logger::getInstance().log("MAG_Ref", ref_mag, getCurrentTimeUs());
#endif
}

void ESKF::updateGPS(const SensorData &gps_data)
{
    MatrixXf H_x(6, 19);
    H_x.setZero();
    H_x(seq(0, 5), seq(0, 5)) = MatrixXf::Identity(6, 6);

    MatrixXf X_deltax(19, 18);
    X_deltax.setZero();
    X_deltax(seq(0, 5), seq(0, 5)) = MatrixXf::Identity(6, 6);

    X_deltax(6, seq(6, 8)) << -nominalQuat.x(), -nominalQuat.y(), -nominalQuat.z();
    X_deltax(7, seq(6, 8)) << nominalQuat.w(), -nominalQuat.z(), nominalQuat.y();
    X_deltax(8, seq(6, 8)) << nominalQuat.z(), nominalQuat.w(), -nominalQuat.x();
    X_deltax(9, seq(6, 8)) << -nominalQuat.y(), nominalQuat.x(), nominalQuat.w();
    X_deltax(seq(6, 9), seq(6, 8)) /= 2;

    MatrixXf H = H_x * X_deltax;
    MatrixXf V = MatrixXf::Identity(6, 6);
    V(seq(0, 2), seq(0, 2)) *= gpsPosVar;
    V(seq(3, 5), seq(3, 5)) *= gpsVelVar;
    if (refLLA.isZero())
        refLLA=Eigen::Map<const Eigen::Vector3f>(gps_data.data.gps.lla);
    else
    {
        Vector3f diff = Eigen::Map<const Eigen::Vector3f>(gps_data.data.gps.lla) - refLLA;
        float e_coef = (R_N + refLLA.z() * cos(refLLA.y()));
        float n_coef = (R_M + refLLA.z());
        // Should be NED
        Vector3f ned = Vector3f(n_coef * diff.x(),
                                e_coef * diff.y(),
                                -diff.z());

        VectorXf meas(6);
        meas << ned, gps_data.data.gps.vel;
        VectorXf pred(6);
        pred << nominalPos, nominalVel;
        correctionStep(H, V, meas, pred);
        refLLA.x() += nominalPos.y() / e_coef;
        refLLA.y() += nominalPos.x() / n_coef;
        refLLA.z() += -nominalPos.z();
        nominalPos.setZero();
    }
#ifdef SIM
    VectorXf meas(6);
    meas << gps_data.lla, gps_data.vel;
    Logger::getInstance().log("GPS", meas, gps_data.Timestamp);
#endif
}

void ESKF::correctionStep(MatrixXf H, MatrixXf V, VectorXf measurement, VectorXf prediction)
{
    MatrixXf K = errorStateCovariance * H.transpose() * (H * errorStateCovariance * H.transpose() + V).inverse();
    VectorXf errorStateMean = K * (measurement - prediction);
    errorStateCovariance = (MatrixXf::Identity(18, 18) - K * H) * errorStateCovariance;
    nominalPos += errorStateMean.head(3);
    nominalVel += errorStateMean.segment(3, 3);
    Quaternionf deltaQuat = axisAngleToQuaternion(errorStateMean.segment(6, 3), 1);
    nominalQuat *= deltaQuat;
    nominalQuat.normalize();
    nominalAccBias += errorStateMean.segment(9, 3);
    nominalGyroBias += errorStateMean.segment(12, 3);
    nominalGrav += errorStateMean.segment(15, 3);

    MatrixXf G = MatrixXf::Identity(18, 18);
    G(seq(6, 8), seq(6, 8)) -= skew(0.5f * errorStateMean.segment(6, 3));

    errorStateCovariance = G * errorStateCovariance * G.transpose();
}
