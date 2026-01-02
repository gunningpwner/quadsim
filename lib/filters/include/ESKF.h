#pragma once
#include "DataManager.h"
#include "DataTypes.h"
#include <Eigen/Dense>
#include "timing.h"
#ifdef SIM
#include "Logger.h"
#endif

// Define fixed-size types to prevent mallocs
using Matrix18f = Eigen::Matrix<float, 18, 18>;
using Vector18f = Eigen::Matrix<float, 18, 1>;
using Vector3f = Eigen::Vector3f;
using Quaternionf = Eigen::Quaternionf;

class ESKF
{
public:
    ESKF(DataManager::SensorConsumer m_sensor_consumer, DataManager::StateBuffer &state_buffer);
    void run();

private:
    DataManager::SensorConsumer m_sensor_consumer;
    DataManager::StateBuffer &m_state_buffer;

    void updateIMU(const SensorData &imu_data);
    void updateMag(const SensorData &mag_data);
    void updateGPS(const SensorData &gps_data);

    // Templated correction step handles 3D (Mag) and 6D (GPS) measurements 
    // without dynamic allocation.
    template <int MeasDim>
    void correctionStep(const Eigen::Matrix<float, MeasDim, 18> &H, 
                        const Eigen::Matrix<float, MeasDim, MeasDim> &V, 
                        const Eigen::Matrix<float, MeasDim, 1> &measurement, 
                        const Eigen::Matrix<float, MeasDim, 1> &prediction)
    {
        // K = P * H.T * (H * P * H.T + V).inverse()
        // Intermediate expressions are automatically evaluated as fixed-size stack objects
        Eigen::Matrix<float, 18, MeasDim> K;
        Eigen::Matrix<float, MeasDim, MeasDim> S = H * errorStateCovariance * H.transpose() + V;
        
        // Use LDLT for stable, fast inversion of symmetric positive definite S
        K = errorStateCovariance * H.transpose() * S.ldlt().solve(Eigen::Matrix<float, MeasDim, MeasDim>::Identity());

        Eigen::Matrix<float, 18, 1> errorStateMean = K * (measurement - prediction);
        #ifdef SIM
        if constexpr (MeasDim == 3)
        {
        Logger::getInstance().log("K_MAG", K, getCurrentTimeUs());
        Logger::getInstance().log("Err_MAG", measurement - prediction, getCurrentTimeUs());
        }
        else
        {
            Logger::getInstance().log("K_GPS", K, getCurrentTimeUs());
            Logger::getInstance().log("Err_GPS", measurement - prediction, getCurrentTimeUs());
        }
        Logger::getInstance().log("errorStateMean", errorStateMean, getCurrentTimeUs());
        #endif
        // P = (I - K * H) * P
        // Optimized form: P = P - K * (H * P) to reduce operations
        // Or standard Joseph form for stability if needed, but simple form is faster:
        errorStateCovariance = (Matrix18f::Identity() - K * H) * errorStateCovariance;

        // Injection
        nominalPos += errorStateMean.template head<3>();
        nominalVel += errorStateMean.template segment<3>(3);
        
        // Small angle approximation for quaternion update
        Vector3f angleErr = errorStateMean.template segment<3>(6);
        if constexpr (MeasDim == 3) {
             Quaternionf deltaQuat(Eigen::AngleAxisf(angleErr.norm(), angleErr.normalized()));
             nominalQuat *= deltaQuat;
             nominalQuat.normalize();
             nominalGyroBias += errorStateMean.template segment<3>(12);
        }

        nominalAccBias += errorStateMean.template segment<3>(9);
        
        nominalGrav += errorStateMean.template segment<3>(15);
        // ESKF Reset
        Matrix18f G = Matrix18f::Identity();
        // Uses skew symmetric matrix of half the error
        Vector3f theta = 0.5f * errorStateMean.template segment<3>(6);
        Eigen::Matrix3f skewTheta;
        skewTheta << 0, -theta.z(), theta.y(),
                     theta.z(), 0, -theta.x(),
                     -theta.y(), theta.x(), 0;
                     
        G.template block<3,3>(6,6) -= skewTheta;

        errorStateCovariance = G * errorStateCovariance * G.transpose();
    }
    
    Vector3f nominalPos;
    Vector3f nominalVel;
    Quaternionf nominalQuat;
    Vector3f nominalAccBias;
    Vector3f nominalGyroBias;
    Vector3f nominalGrav;

    Vector3f refLLA;

    Matrix18f errorStateCovariance;
    Matrix18f Fx;

    uint64_t last_timestamp;


    float accVar = .00157f;
    float accBiasVar = .001f;
    float gyroVar = .000122f;
    float gyroBiasVar = .001f;
    float gpsPosVar = 10.0f;
    float gpsVelVar = 10.0f;
    float magVar = 1.0f;
};