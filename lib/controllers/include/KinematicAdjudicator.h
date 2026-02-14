#pragma once
#include "QuadModel.h"
#include "Estimator.h"
#include "INDIController.h"
#include "ExcitationGenerator.h"

class KinematicAdjudicator
{
public:
    enum class FlightMode {
        LEARNING,
        RECOVERY, // Optional: Transition phase
        FLIGHT
    };

    KinematicAdjudicator();

    // Returns the final motor command to be sent to ESCs
    Eigen::Vector4f update(uint64_t timestamp_us, 
                           const Eigen::Vector4f& raw_omega, 
                           const Eigen::Vector3f& raw_acc, 
                           const Eigen::Vector3f& raw_gyro);

private:
    FlightMode current_mode;
    
    // Shared Data Store
    QuadcopterModel model;

    // Sub-modules
    ExcitationGenerator m_exciter;
    Estimator m_estimator;
    INDIController m_indicator;

    uint64_t last_timestamp_us;
};