#pragma once
#include "QuadModel.h"
#include "Estimator.h"
#include "INDIController.h"
#include "ExcitationGenerator.h"
#include "DataTypes.h"

class KinematicAdjudicator
{
public:


    KinematicAdjudicator();

    // Returns the final motor command to be sent to ESCs
    void update(FastIMUData *imu_data, FastRPMData *rpm_data, float *command);
    void setCommand(Eigen::Vector3f cmd, float acc){m_indicator.setCommand( cmd,acc);}
    void launch();
    FlightMode getCurrentMode() const { return model.current_mode; }

private:
    
    // Shared Data Store
    QuadcopterModel model;

    // Sub-modules
    ExcitationGenerator m_exciter;
    Estimator m_estimator;
    INDIController m_indicator;

    uint64_t last_timestamp_us;
};