#include "KinematicAdjudicator.h"

KinematicAdjuticator::KinematicAdjuticator() : 
m_estimator(model),
m_indicator(model)
{
    
};

void KinematicAdjuticator::update(){
 // Method to be called upon receipt of new imu data,
 // 
 // Filter new imu measurement
 
 // if state is learning
        // give to estimator
        // calculate new excitation command
        //send to motors
 // else
    // give to indi controller
    // get output from indi controller
    // send to motors

};