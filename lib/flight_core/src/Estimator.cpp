#include "Estimator.h"

Estimator::Estimator(QuadcopterModel &model) : 
model(model) {};

void Estimator::update_motor_estimate(){
    // O =4x1, Y=1x1,X=4x1 P=4x4
    for (int i=0; i<4; i++){
        
    }
};

void Estimator::update_control_estimate() {
    // specific force
    // O=4x3 Y=3x1 X=4x1 P=4x4

    // weird stuff
    // O=8x3 Y=3x1 X=8x1 P=8x8
}