#pragma once
#include "QuadModel.h"

using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector3f = Eigen::Vector3f;
using Quaternionf = Eigen::Quaternionf;

class INDIController
{
public:
    INDIController(QuadcopterModel &model);
    Vector4f run();
    void setCommand( Vector3f ang_acc_in, float lin_acc_in);
    void temporary(Quaternionf orientation, Vector3f body_rates);
private:
    float solveForDelta(float u_cmd, float kappa);
    QuadcopterModel &model;
    Vector6f ref_command; // lin acc and ang acc
    Vector4f last_u;
    Vector4f u_est;
};