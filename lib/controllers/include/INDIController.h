#pragma once
#include "QuadModel.h"

using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector3f = Eigen::Vector3f;

class INDIController
{
public:
    INDIController(QuadcopterModel &model);
    void run();
    void setCommand(Vector3f lin_acc_in, Vector3f ang_acc_in);
private:
    QuadcopterModel &model;
    Vector6f ref_command; // lin acc and ang acc
};