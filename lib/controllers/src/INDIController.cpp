#include "INDIController.h"
#include <Eigen/QR>
#include "Logger.h"
#include "timing.h"
INDIController::INDIController(QuadcopterModel &model) : 
model(model) {
    ref_command.setZero();
};

void INDIController::setCommand(Vector3f lin_acc_in, Vector3f ang_acc_in)
{
    // Quadcopter can only produce acceleration in body z, so we just ignore any other command
    ref_command[2] = lin_acc_in[2];
    ref_command.tail<3>() = ang_acc_in;
}

void INDIController::run()
{
    // time for some fucky math :D
    Vector6f v_meas;
    // v_meas needs to be lin_acc and ang_acc
    // so we need first 3 states of imu_sig val and last 3 states of imu_sig dot ..... i think
    v_meas << 0,0,model.imu_sig.val[2], model.imu_sig.dot.tail<3>();
    Vector6f lhs = (ref_command-v_meas)+model.B2*model.omega_sig.dot;

    // most of this doesn't change so i can probably just cache them and update whenever the estimate changes.
    // either have the kinematic adjudicator call to update cache or add a timestamp to model to demark when parameters were updated
    Vector4f omega_squared =model.motor_parameters[0].array().square().matrix();
    Eigen::Matrix<float, 6, 4> top_half = model.B2*omega_squared.asDiagonal();
    Vector4f bottom_half = 2*model.omega_sig.val.cwiseProduct(model.motor_parameters[2]);
    Eigen::Matrix<float, 6, 4> rhs = model.B1*omega_squared.asDiagonal() + (top_half.array().rowwise()/bottom_half.transpose().array()).matrix();
    Eigen::Matrix<float, 4, 6> pinv = rhs.completeOrthogonalDecomposition().pseudoInverse();
    Vector4f delta_u = pinv*lhs;
    
    // #ifdef SIM
    //     Logger::getInstance().log("B2", model.B2, getCurrentTimeUs());
    //     Logger::getInstance().log("B1", model.B1, getCurrentTimeUs());
    //     Logger::getInstance().log("v_meas", v_meas, getCurrentTimeUs());
    //     Logger::getInstance().log("lhs", lhs, getCurrentTimeUs());
    //     Logger::getInstance().log("rhs", rhs, getCurrentTimeUs());
    //     Logger::getInstance().log("ref", ref_command, getCurrentTimeUs());
    //     Logger::getInstance().log("omegadot", model.omega_sig.dot, getCurrentTimeUs());
    //     Logger::getInstance().log("omega", model.omega_sig.val, getCurrentTimeUs());
    //     Logger::getInstance().log("pinv", pinv, getCurrentTimeUs());
    //     Logger::getInstance().log("delta_u", delta_u, getCurrentTimeUs());
    // #endif

}