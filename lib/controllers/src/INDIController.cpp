#include "INDIController.h"
#include <Eigen/QR>
#include "Logger.h"
#include "timing.h"
INDIController::INDIController(QuadcopterModel &model) : 
model(model) {
    ref_command.setZero();
    u_est.setZero();
    last_u.setZero();
};

void INDIController::setCommand( Vector3f ang_acc_in, float lin_acc_in)
{
    // Quadcopter can only produce acceleration in body z, so we just ignore any other command
    ref_command[2] = lin_acc_in;
    ref_command.tail<3>() = ang_acc_in;
}


Vector4f INDIController::run()
{
    // time for some fucky math :D
    Vector6f v_meas;
    // v_meas needs to be lin_acc and ang_acc
    // so we need first 3 states of imu_sig val and last 3 states of imu_sig dot ..... i think
    v_meas << model.imu_sig.val.head<3>(), model.imu_sig.dot.tail<3>();
    Vector6f lhs = (ref_command-v_meas)+model.B2*model.omega_sig.dot;

    // most of this doesn't change so i can probably just cache them and update whenever the estimate changes.
    // either have the kinematic adjudicator call to update cache or add a timestamp to model to demark when parameters were updated
    Vector4f omega_squared =model.motor_omega_max.array().square().matrix();
    Eigen::Matrix<float, 6, 4> top_half = model.B2*omega_squared.asDiagonal();
    Vector4f bottom_half = 2*model.omega_sig.val.cwiseProduct(model.motor_tau);
    Eigen::Matrix<float, 6, 4> rhs = model.B1*omega_squared.asDiagonal() + (top_half.array().rowwise()/bottom_half.transpose().array()).matrix();
    Eigen::Matrix<float, 4, 6> pinv = rhs.completeOrthogonalDecomposition().pseudoInverse();
    Vector4f delta_u = pinv*lhs;
    // TODO: Figure out how to actually track the time
    u_est = u_est+model.motor_tau.cwiseInverse().cwiseProduct(last_u-u_est)*0.0005f;
    Vector4f u_cmd = u_est+delta_u;
    // Clip between 0 and 1
    u_cmd = u_cmd.cwiseMax(0.0f).cwiseMin(1.0f);
    last_u = u_cmd;
    Vector4f pwm_cmd;

    for (int i = 0; i < 4; i++)
    {
        pwm_cmd[i] = solveForDelta(u_cmd[i], model.motor_kappa[i]);
    }
    
    #ifdef SIM
    //     Logger::getInstance().log("B2", model.B2, getCurrentTimeUs());
    //     Logger::getInstance().log("B1", model.B1, getCurrentTimeUs());
        // Logger::getInstance().log("v_meas", v_meas, getCurrentTimeUs());
    //     Logger::getInstance().log("lhs", lhs, getCurrentTimeUs());
    //     Logger::getInstance().log("rhs", rhs, getCurrentTimeUs());
    //     Logger::getInstance().log("ref", ref_command, getCurrentTimeUs());
    //     Logger::getInstance().log("omegadot", model.omega_sig.dot, getCurrentTimeUs());
    //     Logger::getInstance().log("omega", model.omega_sig.val, getCurrentTimeUs());
    //     Logger::getInstance().log("pinv", pinv, getCurrentTimeUs());
        // Logger::getInstance().log("delta_u", delta_u, getCurrentTimeUs());
        // Logger::getInstance().log("u_est", u_est, getCurrentTimeUs());

    #endif
    return pwm_cmd;
}

float INDIController::solveForDelta(float u_cmd, float kappa)
{
    if (std::abs(u_cmd) < 1e-6f) return 0.0f;
    float a = kappa;
    float b = 1-kappa;
    float c = -std::sqrt(u_cmd);
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return 0.0f;
    float sqrt_delta = (-b + std::sqrt(discriminant)) / (2 * a);
    return sqrt_delta * sqrt_delta;

}
