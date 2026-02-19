#include "Estimator.h"
#include <cmath>
#ifdef SIM
#include "Logger.h"
#endif
Estimator::Estimator(QuadcopterModel &model)
    : model(model),
      last_timestamp_us(0),
      current_dt(0.0005f), // Default small dt
      lambda_tau(0.2f)
{
    // Initialize Covariances
    for (int i = 0; i < 4; i++)
    {
        rls_motor_covariances[i].setIdentity();
        rls_motor_covariances[i] *= RLS_COV_INIT;
        rls_motor_estimates[i].setZero();
    }

    rls_spf_covariance.setIdentity();
    rls_spf_covariance *= RLS_COV_INIT;
    rls_spf_estimate.setZero();

    rls_ratedot_covariance.setIdentity();
    rls_ratedot_covariance *= RLS_COV_INIT;
    rls_ratedot_estimate.setZero();
}

void Estimator::run()
{

    update_motor_estimate();
    update_control_estimate();
}

void Estimator::update_motor_estimate()
{
    // Motor Model: omega = a*u + b*sqrt(u) + omega_idle - tau*omega_dot
    // Linear Form: Y = X.T * Theta
    // Y = omega
    // X = [u, sqrt(u), 1, -omega_dot]
    // Theta = [a, b, omega_idle, tau]

    Vector4f omega = model.omega_sig.val;
    Vector4f omega_dot = model.omega_sig.dot;
    Vector4f u = model.control_sig.val;
    // for(int i = 0; i < 4; i++) {
    if (model.current_mode == FlightMode::LEARNING)
    {
        int i = model.current_motor;
        // Construct Regressor
        Eigen::Matrix<float, 4, 1> X;
        float sqrt_u = (u(i) > 0) ? std::sqrt(u(i)) : 0.0f;

        X << u(i),
            sqrt_u,
            1.0f,
            -omega_dot(i);

        float Y = omega(i);
#ifdef SIM
        Logger::getInstance().log("Motor" + std::to_string(i + 1) + "Est", rls_motor_estimates[i], getCurrentTimeUs());
        Logger::getInstance().log("Motor" + std::to_string(i + 1) + "Cov", rls_motor_covariances[i], getCurrentTimeUs());
// Logger::getInstance().log("Motor1X", X,  getCurrentTimeUs());
// Logger::getInstance().log("Motor1Y", Y,  getCurrentTimeUs());
#endif
        apply_miso_rls<4>(X, Y, rls_motor_estimates[i], rls_motor_covariances[i]);
    }
}

void Estimator::update_control_estimate()
{
    // B1: Maps Thrust to Specific Force
    // Model: v_dot_lin ~ B1 * (omega^2 - omega_prev^2)
    // Target Y: Delta Specific Force (imu_sig.diff.head(3))
    // Regressor X: Delta Omega Squared (InputDim 4)

    Vector4f omega_curr = model.omega_sig.val;
    Vector4f omega_prev = model.omega_sig.prev_val;

    // X_B1 = omega_k^2 - omega_{k-1}^2
    Eigen::Matrix<float, 4, 1> X_B1 = omega_curr.array().square() - omega_prev.array().square();

    // Check for excitation
    if (X_B1.squaredNorm() < 1e-6f)
        return;

    // Y_B1 = Delta Specific Force
    Eigen::Vector3f Y_B1 = model.imu_sig.diff.head<3>();

    apply_mimo_rls<4, 3>(X_B1, Y_B1, rls_spf_estimate, rls_spf_covariance);

    // B2: Maps Motor Acceleration to Body Angular Acceleration
    // Model: alpha_diff ~ B1_term + B2 * (omega_dot_diff)
    // The Python code constructs a joint regressor for B2 including the B1 term,
    // effectively refining B1 info or just using the full state for the rotational dynamics.

    // Regressor X_B2 = [X_B1, Delta_Omega_Dot] (Size 8)
    Eigen::Matrix<float, 8, 1> X_B2;
    X_B2.head<4>() = X_B1;                     // Reuse B1 regressor
    X_B2.tail<4>() = model.omega_sig.dot_diff; // Change in motor acceleration

    // Y_B2 = Change in Angular Acceleration
    // imu_sig contains [acc, gyro].
    // dot_diff of imu_sig is [jerk_lin, alpha_diff]. We want tail(3).
    Eigen::Vector3f Y_B2 = model.imu_sig.dot_diff.tail<3>();

    apply_mimo_rls<8, 3>(X_B2, Y_B2, rls_ratedot_estimate, rls_ratedot_covariance);
    #ifdef SIM
        Logger::getInstance().log("ratedot_cov", rls_ratedot_covariance, getCurrentTimeUs());
        Logger::getInstance().log("ratedot", rls_ratedot_estimate, getCurrentTimeUs());
        Logger::getInstance().log("spf", rls_spf_estimate, getCurrentTimeUs());
        Logger::getInstance().log("spf_cov", rls_spf_covariance, getCurrentTimeUs());
        Logger::getInstance().log("spf_cov", rls_spf_covariance, getCurrentTimeUs());
    #endif
}

void Estimator::estimates_to_model()
{
    for (int i = 0; i < 4; i++)
    {
        // model has 3 vectors of length 4
        // vectors are omega_max, kappa and tau for each motor
        // rls_motor estimate vector is [a b omega_idle and tau]
        float a = rls_motor_estimates[i][0];
        float b = rls_motor_estimates[i][1];
        float tau = rls_motor_estimates[i][3];
        float omega_max = a + b;
        model.motor_omega_max[i] = omega_max;                                    
        model.motor_kappa[i] = (omega_max > 1e-4f) ? (a / omega_max) : 0.0f; 
        model.motor_tau[i] = tau;                                          
    }

    // B1k matrix is made by stacking rls_spf_estimate on top of first 4 rows of rls_ratedot_estimate
    model.B1.block<3, 4>(0, 0) = rls_spf_estimate.transpose();
    model.B1.block<3, 4>(3, 0) = rls_ratedot_estimate.block<4, 3>(0, 0).transpose();

    // B2k matrix is made by stacking zeros on top of last 4 rows of rls_ratedot_estimate
    model.B2.block<3, 4>(0, 0).setZero();
    model.B2.block<3, 4>(3, 0) = rls_ratedot_estimate.block<4, 3>(4, 0).transpose();
}