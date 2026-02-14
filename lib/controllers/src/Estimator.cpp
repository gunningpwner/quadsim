#include "Estimator.h"
#include <cmath>

Estimator::Estimator(QuadcopterModel& model) 
    : model(model), 
      last_timestamp_us(0),
      current_dt(0.0005f), // Default small dt
      lambda_tau(0.2f)
{
    // Initialize Covariances
    for(int i=0; i<4; i++){
        model.rls_motor_covariances[i].setIdentity();
        model.rls_motor_covariances[i] *= RLS_COV_INIT;
        model.rls_motor_estimates[i].setZero();
    }

    P_B1.setIdentity();
    P_B1 *= RLS_COV_INIT;
    
    P_B2.setIdentity();
    P_B2 *= RLS_COV_INIT;

    model.B1.setZero();
    model.B2.setZero();
}

void Estimator::run(uint64_t timestamp_us, const Vector4f& control_u, const Vector4f& omega, const Eigen::Matrix<float, 6, 1>& imu_data)
{
    if (last_timestamp_us != 0) {
        current_dt = (timestamp_us - last_timestamp_us) * 1e-6f;
        if (current_dt < 1e-6f) current_dt = 1e-6f; // Prevent div by zero
    }
    last_timestamp_us = timestamp_us;

    // 1. Update Signals (computes filtered values, diffs, and derivatives)
    model.control_sig.update(control_u, timestamp_us);
    model.omega_sig.update(omega, timestamp_us);
    model.imu_sig.update(imu_data, timestamp_us);

    // 2. Run Estimates
    // We need valid derivatives (requires at least 2 samples for diff, 3 for dot_diff)
    // The FilteredSignal handles startup gracefully, but RLS might need to wait 
    // for non-zero derivatives to be stable.
    if (last_timestamp_us > 0) {
        update_motor_estimate();
        update_control_estimate();
    }
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

    // If we don't have a valid omega derivative yet, skip
    if (omega_dot.isZero(1e-5f)) return;

    for(int i = 0; i < 4; i++) {
        // Construct Regressor
        Eigen::Matrix<float, 4, 1> X;
        float sqrt_u = (u(i) > 0) ? std::sqrt(u(i)) : 0.0f;
        
        X << u(i), 
             sqrt_u, 
             1.0f, 
             -omega_dot(i);

        float Y = omega(i);

        apply_miso_rls<4>(X, Y, model.rls_motor_estimates[i], model.rls_motor_covariances[i]);
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
    if (X_B1.squaredNorm() < 1e-6f) return;

    // Y_B1 = Delta Specific Force
    Eigen::Vector3f Y_B1 = model.imu_sig.diff.head<3>();

    apply_mimo_rls<4, 3>(X_B1, Y_B1, model.B1, P_B1);


    // B2: Maps Motor Acceleration to Body Angular Acceleration
    // Model: alpha_diff ~ B1_term + B2 * (omega_dot_diff)
    // The Python code constructs a joint regressor for B2 including the B1 term, 
    // effectively refining B1 info or just using the full state for the rotational dynamics.
    
    // Regressor X_B2 = [X_B1, Delta_Omega_Dot] (Size 8)
    Eigen::Matrix<float, 8, 1> X_B2;
    X_B2.head<4>() = X_B1; // Reuse B1 regressor
    X_B2.tail<4>() = model.omega_sig.dot_diff; // Change in motor acceleration

    // Y_B2 = Change in Angular Acceleration
    // imu_sig contains [acc, gyro]. 
    // dot_diff of imu_sig is [jerk_lin, alpha_diff]. We want tail(3).
    Eigen::Vector3f Y_B2 = model.imu_sig.dot_diff.tail<3>();

    apply_mimo_rls<8, 3>(X_B2, Y_B2, model.B2, P_B2);
}