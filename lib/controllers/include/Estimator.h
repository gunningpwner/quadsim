#pragma once
#include "QuadModel.h"

class Estimator
{
public:
    Estimator(QuadcopterModel& model);
    
    // Main update loop
    void run(uint64_t timestamp_us, const Vector4f& control_u, const Vector4f& omega, const Eigen::Matrix<float, 6, 1>& imu_data);

private:
    QuadcopterModel& model;

    // Covariances for Control Effectiveness (kept internal to estimator)
    Eigen::Matrix<float, 4, 4> P_B1; // Covariance for B1
    Eigen::Matrix<float, 8, 8> P_B2; // Covariance for B2

    // Internal State
    uint64_t last_timestamp_us;
    float current_dt;
    
    // Hyperparameters
    float lambda_tau; // Time constant for forgetting factor (0.2s)
    static constexpr float RLS_COV_MAX = 1e10f;
    static constexpr float RLS_COV_INIT = 100.0f;

    void update_motor_estimate();
    void update_control_estimate();

    // --- RLS Helper Functions ---

    /**
     * MISO RLS: Multiple Input, Single Output
     * Used for individual motor parameter estimation.
     * X: Regressor Vector (N x 1)
     * y: Scalar measurement
     * theta: Parameter Vector (N x 1)
     * P: Covariance Matrix (N x N)
     */
    template <int N>
    void apply_miso_rls(const Eigen::Matrix<float, N, 1> &X, 
                        float y,
                        Eigen::Matrix<float, N, 1> &theta,
                        Eigen::Matrix<float, N, N> &P)
    {
        // Calculate dynamic forgetting factor
        float lambda = std::exp(current_dt / -lambda_tau); // lambda = exp(-dt/tau) approx exp(dt/0.2) logic inverted for decay
        // Note: Python code uses lambda = exp(dt/0.2) which is > 1. 
        // Standard RLS lambda is usually < 1 (forgetting). 
        // However, Python implementation uses lambda in the denominator of P update: P = (...) / lambda.
        // If lambda > 1, P decays. 
        // We will match the Python logic: lambda = exp(dt / 0.2f);
        lambda = std::exp(current_dt / 0.2f);

        // Covariance Reset check
        if (P.diagonal().maxCoeff() > RLS_COV_MAX) {
             // Soft reset: Increase lambda to forget faster
             lambda = 1.0f + 0.1f * (1.0f - lambda); // Heuristic from Python
        }

        // K = P * X / (lambda + X' * P * X)
        // Denominator is scalar
        float denom = lambda + X.dot(P * X);
        Eigen::Matrix<float, N, 1> K = (P * X) / denom;

        // P = (P - K * X' * P) / lambda
        // Optimized: P = (I - K * X') * P / lambda
        P = (P - K * X.transpose() * P) / lambda;

        // Error e = y - X' * theta
        float e = y - X.dot(theta);

        // theta = theta + K * e
        theta += K * e;
    }

    /**
     * MIMO RLS: Multiple Input, Multiple Output (Parallel RLS)
     * Used for B1 and B2 estimation where the regressor X is shared across outputs.
     * X: Regressor Vector (InputDim x 1)
     * Y: Measurement Vector (OutputDim x 1)
     * Theta: Parameter Matrix (InputDim x OutputDim)
     * P: Covariance Matrix (InputDim x InputDim) - Shared for all outputs
     */
    template <int InputDim, int OutputDim>
    void apply_mimo_rls(const Eigen::Matrix<float, InputDim, 1> &X, 
                        const Eigen::Matrix<float, OutputDim, 1> &Y,
                        Eigen::Matrix<float, InputDim, OutputDim> &Theta,
                        Eigen::Matrix<float, InputDim, InputDim> &P)
    {
        float lambda = std::exp(current_dt / 0.2f);

        if (P.diagonal().maxCoeff() > RLS_COV_MAX) {
             lambda = 1.0f + 0.1f * (1.0f - lambda);
        }

        // K is (InputDim x 1)
        float denom = lambda + X.dot(P * X);
        Eigen::Matrix<float, InputDim, 1> K = (P * X) / denom;

        // P Update (Shared)
        P = (P - K * X.transpose() * P) / lambda;

        // Error Vector (1 x OutputDim) = Y.T - X.T * Theta
        // Transpose logic: e = Y - Theta.T * X is (OutputDim x 1)
        // Theta columns are outputs. 
        Eigen::Matrix<float, OutputDim, 1> prediction = Theta.transpose() * X;
        Eigen::Matrix<float, OutputDim, 1> e = Y - prediction;

        // Theta Update: Theta += K * e.T
        // (InputDim x 1) * (1 x OutputDim) -> (InputDim x OutputDim)
        Theta.noalias() += K * e.transpose();
    }
};