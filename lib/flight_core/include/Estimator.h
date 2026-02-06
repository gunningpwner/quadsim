#include "QuadModel.h"
class Estimator
{
public:
    Estimator(QuadcopterModel& model);
private:
    void update_motor_estimate();
    void update_control_estimate();

    template <int InputDim, int OutputDim>
    void apply_rls(Eigen::Matrix<float, InputDim, 1> &X, 
        Eigen::Matrix<float, OutputDim, 1> &Y,
        Eigen::Matrix<float, InputDim, OutputDim> &O,
        Eigen::Matrix<float, OutputDim, OutputDim> &P)
        {
            float current_lambda=base_lambda;
            // do check step here

            Eigen::Matrix<float, InputDim, OutputDim> K= P*X / (current_lambda+X.transpose()*P*X);
            P =(P-K*X.transpose*P)/current_lambda;
            Eigen::Matrix<float, OutputDim, 1> e=Y-X.T*O;
            O+= K*e;
        }

    
    QuadcopterModel& model;
    FilteredSignal& omega_sig;
    FilteredSignal& imu_sig;
    FilteredSignal& control_sig;
    float base_lambda;
};