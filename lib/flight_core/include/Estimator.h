#include "QuadModel.h"
class Estimator
{
public:
    Estimator(QuadcopterModel& model);
private:
    QuadcopterModel& model;
    void update_motor_estimate();
    void update_control_estimate();
    template <int MeasDim, int StateDim>
    void apply_rls()
};