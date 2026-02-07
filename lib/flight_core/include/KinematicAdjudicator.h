#include "QuadModel.h"
#include "Estimator.h"
#include "INDIController.h"

class KinematicAdjuticator
{
public:
    KinematicAdjuticator();
    void update();
    
private:
    Estimator m_estimator;
    INDIController m_indicator;
    QuadcopterModel model;
};
