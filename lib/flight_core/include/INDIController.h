#pragma once
#include "QuadModel.h"

class INDIController
{
public:
    INDIController(QuadcopterModel &model);
    void run();
private:
    QuadcopterModel &model;
};