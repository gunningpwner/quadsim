#ifndef HAL_INTERFACE_H
#define HAL_INTERFACE_H

#include "../common/types.h"

class IHAL {
public:
    virtual ~IHAL() {}
    virtual SensorData read_sensors() = 0;
    virtual UserInput read_user_input() = 0;
};

#endif // HAL_INTERFACE_H
