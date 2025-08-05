#ifndef HAL_INTERFACE_H
#define HAL_INTERFACE_H
#include <array>
#include "../common/types.h"

class IHAL {
public:
    virtual ~IHAL() {}
    virtual Vector3 read_gyros() = 0;
    virtual Vector3 read_accelerometer() = 0;
    virtual Vector3 read_magnetometer() = 0;
    virtual std::array<GPSData, 2> read_gps() = 0;
    virtual std::array<float, 4> read_motor_rpms() = 0;
    virtual UserInput read_user_input() = 0;
    virtual void write_motor_commands(const std::array<int, 4>& motor_commands) = 0;
    virtual void output_debug_data(Vector3 enuVelocity, Vector3 bodyOrientation) =0;
    bool newGPSData;
};

#endif // HAL_INTERFACE_H
