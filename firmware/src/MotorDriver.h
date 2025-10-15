#pragma once

#include "Consumer.h"
#include "DataManager.h"
#include "MotorCommands.h"
#include "DShot.h"
#include <array>

class MotorDriver : public Consumer<MotorCommands> {
public:
    MotorDriver(DataManager& dm, DShot& dshot_driver)
        : Consumer(dm, "motor_commands"), _dshot_driver(dshot_driver) {}

    void run() override {
        MotorCommands commands;
        if (get(commands)) {
            // Convert float commands [-1.0, 1.0] to DShot values [0, 1999]
            // Note: For now, we assume throttle is [0.0, 1.0]
            std::array<uint16_t, 4> dshot_values;

            // Standard quad X motor mapping:
            // Motor 1 (Rear Right)
            // Motor 2 (Front Right)
            // Motor 3 (Rear Left)
            // Motor 4 (Front Left)
            dshot_values[0] = static_cast<uint16_t>(commands.motor1 * 1999.0f);
            dshot_values[1] = static_cast<uint16_t>(commands.motor2 * 1999.0f);
            dshot_values[2] = static_cast<uint16_t>(commands.motor3 * 1999.0f);
            dshot_values[3] = static_cast<uint16_t>(commands.motor4 * 1999.0f);

            _dshot_driver.write(dshot_values);
        }
    }

private:
    DShot& _dshot_driver;
};