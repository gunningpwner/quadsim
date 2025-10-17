#pragma once

#include "stm32f4xx_hal.h"
#include "OtherData.h" // For DShot_Command enum
#include <array>
#include <cstdint>

// Forward declaration
class DataManager;

class DShot {
public:
    /**
     * @brief Constructor for the DShot driver.
     * @param htim Pointer to the timer handle (e.g., &htim1).
     */
    DShot(TIM_HandleTypeDef* htim);

    /**
     * @brief Initializes the DShot driver and starts the timer PWM channels.
     * @return 0 on success, -1 on failure.
     */
    int init();
    
    void onMotorCommandPosted();

private:
    /**
     * @brief Prepares a DShot frame for a single motor.
     * @param value The throttle value (0-1999).
     * @param telemetry Whether to request telemetry (default false).
     * @return The 16-bit DShot frame.
     */
    uint16_t prepare_frame(uint16_t value, bool telemetry = false);

    /**
     * @brief Prepares the DMA buffer from the DShot frames.
     * @param motor_frames An array of 4 DShot frames.
     */
    void prepare_dma_buffer(const std::array<uint16_t, 4>& motor_frames);

    TIM_HandleTypeDef* _htim;

    // DShot timing constants for DShot600 at 168MHz timer clock
    // For DShot600 and 168MHz clock, Period=280, T0H=93, T1H=186
    // These are from peripherals.cpp, but defined here for clarity in the driver.
    static constexpr uint32_t DSHOT_BIT_0 = 93;
    static constexpr uint32_t DSHOT_BIT_1 = 186;

    // DMA buffer for 4 motors, 16 bits per motor + 2 idle bits
    // The buffer size needs to be 18 words for each of the 4 motors for burst mode.
    std::array<uint32_t, 18 * 4> _dma_buffer; 
};