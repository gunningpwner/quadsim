#pragma once

#include "stm32f4xx_hal.h"
#include <array>
#include <cstdint>

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

    /**
     * @brief Writes motor values to the ESCs.
     * @param motor_values An array of 4 motor values, range 0-1999.
     */
    void write(const std::array<uint16_t, 4>& motor_values);

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
    static constexpr uint32_t DSHOT_BIT_0 = 35;  // ~33% duty cycle for a '0'
    static constexpr uint32_t DSHOT_BIT_1 = 70;  // ~66% duty cycle for a '1'
    static constexpr uint32_t DSHOT_TIMER_PERIOD = 105; // 168MHz / 105 = 1.6MHz -> 0.625us per tick. Period is 1.66us for DShot600

    // DMA buffer for 4 motors, 16 bits per motor + 2 idle bits
    std::array<uint32_t, 18 * 4> _dma_buffer;
};