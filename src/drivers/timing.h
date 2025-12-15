#pragma once

#include <cstdint>

/**
 * @brief Initializes the high-resolution 64-bit timer.
 * @note Must be called after HAL_Init() and SystemClock_Config().
 */
void initTime();

/**
 * @brief Gets the current time from the DWT cycle counter.
 * @note This requires the DWT cycle counter to be enabled in main().
 * @return The current time in microseconds (us).
 */
uint64_t getCurrentTimeUs();

/**
 * @brief Updates the timer overflow counter. Must be called from SysTick_Handler.
 */
void updateTime();