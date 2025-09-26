#pragma once

#include <cstdint>

/**
 * @brief Gets the current time from the DWT cycle counter.
 * @note This requires the DWT cycle counter to be enabled in main().
 * @return The current time in microseconds (us).
 */
uint64_t getCurrentTimeUs();