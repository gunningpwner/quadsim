#ifndef BMI270_H
#define BMI270_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Expected Chip ID for BMI270
#define BMI270_CHIP_ID 0x24

// Chip ID Register
#define BMI270_CHIP_ID_REG 0x00
#define BMI270_PWR_CONF_REG 0x7C
#define BMI270_INIT_CTRL_REG 0x59
#define BMI270_INIT_DATA_REG 0x5E
#define BMI270_INTERNAL_STATUS_REG 0x21

/**
  * @brief Initializes the BMI270 sensor.
  *        Performs a dummy read to enter SPI mode, then verifies the Chip ID.
  * @return 0 on success, -1 on failure (e.g., wrong chip ID).
  */
int8_t bmi270_init(void);

/**
 * @brief Platform-specific SPI read function for the BMI270 API.
 * @param reg_addr: Register address to read from.
 * @param data: Pointer to the buffer to store the read data.
 * @param len: Number of bytes to read.
 * @return 0 on success, non-zero on failure.
 */
int8_t bmi270_spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len);

/**
 * @brief Platform-specific SPI write function for the BMI270 API.
 * @param reg_addr: Register address to write to.
 * @param data: Pointer to the data to be written.
 * @param len: Number of bytes to write.
 * @return 0 on success, non-zero on failure.
 */
int8_t bmi270_spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len);

/**
  * @brief  Provides a precise microsecond delay using the DWT cycle counter.
  * @param  microseconds: Number of microseconds to wait.
  */
void bmi270_delay_us(uint32_t microseconds);


#endif // BMI270_H