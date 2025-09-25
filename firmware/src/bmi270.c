#include "bmi270.h"
#include "bmi270_config.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// These are defined in main.c
extern SPI_HandleTypeDef hspi1;

// Chip Select Pin configuration
#define BMI270_CS_PORT GPIOC
#define BMI270_CS_PIN  GPIO_PIN_4

/**
  * @brief  Provides a precise microsecond delay using the DWT cycle counter.
  * @param  microseconds: Number of microseconds to wait.
  */
void bmi270_delay_us(uint32_t microseconds) {
  uint32_t clk_cycle_start = DWT->CYCCNT;
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

/**
 * @brief Initializes the BMI270 sensor.
 *        Performs a dummy read to enter SPI mode, then verifies the Chip ID.
 * @return 0 on success, -1 on failure (e.g., wrong chip ID).
 */
int8_t bmi270_init(void) {
    uint8_t chip_id = 0;

    // The first read operation is a dummy read to put the device into SPI mode.
    // See BMI270 Datasheet, section 5.1.2
    bmi270_spi_read(BMI270_CHIP_ID_REG, &chip_id, 1);
    bmi270_delay_us(100); // Small delay after mode switch

    // Second read should return the correct Chip ID
    if (bmi270_spi_read(BMI270_CHIP_ID_REG, &chip_id, 1) != 0) {
        return -1; // SPI communication failure
    }

    if (chip_id != BMI270_CHIP_ID) {
        return -1; // Chip ID mismatch
    }

    // Disable power save mode
    bmi270_spi_write(BMI270_PWR_CONF_REG,0x00,1);
    // Delay for at least 450us
    bmi270_delay_us(500);
    // Upload that config file boi
    bmi270_spi_write(BMI270_INIT_CTRL_REG, 0x00, 1);
    
    bmi270_spi_write(BMI270_INIT_DATA_REG,bmi270_config_file,sizeof(bmi270_config_file));

    // uint8_t test[sizeof(bmi270_config_file)];
    // bmi270_spi_read(BMI270_INIT_DATA_REG,test,sizeof(bmi270_config_file));
    // printf("Verifying config file write...\n");
    // int mismatches = 0;
    // for(int i=0;i<sizeof(bmi270_config_file);i++){
    //     if(test[i]!=bmi270_config_file[i]){
    //         if (mismatches == 0) printf("Mismatched bytes found:\n");
    //         printf("Index %d: Wrote %02X, Read %02X\n", i, bmi270_config_file[i], test[i]);
    //         mismatches++;
    //     }
    // }
    // if (mismatches == 0) printf("Config file verified successfully!\n");
    // bmi270_delay_us(10);
    
    bmi270_spi_write(BMI270_INIT_CTRL_REG, 0x01, 1);
    //wait a bit
    HAL_Delay(100);
    uint8_t status=0;
    bmi270_spi_read(BMI270_INTERNAL_STATUS_REG,&status,1);
    bmi270_spi_read(BMI270_INTERNAL_STATUS_REG,&status,1);
    printf("status is %02X\n",status);

    return 0; 
}

/**
 * @brief Platform-specific SPI read function for the BMI270 
 */
int8_t bmi270_spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len) {
  // The first byte sent is the register address with the read bit set (MSB=1)
  uint8_t tx_addr = reg_addr | 0x80;

  HAL_GPIO_WritePin(BMI270_CS_PORT, BMI270_CS_PIN, GPIO_PIN_RESET); // CS Low

  // The BMI270 requires a dummy byte after the register address for reads
  uint8_t tx_buf[len + 2];
  uint8_t rx_buf[len + 2];
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = tx_addr;

  if (HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 2, HAL_MAX_DELAY) != HAL_OK) {
    HAL_GPIO_WritePin(BMI270_CS_PORT, BMI270_CS_PIN, GPIO_PIN_SET);
    return -1;
  }

  // Data is received after the two dummy bytes
  memcpy(data, &rx_buf[2], len);

  HAL_GPIO_WritePin(BMI270_CS_PORT, BMI270_CS_PIN, GPIO_PIN_SET); // CS High
  return 0;
}

/**
 * @brief Platform-specific SPI write function for the BMI270 
 */
int8_t bmi270_spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len) {
  // The first byte sent is the register address with the write bit cleared (MSB=0)
  uint8_t tx_addr = reg_addr & 0x7F;

  HAL_GPIO_WritePin(BMI270_CS_PORT, BMI270_CS_PIN, GPIO_PIN_RESET); // CS Low
  uint8_t tx_buf[len + 1];
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = tx_addr;
  memcpy(&tx_buf[1], data, len);
  // Transmit the stuff
  if (HAL_SPI_Transmit(&hspi1, tx_buf, len + 1, HAL_MAX_DELAY) != HAL_OK) {
    HAL_GPIO_WritePin(BMI270_CS_PORT, BMI270_CS_PIN, GPIO_PIN_SET);
    return -1;
  }


  HAL_GPIO_WritePin(BMI270_CS_PORT, BMI270_CS_PIN, GPIO_PIN_SET); // CS High
  return 0;
}