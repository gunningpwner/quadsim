#include "bmi270.h"
#include "bmi270_config.h"
#include "stm32f4xx_hal.h"
#include "timing.h"
#include <cstdio> // For printf
#include <string.h>

BMI270::BMI270(SPI_HandleTypeDef* spi_handle, GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : m_spi_handle(spi_handle), m_cs_port(cs_port), m_cs_pin(cs_pin) {}

/**
  * @brief  Provides a precise microsecond delay using the DWT cycle counter.
  * @param  microseconds: Number of microseconds to wait.
  */
void BMI270::delay_us(uint32_t microseconds) {
  uint32_t clk_cycle_start = DWT->CYCCNT;
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

int8_t BMI270::init(void) {
    uint8_t chip_id = 0;

    // The first read operation is a dummy read to put the device into SPI mode.
    // See BMI270 Datasheet, section 5.1.2
    spi_read(REG_CHIP_ID, &chip_id, 1);
    delay_us(100); // Small delay after mode switch

    // Second read should return the correct Chip ID
    if (spi_read(REG_CHIP_ID, &chip_id, 1) != 0) {
        return -1; // SPI communication failure
    }

    if (chip_id != EXPECTED_CHIP_ID) {
        return -1; // Chip ID mismatch
    }

    // Disable power save mode
    spi_write(REG_PWR_CONF,(const uint8_t[]){0x00},1);
    // Delay for at least 450us
    delay_us(500);
    // Upload that config file boi
    spi_write(REG_INIT_CTRL, (const uint8_t[]){0x00}, 1);
    
    spi_write(REG_INIT_DATA,bmi270_config_file,sizeof(bmi270_config_file));

    #ifdef IMU_IO_DEBUG
    uint8_t test[sizeof(bmi270_config_file)];
    spi_read(REG_INIT_DATA,test,sizeof(bmi270_config_file));
    printf("Verifying config file write...\n");
    int mismatches = 0;
    for(int i=0;i<sizeof(bmi270_config_file);i++){
        if(test[i]!=bmi270_config_file[i]){
            if (mismatches == 0) printf("Mismatched bytes found:\n");
            printf("Index %d: Wrote 0x%02X, Read 0x%02X\n", i, bmi270_config_file[i], test[i]);
            mismatches++;
        }
    }
    if (mismatches == 0) printf("Config file verified successfully!\n");
    delay_us(10);
    #endif
    
    spi_write(REG_INIT_CTRL, (const uint8_t[]){0x01}, 1);
    //wait a bit
    HAL_Delay(100);

    uint8_t status=0;
    spi_read(REG_INTERNAL_STATUS,&status,1);
    if (status==0x01){
    } else
    {
      printf("BMI270 init status is 0x%02X, expected 0x01\n",status);
      return -1;
    }

    //Set the sensor range
    spi_write(REG_ACC_RANGE, (const uint8_t[]){ACCEL_RANGE}, 1);
    spi_write(REG_GYRO_RANGE, (const uint8_t[]){GYRO_RANGE}, 1);

    // Pre-configure the DMA transmit buffer since it never changes.
    // This saves a few cycles in the timer interrupt.
    memset(m_spi_tx_buf, 0, sizeof(m_spi_tx_buf));
    m_spi_tx_buf[0] = REG_ACC_DATA | 0x80;

    // Turn on the sensors
    // Enable accelerometer and gyroscope
    spi_write(REG_PWR_CTRL, (const uint8_t[]){0b00001110}, 1);
    return 0; 
}

int8_t BMI270::spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len) {
  // The first byte sent is the register address with the read bit set (MSB=1)
  uint8_t tx_addr = reg_addr | 0x80;

  // The BMI270 requires a dummy byte after the register address for reads
  uint8_t tx_buf[len + 2];
  uint8_t rx_buf[len + 2];
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = tx_addr;

  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(m_spi_handle, tx_buf, rx_buf, len + 2, HAL_MAX_DELAY) != HAL_OK) {
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
    return -1;
  }
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET); // CS High

  // Data is received after the two dummy bytes
  memcpy(data, &rx_buf[2], len);

  #ifdef IMU_IO_DEBUG
    printf("Read ");
    for (uint32_t i = 0; i < len; i++){
        printf("0x%02X ", data[i]);
    }
    printf(" from address 0x%02X\n",reg_addr);
  #endif

  return 0;
}

int8_t BMI270::spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len) {
  // The first byte sent is the register address with the write bit cleared (MSB=0)
  
  uint8_t tx_addr = reg_addr & 0x7F;
  uint8_t tx_buf[len + 1];
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = tx_addr;
  memcpy(&tx_buf[1], data, len);

  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET); // CS Low
  // Transmit the stuff
  if (HAL_SPI_Transmit(m_spi_handle, tx_buf, len + 1, HAL_MAX_DELAY) != HAL_OK) {
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
    return -1;
  }
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET); // CS High

  #ifdef IMU_IO_DEBUG
    printf("Wrote ");
    for (uint32_t i = 0; i < len; i++){
        printf("0x%02X ", data[i]);
    }
    printf(" to address 0x%02X\n",reg_addr);
  #endif

  return 0;
}

bool BMI270::startReadImu_DMA() {
  // Manually assert the chip select pin
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);

  // Start a non-blocking SPI transfer with DMA.
  // We transfer 14 bytes: 2 dummy bytes + 12 data bytes (accel + gyro).
  if (HAL_SPI_TransmitReceive_DMA(m_spi_handle, m_spi_tx_buf, m_spi_rx_buf, 14) != HAL_OK) {
    // If the DMA fails to start, de-assert CS and return failure.
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
    return false;
  }

  return true; // The transfer has started successfully.
}

void BMI270::processRawData() {
  // This function is called from the interrupt context.
  // De-assert the chip select pin to end the transaction.
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);

  // --- Timestamp the data as close to its arrival as possible ---
  uint64_t timestamp = getCurrentTimeUs();

  // Data starts at index 2. Accel is first (6 bytes), then Gyro (6 bytes).
  const uint8_t* data_ptr = &m_spi_rx_buf[2];

  // --- Process Accelerometer Data ---
  int16_t raw_ax = (int16_t)((data_ptr[1] << 8) | data_ptr[0]);
  int16_t raw_ay = (int16_t)((data_ptr[3] << 8) | data_ptr[2]);
  int16_t raw_az = (int16_t)((data_ptr[5] << 8) | data_ptr[4]);

  AccelData accel_data;
  accel_data.Timestamp = timestamp;
  accel_data.Acceleration << (float)raw_ax, (float)raw_ay, (float)raw_az;
  accel_data.Acceleration = accel_data.Acceleration / ACCEL_SENSITIVITY * G_TO_MS2;

  // --- Process Gyroscope Data ---
  int16_t raw_gx = (int16_t)((data_ptr[7] << 8) | data_ptr[6]);
  int16_t raw_gy = (int16_t)((data_ptr[9] << 8) | data_ptr[8]);
  int16_t raw_gz = (int16_t)((data_ptr[11] << 8) | data_ptr[10]);

  GyroData gyro_data;
  gyro_data.Timestamp = timestamp; // Use the same timestamp for both
  gyro_data.AngularVelocity << (float)raw_gx, (float)raw_gy, (float)raw_gz;
  gyro_data.AngularVelocity = gyro_data.AngularVelocity / GYRO_SENSITIVITY * DEG_TO_RAD;

  // For now, we just print the timestamped data.
  // In a real system, you would post this to a thread-safe queue (like a DataManager).
  printf("TS: %.0f, Accel: X=%.3f Y=%.3f Z=%.3f, Gyro: X=%.3f Y=%.3f Z=%.3f\n",
         (double)accel_data.Timestamp,
         (double)accel_data.Acceleration.x(),
         (double)accel_data.Acceleration.y(),
         (double)accel_data.Acceleration.z(),
         (double)gyro_data.AngularVelocity.x(),
         (double)gyro_data.AngularVelocity.y(),
         (double)gyro_data.AngularVelocity.z());
}
