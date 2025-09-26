#ifndef BMI270_H
#define BMI270_H

#include "stm32f4xx_hal.h"
#include <cstdint>
#include "SensorData.h" // For AccelData struct

class BMI270 {
public:
    BMI270(SPI_HandleTypeDef* spi_handle, GPIO_TypeDef* cs_port, uint16_t cs_pin);

    // Blocking init function
    int8_t init();

    // Non-blocking DMA read functions
    bool startReadImu_DMA();
    // This function will be called from the SPI/DMA interrupt handler
    void processRawData();

private:
    // Register Map
    static constexpr uint8_t REG_CHIP_ID         = 0x00;
    static constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
    static constexpr uint8_t REG_ACC_DATA        = 0x0C;
    static constexpr uint8_t REG_ACC_RANGE       = 0x41; 
    static constexpr uint8_t REG_GYRO_DATA       = 0x12;
    static constexpr uint8_t REG_GYRO_RANGE      = 0x43;
    static constexpr uint8_t REG_INIT_CTRL       = 0x59;
    static constexpr uint8_t REG_INIT_DATA       = 0x5E;
    static constexpr uint8_t REG_PWR_CONF        = 0x7C;
    static constexpr uint8_t REG_PWR_CTRL        = 0x7D;
    
    // Constants
    static constexpr uint8_t EXPECTED_CHIP_ID = 0x24;

    // Sensitivity for default +/- 2g range. LSB/g
    // static constexpr float ACCEL_SENSITIVITY = 16384.0f;
    // static constexpr uint8_t ACCEL_RANGE = 0x00; // +/- 2g range

    static constexpr float ACCEL_SENSITIVITY = 8192.0f;
    static constexpr uint8_t ACCEL_RANGE = 0x01; // +/- 4g range

    static constexpr float GYRO_SENSITIVITY = 16.384f;
    static constexpr uint8_t GYRO_RANGE = 0x00; // +/-2000 dps
    // Conversion from g to m/s^2
    static constexpr float G_TO_MS2 = 9.80665f;
    // Conversion from degrees to radians
    static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

    // Platform-specific functions are now private helper methods
    int8_t spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len);
    int8_t spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len);
    void delay_us(uint32_t microseconds);

    // Member variables to hold driver state
    SPI_HandleTypeDef* m_spi_handle;
    GPIO_TypeDef*      m_cs_port;
    uint16_t           m_cs_pin;

    // Buffers for DMA transfers. We read 12 bytes of data (6 accel + 6 gyro)
    // plus 2 dummy bytes, for a total of 14 bytes.
    uint8_t m_spi_tx_buf[14];
    uint8_t m_spi_rx_buf[14];
};

#endif // BMI270_H