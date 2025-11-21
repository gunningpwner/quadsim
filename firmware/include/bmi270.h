#ifndef BMI270_H
#define BMI270_H

#include "stm32f4xx_hal.h"
#include <cstdint>
#include "SensorData.h" 

class BMI270 {
public:
    BMI270(SPI_HandleTypeDef* spi_handle, GPIO_TypeDef* cs_port, uint16_t cs_pin);

    int8_t init();

    bool startReadImu_DMA();
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

    static constexpr float G_TO_MS2 = 9.80665f;
    static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

    int8_t readReg(uint8_t reg_addr);
    void writeReg(uint8_t reg_addr, uint8_t value);
    void startRead_DMA();
    void delay_us(uint32_t microseconds);

    SPI_HandleTypeDef* m_spi_handle;
    GPIO_TypeDef*      m_cs_port;
    uint16_t           m_cs_pin;

    uint8_t m_spi_tx_buf[14];
    uint8_t m_spi_rx_buf[14];
};

#endif // BMI270_H