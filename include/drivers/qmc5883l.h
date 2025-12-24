#pragma once

#include "stm32f4xx_hal.h"
#include <cstdint>
#include "DataTypes.h"
#include "DataManager.h"


class QMC5883L
{
public:
    QMC5883L(DataManager::SensorBuffer &m_buffer);
    int8_t init();
    bool startReadCompass_DMA();
    void processRawData();


private:
    static constexpr uint8_t DEVICE_ADDR = 0x0D<<1;
    static constexpr uint8_t REG_CHIP_ID = 0x0D;
    static constexpr uint8_t EXPECTED_CHIP_ID=0xFF;
    static constexpr uint8_t REG_CONTROL1 = 0x09;
    static constexpr uint8_t REG_CONTROL2 = 0x0A;
    static constexpr uint8_t DATA_CONTROL1 = 0b10000001;
    static constexpr uint8_t REG_DATA = 0x06;
    static constexpr float SENSITIVITY_2G = 12000.0f;

    DataManager::SensorBuffer& m_sensor_buffer;
    uint8_t m_rx_buf[7];

};