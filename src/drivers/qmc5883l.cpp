#include "drivers/qmc5883l.h"
#include "timing.h"

extern I2C_HandleTypeDef hi2c1;

QMC5883L::QMC5883L(DataManager::SensorBuffer &m_buffer) : m_sensor_buffer(m_buffer)
{
}

int8_t QMC5883L::init()
{
    uint8_t reset = 0x80;
    // Soft reset
    HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDR, REG_CONTROL2, 1, &reset, 1, HAL_MAX_DELAY);
    HAL_Delay(350);

    uint8_t chip_id = 0;
    HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDR, REG_CHIP_ID, 1, &chip_id, 1, HAL_MAX_DELAY);
    if (chip_id != EXPECTED_CHIP_ID)
    {
        return -1;
    }
    //idk man. the datasheet said to do it and did not elaborate
    uint8_t reset_period = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDR, 0x0B,1, &reset_period, 1, HAL_MAX_DELAY);
    // Enables automatic rollover
    uint8_t rollover = 0b01000000;
    HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDR, REG_CONTROL2,1, &rollover, 1, HAL_MAX_DELAY);
    // Won't let me just use DATA_CONTROL1, so fuck it just use the chip_id
    chip_id = DATA_CONTROL1;
    HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDR, REG_CONTROL1, 1, &chip_id, 1, HAL_MAX_DELAY);
    return 0;
}

bool QMC5883L::startReadCompass_DMA()
{
    HAL_I2C_Mem_Read_DMA(&hi2c1, DEVICE_ADDR, REG_DATA, 1, m_rx_buf, 7);
}

void QMC5883L::processRawData()
{   
    // Verify drdy flag is set
    if (!(m_rx_buf[0]&0b1))
        return;
    
    uint64_t timestamp = getCurrentTimeUs();

    SensorData *mag_data = m_sensor_buffer.claim();

    mag_data->timestamp = timestamp;
    mag_data->sensor = SensorData::Type::MAG;
    mag_data->data.mag.mag[0] = (int16_t)(m_rx_buf[2] << 8 | m_rx_buf[1]) / SENSITIVITY_2G;
    mag_data->data.mag.mag[1] = (int16_t)(m_rx_buf[4] << 8 | m_rx_buf[3]) / SENSITIVITY_2G;
    mag_data->data.mag.mag[2] = (int16_t)(m_rx_buf[6] << 8 | m_rx_buf[5]) / SENSITIVITY_2G;
    m_sensor_buffer.commit(mag_data);
    // printf("mag: %f %f %f\n", mag_data->data.mag.mag[0], mag_data->data.mag.mag[1], mag_data->data.mag.mag[2]);
    
}