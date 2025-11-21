#include "bmi270.h"
#include "bmi270_config.h"
#include "stm32f4xx_hal.h"
#include "DataManager.h"

extern DataManager *g_data_manager_ptr;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

BMI270::BMI270(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin)
    : m_spi_handle(spi_handle), m_cs_port(cs_port), m_cs_pin(cs_pin) {}

int8_t BMI270::init(void)
{
  // BMI270 will switch to SPI if it sees a rising edge on the chip select wire
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);

  if (readReg(REG_CHIP_ID) != EXPECTED_CHIP_ID)
  {
    return -1; // Chip ID mismatch
  }

  writeReg(REG_PWR_CONF, 0x00); // Disable power save mode
  HAL_Delay(1);// wait at least 450us

  writeReg(REG_INIT_CTRL, 0x00);
  // Manually slam the config down the pipe
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(m_spi_handle, bmi270_config_file, sizeof(bmi270_config_file), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);

  writeReg(REG_INIT_CTRL, 0x01);

  HAL_Delay(50); // Wait for imu to do its thing

  if (readReg(REG_INTERNAL_STATUS) != 0x01)
    // IMU not happy or not ready yet. Testing shows it should be ready after delay so it's probably not happy
    return -1; 

  writeReg(REG_ACC_RANGE, ACCEL_RANGE);
  writeReg(REG_GYRO_RANGE, GYRO_RANGE);

  // Manually configure dma streams so we don't have to use the HAL
  // This assumes we're done doing config stuff and will just periodically read imu data

  hdma_spi1_rx.Instance->M0AR = (uint32_t)m_spi_rx_buf;//where to put
  hdma_spi1_rx.Instance->PAR = (uint32_t)&m_spi_handle->Instance->DR;// where to take from

  
  hdma_spi1_tx.Instance->M0AR = (uint32_t)m_spi_rx_buf;
  hdma_spi1_tx.Instance->PAR = (uint32_t)&m_spi_handle->Instance->DR;

  return 0;
}

int8_t BMI270::readReg(uint8_t reg_addr)
{
  // Read one byte from a register

  // Might as well just use these buffers.
  m_spi_tx_buf[0] = reg_addr | 0x80;
  m_spi_tx_buf[1] = 0x00;
  m_spi_tx_buf[2] = 0x00;
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(m_spi_handle, m_spi_tx_buf, m_spi_rx_buf, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
  return m_spi_rx_buf[2];
}

void BMI270::writeReg(uint8_t reg_addr, uint8_t value)
{
  // Write one byte to a register

  // Might as well just use this buffer.
  m_spi_tx_buf[0] = reg_addr & 0x7F;
  m_spi_tx_buf[1] = value;

  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(m_spi_handle, m_spi_tx_buf, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
}

void BMI270::startRead_DMA(){
  HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);


}