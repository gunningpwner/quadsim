#include "stm32f4xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal_spi.h"

// Global variables
SPI_HandleTypeDef hspi1;
USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_DescriptorsTypeDef FS_Desc;

// Function Prototypes
void SystemClock_Config_HSE(void);
void MX_USB_DEVICE_Init(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void DWT_Delay_us(volatile uint32_t microseconds);


/**
  * @brief System Clock Configuration for an 8MHz HSE crystal.
  */
void SystemClock_Config_HSE(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void MX_USB_DEVICE_Init(void) {
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
  USBD_Start(&hUsbDeviceFS);
}

void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void MX_SPI1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

    __HAL_RCC_SPI1_CLK_ENABLE();
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi1);
}

int _write(int file, char *ptr, int len) {
  if (file != 1) { return -1; }
  CDC_Transmit_FS((uint8_t *)ptr, len);
  return len;
}

/**
  * @brief  Provides a precise microsecond delay using the DWT cycle counter.
  * @param  microseconds: Number of microseconds to wait.
  */
void DWT_Delay_us(volatile uint32_t microseconds) {
  uint32_t clk_cycle_start = DWT->CYCCNT;
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

// Replace your existing bmi270_xxx functions with these two
uint8_t bmi270_read_reg(uint8_t reg_addr) {
    HAL_StatusTypeDef status;
    uint8_t tx_byte = reg_addr | 0x80; // Address with read bit
    uint8_t dummy_tx = 0x00;           // Dummy byte to send
    uint8_t rx_dummy;                  // To receive the first dummy byte from IMU
    uint8_t rx_data;                   // To receive the actual data

    // CS Low
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

    // 1. Send the register address we want to read
    status = HAL_SPI_TransmitReceive(&hspi1, &tx_byte, &rx_dummy, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        printf("SPI TransmitReceive (address phase) failed!\n");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        return 0;
    }

    // 2. Send a dummy byte to clock in the data from the sensor
    status = HAL_SPI_TransmitReceive(&hspi1, &dummy_tx, &rx_data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        printf("SPI TransmitReceive (data phase) failed!\n");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        return 0;
    }

    // CS High
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

    return rx_data;
}

void bmi270_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg_addr & 0x7F; // Address with write bit (MSB=0)
    tx_buffer[1] = data;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    DWT_Delay_us(450); // After a write, especially to a config reg, a longer delay is safer.
}


// Replace your main function with this
int main(void) {
  HAL_Init();
  SystemClock_Config_HSE();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  HAL_Delay(1000);
  printf("\n--- Last Attempt: Low-Level Communication Test ---\n");

  // Perform a dummy read to set SPI mode.
  bmi270_read_reg(0x00); 
  HAL_Delay(1); // Small delay

  // Now, try reading the Chip ID.
  uint8_t chip_id = bmi270_read_reg(0x00);
  printf("Chip ID: 0x%02X (Expected: 0x24)\n", chip_id);

  if (chip_id == 0x24) {
    printf("SUCCESS! It was a HAL timing issue.\n");
  } else {
    printf("FAILURE. The issue is confirmed to be a deep HAL/Betaflight difference.\n");
    printf("Next step is to port the exact SPI driver functions from Betaflight source.\n");
  }

  while (1) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    HAL_Delay(500);
  }
}

void SysTick_Handler(void) {
  HAL_IncTick();
}

void OTG_FS_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}