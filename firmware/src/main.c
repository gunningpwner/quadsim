#include "stm32f4xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include <string.h>
#include <stdio.h>

USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_DescriptorsTypeDef FS_Desc;

// The working clock config that uses the internal HSI oscillator
void SystemClock_Config_HSI(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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

/**
  * @brief  Retargets the C library printf function to the VIRTUAL COM PORT.
  * @note   This is a weak function implementation that can be overridden.
  *         It routes stdout (file descriptor 1) to the USB CDC port.
  * @param  file: File descriptor.
  * @param  ptr: Pointer to the data to be written.
  * @param  len: Length of the data.
  * @retval The number of characters written.
  */
int _write(int file, char *ptr, int len)
{
  // We only want to handle stdout (file descriptor 1)
  if (file != 1) {
    return -1;
  }

  // Define a timeout period in milliseconds.
  const uint32_t timeout = 10; // Give up after 10ms
  uint32_t start_time = HAL_GetTick();

  // Attempt to transmit the data. If the port is busy, loop until it's free
  // or until the timeout is reached.
  while (CDC_Transmit_FS((uint8_t *)ptr, len) == USBD_BUSY) {
    if (HAL_GetTick() - start_time > timeout) {
      // Timeout occurred, data is dropped.
      return len; // Lie and say we wrote it all to not confuse printf.
    }
  }
  return len;
}

int main(void) {
  HAL_Init();
  SystemClock_Config_HSI();

  // --- Initialize GPIO Pin for LED ---
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  // --- Initialize USB ---
  MX_USB_DEVICE_Init();

  // --- Infinite Loop ---
  while (1) {
    static uint32_t counter = 0;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    // Now you can use printf!
    printf("It's alive! Count: %lu\r\n", counter++);
    HAL_Delay(500);
  }
}

void SysTick_Handler(void) {
  HAL_IncTick();
}

void OTG_FS_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}