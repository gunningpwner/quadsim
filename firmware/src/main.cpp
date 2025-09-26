#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "bmi270.h"
#include "timing.h"

#include <string.h>
#include <stdio.h>

// Global variables
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_DescriptorsTypeDef FS_Desc;

// Function Prototypes
void SystemClock_Config_HSE(void);
void MX_USB_DEVICE_Init(void);
void MX_DMA_Init(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_SPI1_Init(void);

// Global pointer to the IMU object for the interrupt handler to use.
BMI270* g_imu_ptr = nullptr;

uint64_t getCurrentTimeUs() {
    // Return the value of the DWT cycle counter, scaled to microseconds
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000);
}

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

/**
  * @brief Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration (for SPI1_RX) */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration (for SPI1_TX) */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void) {
    // TIM2 is on APB1, which has a timer clock of 84 MHz.
    // We want to trigger at 100 Hz (every 10ms).
    // For 100Hz (10ms): (839 + 1) * (999 + 1) / 84MHz = 840 * 1000 / 84M = 0.01s = 10ms.

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 840 - 1; // Gives a 100kHz timer clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1; // Count up to 1000 for a 10ms period (100Hz)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // Timer has elapsed, kick off a DMA read.
        if (g_imu_ptr) g_imu_ptr->startReadImu_DMA();
    }
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    // This function is called when the SPI DMA transfer is complete.
    // We can now process the raw data that the DMA has moved for us.
    if (g_imu_ptr != nullptr) {
      g_imu_ptr->processRawData();
    }
  }
}


void MX_SPI1_Init(void) {
  // Used by the IMU
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

  // DMA handles must be static or global
  static DMA_HandleTypeDef hdma_spi1_rx;
  static DMA_HandleTypeDef hdma_spi1_tx;
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

  // Configure DMA for SPI1 RX
  hdma_spi1_rx.Instance = DMA2_Stream0;
  hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
  hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi1_rx.Init.Mode = DMA_NORMAL;
  hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_spi1_rx);
  __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);

  // Configure DMA for SPI1 TX
  hdma_spi1_tx.Instance = DMA2_Stream3;
  hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
  hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi1_tx.Init.Mode = DMA_NORMAL;
  hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_spi1_tx);
  __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);

  HAL_SPI_Init(&hspi1);
}

void MX_SPI2_Init(void){
  // Used by the Flash storage
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Data pins
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //CS Pin
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  __HAL_RCC_SPI2_CLK_ENABLE();
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {
    if(tim_baseHandle->Instance==TIM2) {
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

extern "C" int _write(int file, char *ptr, int len) {
  if (file != 1) { return -1; }
  CDC_Transmit_FS((uint8_t *)ptr, len);
  return len;
}


int main(void) {
  SystemClock_Config_HSE(); // Configure the clock first.
  HAL_Init(); // Then initialize the HAL, which sets up the SysTick based on the new clock.

  // Enable DWT Cycle Counter for microsecond delays
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();

  // Create an instance of the BMI270 driver
  BMI270 imu(&hspi1, GPIOC, GPIO_PIN_4);
  g_imu_ptr = &imu; // Set the global pointer

  HAL_Delay(2000); // Wait for USB to enumerate
  printf("\n--- BMI270 Initialization ---\n");

  if (imu.init() == 0) {
      printf("BMI270 Initialized Successfully.\n");
  } else {
      printf("BMI270 Initialization Failed!\n");
      // It's good practice to halt if essential hardware fails
      while(1);
  }

  // Start the timer. It will now trigger DMA reads automatically in the background.
  HAL_TIM_Base_Start_IT(&htim2);

  while (1) {
    // The main loop is now free! Sensor reads are happening automatically.
    // We can use a simple blocking delay here to toggle an LED and prove
    // that the main loop's activity doesn't affect the sensor reads.
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
  }
}

extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
}

extern "C" void OTG_FS_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

extern "C" void TIM2_IRQHandler(void) {
    // This is the interrupt handler for TIM2
    HAL_TIM_IRQHandler(&htim2);
}

extern "C" void DMA2_Stream0_IRQHandler(void) {
    // This is the interrupt handler for SPI1_RX
    HAL_DMA_IRQHandler(hspi1.hdmarx);
}

extern "C" void DMA2_Stream3_IRQHandler(void) {
    // This is the interrupt handler for SPI1_TX
    HAL_DMA_IRQHandler(hspi1.hdmatx);
}