#include "stm32f4xx_hal.h"
// #include "usbd_cdc_if.h"

#include "peripherals.h"
// #include <string.h>
// #include <stdio.h>

uint16_t prepare_frame(uint16_t value, bool telemetry);
void prepare_buffer(uint32_t* _dma_buffer, uint16_t frame);

 uint32_t pwmData[18]; 

 int main(void) {
  SystemClock_Config_HSE(); 
  HAL_Init(); 



  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init(); 
  MX_TIM5_Init();
  // MX_USB_DEVICE_Init();


  __HAL_DBGMCU_FREEZE_TIM2();
  __HAL_DBGMCU_FREEZE_TIM5();

  // HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  uint16_t frame = prepare_frame(200,false);
  prepare_buffer(pwmData,frame);
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_2, pwmData, 18);
  // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
}


#define DSHOT_BIT_1 104
#define DSHOT_BIT_0 52

uint16_t prepare_frame(uint16_t value, bool telemetry) {
    uint16_t dshot_val = value; // The value is now expected to be in the final 0-2047 range.

    uint16_t frame = (dshot_val << 1) | (telemetry ? 1 : 0);

    // Compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = frame;
    for (int i = 0; i < 3; ++i) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x0F;

    frame = (frame << 4) | csum;

    return frame;
}

void prepare_buffer(uint32_t* _dma_buffer, uint16_t frame) {
  //convert frame to buffer
  for (int bit = 0; bit < 16; ++bit) {
    if (frame & (1 << (15 - bit))) {
      _dma_buffer[bit] = DSHOT_BIT_1;
    } else {
      _dma_buffer[bit] = DSHOT_BIT_0;
    }
  }
  // Add two "zero" bits at the end for spacing between DShot frames.
  // This writes 0 to the CCRs, ensuring the line is low.
  for (int i = 16; i < 18; ++i) {
    _dma_buffer[i] = 0;
  }
}


extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

}
extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
  // The overflow counter is now updated atomically within getCurrentTimeUs().
}

// extern "C" void OTG_FS_IRQHandler(void) {
//   HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
// }



extern "C" void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim5_ch2);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
extern "C" void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

extern "C" void HardFault_Handler(void) {
    // Stop the processor and enter an infinite loop
    // BUT, first, toggle your LED to signal the fault!
    
    // Example using your existing GPIO init on PC15
    // Note: You must ensure the HAL functions work inside the fault handler,
    // which they usually do unless the core HAL data is corrupted.
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15); 
    
    // Loop indefinitely to keep the LED in the toggled state
    while (1) {
        // You can add a small delay here if you want a blinking fault indicator
        HAL_Delay(200);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    }
}

extern "C" void BusFault_Handler(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // Turn LED ON for Bus Fault
    while (1) {}
}

extern "C" void UsageFault_Handler(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // Turn LED OFF for Usage Fault
    while (1) {}
}