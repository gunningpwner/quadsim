#include "stm32f4xx_hal.h"
// #include "usbd_cdc_if.h"

#include "peripherals.h"
// #include <string.h>
// #include <stdio.h>



uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0;

/* Measure Frequency */
float frequency = 0;

// extern "C" int _write(int file, char *ptr, int len)
// {
//     if (file != 1) { // stdout
//         return -1;
//     }

//     // Original behavior: send raw data over USB
//     CDC_Transmit_FS((uint8_t *)ptr, len);

//     return len;
// }

volatile uint16_t pwmData[13]; // This is correct, DMA is configured for 16-bit (half-word)

int main(void) {
  SystemClock_Config_HSE(); 
  HAL_Init(); 



  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init(); 
  MX_TIM5_Init();
  // MX_USB_DEVICE_Init();

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  pwmData[0]=9;
  pwmData[1]=9;
  pwmData[2]=9;
  pwmData[3]=9;
  pwmData[4]=9;
  pwmData[5]=9;
  pwmData[6]=9;
  pwmData[7]=9;
  pwmData[8]=9;
  pwmData[9]=9;
  pwmData[10]=9;
  pwmData[11]=9;
  pwmData[12]=9;

  // The DMA is configured for Half-Word (16-bit) transfers in peripherals.cpp.
  // Therefore, we must pass a uint16_t pointer. The cast to (uint32_t*) was
  // causing a data type mismatch and a hard fault.
  // HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_2, (uint32_t *)pwmData, 13);
  __HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC2);
  HAL_DMA_Start(&hdma_tim5_ch2, (uint32_t)pwmData, (uint32_t)&(htim5.Instance->CCR2), 13);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  while(1){
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    HAL_Delay(100);
    // printf("Difference: \n");
  }
  
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (Is_First_Captured==0) // if the first rising edge is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
		}

		else   // If the first rising edge is captured, now we will capture the second edge
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffffffff - IC_Val1) + IC_Val2;
			}


			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_First_Captured = 0; // set it back to false
		}
	}
}
extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
  // The overflow counter is now updated atomically within getCurrentTimeUs().
}

// extern "C" void OTG_FS_IRQHandler(void) {
//   HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
// }



void DMA1_Stream4_IRQHandler(void)
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
void TIM2_IRQHandler(void)
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