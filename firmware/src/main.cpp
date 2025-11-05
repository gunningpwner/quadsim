#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#include "peripherals.h"
#include <string.h>
#include <stdio.h>


extern "C" int _write(int file, char *ptr, int len)
{
    if (file != 1) { // stdout
        return -1;
    }

    // Original behavior: send raw data over USB
    CDC_Transmit_FS((uint8_t *)ptr, len);

    return len;
}

int main(void) {
  SystemClock_Config_HSE(); 
  HAL_Init(); 



  MX_GPIO_Init();
  MX_TIM2_Init(); 
  MX_USB_DEVICE_Init();

  while(1){
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    HAL_Delay(100);
    printf("hey\n");
  }
  
}

extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
  // The overflow counter is now updated atomically within getCurrentTimeUs().
}

extern "C" void OTG_FS_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}



extern "C" void TIM2_IRQHandler(void) {
    // This is the interrupt handler for TIM2
    HAL_TIM_IRQHandler(&htim2);
}


extern "C" void DMA2_Stream5_IRQHandler(void) {
    // This is the interrupt handler for TIM1_UP
    HAL_DMA_IRQHandler(htim1.hdma[TIM_DMA_ID_UPDATE]);
}