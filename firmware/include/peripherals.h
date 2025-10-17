#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "stm32f4xx_hal.h"
#include "usbd_core.h"

// --- Global Hardware Handles ---
// These are defined in peripherals.cpp and declared here as 'extern'
// to be accessible from other files (like main.cpp).
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;


// --- Function Prototypes for Initializers ---
void SystemClock_Config_HSE(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_SPI1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM5_Init(void);
void MX_USART3_UART_Init(void);
void MX_USB_DEVICE_Init(void);


#endif // PERIPHERALS_H