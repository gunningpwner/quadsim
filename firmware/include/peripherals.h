#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "usbd_def.h"
#include "stm32f4xx_hal.h"

// Declare global handles for all used peripherals
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

// These are defined in peripherals.cpp
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

// Declare initialization functions
void SystemClock_Config_HSE(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_SPI1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_USART3_UART_Init(void);
void MX_USB_DEVICE_Init(void);
void MX_ADC1_Init(void);


#endif // PERIPHERALS_H