#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "usbd_def.h"
#include "stm32f4xx_hal.h"

// Declare global handles for all used peripherals
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern I2C_HandleTypeDef hi2c1;
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
void MX_TIM2_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
void MX_USART3_UART_Init(void);
void MX_UART4_Init(void);
void MX_USB_DEVICE_Init(void);
void MX_ADC1_Init(void);
void MX_I2C1_Init(void);


#endif // PERIPHERALS_H