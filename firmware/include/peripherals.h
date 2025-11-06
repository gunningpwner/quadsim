#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "usbd_def.h"
#include "stm32f4xx_hal.h"

// Declare global handles for all used peripherals

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern DMA_HandleTypeDef hdma_tim5_ch2;
// These are defined in peripherals.cpp
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

// Declare initialization functions
void SystemClock_Config_HSE(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_USB_DEVICE_Init(void);
void MX_TIM2_Init(void);
void MX_TIM5_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

#endif // PERIPHERALS_H