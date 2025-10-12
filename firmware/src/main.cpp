#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"


#include <string.h>
#include <stdio.h>

// MAVLink system and component IDs for our vehicle
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1

// Global variables
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart3;
USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_DescriptorsTypeDef FS_Desc;

// Function Prototypes
void SystemClock_Config_HSE(void);
void MX_USB_DEVICE_Init(void);
void MX_USART3_UART_Init(void);
TIM_HandleTypeDef htim2;


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

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources needed for the UART:
  * - Peripheral's clock enable
  * - GPIO Configuration
  * @param huart: UART handle pointer
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART3)
  {
    // 1. Enable peripheral and GPIO clocks
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 2. Configure UART GPIO pins (PC10 -> TX, PC11 -> RX)
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 420000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    // Initialization Error
    while(1);
  }
}


void setup_freerunning_timer(void) {
  // 1. Enable Timer Clock
  __HAL_RCC_TIM2_CLK_ENABLE();

  // Configure the timer handle
  htim2.Instance = TIM2;
  // 2. Set the prescaler
  htim2.Init.Prescaler = 83; // 84MHz / (83+1) = 1MHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  // 3. Set the period to maximum
  htim2.Init.Period = 0xFFFFFFFF; // Max 32-bit value
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  // Initialize the timer with the settings
  HAL_TIM_Base_Init(&htim2);

  // 4. Start the timer
  HAL_TIM_Base_Start(&htim2);
}

extern "C" int _write(int file, char *ptr, int len)
{
    if (file != 1) { // stdout
        return -1;
    }
    // Original behavior: send raw data over USB
    CDC_Transmit_FS((uint8_t *)ptr, len);
    return len;
}

struct CRSFChannelDataStruct{
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));
typedef struct CRSFChannelDataStruct CRSFChannelData;

int main(void) {
  SystemClock_Config_HSE(); // Configure the clock first.
  HAL_Init(); // Then initialize the HAL, which sets up the SysTick based on the new clock.
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  setup_freerunning_timer();

  // Create an instance of the BMI270 driver

  HAL_Delay(2000); // Wait for USB to enumerate

  uint8_t rx_byte; // Buffer to hold one received byte

  enum CrsfState {
    IDLE,
    LENGTH,
    TYPE,
    RECEIVING,
  };

  CrsfState state = IDLE;
  uint32_t frame_start = __HAL_TIM_GET_COUNTER(&htim2);
  int byte_count=0;
  int bytes_left=0;
  int message_type = 0;
  uint8_t  rx_buffer[22];
  while (1) {
    // Run the filter. This will consume new IMU data and post a new state estimate.
    // Let the CPU rest briefly if there's nothing to do.
    // Check for incoming data on UART3.
    // This is a blocking call that will wait for 1 byte for up to 10ms.
    // If no data arrives, it will time out and the loop will continue.
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, &rx_byte, 1, 1);


    if (status == HAL_OK)
    {
    uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t elapsed_time = current_time - frame_start;
    if (elapsed_time>=1500){
            state=IDLE;
            printf("Timeout\n");
            
    }
    frame_start=current_time;

      if (state == IDLE) {
        frame_start = __HAL_TIM_GET_COUNTER(&htim2);
        state = LENGTH;
        byte_count=0;
        bytes_left=0;
      }
      else if (state == LENGTH) {
        bytes_left = rx_byte;
        state = TYPE;
      }
      else if (state == TYPE) {
        message_type = rx_byte;
        state = RECEIVING;
        bytes_left--;
        if (message_type!=0x16){
            printf("Got type 0x%X\n",message_type);
        }
      }
      else if (state == RECEIVING) {
        if (message_type==0x16){
            rx_buffer[byte_count]=rx_byte;
        }
        byte_count++;
        bytes_left--;
        if (bytes_left==0){
            state=IDLE;
            if (message_type==0x16){
                printf("RAW 0: %X\n",rx_buffer[0]);
                CRSFChannelData* data = (CRSFChannelData*)&rx_buffer;
                
                printf("Channel 1: %d\n",data->chan0);
            }
        }
        
      }
    
    }
    else if (status == HAL_TIMEOUT)
    {
    }
  }
  

}

extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
}

extern "C" void OTG_FS_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
