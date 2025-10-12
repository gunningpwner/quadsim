#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

#include <string.h>
#include <stdio.h>

// Function Prototypes
void SystemClock_Config_HSE(void);
void MX_USB_DEVICE_Init(void);
void MX_USART3_UART_Init(void);
void setup_freerunning_timer(void);

// Global variables
UART_HandleTypeDef huart3;
USBD_HandleTypeDef hUsbDeviceFS;
TIM_HandleTypeDef htim2;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_DescriptorsTypeDef FS_Desc;


// --- NEW: CRSF Parsing Globals ---

// Use a volatile flag to signal a new frame from the ISR to the main loop
volatile bool new_crsf_frame_ready = false;

// Buffers and state for the ISR to parse incoming frames
uint8_t crsf_rx_buffer[64];      // Buffer to build the incoming frame
uint8_t crsf_frame_position = 0; // Current position in the rx buffer
uint32_t crsf_frame_start_time = 0; // Timestamp of the first byte

// A separate buffer to hold the last complete, validated frame for the main loop to process
uint8_t crsf_latest_frame[64];
uint8_t crsf_latest_frame_len = 0;
uint8_t rx_isr_byte; // Single byte buffer for the HAL_UART_Receive_IT function

// CRSF defines from Betaflight for clarity
#define CRSF_TIME_NEEDED_PER_FRAME_US 1750
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAME_LENGTH_ADDRESS 1
#define CRSF_FRAME_LENGTH_FRAMELENGTH 1
#define CRSF_FRAME_LENGTH_TYPE_CRC 2 // Type and CRC are 2 bytes

// Packed struct for CRSF channel data (11 bits per channel)
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


// --- NEW: CRC8 calculation function ---
// This is the standard CRC8-DVB-S2 used by CRSF
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// --- NEW: UART Interrupt Callback ---
// This function is called by the HAL driver every time a byte is received.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim2);

        // Check for a timeout between bytes. If it's too long, reset.
        // This prevents the parser from getting stuck on a partial frame.
        if (crsf_frame_position > 0 && (current_time - crsf_frame_start_time) > CRSF_TIME_NEEDED_PER_FRAME_US) {
            crsf_frame_position = 0;
        }

        if (crsf_frame_position == 0) {
            // This must be the first byte of a new frame.
            // Note: CRSF sync byte (the address) is not checked here but is included in the frame.
            crsf_frame_start_time = current_time;
        }
        
        // The full frame length is the value of the 'length' byte plus the address and length bytes themselves.
        // We guess a minimum length until we've received the actual length byte.
        const int full_frame_length = crsf_frame_position < 2 ? 5 : crsf_rx_buffer[1] + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;

        if (crsf_frame_position < sizeof(crsf_rx_buffer)) {
            crsf_rx_buffer[crsf_frame_position++] = rx_isr_byte;

            if (crsf_frame_position >= full_frame_length) {
                // We have a complete frame, now validate CRC
                uint8_t calculated_crc = 0;
                // CRC includes Type and Payload
                for (int i = 2; i < full_frame_length - 1; i++) {
                     calculated_crc = crc8_dvb_s2(calculated_crc, crsf_rx_buffer[i]);
                }
                uint8_t received_crc = crsf_rx_buffer[full_frame_length - 1];

                if (calculated_crc == received_crc) {
                    // CRC is valid! Copy to the main buffer and set the flag.
                    memcpy(crsf_latest_frame, crsf_rx_buffer, full_frame_length);
                    crsf_latest_frame_len = full_frame_length;
                    new_crsf_frame_ready = true;
                }
                // Reset for the next frame regardless of CRC outcome
                crsf_frame_position = 0;
            }
        } else {
             // Buffer overflow, reset
            crsf_frame_position = 0;
        }

        // IMPORTANT: Re-arm the interrupt to receive the next byte
        HAL_UART_Receive_IT(&huart3, &rx_isr_byte, 1);
    }
}


int main(void) {
  SystemClock_Config_HSE();
  HAL_Init();
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  setup_freerunning_timer();

  HAL_Delay(2000); // Wait for USB to enumerate
  printf("CRSF Parser Initialized.\r\n");

  // --- MODIFIED: Start interrupt-driven UART reception ---
  // This kicks off the process. The callback will re-arm the interrupt each time.
  HAL_UART_Receive_IT(&huart3, &rx_isr_byte, 1);


  while (1) {
    // --- MODIFIED: Main loop logic ---
    // The main loop is now non-blocking. It just checks a flag.
    
    if (new_crsf_frame_ready) {
        // A new, CRC-validated frame has been received by the ISR.
        // Let's process it.

        // Reset the flag so we only process this frame once
        new_crsf_frame_ready = false;

        // The 'type' is the 3rd byte (index 2) of the frame
        uint8_t frame_type = crsf_latest_frame[2];

        if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            // The frame payload starts after address, length, and type bytes.
            CRSFChannelData* data = (CRSFChannelData*)&crsf_latest_frame[3];
            
            // Print out one of the channels to verify
            printf("Ch0: %4u | Ch1: %4u | Ch2: %4u | Ch3: %4u\r\n", 
                    data->chan0, data->chan1, data->chan2, data->chan3);
        }
        else {
            // You can add handlers for other frame types here if needed
            printf("Received frame of type: 0x%02X\r\n", frame_type);
        }
    }

    // The CPU is now free to do other things here, like:
    // - Run sensor fusion algorithms
    // - Calculate PID loops
    // - Log telemetry data
    // etc.
  }
}

// System configuration functions (no changes needed)

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
    
    // --- NEW: Enable UART Interrupt ---
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
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

extern "C" void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart3);
}

extern "C" int _write(int file, char *ptr, int len)
{
    if (file != 1) { // stdout
        return -1;
    }
    CDC_Transmit_FS((uint8_t *)ptr, len);
    return len;
}

extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
}

extern "C" void OTG_FS_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}