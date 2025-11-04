#include "stm32f4xx_hal.h"
#include "common/mavlink.h"
#include "usbd_cdc_if.h"
#include "bmi270.h"
#include "timing.h"
#include "Crsf.h"
#include "DShot.h"
#include "MCE.h"
#include "MavlinkPublisher.h"
#include "peripherals.h"
#include <string.h>
#include <stdio.h>

// MAVLink system and component IDs for our vehicle
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1
// --- CRSF Integration ---
Crsf* g_crsf_ptr = nullptr;
uint8_t g_crsf_rx_byte; // Single byte buffer for the HAL_UART_Receive_IT function
// --- End CRSF Integration ---

// Global pointer to the IMU object for the interrupt handler to use.
BMI270* g_imu_ptr = nullptr;

// Global pointer to the DataManager instance, managed by the MCE.
DataManager* g_data_manager_ptr = nullptr;

// --- Robust 64-bit Timer Implementation ---
static volatile uint32_t s_overflow_count = 0;
static volatile uint32_t s_last_dwt_cyccnt = 0;

void initTime() {
    // Enable DWT Cycle Counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_last_dwt_cyccnt = DWT->CYCCNT;
}

uint64_t getCurrentTimeUs() {
    // This is the most critical section for robust timekeeping.
    // We must read the hardware counter and check for an overflow atomically.
    // Disabling interrupts briefly is the standard, safest way to do this.
    __disable_irq();

    const uint32_t current_dwt_cyccnt = DWT->CYCCNT;

    // Check for overflow against the last known value
    if (current_dwt_cyccnt < s_last_dwt_cyccnt) {
        s_overflow_count++;
    }
    s_last_dwt_cyccnt = current_dwt_cyccnt;

    // Construct the 64-bit time value *after* the overflow check.
    const uint64_t total_cycles = ((uint64_t)s_overflow_count << 32) | current_dwt_cyccnt;

    __enable_irq();

    // Scale to microseconds
    return total_cycles / (HAL_RCC_GetHCLKFreq() / 1000000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // Timer has elapsed, kick off a DMA read.
        if (g_imu_ptr) g_imu_ptr->startReadImu_DMA();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        if (g_crsf_ptr != nullptr) {
            // Process the received chunk of data
            g_crsf_ptr->handleRxChunk(g_crsf_ptr->getRxBuffer(), Size);
        }
        // Re-arm the DMA reception to catch the next frame
        if (g_crsf_ptr != nullptr) {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_crsf_ptr->getRxBuffer(), 64);
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); // Disable half-transfer interrupt
        }
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

extern "C" int _write(int file, char *ptr, int len)
{
    if (file != 1) { // stdout
        return -1;
    }

#ifdef SEND_MAVLINK_STATUSTEXT
    static char printf_buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
    static uint16_t buffer_index = 0;

    for (int i = 0; i < len; i++) {
        bool is_newline = (ptr[i] == '\n');
        bool is_buffer_full = (buffer_index == MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);

        // Send the buffer if it's full OR if we encounter a newline and it's not empty.
        if (is_buffer_full || (is_newline && buffer_index > 0)) {
            printf_buffer[buffer_index] = '\0'; // Null-terminate the string

            mavlink_message_t msg;
            uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];
            // The pack function populates the msg structure.
            mavlink_msg_statustext_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, 
                                        MAV_SEVERITY_INFO, printf_buffer);

            // mavlink_msg_to_send_buffer serializes the message and returns the correct final length.
            const uint16_t mav_len = mavlink_msg_to_send_buffer(mav_buf, &msg);
            CDC_Transmit_FS(mav_buf, mav_len);

            buffer_index = 0; // Reset buffer for the next message
        }

        // Add the character to the buffer if it's not a newline.
        if (!is_newline) {
            if (buffer_index < MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN) {
                printf_buffer[buffer_index++] = ptr[i];
            }
        }
    }
#else
    // Original behavior: send raw data over USB
    CDC_Transmit_FS((uint8_t *)ptr, len);
#endif

    return len;
}

int main(void) {
  SystemClock_Config_HSE(); 
  HAL_Init(); 

  initTime(); 

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init(); 
  MX_USART3_UART_Init(); 
  MX_USB_DEVICE_Init();

  
  MonolithicControlEntity mce;
  mce.initialize(getCurrentTimeUs);

  g_data_manager_ptr = &mce.getDataManager();

  BMI270 imu(&hspi1, GPIOC, GPIO_PIN_4);
  g_imu_ptr = &imu; 
  Crsf crsf_receiver(&huart3);
  g_crsf_ptr = &crsf_receiver;
  DShot dshot_driver(&htim1);
  MavlinkPublisher mavlink_publisher(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID);

  // Register the DShot driver's callback for motor commands
  g_data_manager_ptr->registerMotorCommandPostCallback([&dshot_driver](){ dshot_driver.onMotorCommandPosted(); });
  HAL_Delay(2000); // Wait for USB to enumerate
  printf("\n--- BMI270 Initialization ---\n");

  if (imu.init() == 0) {
      printf("BMI270 Initialized Successfully.\n");
  } else {
      printf("BMI270 Initialization Failed!\n");
      while(1);
  }

  printf("--- DShot Initialization ---\n");
  if (dshot_driver.init() == 0) {
      printf("DShot Initialized Successfully.\n");
  } else {
      printf("DShot Initialization Failed!\n");
  }


  // Start CRSF receiver using DMA and IDLE line detection
  // This is more efficient than a per-byte interrupt.
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, crsf_receiver.getRxBuffer(), 64);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); // We don't need the half-transfer interrupt

  printf("CRSF Receiver Started.\n");

  // Start the timer. It will now trigger DMA reads automatically in the background.
  HAL_TIM_Base_Start_IT(&htim2);
  

  uint64_t last_mavlink_send_time = 0;
  const uint64_t mavlink_send_interval_us = 1000000; // 50 Hz

  while (1) {
    // Run the main flight software logic.
    mce.run();
    
    // Periodically send MAVLink attitude message
    if (is_usb_vcp_connected() && (getCurrentTimeUs() - last_mavlink_send_time > mavlink_send_interval_us)) {
        mavlink_publisher.run();
        last_mavlink_send_time = getCurrentTimeUs();
    }

    // Control the on-board LED based on the MCE state
    if (mce.getCurrentState() == DisarmedState::instance()) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // Turn LED ON when disarmed
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // Turn LED OFF otherwise
    }
  }
}

extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
  // The overflow counter is now updated atomically within getCurrentTimeUs().
}

extern "C" void OTG_FS_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

extern "C" void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart3);
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

extern "C" void DMA1_Stream1_IRQHandler(void) {
    // This is the interrupt handler for USART3_RX
    HAL_DMA_IRQHandler(huart3.hdmarx);
}

extern "C" void DMA2_Stream5_IRQHandler(void) {
    // This is the interrupt handler for TIM1_UP
    HAL_DMA_IRQHandler(htim1.hdma[TIM_DMA_ID_UPDATE]);
}