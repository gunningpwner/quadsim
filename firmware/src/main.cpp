#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_uart.h"
#include "common/mavlink.h"

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "bmi270.h"
#include "timing.h"
#include "DataManager.h"
#include "Consumer.h"
#include "MahonyFilter.h"
#include "OtherData.h"
#include "SensorData.h"
#include "BodyRateController.h"
#include "Crsf.h"

#include <string.h>
#include <stdio.h>

// MAVLink system and component IDs for our vehicle
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1

// --- Flight State Machine ---
enum class FlightState {
    UNINITIALIZED,
    DISARMED,
    ARMING,
    ARMED_RATE,
    // Add future modes here: ARMED_ANGLE, ARMED_POS_HOLD, etc.
    FAILSAFE
};

// Global variables
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3; // For CRSF
TIM_HandleTypeDef htim5;   // For CRSF timeout
USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_DescriptorsTypeDef FS_Desc;

// Function Prototypes
void SystemClock_Config_HSE(void);
void MX_USART3_UART_Init(void);
void MX_TIM5_Init(void);
void MX_USB_DEVICE_Init(void);
void MX_DMA_Init(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_SPI1_Init(void);

// --- CRSF Integration ---
Crsf* g_crsf_ptr = nullptr;
uint8_t g_crsf_rx_byte; // Single byte buffer for the HAL_UART_Receive_IT function
// --- End CRSF Integration ---

// Global pointer to the IMU object for the interrupt handler to use.
BMI270* g_imu_ptr = nullptr;

// Global DataManager instance, providing the time source function.
DataManager g_data_manager(getCurrentTimeUs);

// --- Robust 64-bit Timer Implementation ---
static volatile uint32_t s_overflow_count = 0;
static volatile uint32_t s_last_dwt_cyccnt = 0;

void initTime() {
    // Enable DWT Cycle Counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_last_dwt_cyccnt = DWT->CYCCNT;
}

void updateTime() {
    const uint32_t current_dwt_cyccnt = DWT->CYCCNT;
    // Check for overflow
    if (current_dwt_cyccnt < s_last_dwt_cyccnt) {
        s_overflow_count++;
    }
    s_last_dwt_cyccnt = current_dwt_cyccnt;
}

uint64_t getCurrentTimeUs() {
    // Combine the 32-bit overflow count and the 32-bit cycle counter
    // to create a full 64-bit cycle count.
    const uint64_t total_cycles = ((uint64_t)s_overflow_count << 32) | DWT->CYCCNT;
    // Scale to microseconds
    return total_cycles / (HAL_RCC_GetHCLKFreq() / 1000000);
}

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

void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration (for SPI1_RX) */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration (for SPI1_TX) */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void) {
    // TIM2 is on APB1, which has a timer clock of 84 MHz.
    // We want to trigger at 100 Hz (every 10ms).
    // For 100Hz (10ms): (839 + 1) * (999 + 1) / 84MHz = 840 * 1000 / 84M = 0.01s = 10ms.

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 840 - 1; // Gives a 100kHz timer clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1; // Count up to 1000 for a 10ms period (100Hz)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // Timer has elapsed, kick off a DMA read.
        if (g_imu_ptr) g_imu_ptr->startReadImu_DMA();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        if (g_crsf_ptr) {
            g_crsf_ptr->handleRxByte(g_crsf_rx_byte);
        }
        HAL_UART_Receive_IT(&huart3, &g_crsf_rx_byte, 1); // Re-arm interrupt
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


void MX_SPI1_Init(void) {
  // Used by the IMU
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

  // DMA handles must be static or global
  static DMA_HandleTypeDef hdma_spi1_rx;
  static DMA_HandleTypeDef hdma_spi1_tx;
  __HAL_RCC_SPI1_CLK_ENABLE();
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;

  // Configure DMA for SPI1 RX
  hdma_spi1_rx.Instance = DMA2_Stream0;
  hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
  hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi1_rx.Init.Mode = DMA_NORMAL;
  hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_spi1_rx);
  __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);

  // Configure DMA for SPI1 TX
  hdma_spi1_tx.Instance = DMA2_Stream3;
  hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
  hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi1_tx.Init.Mode = DMA_NORMAL;
  hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_spi1_tx);
  __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);

  HAL_SPI_Init(&hspi1);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // PC10 -> TX, PC11 -> RX
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0); // Lower priority than IMU
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}

void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 420000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX; // We only need to receive from the RC receiver
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
}

void MX_TIM5_Init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84 - 1; // 84MHz / 84 = 1MHz timer clock (1us per tick)
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF; // 32-bit free-running
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim5);
  HAL_TIM_Base_Start(&htim5);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {
    if(tim_baseHandle->Instance==TIM2) {
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
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
        if (ptr[i] == '\n' || buffer_index == MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN) {
            if (buffer_index > 0) {
                printf_buffer[buffer_index] = '\0'; // Null-terminate the string

                mavlink_message_t msg;
                uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];
                mavlink_msg_statustext_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                            MAV_SEVERITY_INFO, printf_buffer);

                uint16_t mav_len = mavlink_msg_to_send_buffer(mav_buf, &msg);
                CDC_Transmit_FS(mav_buf, mav_len);

                buffer_index = 0; // Reset buffer
            }
        } else {
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

/**
 * @brief Sends the attitude quaternion over MAVLink.
 * @param state The state data containing the orientation.
 */
void send_attitude_quaternion(const StateData& state) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_attitude_quaternion_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                         state.Timestamp / 1000, // time_boot_ms
                                         state.orientation.w(),
                                         state.orientation.x(),
                                         state.orientation.y(),
                                         state.orientation.z(),
                                         0.0f, 0.0f, 0.0f); // rollspeed, pitchspeed, yawspeed (not available in StateData)

    // Copy the message to a buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the buffer over USB VCP
    CDC_Transmit_FS(buf, len);
}

int main(void) {
  SystemClock_Config_HSE(); // Configure the clock first.
  HAL_Init(); // Then initialize the HAL, which sets up the SysTick based on the new clock.

  initTime(); // Initialize our robust 64-bit timer

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init(); // For CRSF
  MX_USART3_UART_Init(); // For CRSF
  MX_USB_DEVICE_Init();

  // Create an instance of the BMI270 driver
  BMI270 imu(&hspi1, GPIOC, GPIO_PIN_4);
  g_imu_ptr = &imu; // Set the global pointer
  Crsf crsf_receiver(&huart3, &htim5);
  g_crsf_ptr = &crsf_receiver;

  HAL_Delay(2000); // Wait for USB to enumerate
  printf("\n--- BMI270 Initialization ---\n");

  if (imu.init() == 0) {
      printf("BMI270 Initialized Successfully.\n");
  } else {
      printf("BMI270 Initialization Failed!\n");
      // It's good practice to halt if essential hardware fails
      while(1);
  }

  // Start CRSF receiver (kicks off the first interrupt-driven receive)
  HAL_UART_Receive_IT(&huart3, &g_crsf_rx_byte, 1);
  printf("CRSF Receiver Started.\n");

  // Start the timer. It will now trigger DMA reads automatically in the background.
  HAL_TIM_Base_Start_IT(&htim2);

  // --- Instantiate the Orientation Filter ---
  MahonyFilter mahony_filter(g_data_manager);

  // --- Instantiate Controllers ---
  BodyRateController rate_controller(g_data_manager);

  // --- State Machine Initialization ---
  FlightState current_state = FlightState::DISARMED;
  RCChannelsData last_rc_frame;
  uint64_t last_rc_frame_time = 0;
  const uint64_t rc_timeout_us = 500000; // 500ms

  while (1) {
    // --- Universal Tasks (Run in every state) ---

    // 1. Run the orientation filter. It consumes IMU data and produces state estimates
    //    that are needed by controllers and for telemetry.
    mahony_filter.run();

    // 2. Check for and process new RC commands. This is the primary input for state transitions.
    if (crsf_receiver.processFrame()) {
        g_data_manager.getLatest(last_rc_frame);
        last_rc_frame_time = last_rc_frame.Timestamp;
    }

    // 3. Check for RC link failure (Failsafe)
    if (current_state != FlightState::DISARMED && (getCurrentTimeUs() - last_rc_frame_time > rc_timeout_us)) {
        current_state = FlightState::FAILSAFE;
    }

    // --- Main State Machine ---
    switch (current_state) {
        case FlightState::UNINITIALIZED:
            // Should not be in this state after setup.
            // Maybe flash an error LED pattern.
            break;

        case FlightState::DISARMED:
        {
            // Action: Ensure motors are off.
            MotorCommands zero_commands = {}; // All motors at 0.0f
            g_data_manager.post(zero_commands);

            // Transition: Check for arming sequence.
            // Example: Throttle low (chan2 < 200) and Arm switch high (chan4 > 1800)
            // CRSF values range from ~172 to 1811.
            const auto& channels = crsf_receiver.getChannels();
            if (channels.chan2 < 200 && channels.chan4 > 1800) {
                printf("ARMING...\n");
                current_state = FlightState::ARMED_RATE;
            }
            break;
        }

        case FlightState::ARMED_RATE:
        {
            // Action: Run the body rate controller.
            rate_controller.run();

            // Transition: Check for disarming sequence.
            // Example: Arm switch low (chan4 < 200)
            const auto& channels = crsf_receiver.getChannels();
            if (channels.chan4 < 200) {
                printf("DISARMING...\n");
                current_state = FlightState::DISARMED;
            }

            // Transition: Check for mode switch (future).
            // if (channels.chan5 > 1800) { current_state = FlightState::ARMED_ANGLE; }

            break;
        }

        case FlightState::FAILSAFE:
        {
            // Action: Failsafe behavior. For now, just cut the motors.
            // A more advanced implementation might try to auto-land.
            printf("FAILSAFE: RC link lost!\n");
            MotorCommands zero_commands = {};
            g_data_manager.post(zero_commands);

            // Transition: Check if RC link is restored.
            if (getCurrentTimeUs() - last_rc_frame_time < rc_timeout_us) {
                printf("RC link restored. Returning to DISARMED state.\n");
                current_state = FlightState::DISARMED; // Go to a safe state
            }
            break;
        }

        default:
            // Should not happen. Go to a safe state.
            printf("Unknown state! Forcing DISARMED.\n");
            current_state = FlightState::DISARMED;
            break;
    }


  }
}

extern "C" void SysTick_Handler(void) {
  HAL_IncTick();
  updateTime(); // Update our overflow counter
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