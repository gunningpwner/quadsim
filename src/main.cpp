#include "stm32f4xx_hal.h"
#include "timing.h"
#include "drivers/usbd_cdc_if.h"
#include "drivers/bmi270.h"
#include "drivers/qmc5883l.h"
#include "drivers/W25Q128FV.h"
#include "drivers/Crsf.h"
#include "drivers/DShot.h"
#include "drivers/GPS.h"
#include "MCE.h"

#include "peripherals.h"
#include <string.h>
#include <stdio.h>

#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1

Crsf *g_crsf_ptr = nullptr;
BMI270 *g_imu_ptr = nullptr;
GPS *g_gps_ptr = nullptr;
QMC5883L *g_compass_ptr = nullptr;
W25Q128FV *g_flash_ptr = nullptr;

static volatile uint32_t s_overflow_count = 0;
static volatile uint32_t s_last_dwt_cyccnt = 0;

extern uint8_t RxLastPacket[];
extern volatile uint32_t RxPacketLen;
extern volatile uint8_t NewDataFlag;


void initTime()
{
    // Enable DWT Cycle Counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_last_dwt_cyccnt = DWT->CYCCNT;
}

uint64_t getCurrentTimeUs()
{

    // If nothing calls this for a while, we could miss an overflow
    // There would have to be >~25.6 seconds for this to happen
    // Seems unlikely so leaving for now

    // Disables interrupts
    __disable_irq();

    const uint32_t current_dwt_cyccnt = DWT->CYCCNT;
    if (current_dwt_cyccnt < s_last_dwt_cyccnt)
    {
        s_overflow_count++;
    }
    s_last_dwt_cyccnt = current_dwt_cyccnt;

    const uint64_t total_cycles = ((uint64_t)s_overflow_count << 32) | current_dwt_cyccnt;
    __enable_irq();

    // Scale to microseconds
    return total_cycles / (HAL_RCC_GetHCLKFreq() / 1000000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        // Timer 2 set to run at 100Hz. Use counter to trigger other functions at lower rates
        static size_t counter = 0;
        if (g_imu_ptr)
            g_imu_ptr->startReadImu_DMA();

        if (counter % 10 == 0)
        {
            if (g_compass_ptr)
            {
                g_compass_ptr->startReadCompass_DMA();
            }
        }
        counter++;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        if (g_crsf_ptr != nullptr)
        {
            g_crsf_ptr->handleRxChunk(g_crsf_ptr->getRxBuffer(), Size);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_crsf_ptr->getRxBuffer(), 64);

            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); // Disable half-transfer interrupt
        }
    }
    else if (huart->Instance == UART4)
    {
        g_gps_ptr->handleRxChunk(g_gps_ptr->getRxBuffer(), Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_gps_ptr->getRxBuffer(), 100);
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        // This function is called when the SPI DMA transfer is complete.
        // We can now process the raw data that the DMA has moved for us.
        if (g_imu_ptr != nullptr)
        {
            g_imu_ptr->processRawData();
        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) //Flash
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        if (g_flash_ptr) {
            g_flash_ptr->m_dma_transfer_active = false; // Clear flag
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        if (g_compass_ptr != nullptr)
        {
            g_compass_ptr->processRawData();
        }
    }
}

extern "C" int _write(int file, char *ptr, int len)
{
    if (file != 1)
    { // stdout
        return -1;
    }

    // Original behavior: send raw data over USB
    CDC_Transmit_FS((uint8_t *)ptr, len);

    return len;
}

void CheckUsb(void) {
    if (NewDataFlag == 1) {
        // Optional: Null-terminate the string for easier comparison
        // (Make sure RxPacketLen < 64 before doing this to be safe)
        RxLastPacket[RxPacketLen] = 0; 

        if (strncmp((char*)RxLastPacket, "DUMP", 4) == 0) {
            g_flash_ptr->stopWriting();
            g_flash_ptr->dumpDataToUSB();

        } else if ((strncmp((char*)RxLastPacket, "START", 5) == 0))
        {
            printf("Starting flash write\n");
            g_flash_ptr->startWriting();
        } else if ((strncmp((char*)RxLastPacket, "STOP", 4) == 0))
        {
            printf("Stopping flash write\n");
            g_flash_ptr->stopWriting();
        }
        
        
        // Clear flag so we don't re-read the same command
        NewDataFlag = 0;
    }
}

/**
 * @brief Reads a value from a specific ADC channel.
 * @param channel The ADC channel to read (e.g., ADC_CHANNEL_10).
 * @return The 12-bit ADC value, or 0 on failure.
 */
uint16_t read_adc_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure channel
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0;
    }

    // Start, wait for, and stop conversion
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return value;
}

/**
 * @brief Handles sending various CRSF telemetry packets.
 *
 * This function is designed to be called periodically. It alternates between sending
 * different types of telemetry data to avoid flooding the CRSF link.
 *
 * @param crsf_receiver A reference to the Crsf object used for sending packets.
 */
void handle_telemetry(Crsf &crsf_receiver)
{
    static int telemetry_phase = 0;

    switch (telemetry_phase)
    {
    case 0:
    {
        // Phase 0: Send Battery Sensor packet (CRSF Frame Type 0x08)
        // Payload:
        // uint16_t voltage (100mV units)
        // uint16_t current (100mA units)
        // uint24_t capacity (mAh)
        // uint8_t  remaining (%)
        uint8_t battery_payload[8];

        // --- Read and calculate real voltage ---
        // Voltage divider: R1=10k, R2=1k. Scale factor = (10+1)/1 = 11.
        // ADC_REF = 3.3V, ADC_MAX = 4095.
        // Voltage = (ADC_Value / 4095) * 3.3 * 11
        const float VOLTAGE_DIVIDER_SCALE = 11.0f;
        uint16_t adc_vbat = read_adc_channel(ADC_CHANNEL_10);
        float voltage_f = ((float)adc_vbat / 4095.0f) * 3.3f * VOLTAGE_DIVIDER_SCALE;
        uint16_t voltage_crsf = (uint16_t)(voltage_f * 10); // CRSF units are 0.1V

        // --- Read and calculate real current ---
        // Current sensor scale: 118mV/A (from config.h). Offset is VCC/2 for no current.
        // Current = ( (ADC_Value / 4095) * 3.3 - 1.65) / 0.118
        uint16_t adc_curr = read_adc_channel(ADC_CHANNEL_11);
        float current_f = (((float)adc_curr / 4095.0f) * 3.3f - 1.65f) / 0.118f;
        uint16_t current_crsf = (uint16_t)(fmax(0.0f, current_f) * 10); // CRSF units are 0.1A

        uint32_t capacity_drawn = 500;     // Placeholder for 500mAh drawn
        uint8_t remaining_percentage = 75; // Placeholder for 75%

        battery_payload[0] = (voltage_crsf >> 8) & 0xFF;
        battery_payload[1] = voltage_crsf & 0xFF;
        battery_payload[2] = (current_crsf >> 8) & 0xFF;
        battery_payload[3] = current_crsf & 0xFF;
        battery_payload[4] = (capacity_drawn >> 16) & 0xFF;
        battery_payload[5] = (capacity_drawn >> 8) & 0xFF;
        battery_payload[6] = capacity_drawn & 0xFF;
        battery_payload[7] = remaining_percentage;

        crsf_receiver.sendPacket(CRSF_FRAMETYPE_BATTERY_SENSOR, battery_payload, sizeof(battery_payload));
        break;
    }
    case 1:
    {
        // Phase 1: Send GPS packet (CRSF Frame Type 0x02)
        // Payload:
        // int32_t latitude, int32_t longitude (degrees * 1e7)
        // uint16_t groundspeed (km/h * 10)
        // uint16_t heading (deg * 100)
        // uint16_t altitude (meters, offset 1000)
        // uint8_t satellites
        uint8_t gps_payload[15];
        // Using memcpy for strict aliasing safety with multi-byte types.
        int32_t lat = 476432130;    // Placeholder for 47.6432130 degrees
        int32_t lon = -1221034230;  // Placeholder for -122.1034230 degrees
        uint16_t groundspeed = 500; // Placeholder for 50.0 km/h
        uint16_t heading = 18000;   // Placeholder for 180.00 degrees
        uint16_t altitude = 1123;   // Placeholder for 123m (1000m offset)
        uint8_t satellites = 15;    // Placeholder for 15 satellites

        memcpy(&gps_payload[0], &lat, 4);
        memcpy(&gps_payload[4], &lon, 4);
        memcpy(&gps_payload[8], &groundspeed, 2);
        memcpy(&gps_payload[10], &heading, 2);
        memcpy(&gps_payload[12], &altitude, 2);
        gps_payload[14] = satellites;

        crsf_receiver.sendPacket(CRSF_FRAMETYPE_GPS, gps_payload, sizeof(gps_payload));
        break;
    }
    }

    // Cycle to the next telemetry type for the next call
    telemetry_phase = (telemetry_phase + 1) % 2;
}

int main(void)
{
    SystemClock_Config_HSE();
    HAL_Init();

    initTime();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_TIM8_Init();
    MX_USART3_UART_Init();
    MX_UART4_Init();
    MX_ADC1_Init();
    MX_USB_DEVICE_Init();
    MX_I2C1_Init();

    MonolithicControlEntity mce;
    DataManager &data_manager = mce.getDataManager();

    BMI270 imu(data_manager.getSensorBuffer(), &hspi1, GPIOC, GPIO_PIN_4);
    g_imu_ptr = &imu;
    Crsf crsf_receiver(&huart3, data_manager.getRCChannelsBuffer());
    g_crsf_ptr = &crsf_receiver;
    GPS gps_driver(data_manager.getSensorBuffer());
    g_gps_ptr = &gps_driver;
    QMC5883L compass(data_manager.getSensorBuffer());
    g_compass_ptr = &compass;
    W25Q128FV flash(data_manager.makeSensorConsumer());
    g_flash_ptr = &flash;
    DShot dshot_driver(data_manager.makeMotorCommandsConsumer());

    HAL_Delay(2000); // Wait for USB to enumerate
    printf("\n--- BMI270 Initialization ---\n");

    if (imu.init() == 0)
    {
        printf("BMI270 Initialized Successfully.\n");
    }
    else
    {
        printf("BMI270 Initialization Failed!\n");
    }

    printf("--- DShot Initialization ---\n");
    if (dshot_driver.init() == 0)
    {
        printf("DShot Initialized Successfully.\n");
    }
    else
    {
        printf("DShot Initialization Failed!\n");
    }

    printf("--- GPS Initialization ---\n");
    if (gps_driver.init() == 0)
    {
        printf("GPS Initialized Successfully.\n");
    }
    else
    {
        printf("GPS Initialization Failed!\n");
    }

    printf("--- Compass Initialization ---\n");

    if (compass.init() == 0)
    {
        printf("Compass Initialized Successfully.\n");
    }
    else
    {
        printf("Compass Initialization Failed!\n");
    }

    printf("--- Flash Initialization ---\n");
    if (flash.init() == 0)
    {
        printf("Flash Initialized Successfully.\n");
    }
    else
    {
        printf("Flash Initialization Failed!\n");
    }

    //
    mce.initialize(&dshot_driver);

    // Start CRSF receiver using DMA and IDLE line detection
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, crsf_receiver.getRxBuffer(), 64);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

    HAL_TIM_Base_Start_IT(&htim2);

    printf("CRSF Receiver Started.\n");

    uint64_t last_mavlink_send_time = 0;
    const uint64_t mavlink_send_interval_us = 10000;

    volatile uint64_t last_crsf_telemetry_time = 0;
    const uint64_t crsf_telemetry_interval_us = 100000;

    while (1)
    {
        flash.run();
        CheckUsb();
        // Run the main flight software logic.

        // mce.run();

        // // Periodically send CRSF telemetry
        // if (getCurrentTimeUs() - last_crsf_telemetry_time > crsf_telemetry_interval_us) {
        //     handle_telemetry(crsf_receiver);
        //     last_crsf_telemetry_time = getCurrentTimeUs();
        // }

        // // Periodically send MAVLink attitude message
        // if (is_usb_vcp_connected() && (getCurrentTimeUs() - last_mavlink_send_time > mavlink_send_interval_us)) {
        //     mavlink_publisher.run();
        //     last_mavlink_send_time = getCurrentTimeUs();
        // }

        // // Control the on-board LED based on the MCE state
        // if (mce.getCurrentState() == DisarmedState::instance()) {
        //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // Turn LED ON when disarmed
        // } else {
        //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // Turn LED OFF otherwise
        // }
    }
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

extern "C" void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

extern "C" void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

extern "C" void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

extern "C" void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

extern "C" void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hspi1.hdmarx);
}

extern "C" void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hspi1.hdmatx);
}

extern "C" void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hi2c1.hdmarx);
}

extern "C" void DMA1_Stream1_IRQHandler(void)
{

    HAL_DMA_IRQHandler(huart3.hdmarx);
}

extern "C" void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(huart4.hdmarx);
}

extern "C" void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hspi2.hdmarx);
}

extern "C" void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hspi2.hdmatx);
}

extern "C" void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hi2c1.hdmatx);
}