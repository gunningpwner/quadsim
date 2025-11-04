#ifndef CRSF_H
#define CRSF_H

#include "stm32f4xx_hal.h"
#include "SensorData.h"
#include <stdbool.h>
#include <stdint.h>

// CRSF defines from Betaflight for clarity
#define CRSF_TIME_NEEDED_PER_FRAME_US 1750
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAME_LENGTH_ADDRESS 1
#define CRSF_FRAME_LENGTH_FRAMELENGTH 1
#define CRSF_FRAME_LENGTH_TYPE_CRC 2 // Type and CRC are 2 bytes
typedef CRSFPackedChannels CRSFChannelData; // Alias for backward compatibility if needed

class Crsf {
public:
    Crsf(UART_HandleTypeDef* huart);
    void init();
    bool processFrame(const uint8_t* frame_buffer, uint8_t frame_len);
    const CRSFPackedChannels& getChannels() const;

    // Public method to be called from the UART RxEvent ISR
    void handleRxChunk(uint8_t* buf, uint16_t len);

    // Public getter for the DMA buffer
    uint8_t* getRxBuffer() { return m_dma_rx_buffer; }

private:
    UART_HandleTypeDef* m_huart;
    uint8_t m_rx_buffer[64];
    uint8_t m_frame_position;
    uint64_t m_frame_start_time;
    
    RCChannelsData m_rc_channels_data;

    // DMA buffer for HAL_UARTEx_ReceiveToIdle_DMA
    uint8_t m_dma_rx_buffer[64];
};

#endif // CRSF_H