#ifndef CRSF_H
#define CRSF_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// CRSF defines from Betaflight for clarity
#define CRSF_TIME_NEEDED_PER_FRAME_US 1750
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAME_LENGTH_ADDRESS 1
#define CRSF_FRAME_LENGTH_FRAMELENGTH 1
#define CRSF_FRAME_LENGTH_TYPE_CRC 2 // Type and CRC are 2 bytes

// Packed struct for CRSF channel data (11 bits per channel)
struct CRSFChannelDataStruct {
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
} __attribute__((__packed__));
typedef struct CRSFChannelDataStruct CRSFChannelData;

class Crsf {
public:
    Crsf(UART_HandleTypeDef* huart, TIM_HandleTypeDef* htim);
    void init();
    bool processFrame();
    const CRSFChannelData& getChannels() const;

    // Public method to be called from the UART ISR
    void handleRxByte(uint8_t byte);

private:
    UART_HandleTypeDef* m_huart;
    TIM_HandleTypeDef* m_htim; // For inter-byte timeout

    volatile bool m_new_frame_ready;
    uint8_t m_rx_buffer[64];
    uint8_t m_frame_position;
    uint32_t m_frame_start_time;

    uint8_t m_latest_frame[64];
    uint8_t m_latest_frame_len;
    CRSFChannelData m_channel_data;
};

#endif // CRSF_H