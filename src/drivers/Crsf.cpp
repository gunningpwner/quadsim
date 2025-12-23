#include "drivers/Crsf.h"
#include <string.h> // For memcpy
#include "timing.h"

// Make the global DataManager instance from main.cpp available here.
// This allows the non-ISR context function to post data.

// This is the standard CRC8-DVB-S2 used by CRSF
static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii)
    {
        if (crc & 0x80)
        {
            crc = (crc << 1) ^ 0xD5;
        }
        else
        {
            crc = crc << 1;
        }
    }
    return crc;
}

Crsf::Crsf(UART_HandleTypeDef *huart, DataManager::RCChannelsBuffer &m_rc_channel_buffer)
    : m_huart(huart),
      m_rc_channel_buffer(m_rc_channel_buffer),
      m_frame_position(0),
      m_frame_start_time(0)
{
    memset(m_dma_rx_buffer, 0, sizeof(m_dma_rx_buffer));
}

void Crsf::init()
{
    // The caller is responsible for starting the timer and the initial UART receive interrupt.
}

void Crsf::handleRxChunk(uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t byte = buf[i];
        uint64_t current_time = getCurrentTimeUs();

        // The idle line interrupt should handle frame timeouts, but as a fallback,
        // we can still check for inter-byte timeouts if needed, though it's less critical now.
        if (m_frame_position > 0 && (current_time - m_frame_start_time) > CRSF_TIME_NEEDED_PER_FRAME_US)
        {
            m_frame_position = 0;
        }

        if (m_frame_position == 0)
        {
            m_frame_start_time = current_time;
        }

        if (m_frame_position < sizeof(m_rx_buffer))
        {
            m_rx_buffer[m_frame_position++] = byte;

            // The full frame length is the value of the 'length' byte plus address, length, type, and CRC bytes.
            const int full_frame_length = m_frame_position < 2 ? 5 : m_rx_buffer[1] + CRSF_FRAME_LENGTH_TYPE_CRC;

            if (m_frame_position >= 2 && m_frame_position >= full_frame_length)
            {
                // We have a complete frame, now validate CRC
                uint8_t calculated_crc = 0;
                // CRC includes Type and Payload (from index 2 up to but not including the CRC byte)
                for (int j = 2; j < full_frame_length - 1; j++)
                {
                    calculated_crc = crc8_dvb_s2(calculated_crc, m_rx_buffer[j]);
                }
                uint8_t received_crc = m_rx_buffer[full_frame_length - 1];

                if (calculated_crc == received_crc)
                {
                    // CRC is valid! Process the frame.
                    processFrame(m_rx_buffer, full_frame_length);
                }
                // Reset for the next frame regardless of CRC outcome
                m_frame_position = 0;
            }
        }
        else
        {
            // Buffer overflow, reset
            m_frame_position = 0;
        }
    }
}

bool Crsf::processFrame(const uint8_t *frame_buffer, uint8_t frame_len)
{
    uint8_t frame_type = frame_buffer[2];

    if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {   
        uint64_t timestamp = getCurrentTimeUs();

        RCChannelsData *rc_data = m_rc_channel_buffer.claim();
        rc_data->timestamp = timestamp;
        memcpy(&rc_data->channels, &frame_buffer[3], sizeof(CRSFPackedChannels));
        m_rc_channel_buffer.commit(rc_data);

        return true;
    }

    return false; // Not a channel data frame
}



void Crsf::sendPacket(uint8_t type, const uint8_t *payload, uint8_t len)
{
    // Max CRSF payload is 60 bytes, so a 64-byte buffer is safe.
    uint8_t tx_buffer[64];

    // Frame length = Type (1) + Payload (len) + CRC (1)
    uint8_t frame_len = len + 2;

    tx_buffer[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx_buffer[1] = frame_len;
    tx_buffer[2] = type;
    memcpy(&tx_buffer[3], payload, len);

    uint8_t crc = crc8_dvb_s2(0, type);
    for (uint8_t i = 0; i < len; i++)
    {
        crc = crc8_dvb_s2(crc, payload[i]);
    }
    tx_buffer[3 + len] = crc;

    HAL_UART_Transmit(m_huart, tx_buffer, frame_len + 2, 10); // +2 for address and length bytes
}