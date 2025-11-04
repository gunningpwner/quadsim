#include "Crsf.h"
#include <string.h> // For memcpy
#include "timing.h"
#include "DataManager.h"

// Make the global DataManager instance from main.cpp available here.
// This allows the non-ISR context function to post data.
extern DataManager* g_data_manager_ptr;

// This is the standard CRC8-DVB-S2 used by CRSF
static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
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

Crsf::Crsf(UART_HandleTypeDef* huart)
    : m_huart(huart),
      m_frame_position(0),
      m_frame_start_time(0) {
    memset(&m_rc_channels_data, 0, sizeof(m_rc_channels_data));
    memset(m_dma_rx_buffer, 0, sizeof(m_dma_rx_buffer));
}

void Crsf::init() {
    // The caller is responsible for starting the timer and the initial UART receive interrupt.
}

void Crsf::handleRxChunk(uint8_t* buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = buf[i];
        uint64_t current_time = getCurrentTimeUs();

        // The idle line interrupt should handle frame timeouts, but as a fallback,
        // we can still check for inter-byte timeouts if needed, though it's less critical now.
        if (m_frame_position > 0 && (current_time - m_frame_start_time) > CRSF_TIME_NEEDED_PER_FRAME_US) {
            m_frame_position = 0;
        }

        if (m_frame_position == 0) {
            m_frame_start_time = current_time;
        }

        if (m_frame_position < sizeof(m_rx_buffer)) {
            m_rx_buffer[m_frame_position++] = byte;

            // The full frame length is the value of the 'length' byte plus address, length, type, and CRC bytes.
            const int full_frame_length = m_frame_position < 2 ? 5 : m_rx_buffer[1] + CRSF_FRAME_LENGTH_TYPE_CRC;

            if (m_frame_position >= 2 && m_frame_position >= full_frame_length) {
                // We have a complete frame, now validate CRC
                uint8_t calculated_crc = 0;
                // CRC includes Type and Payload (from index 2 up to but not including the CRC byte)
                for (int j = 2; j < full_frame_length - 1; j++) {
                    calculated_crc = crc8_dvb_s2(calculated_crc, m_rx_buffer[j]);
                }
                uint8_t received_crc = m_rx_buffer[full_frame_length - 1];

                if (calculated_crc == received_crc) {
                    // CRC is valid! Process the frame.
                    processFrame(m_rx_buffer, full_frame_length);
                }
                // Reset for the next frame regardless of CRC outcome
                m_frame_position = 0;
            }
        } else {
            // Buffer overflow, reset
            m_frame_position = 0;
        }
    }
}

bool Crsf::processFrame(const uint8_t* frame_buffer, uint8_t frame_len) {
    uint8_t frame_type = frame_buffer[2];

    if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        // The frame payload starts after address, length, and type bytes.
        // We copy directly into the 'channels' member of our timestamped struct.
        memcpy(&m_rc_channels_data.channels, &frame_buffer[3], sizeof(CRSFPackedChannels));

        // Set the timestamp for this new data.
        m_rc_channels_data.Timestamp = getCurrentTimeUs();

        // Post the new, timestamped RC data to the central DataManager.
        if (g_data_manager_ptr) {
            g_data_manager_ptr->post(m_rc_channels_data);
        }

        return true;
    }

    return false; // Not a channel data frame
}

const CRSFPackedChannels& Crsf::getChannels() const {
    return m_rc_channels_data.channels;
}