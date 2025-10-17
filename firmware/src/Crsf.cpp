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

Crsf::Crsf(UART_HandleTypeDef* huart, TIM_HandleTypeDef* htim)
    : m_huart(huart),
      m_htim(htim),
      m_new_frame_ready(false),
      m_frame_position(0),
      m_frame_start_time(0),
      m_latest_frame_len(0) {
    memset(&m_rc_channels_data, 0, sizeof(m_rc_channels_data));
}

void Crsf::init() {
    // The caller is responsible for starting the timer and the initial UART receive interrupt.
}

void Crsf::handleRxByte(uint8_t byte) {
    uint32_t current_time = __HAL_TIM_GET_COUNTER(m_htim);

    // Check for a timeout between bytes. If it's too long, reset.
    if (m_frame_position > 0 && (current_time - m_frame_start_time) > CRSF_TIME_NEEDED_PER_FRAME_US) {
        m_frame_position = 0;
    }

    if (m_frame_position == 0) {
        // This must be the first byte of a new frame.
        m_frame_start_time = current_time;
    }

    // The full frame length is the value of the 'length' byte plus the address and length bytes themselves.
    const int full_frame_length = m_frame_position < 2 ? 5 : m_rx_buffer[1] + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;

    if (m_frame_position < sizeof(m_rx_buffer)) {
        m_rx_buffer[m_frame_position++] = byte;

        if (m_frame_position >= full_frame_length) {
            // We have a complete frame, now validate CRC
            uint8_t calculated_crc = 0;
            // CRC includes Type and Payload
            for (int i = 2; i < full_frame_length - 1; i++) {
                 calculated_crc = crc8_dvb_s2(calculated_crc, m_rx_buffer[i]);
            }
            uint8_t received_crc = m_rx_buffer[full_frame_length - 1];

            if (calculated_crc == received_crc) {
                // CRC is valid! Copy to the main buffer and set the flag.
                memcpy(m_latest_frame, m_rx_buffer, full_frame_length);
                m_latest_frame_len = full_frame_length;
                m_new_frame_ready = true;
            }
            // Reset for the next frame regardless of CRC outcome
            m_frame_position = 0;
        }
    } else {
         // Buffer overflow, reset
        m_frame_position = 0;
    }
}

bool Crsf::processFrame() {
    if (!m_new_frame_ready) {
        return false;
    }

    // Reset the flag so we only process this frame once
    m_new_frame_ready = false;

    uint8_t frame_type = m_latest_frame[2];

    if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        // The frame payload starts after address, length, and type bytes.
        // We copy directly into the 'channels' member of our timestamped struct.
        memcpy(&m_rc_channels_data.channels, &m_latest_frame[3], sizeof(CRSFPackedChannels));

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