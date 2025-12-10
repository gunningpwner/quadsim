#pragma once
#include "stm32f4xx_hal.h"

class GPS
{
public:
    GPS();

    int8_t init();

    void handleRxChunk(uint8_t *buf, uint16_t len);

    uint8_t* getRxBuffer() { return m_dma_rx_buffer; }

private:
    uint8_t m_dma_rx_buffer[256];

    void writeUBXMessage(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);
};