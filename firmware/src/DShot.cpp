#include "DShot.h"

DShot::DShot(TIM_HandleTypeDef* htim) : _htim(htim) {
    _dma_buffer.fill(0);
}

int DShot::init() {
    // Start the PWM channels for all 4 motors
    if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4) != HAL_OK) return -1;

    // Start the DMA requests
    // This is a bit of a trick: we use the TIM_DMA_ID_UPDATE source, but we'll
    // trigger it manually. The important part is to link the timer to the DMA.
    if (HAL_TIM_Base_Start_DMA(_htim, (uint32_t*)_dma_buffer.data(), _dma_buffer.size()) != HAL_OK) {
        return -1;
    }
    // Immediately stop it, we will trigger it manually
    __HAL_TIM_DISABLE_DMA(_htim, TIM_DMA_UPDATE);

    return 0;
}

uint16_t DShot::prepare_frame(uint16_t value, bool telemetry) {
    // DShot values are 11 bits (0-2047).
    // 0 is reserved. 1-47 are for commands. 48-2047 are for throttle.
    // We'll map our 0-1999 input to 48-2047.
    uint16_t dshot_val = (value > 1999) ? 2047 : (value + 48);

    uint16_t frame = (dshot_val << 1) | (telemetry ? 1 : 0);

    // Compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = frame;
    for (int i = 0; i < 3; ++i) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x0F;

    frame = (frame << 4) | csum;

    return frame;
}

void DShot::prepare_dma_buffer(const std::array<uint16_t, 4>& motor_frames) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 16; ++j) {
            // Check the j-th bit of the i-th motor frame
            if (motor_frames[i] & (1 << (15 - j))) {
                _dma_buffer[i * 18 + j] = DSHOT_BIT_1;
            } else {
                _dma_buffer[i * 18 + j] = DSHOT_BIT_0;
            }
        }
        // Add two "zero" bits at the end for spacing between frames
        _dma_buffer[i * 18 + 16] = 0;
        _dma_buffer[i * 18 + 17] = 0;
    }
}

void DShot::write(const std::array<uint16_t, 4>& motor_values) {
    std::array<uint16_t, 4> frames;
    for (int i = 0; i < 4; ++i) {
        frames[i] = prepare_frame(motor_values[i]);
    }

    prepare_dma_buffer(frames);

    // Manually trigger a DMA transfer
    // 1. Disable the timer
    __HAL_TIM_DISABLE(_htim);
    // 2. Clear any pending DMA requests
    __HAL_DMA_CLEAR_FLAG(_htim->hdma[TIM_DMA_ID_UPDATE], __HAL_DMA_GET_TC_FLAG_INDEX(_htim->hdma[TIM_DMA_ID_UPDATE]));
    __HAL_DMA_CLEAR_FLAG(_htim->hdma[TIM_DMA_ID_UPDATE], __HAL_DMA_GET_HT_FLAG_INDEX(_htim->hdma[TIM_DMA_ID_UPDATE]));
    // 3. Set the number of data units to transfer
    _htim->hdma[TIM_DMA_ID_UPDATE]->Instance->NDTR = _dma_buffer.size();
    // 4. Set the DMA source address
    _htim->hdma[TIM_DMA_ID_UPDATE]->Instance->M0AR = (uint32_t)_dma_buffer.data();
    // 5. Set the timer counter to 0
    __HAL_TIM_SET_COUNTER(_htim, 0);
    // 6. Enable the DMA Update request
    __HAL_TIM_ENABLE_DMA(_htim, TIM_DMA_UPDATE);
    // 7. Enable the DMA stream
    __HAL_DMA_ENABLE(_htim->hdma[TIM_DMA_ID_UPDATE]);
    // 8. Enable the timer
    __HAL_TIM_ENABLE(_htim);
}