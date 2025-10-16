#include "DShot.h"

// Note: The DMA buffer must be in a region accessible by the DMA controller (e.g., SRAM1/SRAM2).
DShot::DShot(TIM_HandleTypeDef* htim) : _htim(htim) {
    _dma_buffer.fill(0);
}

int DShot::init() {
    // Start the PWM channels for all 4 motors
    if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4) != HAL_OK) return -1;

    // Configure the timer to trigger a DMA transfer on its update event (when the counter overflows).
    // This DMA request will perform a "burst" transfer to update all 4 CCR registers at once.
    // 1. Set the DMA destination to the special DMAR register.
    // 2. Specify the burst length (4 registers: CCR1, CCR2, CCR3, CCR4).
    // 3. The DMA source will be our prepared buffer.
    // The HAL_TIM_Base_Start_DMA function is not suitable for this burst mode setup.
    // We will manage the DMA transfer manually in the write() function.
    // The key linkage is done in HAL_TIM_Base_MspInit by __HAL_LINKDMA.

    // Set the DMA burst transfer properties for the timer.
    _htim->Instance->DCR = (TIM_DMABASE_CCR1 << 8) | (4 - 1); // Start at CCR1, burst length of 4

    return 0;
}

uint16_t DShot::prepare_frame(uint16_t value, bool telemetry) {
    uint16_t dshot_val = value; // The value is now expected to be in the final 0-2047 range.

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
    // For burst DMA, we must interleave the data: [M1_B0, M2_B0, M3_B0, M4_B0, M1_B1, M2_B1, ...]
    for (int bit = 0; bit < 16; ++bit) {
        for (int motor = 0; motor < 4; ++motor) {
            // Check the current bit (from MSB to LSB) for the current motor
            if (motor_frames[motor] & (1 << (15 - bit))) {
                _dma_buffer[bit * 4 + motor] = DSHOT_BIT_1;
            } else {
                _dma_buffer[bit * 4 + motor] = DSHOT_BIT_0;
            }
        }
    }
    // Add two "zero" bits at the end for spacing between DShot frames.
    // This writes 0 to the CCRs, ensuring the line is low.
    for (int i = 16 * 4; i < 18 * 4; ++i) {
        _dma_buffer[i] = 0;
    }
}

void DShot::write(const std::array<uint16_t, 4>& motor_values) {
    std::array<uint16_t, 4> frames;
    // This function handles throttle values.
    // We map the input range [0, 1999] to the DShot throttle range [48, 2047].
    // A value of 0 from the controller means motor stop (DShot value 0).
    for (int i = 0; i < 4; ++i) {
        uint16_t dshot_val = (motor_values[i] == 0) ? 0 : motor_values[i] + 48;
        dshot_val = (dshot_val > 2047) ? 2047 : dshot_val;
        frames[i] = prepare_frame(dshot_val);
    }

    prepare_dma_buffer(frames);

    // Manually trigger a DMA transfer
    // This sequence ensures a clean start for each DShot packet train.

    // Ensure DMA is disabled before reconfiguring
    __HAL_DMA_DISABLE(_htim->hdma[TIM_DMA_ID_UPDATE]);

    // 1. Disable the timer
    __HAL_TIM_DISABLE(_htim);

    // 2. Clear any pending DMA transfer flags for the associated stream.
    // This is more portable than hardcoding the flag values.
    DMA_HandleTypeDef* hdma = _htim->hdma[TIM_DMA_ID_UPDATE];
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma)); // Transfer Complete
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma)); // Half Transfer
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma)); // Transfer Error
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma)); // FIFO Error
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));// Direct Mode Error

    // 3. Set the number of data units to transfer
    _htim->hdma[TIM_DMA_ID_UPDATE]->Instance->NDTR = _dma_buffer.size();
    // 4. Set the DMA source and destination addresses
    _htim->hdma[TIM_DMA_ID_UPDATE]->Instance->M0AR = (uint32_t)_dma_buffer.data();
    _htim->hdma[TIM_DMA_ID_UPDATE]->Instance->PAR = (uint32_t)&_htim->Instance->DMAR;

    // 5. Set the timer counter to 0
    __HAL_TIM_SET_COUNTER(_htim, 0);

    // 6. Enable the DMA stream and the timer's DMA request
    __HAL_DMA_ENABLE(_htim->hdma[TIM_DMA_ID_UPDATE]);
    __HAL_TIM_ENABLE_DMA(_htim, TIM_DMA_UPDATE);

    // 7. Enable the timer to start the process
    __HAL_TIM_ENABLE(_htim);
}

void DShot::write_command(DShot_Command command, bool telemetry) {
    // This function sends a command value directly, without any mapping.
    // The same command is sent to all 4 motors.
    uint16_t command_val = static_cast<uint16_t>(command);

    std::array<uint16_t, 4> frames;
    for (int i = 0; i < 4; ++i) {
        frames[i] = prepare_frame(command_val, telemetry);
    }

    prepare_dma_buffer(frames);


    // Ensure DMA is disabled before reconfiguring
    __HAL_DMA_DISABLE(_htim->hdma[TIM_DMA_ID_UPDATE]);

    // 1. Disable the timer
    __HAL_TIM_DISABLE(_htim);

    // 2. Clear any pending DMA transfer flags
    DMA_HandleTypeDef* hdma = _htim->hdma[TIM_DMA_ID_UPDATE];
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));

    // 3. Set the number of data units to transfer
    _htim->hdma[TIM_DMA_ID_UPDATE]->Instance->NDTR = _dma_buffer.size();
    // 4. Set the DMA source and destination addresses
    _htim->hdma[TIM_DMA_ID_UPDATE]->Instance->M0AR = (uint32_t)_dma_buffer.data();
    _htim->hdma[TIM_DMA_ID_UPDATE]->Instance->PAR = (uint32_t)&_htim->Instance->DMAR;

    // 5. Set the timer counter to 0
    __HAL_TIM_SET_COUNTER(_htim, 0);

    // 6. Enable the DMA stream and the timer's DMA request
    __HAL_DMA_ENABLE(_htim->hdma[TIM_DMA_ID_UPDATE]);
    __HAL_TIM_ENABLE_DMA(_htim, TIM_DMA_UPDATE);

    // 7. Enable the timer to start the process
    __HAL_TIM_ENABLE(_htim);
}