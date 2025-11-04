#include "DShot.h"
#include "DataManager.h"
#include "OtherData.h" // For MotorCommands

// Make the global DataManager instance from main.cpp available here.
extern DataManager* g_data_manager_ptr;

// Note: The DMA buffer must be in a region accessible by the DMA controller (e.g., SRAM1/SRAM2).
DShot::DShot(TIM_HandleTypeDef* htim)
    : _htim(htim),
      m_motor_commands_consumer(g_data_manager_ptr->getMotorCommandsChannel())
{
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

    // The ESC apparently needs several repeated messages in order to arm
    MotorCommands armCmd = {.command=DShot_Command::MOTOR_STOP,
                            .is_throttle_command=false};

    for (int i = 0; i < 100; ++i) {
        sendMotorCommand(armCmd);
        // Also apparently need at least 2us between frames
        HAL_Delay(1);
    }
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
void DShot::sendMotorCommand(MotorCommands& cmd){
    std::array<uint16_t, 4> frames;

    if (cmd.is_throttle_command) {
        // This is a throttle command.
        // We map the input range [0, 1999] to the DShot throttle range [48, 2047].
        // A value of 0 from the controller means motor stop (DShot value 0).
        for (int i = 0; i < 4; ++i) {
            uint16_t dshot_val = (cmd.throttle[i] == 0) ? 0 : cmd.throttle[i] + 48;
            dshot_val = (dshot_val > 2047) ? 2047 : dshot_val;
            frames[i] = prepare_frame(dshot_val);
        }
    } else {
        // This is a special command (e.g., MOTOR_STOP, BEEP).
        // The same command is sent to all 4 motors.
        uint16_t command_val = static_cast<uint16_t>(cmd.command);
        for (int i = 0; i < 4; ++i) {
            frames[i] = prepare_frame(command_val, false); // Telemetry is false for commands for now
        }
    }

    prepare_dma_buffer(frames);

    HAL_TIM_Base_Stop_DMA(_htim);
    __HAL_DMA_DISABLE(_htim->hdma[TIM_DMA_ID_UPDATE]);

    HAL_TIM_Base_Start_DMA(_htim, (uint32_t*)_dma_buffer.data(), _dma_buffer.size());
}
void DShot::onMotorCommandPosted() {
    if (!g_data_manager_ptr) {
        return; // DataManager not available
    }

    if (!m_motor_commands_consumer.consumeLatest()) {
        return; // No new motor commands
    }
    const MotorCommands& latest_commands = m_motor_commands_consumer.get_span().first[0];

    sendMotorCommand(latest_commands);

}