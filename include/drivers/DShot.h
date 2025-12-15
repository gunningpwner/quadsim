#pragma once
#include "stm32f4xx_hal.h"
#include "Consumer.h"
#include "OtherData.h"

struct MotorTable{
    // Struct to store hardware related info about motor
    // What timer, what dma etc..

    TIM_HandleTypeDef* htim;
    uint32_t channel;

    DMA_HandleTypeDef* hdma;

    volatile uint32_t* ccr_reg; 
    uint32_t dma_bit;

    uint32_t cmd_buffer[18] __attribute__((aligned(4)));

    uint16_t duty_bit_0; 
    uint16_t duty_bit_1;
};

class DShot {
public:
    DShot();

    int init();
    void update();
    void disarm();
    void arm();
    void sendMotorCommand(MotorCommands& cmd);

private:
    void createMotorTable(uint8_t index, TIM_HandleTypeDef& htim, uint32_t channel, DMA_HandleTypeDef& hdma);
    void fillMotorTableBuffer(MotorTable* m, uint16_t cmd, bool telemetry);
    void startCmdXmit();
    int8_t is_armed = 0;

    MotorTable motor_tables[4];

    Consumer<MotorCommands, 1> m_motor_commands_consumer;
};
