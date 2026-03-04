#pragma once
#include "stm32f4xx_hal.h"
#include "ClaimCommitRingBuffer.h"
#include "DataTypes.h"

#include <cstdint>

struct MotorTable{
    // Struct to store hardware related info about motor
    // What timer, what dma etc..

    TIM_HandleTypeDef* htim;
    uint32_t channel;

    DMA_HandleTypeDef* hdma;

    volatile uint32_t* ccr_reg; 
    uint32_t dma_bit;

    uint32_t cmd_buffer[18] __attribute__((aligned(4)));
    uint32_t rx_buffer[22];
    uint16_t duty_bit_0; 
    uint16_t duty_bit_1;
};
enum DMAState { IDLE, TRANSMITTING, RECEIVING };
enum DriverState { DISARMED, UNINITIALIZED, ARMED};

class DShot {
public:
    DShot();

    int init();
    void update();
    void disarm();
    void arm();
    void sendMotorCommand(MotorCommands& cmd);
    void sendMotorThrottle(float cmds[4]);
    void handleInterrupt();
    
private:
    void createMotorTable(uint8_t index, TIM_HandleTypeDef& htim, uint32_t channel, DMA_HandleTypeDef& hdma);
    void fillMotorTableBuffer(MotorTable* m, uint16_t cmd, bool telemetry);
    void startCmdXmit();
    void reconfigureForTelemetry();
    DriverState armedState = UNINITIALIZED;
    DMAState dmaState = IDLE;
    uint64_t receive_start=0;
    MotorTable motor_tables[4];

};


