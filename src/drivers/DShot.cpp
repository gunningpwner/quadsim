#include "drivers/DShot.h"


#define DSHOT_RATE 150000 // in kbit/s
#define DSHOT_MIN_THROTTLE 50

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern DMA_HandleTypeDef hdma_tim8_ch3;
extern DMA_HandleTypeDef hdma_tim8_ch4;

IRQn_Type Get_DMA_Stream_IRQn(DMA_HandleTypeDef *hdma);
/*
    4 states are DISARMED, IDLE, TRANSMITTING, RECEIVING

    IDLE:
        dma and timer are off
        DMA_SxCR_EN off
        TIM_CR1_CEN off
        unknown configuration
    DISARMED
        DMA_SxCR_EN on
        TIM_CR1_CEN on
        DMA circular buffer


*/
DShot::DShot() {
}

int DShot::init()
{
    createMotorTable(0, htim8, TIM_CHANNEL_3, hdma_tim8_ch3);
    createMotorTable(1, htim4, TIM_CHANNEL_1, hdma_tim4_ch1);
    createMotorTable(2, htim8, TIM_CHANNEL_4, hdma_tim8_ch4);
    createMotorTable(3, htim4, TIM_CHANNEL_2, hdma_tim4_ch2);
    return 0;
}

void DShot::createMotorTable(uint8_t index, TIM_HandleTypeDef &htim, uint32_t channel, DMA_HandleTypeDef &hdma)
{

    MotorTable *m = &motor_tables[index];

    m->htim = &htim;
    m->channel = channel;
    m->hdma = &hdma;

    switch (channel)
    {
    case TIM_CHANNEL_1:
        m->ccr_reg = &htim.Instance->CCR1;
        m->dma_bit = TIM_DMA_CC1;
        break;
    case TIM_CHANNEL_2:
        m->ccr_reg = &htim.Instance->CCR2;
        m->dma_bit = TIM_DMA_CC2;
        break;
    case TIM_CHANNEL_3:
        m->ccr_reg = &htim.Instance->CCR3;
        m->dma_bit = TIM_DMA_CC3;
        break;
    case TIM_CHANNEL_4:
        m->ccr_reg = &htim.Instance->CCR4;
        m->dma_bit = TIM_DMA_CC4;
        break;
    default:
        return;
    }
    uint32_t tim_clock = 0;
    // Now we calculate the duty cycle for a 0 and 1 based off the timer and clock configuration
    TIM_TypeDef *tim = htim.Instance;
    if (tim == TIM1 || tim == TIM8 || tim == TIM9 || tim == TIM10 || tim == TIM11)
    {
        tim_clock = HAL_RCC_GetPCLK2Freq();
        // If the APB2 Prescaler !=, Timer clock automatically gets doubled
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0)
            tim_clock *= 2;
    }
    else
    {
        tim_clock = HAL_RCC_GetPCLK1Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
            tim_clock *= 2;
    }
    uint32_t arr = (tim_clock + (DSHOT_RATE / 2)) / DSHOT_RATE;
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr - 1);
    __HAL_TIM_SET_PRESCALER(m->htim, 0);

    m->duty_bit_0 = (arr * 3) / 8;
    m->duty_bit_1 = (arr * 3) / 4;
}

void DShot::arm()
{
    if (armedState==ARMED){
        return;
    }

    MotorTable *m = &motor_tables[0];

    DMA_Stream_TypeDef *dmaStreamM1 = (DMA_Stream_TypeDef *)m->hdma->Instance;

    // Turn on transfer complete interrupt 
    dmaStreamM1->CR |= DMA_SxCR_TCIE;
    IRQn_Type irq = Get_DMA_Stream_IRQn(m->hdma);
    HAL_NVIC_SetPriority(irq, 0, 0);
    HAL_NVIC_EnableIRQ(irq);
    // Wait for stream to be disabled.
    // We don't want to interrupt the transfer while the dma is mid transmit
    // So we turn on interrupt flag, and in the interrupt, disable the stream.
    // The same interrupt also turned off the interrupt enable.
    __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_TC_FLAG_INDEX(m->hdma));

    // while (dmaStreamM1->CR & DMA_SxCR_EN)
    //     asm volatile("nop");

    while (dmaState!=IDLE){
        asm volatile("nop");
    }

    __HAL_TIM_DISABLE(&htim4);
    __HAL_TIM_DISABLE(&htim8);

    for (int i = 0; i < 4; ++i)
    {
        MotorTable *m = &motor_tables[i];
        DMA_Stream_TypeDef *dmaStream = (DMA_Stream_TypeDef *)m->hdma->Instance;
        dmaStream->CR &= ~DMA_SxCR_CIRC;
    }
    armedState = ARMED;
}

void DShot::disarm()
{
    if (armedState==DISARMED){
        return;
    }
    

    // Disable interrupts to stop state switch logic.
    hdma_tim8_ch3.Instance->CR &= ~DMA_SxCR_TCIE;

    // Wait until last command has finished sending so we don't corrupt the signal
    // Idk how the esc would handle that so just play it safe
    
    DMA_Stream_TypeDef *dmaStreamM1 = (DMA_Stream_TypeDef *)motor_tables[0].hdma->Instance;
    if (dmaState==TRANSMITTING){
        while (dmaStreamM1->CR & DMA_SxCR_EN)
            asm volatile("nop");
    }
    else {

    }

    

    // Fill motor tables with zero throttle command and switch DMAs to circular buffer
    for (int i = 0; i < 4; ++i)
    {
        MotorTable *m = &motor_tables[i];
        fillMotorTableBuffer(m, 0, false);
        DMA_Stream_TypeDef *dmaStream = (DMA_Stream_TypeDef *)m->hdma->Instance;
        dmaStream->CR |= DMA_SxCR_CIRC;
    }

    startCmdXmit();
    armedState = DISARMED;
    printf("d\n");
}

void DShot::reconfigureForTelemetry()
{
    if (dmaState!=IDLE){
        return;
    }

    __HAL_TIM_DISABLE(&htim4);
    __HAL_TIM_DISABLE(&htim8);

    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    for (int i = 0; i < 4; ++i)
    {
        MotorTable *m = &motor_tables[i];
        if (m->htim == nullptr)
            continue;

       
        DMA_Stream_TypeDef *dmaStream = (DMA_Stream_TypeDef *)m->hdma->Instance;
        dmaStream->CR &= ~DMA_SxCR_EN;
        // Clears flags
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_TC_FLAG_INDEX(m->hdma));
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_HT_FLAG_INDEX(m->hdma));
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_TE_FLAG_INDEX(m->hdma));
        // Manually sets transfer target and size
        
        dmaStream->NDTR = 22;                     // Bi-DShot frame usually generates 21 or 22 edges
        dmaStream->M0AR = (uint32_t)m->rx_buffer; // Point to your new RX buffer

        dmaStream->CR &= ~DMA_SxCR_DIR;           // 00 = Peripheral-to-memory

        dmaStream->CR |= DMA_SxCR_EN;

        TIM_TypeDef *tim = m->htim->Instance;

        // Disable channel so we can change the settings
        tim->CCER &= ~(1 << ((m->channel >> 2) * 4));
        switch (m->channel)
        {
        case TIM_CHANNEL_1:
            tim->CCMR1 &= ~TIM_CCMR1_CC1S;       // Clear selection
            tim->CCMR1 |= TIM_CCMR1_CC1S_0;      // Set to 01 (Input mapped to TI1)
            tim->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC1NP); // Capture both edges
            tim->CCER |= TIM_CCER_CC1E;          // Re-enable capture
            break;
        case TIM_CHANNEL_2:
            tim->CCMR1 &= ~TIM_CCMR1_CC2S;
            tim->CCMR1 |= TIM_CCMR1_CC2S_0;
            tim->CCER |= (TIM_CCER_CC2P | TIM_CCER_CC2NP);
            tim->CCER |= TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            tim->CCMR2 &= ~TIM_CCMR2_CC3S;
            tim->CCMR2 |= TIM_CCMR2_CC3S_0;
            tim->CCER |= (TIM_CCER_CC3P | TIM_CCER_CC3NP);
            tim->CCER |= TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            tim->CCMR2 &= ~TIM_CCMR2_CC4S;
            tim->CCMR2 |= TIM_CCMR2_CC4S_0;
            tim->CCER |= (TIM_CCER_CC4P | TIM_CCER_CC4NP);
            tim->CCER |= TIM_CCER_CC4E;
            break;
        }

    }
    __HAL_TIM_MOE_ENABLE(&htim8);

    __HAL_TIM_ENABLE(&htim4);
    __HAL_TIM_ENABLE(&htim8);
    dmaState=RECEIVING;
}

void DShot::startCmdXmit()
{
    // Check if motor 1 transfer is finished

    if (dmaState!=IDLE){
        return;
    }

    __HAL_TIM_DISABLE(&htim4);
    __HAL_TIM_DISABLE(&htim8);

    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    for (int i = 0; i < 4; ++i)
    {
        MotorTable *m = &motor_tables[i];
        if (m->htim == nullptr)
            continue;

        DMA_Stream_TypeDef *dmaStream = (DMA_Stream_TypeDef *)m->hdma->Instance;
        dmaStream->CR &= ~DMA_SxCR_EN;
        // Clears flags
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_TC_FLAG_INDEX(m->hdma));
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_HT_FLAG_INDEX(m->hdma));
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_TE_FLAG_INDEX(m->hdma));
        // Manually sets transfer target and size
        dmaStream->NDTR = 18;
        dmaStream->M0AR = (uint32_t)m->cmd_buffer;
        dmaStream->PAR = (uint32_t)m->ccr_reg;
        //  Reconfigure DMA for Memory-to-Peripheral
        dmaStream->CR &= ~DMA_SxCR_DIR;           // Clear direction bits
        dmaStream->CR |= DMA_SxCR_DIR_0;          // 01 = Memory-to-peripheral

        dmaStream->CR |= DMA_SxCR_EN;

        TIM_TypeDef *tim = m->htim->Instance;

        tim->DIER |= m->dma_bit;

        // Disable channel so we can change the settings
        tim->CCER &= ~(1 << ((m->channel >> 2) * 4));
        switch (m->channel)
        {
        case TIM_CHANNEL_1:
            tim->CCMR1 &= ~TIM_CCMR1_CC1S;       // 00: Output
            tim->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM mode 1 (110)
            tim->CCMR1 |= TIM_CCMR1_OC1PE;       // Preload enable
            tim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); // Clear polarity (active high)
            tim->CCER |= TIM_CCER_CC1E;          // Enable output
            break;
        case TIM_CHANNEL_2:
            tim->CCMR1 &= ~TIM_CCMR1_CC2S;
            tim->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
            tim->CCMR1 |= TIM_CCMR1_OC2PE;
            tim->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP);
            tim->CCER |= TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            tim->CCMR2 &= ~TIM_CCMR2_CC3S;
            tim->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);
            tim->CCMR2 |= TIM_CCMR2_OC3PE;
            tim->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
            tim->CCER |= TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            tim->CCMR2 &= ~TIM_CCMR2_CC4S;
            tim->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);
            tim->CCMR2 |= TIM_CCMR2_OC4PE;
            tim->CCER &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP);
            tim->CCER |= TIM_CCER_CC4E;
            break;
        }

        tim->CCER |= (1 << (m->channel >> 2) * 4);

    }
    // Im just gonna hard code starting the timers for now lol
    // One day I'll find a smarter way to do this
    // Surely I won't forget about this and it bites me in the ass
    __HAL_TIM_MOE_ENABLE(&htim8);

    __HAL_TIM_ENABLE(&htim4);
    __HAL_TIM_ENABLE(&htim8);

    dmaState=TRANSMITTING;
}



void DShot::handleInterrupt()
{
    if (armedState==DISARMED){
        // Should only get here when arm function enables interrupts
        for (int i = 0; i < 4; ++i)
        {
            MotorTable *m = &motor_tables[i];
            DMA_Stream_TypeDef *dmaStream = (DMA_Stream_TypeDef *)m->hdma->Instance;
            dmaStream->CR &= ~DMA_SxCR_EN;
            __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_TC_FLAG_INDEX(m->hdma));
        }
        printf("a\n");
        dmaState=IDLE;
        return;
    }
    if (dmaState==TRANSMITTING){
        dmaState=IDLE;
        reconfigureForTelemetry();
        printf("b\n");
        return;
    }
    if (dmaState==RECEIVING){
        printf("c\n");
        dmaState=IDLE;
        return;
    }
}



void DShot::fillMotorTableBuffer(MotorTable *m, uint16_t cmd, bool telemetry)
{
    cmd = (cmd << 1) | (telemetry ? 1 : 0);

    // Compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = cmd;
    for (int i = 0; i < 3; ++i)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x0F;

    cmd = (cmd << 4) | csum;

    for (int bit = 0; bit < 16; ++bit)
    {
        if (cmd & 0x8000)
            m->cmd_buffer[bit] = m->duty_bit_1;
        else
            m->cmd_buffer[bit] = m->duty_bit_0;

        cmd <<= 1;
    }
    // 0 out the CCR to create a buffer between messages.
    // Also stops transient pulses at the end.
    m->cmd_buffer[16] = 0;
    m->cmd_buffer[17] = 0;
}
void DShot::sendMotorThrottle(float cmds[4])
{
    if (armedState==DISARMED){
        return;
    }
    // Commands come in with range 0-1
    for (int i = 0; i < 4; ++i)
    {
        uint16_t dshot_val = (uint16_t)(cmds[i] * 1999.0f) + 48;
        dshot_val = (dshot_val > 2047) ? 2047 : dshot_val;
        fillMotorTableBuffer(&motor_tables[i], dshot_val, true);
    }
    startCmdXmit();
}

void DShot::sendMotorCommand(MotorCommands &cmd)
{
    if (cmd.is_throttle_command)
    {
        // Throttle value will come to us in range 0-1999 so we map it to 48-2047.
        // Prolly make this better
        for (int i = 0; i < 4; ++i)
        {
            uint16_t dshot_val = cmd.throttle[i] + 48 + DSHOT_MIN_THROTTLE;
            dshot_val = (dshot_val > 2047) ? 2047 : dshot_val;
            fillMotorTableBuffer(&motor_tables[i], dshot_val, false);
        }
    }
    else
    {
        uint16_t command_val = static_cast<uint16_t>(cmd.command);
        for (int i = 0; i < 4; ++i)
        {
            fillMotorTableBuffer(&motor_tables[i], command_val, false);
        }
    }

    startCmdXmit();
}


IRQn_Type Get_DMA_Stream_IRQn(DMA_HandleTypeDef *hdma)
{
    uint32_t stream_addr = (uint32_t)hdma->Instance;

    // DMA1 Streams
    if (stream_addr == (uint32_t)DMA1_Stream0)
        return DMA1_Stream0_IRQn;
    if (stream_addr == (uint32_t)DMA1_Stream1)
        return DMA1_Stream1_IRQn;
    if (stream_addr == (uint32_t)DMA1_Stream2)
        return DMA1_Stream2_IRQn;
    if (stream_addr == (uint32_t)DMA1_Stream3)
        return DMA1_Stream3_IRQn;
    if (stream_addr == (uint32_t)DMA1_Stream4)
        return DMA1_Stream4_IRQn;
    if (stream_addr == (uint32_t)DMA1_Stream5)
        return DMA1_Stream5_IRQn;
    if (stream_addr == (uint32_t)DMA1_Stream6)
        return DMA1_Stream6_IRQn;
    if (stream_addr == (uint32_t)DMA1_Stream7)
        return DMA1_Stream7_IRQn;

    // DMA2 Streams
    if (stream_addr == (uint32_t)DMA2_Stream0)
        return DMA2_Stream0_IRQn;
    if (stream_addr == (uint32_t)DMA2_Stream1)
        return DMA2_Stream1_IRQn;
    if (stream_addr == (uint32_t)DMA2_Stream2)
        return DMA2_Stream2_IRQn;
    if (stream_addr == (uint32_t)DMA2_Stream3)
        return DMA2_Stream3_IRQn;
    if (stream_addr == (uint32_t)DMA2_Stream4)
        return DMA2_Stream4_IRQn;
    if (stream_addr == (uint32_t)DMA2_Stream5)
        return DMA2_Stream5_IRQn;
    if (stream_addr == (uint32_t)DMA2_Stream6)
        return DMA2_Stream6_IRQn;
    if (stream_addr == (uint32_t)DMA2_Stream7)
        return DMA2_Stream7_IRQn;
}