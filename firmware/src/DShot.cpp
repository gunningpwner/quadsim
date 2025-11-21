#include "DShot.h"
#include "DataManager.h"

#define DSHOT_RATE 150000 // in kbit/s
extern DataManager *g_data_manager_ptr;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern DMA_HandleTypeDef hdma_tim8_ch3;
extern DMA_HandleTypeDef hdma_tim8_ch4;

DShot::DShot() : m_motor_commands_consumer(g_data_manager_ptr->getMotorCommandsChannel())
{
}

int DShot::init()
{
    createMotorTable(0, htim8, TIM_CHANNEL_3, hdma_tim8_ch3);
    createMotorTable(1, htim4, TIM_CHANNEL_1, hdma_tim4_ch1);
    createMotorTable(2, htim8, TIM_CHANNEL_4, hdma_tim8_ch4);
    createMotorTable(3, htim4, TIM_CHANNEL_2, hdma_tim4_ch2);
    return 0;
}
void DShot::arm(){
    DMA_Stream_TypeDef *dmaStreamM1 = (DMA_Stream_TypeDef *)motor_tables[0].hdma->Instance;
    while (dmaStreamM1->CR & DMA_SxCR_EN)
        asm volatile ("nop");

    for (int i = 0; i < 4; ++i)
    {
        MotorTable *m = &motor_tables[i];
        DMA_Stream_TypeDef *dmaStream = (DMA_Stream_TypeDef *)m->hdma->Instance;
        dmaStream->CR &= ~DMA_SxCR_CIRC;
    }
    is_armed=true;
}

void DShot::disarm()
{
    is_armed=false;
    //Wait until last command has finished sending so we don't corrupt the signal
    //Idk how the esc would handle that so just play it safe
    DMA_Stream_TypeDef *dmaStreamM1 = (DMA_Stream_TypeDef *)motor_tables[0].hdma->Instance;
    while (dmaStreamM1->CR & DMA_SxCR_EN)
        asm volatile ("nop");
    
    // Fill motor tables with zero throttle command and switch DMAs to circular buffer
    for (int i = 0; i < 4; ++i)
    {
        MotorTable *m = &motor_tables[i];
        fillMotorTableBuffer(m, 0, false);
        DMA_Stream_TypeDef *dmaStream = (DMA_Stream_TypeDef *)m->hdma->Instance;
        dmaStream->CR |= DMA_SxCR_CIRC;
    }

    startCmdXmit();
    
}

void DShot::update()
{
    if (!is_armed)
        return;
        
    if (!g_data_manager_ptr)
        return;

    if (!m_motor_commands_consumer.consumeLatest())
        return;

    MotorCommands latest_commands = m_motor_commands_consumer.get_span().first[0];

    sendMotorCommand(latest_commands);
}


void DShot::createMotorTable(uint8_t index, TIM_HandleTypeDef& htim, uint32_t channel, DMA_HandleTypeDef& hdma)
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

void DShot::sendMotorCommand(MotorCommands &cmd)
{
    if (cmd.is_throttle_command)
    {
        // Throttle value will come to us in range 0-1999 so we map it to 48-2047.
        // Except 0, that stays 0.
        // Prolly make this better
        for (int i = 0; i < 4; ++i)
        {
            uint16_t dshot_val = (cmd.throttle[i] == 0) ? 0 : cmd.throttle[i] + 48;
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

void DShot::startCmdXmit()
{
    // Check if motor 1 transfer is finished
    DMA_Stream_TypeDef *dmaStreamM1 = (DMA_Stream_TypeDef *)motor_tables[0].hdma->Instance;
    if (dmaStreamM1->CR & DMA_SxCR_EN) 
        return;

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
        // Clears flags
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_TC_FLAG_INDEX(m->hdma));
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_HT_FLAG_INDEX(m->hdma));
        __HAL_DMA_CLEAR_FLAG(m->hdma, __HAL_DMA_GET_TE_FLAG_INDEX(m->hdma));
        //Manually sets transfer target and size
        dmaStream->NDTR = 18; 
        dmaStream->M0AR = (uint32_t)m->cmd_buffer;
        dmaStream->PAR = (uint32_t)m->ccr_reg;
        dmaStream->CR |= DMA_SxCR_EN;
        
        m->htim->Instance->DIER |= m->dma_bit;
        m->htim->Instance->CCER |= (1 << (m->channel >> 2) * 4);
    }
    // Im just gonna hard code starting the timers for now lol
    // One day I'll find a smarter way to do this
    // Surely I won't forget about this and it bites me in the ass
    __HAL_TIM_MOE_ENABLE(&htim8);
    
    __HAL_TIM_ENABLE(&htim4);
    __HAL_TIM_ENABLE(&htim8);
}