#include "GPS.h"
#include "DataManager.h"

extern DataManager* g_data_manager_ptr;
extern UART_HandleTypeDef huart4;

GPS::GPS(){}

int8_t GPS::init() {

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, getRxBuffer(), 256);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

    return 0;
}

void GPS::handleRxChunk(uint8_t* buf, uint16_t len) {
    if(len==256)
        return;

    printf("yay\n");
}