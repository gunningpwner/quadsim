#include "w25.h"

// Functions this driver needs to perform
// 1. Manage data
// 2. It's just that simple
 
void init(){



}

void read_device_information(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    uint8_t data[3+1] ={0};
    uint8_t tx[3+1] = {0};
    tx[0] = READ_JEDEC_ID;

    if (HAL_SPI_TransmitReceive(&hspi2, tx, data, 3+1, HAL_MAX_DELAY) != HAL_OK){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        return;
    }
    // First byte is manufacturer id (0xEF)
    // Second byte is 
    

}