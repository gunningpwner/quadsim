#include "drivers/W25Q128FV.h"
#include "drivers/usbd_cdc_if.h"
#include <cstring>

extern SPI_HandleTypeDef hspi2;
extern int8_t usbStatus;

W25Q128FV::W25Q128FV() : m_flash_full(false)
{
}

int8_t W25Q128FV::init()
{

    uint8_t cmd = CMD_RESET_EN;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(1);

    cmd = CMD_RESET;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(3);

    cmd = CMD_READ_ID;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi2, m_rx_buf, 3, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

    if (!(m_rx_buf[0] == 0xEF && m_rx_buf[1] == 0x40 && m_rx_buf[2] == 0x18))
    {
        return -1;
    }

    write_head = findWriteHead();
    if (write_head == 0xFFFFFFFF)
    {
        m_flash_full = false;
        return -1;
    }

    return 0;
}

bool W25Q128FV::writePageDMA_Raw(uint8_t *buffer_with_header)
{
    if (m_flash_full)
    {
        return false;
    }
    // Check if flash is full or will be full
    //  W25Q128FV capacity is 16MB (0x1000000)
    if (write_head >= 0x1000000 - 256)
    {
        printf("Dis bitch full\n");
        m_flash_full = false;
        return false;
    }

    if (isBusy())
    {
        return false;
    }
    m_dma_transfer_active = true;
    
    buffer_with_header[0] = 0x02;
    buffer_with_header[1] = (write_head >> 16) & 0xFF;
    buffer_with_header[2] = (write_head >> 8) & 0xFF;
    buffer_with_header[3] = (write_head) & 0xFF;
    write_head += 256;
    
    enableWrite();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(&hspi2, buffer_with_header, 260);
    return true;
}
bool W25Q128FV::isBusy()
{
    if (m_dma_transfer_active)
        return true;

    uint8_t cmd = CMD_READ_SR1;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi2, m_rx_buf, 1, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    return (m_rx_buf[0] & 0x01);
}
uint32_t W25Q128FV::findWriteHead()
{
    // Uses binary search to find first empty sector (4KB)
    // Returns FFFFFFFF if flash is full
    uint32_t low_sector = 0;
    uint32_t high_sector = 4095;
    uint32_t first_empty_sector = 0xFFFFFFFF;

    uint8_t check_buff[4];

    while (low_sector <= high_sector)
    {
        uint32_t mid_sector = (low_sector + high_sector) / 2;
        readData(mid_sector * 4096, check_buff, 4);
        // Check if the sector is empty (all 0xFF)
        if (check_buff[0] == 0xFF && check_buff[1] == 0xFF && check_buff[2] == 0xFF && check_buff[3] == 0xFF)
        {
            first_empty_sector = mid_sector;
            if (mid_sector == 0)
                break;
            high_sector = mid_sector - 1; // Try to find an earlier empty sector
        }
        else
        {
            low_sector = mid_sector + 1; // This sector is not empty, search later
        }
    }
    return first_empty_sector * 4096; // Return the address of the first empty sector
}

void W25Q128FV::readData(uint32_t address, uint8_t *buffer, uint32_t size)
{

    uint8_t cmd[4];
    cmd[0] = CMD_READ_DATA;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = (address) & 0xFF;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, cmd, 4, 100);
    HAL_SPI_Receive(&hspi2, buffer, size, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void W25Q128FV::dumpDataToUSB()
{
    while (isBusy())
    {
        HAL_Delay(1);
    }

    for (uint32_t addr = 0; addr < write_head; addr += 256)
    {
        readData(addr, page_buffer, 256);
        CDC_Transmit_FS(page_buffer, 256);
        while (usbStatus == USBD_BUSY)
        {
            HAL_Delay(1);
        }
    }
}

void W25Q128FV::enableWrite()
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void W25Q128FV::erase()
{
    // 1. Safety check: If head is 0, the chip is effectively empty already.
    if (write_head == 0)
    {
        return;
    }

    // 2. Iterate through 4KB sectors up to the write_head
    // We start at address 0 and jump 4096 bytes at a time.
    // The loop condition (addr < write_head) ensures we erase any sector
    // that has even a single byte of data in it.
    for (uint32_t addr = 0; addr < write_head; addr += 4096)
    {
        enableWrite();

        uint8_t cmd[4];
        cmd[0] = CMD_SECTOR_ERASE; // 0x20
        cmd[1] = (addr >> 16) & 0xFF;
        cmd[2] = (addr >> 8) & 0xFF;
        cmd[3] = (addr) & 0xFF;

        // Send Erase Command
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, cmd, 4, 100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

        // Wait for sector to erase (typically ~45ms per sector)
        while (isBusy())
        {
            // Optional: Toggle an LED here if you want visual feedback
            // HAL_Delay(1);
        }
    }

    // 3. Reset the write head so the next log starts at 0
    write_head = 0;
}