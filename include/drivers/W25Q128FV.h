#pragma once
#include "stm32f4xx_hal.h"
#include <cstdint>
#include "DataManager.h"
#include "DataTypes.h"

class W25Q128FV
{
public:
    W25Q128FV(DataManager::SensorConsumer m_consumer);
    void run();
    int8_t init();
    uint32_t findWriteHead();
    void readData(uint32_t address, uint8_t* buffer, uint32_t size);
    bool isBusy();
    void startWriting();
    void stopWriting();

    // NOTE: This function assumes you have prepended the Command (0x02) 
    // and 3-byte Address to your buffer
    void writePageDMA_Raw(uint8_t* buffer_with_header, uint16_t total_len);
    void dumpDataToUSB();
    void erase();
    volatile bool m_dma_transfer_active = false;
private:
    static constexpr uint8_t CMD_WRITE_ENABLE = 0x06;
    static constexpr uint8_t CMD_VOLATILE_SR = 0x50;
    static constexpr uint8_t CMD_READ_SR1 = 0x05;
    static constexpr uint8_t CMD_WRITE_SR1 = 0x01;
    static constexpr uint8_t CMD_PAGE_PROG = 0x02;
    static constexpr uint8_t CMD_SECTOR_ERASE = 0x20; // 4KB
    static constexpr uint8_t CMD_CHIP_ERASE = 0xC7;
    static constexpr uint8_t CMD_READ_DATA = 0x03;
    static constexpr uint8_t CMD_READ_ID = 0x9F; // JEDEC ID
    static constexpr uint8_t CMD_RESET_EN = 0x66;
    static constexpr uint8_t CMD_RESET = 0x99;
    static constexpr uint8_t CMD_POWER_DOWN = 0xB9;
    static constexpr uint8_t CMD_RELEASE_PD = 0xAB;

    static constexpr size_t ITEM_SIZE = sizeof(SensorData);
    static constexpr size_t ITEMS_PER_PAGE = 256 / ITEM_SIZE; 
    static constexpr size_t PADDING_SIZE = 256 - (ITEMS_PER_PAGE * ITEM_SIZE);

    DataManager::SensorConsumer m_sensor_consumer;

    bool m_should_write;

    uint32_t write_head;
    uint8_t m_rx_buf[4];
    uint8_t page_buffer[256+4];

    void enableWrite();
};