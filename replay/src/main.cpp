#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <iomanip>
#include "ESKF.h"
#include "Logger.h"
#include "timing.h"
#include "ClaimCommitRingBuffer.h"
#include "DataManager.h"
#include "DataTypes.h"

static constexpr size_t ITEM_SIZE = sizeof(SensorData);
const size_t PAGE_SIZE = 256;
static constexpr size_t ITEMS_PER_PAGE = 256 / ITEM_SIZE;
static constexpr size_t PADDING_SIZE = 256 - (ITEMS_PER_PAGE * ITEM_SIZE);

uint64_t g_current_time;
uint64_t getCurrentTimeUs()
{
    return g_current_time;
}

int main(int argc, char *argv[])
{
    std::string input_bin;
    if (argc > 1)
    {
        input_bin = argv[1];
    }
    else
    {
        std::cout << "No file specified." << std::endl;
        return -1;
    }
    std::ifstream bin_file(input_bin, std::ios::binary);
    if (!bin_file.is_open())
    {
        std::cerr << "Failed to open file: " << input_bin << std::endl;
        return -1;
    }

    DataManager::SensorBuffer m_sensor_buffer;
    DataManager::SensorConsumer m_sensor_consumer(m_sensor_buffer);
    DataManager::StateBuffer m_state_buffer;
    ESKF eskf(m_sensor_consumer, m_state_buffer);

    std::vector<char> page_buffer(PAGE_SIZE);
    int records_processed = 0;
    std::cout << "Starting Replay..." << std::endl;
    while (bin_file.read(page_buffer.data(), PAGE_SIZE))
    {
        for (size_t i = 0; i < ITEMS_PER_PAGE; i++)
        {
            SensorData current_data = reinterpret_cast<SensorData *>(page_buffer.data())[i];

            SensorData *buf = m_sensor_buffer.claim();
            memcpy(buf, &current_data, sizeof(SensorData));
            m_sensor_buffer.commit(buf);
            records_processed++;
            g_current_time = current_data.timestamp;
            eskf.run();
        }
    }
}