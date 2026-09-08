#pragma once
#include <cstdint>
#include "DataTypes.h"
#include "stm32f4xx_hal.h"


#define LOG_DATA(data) (Logger::getInstance().log(data))


template <typename T>
struct LogSettings;

template <> struct LogSettings<FastIMUData> {
    static constexpr uint8_t id = 0x01;
    inline static bool logEnabled = false;
};
template <> struct LogSettings<FastRPMData> {
    static constexpr uint8_t id = 0x02;
    inline static bool logEnabled = false;
};
template <> struct LogSettings<SensorData> {
    static constexpr uint8_t id = 0x03;
    inline static bool logEnabled = true;
};

template <> struct LogSettings<StateEstimate> {
    static constexpr uint8_t id = 0x04;
    inline static bool logEnabled = true;
};


class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }
    

    template <typename T>
    void log(const T& data){ // pass by reference, no copy
        if(!(LogSettings<T>::logEnabled&&global_log_enabled)||is_fucked){
            return;
        }
        __disable_irq();
        writePacket(reinterpret_cast<const uint8_t*>(&data), sizeof(data),LogSettings<T>::id);
        __enable_irq();
    }

    void yourGodRequiresFurtherOfferingsToBeSated();
    void enableLogging();
    void stopLogging();
    bool isStopped();
    
    enum class BufferState {
        FREE,       // Available to be written to
        ACTIVE,     // Currently being written to by log()
        READY,      // Full buffer ready to be written to permanent storage
        FLUSHING    // Buffer is currently being written to permanent storage
    };


private:
    Logger();
    void writePacket(const uint8_t* data, uint8_t size, uint8_t id);
    int findFreeBuffer();
    int findReadyBuffer();
    void advanceBuffer();
    void presentBufferToFlashDriverInHopesThatItMayBlessThisCodeToBeBugFree(int idx_of_buf_to_be_offered);
    static constexpr uint16_t BUFFER_CAPACITY = 260; //first 4 bytes are reserved for the spi to write address into.
    static constexpr uint8_t OVERFLOW_BYTE = 0xFF;

    uint8_t* in_buf; // Buffer that data gets written to
    uint8_t buffer_head;

    uint8_t buffer_1[BUFFER_CAPACITY]; 
    uint8_t buffer_2[BUFFER_CAPACITY]; 
    uint8_t buffer_3[BUFFER_CAPACITY];
    uint8_t* buf_ptrs[3];

    BufferState buf_states[3];
    uint8_t active_idx; // index of currently active buffer
    bool global_log_enabled;
    bool is_fucked; // Current buffer is full and no other buffers are free.
};