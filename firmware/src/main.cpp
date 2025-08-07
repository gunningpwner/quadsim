#include <Arduino.h>

RawImuData raw_imu_buffer[...]; 

// Layer 2 Buffer: Your existing DataManager for clean data
DataManager data_manager;

// LAYER 1: THE COLLECTOR (ISR)
void imu_collector_ISR() {
    // Just grab raw bytes and put them in the buffer. No math!
    RawImuData raw_data = readRawBytesFromImu();
    raw_imu_buffer.push(raw_data); 
}


void setup() {
    elapsedMicros teensy_hw_timer;
    TimeSource teensy_time_source = [&]() { return (uint64_t)teensy_hw_timer; };

    // 2. Initialize the DataManager with the time source
    DataManager data_manager(teensy_time_source);
}


void loop() {
    // --- LAYER 2: THE PROCESSOR ---
    if (raw_imu_buffer.has_new_data()) {
        RawImuData raw_data = raw_imu_buffer.get();
        
        // This is your modular "driver" section
        // Do all the math and conversion here
        GyroData clean_gyro = processRawImuDataIntoStandardFormat(raw_data);

        // Post the clean, standardized data for the filter
        data_manager.postRawGyro(clean_gyro);
    }

    // --- LAYER 3: THE CONSUMER ---
    std::vector<GyroData> samples;
    unsigned int last_count = 0;
    if (data_manager.consumeRawGyro(samples, last_count)) {
        runFilter(samples);
        // ...
    }
}