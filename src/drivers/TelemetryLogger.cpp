#include "drivers/TelemetryLogger.h"
#include "drivers/W25Q128FV.h"
#include "SEGGER_SYSVIEW.h"
#include <cstring>

extern W25Q128FV *g_flash_ptr;

Logger::Logger(){
    buf_ptrs[0] = buffer_1;
    buf_ptrs[1] = buffer_2;
    buf_ptrs[2] = buffer_3;
    active_idx = 0;
    buf_states[0] = BufferState::FREE;
    buf_states[1] = BufferState::FREE;
    buf_states[2] = BufferState::FREE;
    in_buf = buffer_1;
    buffer_head=4;
    is_fucked=false;
    global_log_enabled=false;
}

void Logger::enableLogging(){
    buf_states[active_idx] = BufferState::ACTIVE;
    global_log_enabled=true;
}
void Logger::stopLogging(){
    if (isStopped()){
        return;
    }

    global_log_enabled=false;

    // Fill rest of buffer with 0s
    std::fill(in_buf+buffer_head,in_buf+BUFFER_CAPACITY,0);
    int last_buf = active_idx;
    advanceBuffer();
    buf_states[active_idx] = BufferState::FREE;
    presentBufferToFlashDriverInHopesThatItMayBlessThisCodeToBeBugFree(last_buf);
}
bool Logger::isStopped(){
    if (global_log_enabled){
        return false;
    }
    
    int buffs_done = 0;
    for (int i=0;i<3;i++){
        buffs_done+=(buf_states[i]==BufferState::FREE);
    }
    // Flash driver is not currently reading one of the logs. All data has been flushed to flash
    return buffs_done==3;

}
void Logger::writePacket(const uint8_t* data, uint8_t size, uint8_t id){
    SEGGER_SYSVIEW_MarkStart(LOGGER_MARKER);
    size_t total_size = size+2;
    size_t buffer_left = BUFFER_CAPACITY-buffer_head;
    
    if (buffer_left<=total_size){ // Packet needs to be split across buffers to fit (or it fills it perfectly)
        int next_buf = (active_idx+1)%3;
        if(buf_states[next_buf]==BufferState::FREE){ // Next buffer is free
            int buf_right_now = active_idx;
            // this is gross. but whatever
            in_buf[buffer_head] = id;
            buffer_head++;
            // Did this byte fill the buffer?
            if (buffer_head==BUFFER_CAPACITY){
                advanceBuffer();
            }

            in_buf[buffer_head] = size;
            buffer_head++;
            // Did this byte fill the buffer?
            if (buffer_head==BUFFER_CAPACITY){
                advanceBuffer();
            }
            // Does this fill the buffer?
            // Or was it filled prior? 
            // Does it even matter?
            uint16_t bytes_written = 0;
            while(bytes_written<size){
                buffer_left = BUFFER_CAPACITY-buffer_head;
                uint16_t bytes_left = size-bytes_written;
                // more bytes than buffer. have to swap
                if(buffer_left<bytes_left){
                    std::memcpy(in_buf+buffer_head,data+bytes_written,buffer_left);
                    bytes_written+=buffer_left;
                    buffer_head+=buffer_left;
                    advanceBuffer();
                } else { // more buffer than bytes
                    std::memcpy(in_buf+buffer_head,data+bytes_written,bytes_left);
                    bytes_written+=bytes_left;
                    buffer_head+=bytes_left;
                }
            }

            if (buf_right_now==active_idx){// I guess we perfectly filled the buffer.... this function sucks
                advanceBuffer();
            }
            SEGGER_SYSVIEW_Mark(LOGGER_MARKER);
            presentBufferToFlashDriverInHopesThatItMayBlessThisCodeToBeBugFree(buf_right_now);

        /* END OF Next buffer is free*/
        } else if (buffer_left!=0){ // No buffers free but current 
            in_buf[buffer_head]=OVERFLOW_BYTE;
            buffer_head++;
            // Check to see if that filled the current buffer (the last one)
            if (buffer_head==BUFFER_CAPACITY) {
                buf_states[active_idx]=BufferState::READY;
                is_fucked = true;
                // flash driver guaranteed to reject our offering. 
            }
        }
    /* END OF Packet needs to be split across buffers to fit*/
    } else { // Packet no fill buffer
        in_buf[buffer_head] = id;
        buffer_head++;
        in_buf[buffer_head] = size;
        buffer_head++;
        std::memcpy(in_buf+buffer_head,data,size);
        buffer_head+=size;
    }
    SEGGER_SYSVIEW_MarkStop(LOGGER_MARKER);
    // This code SUCKS
}

void Logger::presentBufferToFlashDriverInHopesThatItMayBlessThisCodeToBeBugFree(int idx_of_buf_to_be_offered){
    if (g_flash_ptr==nullptr){ // our god has fallen, billions will die
        return;
    }
    // buf_states[idx_of_buf_to_be_offered]=BufferState::FREE;
    if (g_flash_ptr->writePageDMA_Raw(buf_ptrs[idx_of_buf_to_be_offered])){
        // offering was accepted, good harvest this clock cycle
        buf_states[idx_of_buf_to_be_offered]=BufferState::FLUSHING;
    } 
}

void Logger::yourGodRequiresFurtherOfferingsToBeSated(){
    int buffs_done = 0;
    for (int i=0;i<3;i++){
        if(buf_states[i]==BufferState::FLUSHING)
        {
            buffs_done++;
            buf_states[i]=BufferState::FREE;
        }
    }
    is_fucked=false;
    if(buffs_done>1){
        printf("This shouldn't be possible\n");
    }
    int next_buf = findReadyBuffer();
    if (next_buf>=0){
        presentBufferToFlashDriverInHopesThatItMayBlessThisCodeToBeBugFree(next_buf);
    }
}

void Logger::advanceBuffer(){

    // might need to seperatley disable irq or od the primask
    buf_states[active_idx] = BufferState::READY;
    active_idx = (1+active_idx)%3;
    in_buf = buf_ptrs[active_idx];
    buffer_head=4;
    buf_states[active_idx] = BufferState::ACTIVE;
}

int Logger::findFreeBuffer(){

    for (int i=0; i<3; i++){
        if (buf_states[i]==BufferState::FREE){
            return i;
        }
    }
    // No free buffer
    return -1;
}
int Logger::findReadyBuffer(){
    // Check forward from the current active buffer. (Might have looped)
    for (int i=0; i<3; i++){
        if (buf_states[(i+active_idx)%3]==BufferState::READY){
            return (i+active_idx)%3;
        }
    }
    // No ready buffer
    return -1;
}