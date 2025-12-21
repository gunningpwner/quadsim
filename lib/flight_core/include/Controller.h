#pragma once

#include "DataTypes.h"
#include "DataManager.h"

class Controller{
public:
    Controller(DataManager& data_manager):
    m_data_manager(data_manager){};
    virtual ~Controller() = default;
    virtual void run() = 0; // Pure virtual function

protected:
    DataManager& m_data_manager;

    uint64_t m_current_time_us;
    uint64_t m_last_run_time_us;

    

};