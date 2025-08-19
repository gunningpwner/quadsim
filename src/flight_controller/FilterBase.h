#pragma once

#include "DataManager.h" // Include the DataManager header

class FilterBase {
public:
    // Constructor now takes a DataManager reference
    FilterBase(DataManager& data_manager) : m_data_manager(data_manager) {}

    // Virtual destructor
    virtual ~FilterBase() = default;

    // Pure virtual run function
    virtual void run() = 0;

protected:
    // Common data member for all derived filters
    DataManager& m_data_manager;
};