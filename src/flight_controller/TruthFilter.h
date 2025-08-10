#pragma once

#include "Consumer.h"
#include "SensorData.h"
#include "FilterBase.h"

// TruthFilter no longer declares m_data_manager
class TruthFilter : public FilterBase {
public:
    TruthFilter(DataManager& data_manager);

    void run() override;

private:
    // m_data_manager is removed from here
    Consumer<GyroData> m_gyro_consumer;
    Consumer<GPSPositionData> m_gps_consumer;
};