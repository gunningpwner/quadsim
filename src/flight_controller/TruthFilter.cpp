#include "TruthFilter.h"
#include "OtherData.h"
#include "coord_trans.h"

TruthFilter::TruthFilter(DataManager& data_manager) :
    FilterBase(data_manager), // Pass data_manager to the base class
    m_gyro_consumer(data_manager),
    m_gps_consumer(data_manager)
{
}

void TruthFilter::run() {
    std::vector<GyroData> gyro_samples;
    m_gyro_consumer.consume(gyro_samples);

    std::vector<GPSPositionData> gps_samples;
    m_gps_consumer.consume(gps_samples);

    if (!gyro_samples.empty() || !gps_samples.empty()) {
        StateData new_state;

        if (!gps_samples.empty()) {
            new_state.position_ecef = lla_to_ecef(gps_samples.back().lla);
        } else {
            new_state.position_ecef = {0, 0, 0};
        }

        // Set velocity and orientation to default values
        new_state.velocity_ecef = {0, 0, 0};
        new_state.orientation = {1, 0, 0, 0}; // Identity quaternion

        m_data_manager.post(new_state);
    }
}
