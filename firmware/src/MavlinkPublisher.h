#pragma once

#include "common/mavlink.h"

/**
 * @brief Handles the creation and transmission of MAVLink messages.
 */
class MavlinkPublisher {
public:
    /**
     * @brief Construct a new Mavlink Publisher object.
     * @param system_id The MAVLink system ID.
     * @param component_id The MAVLink component ID.
     */
    MavlinkPublisher(uint8_t system_id, uint8_t component_id);

    /**
     * @brief Fetches the latest state and publishes the attitude message.
     */
    void run();

private:
    uint8_t m_system_id;
    uint8_t m_component_id;
};