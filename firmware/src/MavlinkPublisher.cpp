#include "MavlinkPublisher.h"
#include "usbd_cdc_if.h" // For CDC_Transmit_FS
#include "DataManager.h"
#include "SensorData.h"
extern DataManager* g_data_manager_ptr;

MavlinkPublisher::MavlinkPublisher(uint8_t system_id, uint8_t component_id)
    : m_system_id(system_id), m_component_id(component_id) {}

/**
 * @brief Fetches the latest state and publishes the attitude message.
 */
void MavlinkPublisher::run() {
    if (!g_data_manager_ptr) {
        return;
    }

    StateData current_state;
    if (g_data_manager_ptr->getLatest(current_state)) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        // Pack the message
        mavlink_msg_attitude_quaternion_pack(m_system_id, m_component_id, &msg,
                                             current_state.Timestamp / 1000, // time_boot_ms
                                             current_state.orientation.w(),
                                             current_state.orientation.x(),
                                             current_state.orientation.y(),
                                             current_state.orientation.z(),
                                             current_state.angular_velocity_body.x(), // rollspeed
                                             current_state.angular_velocity_body.y(), // pitchspeed
                                             current_state.angular_velocity_body.z()); // yawspeed

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        CDC_Transmit_FS(buf, len);
    }
}