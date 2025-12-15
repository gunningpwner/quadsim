#include "GPS.h"
#include "GPS_config.h"
#include "DataManager.h"
#include "timing.h"
#include "SensorData.h"

extern DataManager *g_data_manager_ptr;
extern UART_HandleTypeDef huart4;

GPS::GPS() {}

struct __attribute__((packed)) UbxHeader
{
    uint8_t sync1; // Always 0xB5
    uint8_t sync2; // Always 0x62
    uint8_t cls;   // Message Class
    uint8_t id;    // Message ID
    uint16_t len;  // Length of the PAYLOAD only (Little Endian)
};

struct __attribute__((packed)) UBX_NAV_PVT_MSG
{
    uint32_t iTOW;    // GPS time of week [ms]
    uint16_t year;    // Year [AD]
    uint8_t month;    // Month [1-12]
    uint8_t day;      // Day of month [1-31]
    uint8_t hour;     // Hour of day [0-23]
    uint8_t min;      // Minute of hour [0-59]
    uint8_t sec;      // Second of minute [0-60]
    // Validity Flags
    uint8_t validDate : 1; // Date Validity Flag
    uint8_t validTime : 1; // Time Validity Flag
    uint8_t fullyResolved : 1; // 1 if UTC time of day has been fully resolved
    uint8_t validMag : 1; // Magnetic Declination Validity Flag
    uint8_t validFiller:4; // Filler
    uint32_t tAcc;    // Time accuracy estimate [ns]
    int32_t nano;     // Nanoseconds of second [ns]
    uint8_t fixType;  // GNSSfix Type (e.g., no fix, 2D, 3D)
    // Fix Status Flags
    uint8_t gnssFixOk:1; // 1 if valid fix
    uint8_t diffSoln:1; // 1 if differential corrections applied
    uint8_t psmState:3; // Power Save Mode State
    uint8_t headVehValid:1; // Vehicle Heading Validity Flag
    uint8_t carrSoln:2; // Carrier phase range solution status
    uint8_t flags2;   // Additional fix status flags
    uint8_t numSV;    // Number of satellites used in fix
    int32_t lon;      // Longitude [1e-7 deg]
    int32_t lat;      // Latitude [1e-7 deg]
    int32_t height;   // Height above ellipsoid [mm]
    int32_t hMSL;     // Height above mean sea level [mm]
    uint32_t hAcc;    // Horizontal accuracy estimate [mm]
    uint32_t vAcc;    // Vertical accuracy estimate [mm]
    int32_t velN;     // NED north velocity [mm/s]
    int32_t velE;     // NED east velocity [mm/s]
    int32_t velD;     // NED down velocity [mm/s]
    int32_t gSpeed;   // Ground Speed (2-D) [mm/s]
    int32_t headMot;  // Heading of motion (2-D) [1e-5 deg]
    uint32_t sAcc;    // Speed accuracy estimate [mm]
    uint32_t headAcc; // Heading accuracy estimate [1e-5 deg]
};

int8_t GPS::init()
{

    // Send reset cmd
    writeUBXMessage(0x06, 0x04, (const uint8_t[]){0x00, 0x00, 0x01, 0x00}, 4);
    HAL_UARTEx_ReceiveToIdle(&huart4, getRxBuffer(), 256, 0, HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart4, m10_config, sizeof(m10_config), 1000);
    HAL_UART_Receive(&huart4, getRxBuffer(), 256, 1000);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, getRxBuffer(), 100);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

    return 0;
}

void GPS::handleRxChunk(uint8_t *buf, uint16_t len)
{
    if (len <= sizeof(UbxHeader))
        return;

    UbxHeader *h = reinterpret_cast<UbxHeader *>(buf);
    if (h->sync1 != 0xb5 || h->sync2 != 0x62)
        return;

    uint64_t timestamp = getCurrentTimeUs();
    // Class and ID for UBX-NAV-PVT
    if (h->cls == 0x01 && h->id == 0x07)
    {
        UBX_NAV_PVT_MSG *msg = reinterpret_cast<UBX_NAV_PVT_MSG *>(buf+sizeof(UbxHeader));
        if (msg->gnssFixOk==1 && msg->fixType==3)
        {
            if(sizeof(GPSData)==0)
                return;
            GPSData gps_data;
            gps_data.Timestamp = timestamp;
            gps_data.lla.x() = msg->lat * 1e-7;
            gps_data.lla.y() = msg->lon * 1e-7;
            gps_data.lla.z() = msg->height * 1e-3;
            gps_data.vel.x() = msg->velN * 1e-3;
            gps_data.vel.y() = msg->velE * 1e-3;
            gps_data.vel.z() = msg->velD * 1e-3;
            gps_data.Satellites = msg->numSV;
            g_data_manager_ptr->post(gps_data);
        }
    }
    
}

void GPS::writeUBXMessage(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
    UbxHeader header;
    header.sync1 = 0xB5;
    header.sync2 = 0x62;
    header.cls = msg_class;
    header.id = msg_id;
    header.len = payload_len;

    uint8_t footer[2] = {0, 0};
    uint8_t *header_ptr = (uint8_t *)&header;
    // Checksum includes from cls to end of payload
    for (int i = 2; i < 6; i++)
    {
        footer[0] += header_ptr[i];
        footer[1] += footer[0];
    }

    for (int i = 0; i < payload_len; i++)
    {
        footer[0] += payload[i];
        footer[1] += footer[0];
    }

    HAL_UART_Transmit(&huart4, (uint8_t *)&header, sizeof(header), 1000);
    HAL_UART_Transmit(&huart4, payload, payload_len, 1000);
    HAL_UART_Transmit(&huart4, footer, 2, 100);
}
