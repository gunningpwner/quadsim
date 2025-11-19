/*
#pragma once

#include "Consumer.h"
#include "DataManager.h"
#include "OtherData.h"

class GeometricController
{
public:
    explicit GeometricController(DataManager &data_manager);
    void run();

private:
    Consumer<RCChannelsData, RC_CHANNELS_BUFFER_SIZE> m_rc_channels_consumer;
    Consumer<StateData, STATE_BUFFER_SIZE> m_state_consumer;

    RCChannelsData m_rc_data;
    StateData m_state_data;

    GeomFlightMode m_flight_mode;

    float kx = 0;
    float kv = 0;
    float kr = 1;
    float komega = 1;
    float mass = .35; // drone mass in kg
}

enum GeomFlightMode {
    ATTITUDE,
    POSITION,
    VELOCITY
}
    */