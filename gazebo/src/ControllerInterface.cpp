#include "ControllerInterface.h"
#include <iostream>

ControllerInterface::ControllerInterface(DataManager& dataManager)
    : m_dataManager(dataManager), m_run_polling(false) {}

ControllerInterface::~ControllerInterface() {
    // Safely stop the polling thread
    if (m_run_polling.load()) {
        m_run_polling.store(false);
        if (m_pollingThread.joinable()) {
            m_pollingThread.join();
        }
    }

    // Clean up SDL resources
    if (m_joystick) {
        SDL_JoystickClose(m_joystick);
    }
    SDL_Quit();
}

bool ControllerInterface::initialize() {
    // Initialize the SDL joystick subsystem
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
        return false;
    }

    // Check for available joysticks
    if (SDL_NumJoysticks() < 1) {
        std::cerr << "Warning: No joysticks connected!" << std::endl;
        return false;
    }

    // Open the first joystick
    m_joystick = SDL_JoystickOpen(0);
    if (m_joystick == nullptr) {
        std::cerr << "Warning: Unable to open game controller! SDL Error: " << SDL_GetError() << std::endl;
        return false;
    }

    std::cout << "Found Joystick: " << SDL_JoystickName(m_joystick) << std::endl;
    return true;
}

void ControllerInterface::startPolling() {
    if (!m_joystick) {
        std::cerr << "Cannot start polling, controller not initialized." << std::endl;
        return;
    }
    m_run_polling.store(true);
    m_pollingThread = std::thread(&ControllerInterface::runPollingLoop, this);
}

void ControllerInterface::runPollingLoop() {
    while (m_run_polling.load()) {
        // Update the state of all joysticks
        SDL_JoystickUpdate();

        // Updated Xbox Controller Axis Mappings:
        // Axis 0: Left Stick X  (-1.0 to 1.0) -> Roll
        // Axis 1: Left Stick Y  (-1.0 to 1.0) -> Pitch
        // Axis 3: Right Stick X (-1.0 to 1.0) -> Yaw
        // Axis 5: Right Trigger ( 0.0 to 1.0) -> Throttle

        RCChannelsData rc_data;
        rc_data.Timestamp = m_dataManager.getCurrentTimeUs();

        // Map joystick axes to CRSF channels.
        // CRSF Channel Order: 0:Roll, 1:Pitch, 2:Throttle, 3:Yaw
        rc_data.channels.chan0 = mapToCrsf(normalizeAxis(SDL_JoystickGetAxis(m_joystick, 0))); // Roll
        rc_data.channels.chan1 = mapToCrsf(-normalizeAxis(SDL_JoystickGetAxis(m_joystick, 1))); // Pitch (inverted)
        rc_data.channels.chan2 = mapToCrsf(normalizeTrigger(SDL_JoystickGetAxis(m_joystick, 5)), true); // Throttle
        rc_data.channels.chan3 = mapToCrsf(normalizeAxis(SDL_JoystickGetAxis(m_joystick, 3))); // Yaw

        // For now, we'll set the arm switch (AUX1/ch4) to a fixed "armed" value for testing.
        // A real implementation would map this to a joystick button.
        const uint16_t CRSF_CHANNEL_MAX = 1811;
        rc_data.channels.chan4 = CRSF_CHANNEL_MAX; // AUX1 (Arm Switch)

        m_dataManager.post(rc_data);
        
        // Poll at a reasonable rate (e.g., 50 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

float ControllerInterface::normalizeAxis(Sint16 value) {
    // Add a small deadzone
    if (std::abs(value) < 327) { // 1% deadzone
        return 0.0f;
    }
    return static_cast<float>(value) / 32767.0f;
}

/**
 * @brief Normalizes a raw trigger axis value from [0, 32767] to [0.0, 1.0].
 * @param value The raw integer value from the trigger axis.
 * @return The normalized floating-point value.
 */
float ControllerInterface::normalizeTrigger(Sint16 value) {
    return static_cast<float>(value) / 32767.0f;
}

uint16_t ControllerInterface::mapToCrsf(float value, bool is_throttle) {
    // CRSF channel values range from 172 to 1811.
    const float CRSF_MIN = 172.0f;
    const float CRSF_MAX = 1811.0f;
    const float CRSF_RANGE = CRSF_MAX - CRSF_MIN;

    if (is_throttle) {
        // Map [0.0, 1.0] to [172, 1811]
        return static_cast<uint16_t>(CRSF_MIN + value * CRSF_RANGE);
    } else {
        // Map [-1.0, 1.0] to [172, 1811]
        // First, shift the value from [-1, 1] to [0, 1]
        float scaled_value = (value + 1.0f) / 2.0f;
        return static_cast<uint16_t>(CRSF_MIN + scaled_value * CRSF_RANGE);
    }
}
