#include "ControllerInterface.h"
#include "SensorData.h"
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
    if (m_window) {
        SDL_DestroyWindow(m_window);
    }
    SDL_Quit();
}

bool ControllerInterface::initialize() {
    // Initialize SDL joystick and video subsystems (video is needed for keyboard events)
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
        return false;
    }

    // Check for available joysticks
    if (SDL_NumJoysticks() < 1) {
        std::cout << "No joysticks connected. Falling back to keyboard control." << std::endl;
        m_use_keyboard = true;
        // Create a visible window to capture keyboard events. It must have focus.
        m_window = SDL_CreateWindow("Keyboard Input", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 320, 240, 0);
        if (!m_window) {
            std::cerr << "Could not create window for keyboard input! SDL Error: " << SDL_GetError() << std::endl;
            return false;
        }
    } else {
        // Open the first joystick
        m_joystick = SDL_JoystickOpen(0);
        if (m_joystick == nullptr) {
            std::cerr << "Warning: Unable to open game controller! SDL Error: " << SDL_GetError() << std::endl;
            // Even if joystick fails, we can still proceed with keyboard
            m_use_keyboard = true;
        } else {
            std::cout << "Found Joystick: " << SDL_JoystickName(m_joystick) << std::endl;
            m_use_keyboard = false;
        }
    }

    return true;
}

void ControllerInterface::startPolling() {
    m_run_polling.store(true);
    m_pollingThread = std::thread(&ControllerInterface::runPollingLoop, this);
}

void ControllerInterface::runPollingLoop() {
    SDL_Event event;
    while (m_run_polling.load()) {
        // Process all pending events.
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                m_run_polling.store(false);
                break;
            }
            if (m_use_keyboard && (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)) {
                handleKeyEvent(event.key);
            }
        }

        if (m_use_keyboard) {
            buildAndPostKeyboardData();
        } else {
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
            rc_data.channels.chan4 = CRSF_CHANNEL_MAX; // AUX1 (Arm Switch)

            m_dataManager.post(rc_data);
        }
        
        // Poll at a reasonable rate (e.g., 50 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void ControllerInterface::handleKeyEvent(const SDL_KeyboardEvent& key_event) {
    const bool is_pressed = (key_event.type == SDL_KEYDOWN);

    switch (key_event.keysym.sym) {
        // Pitch
        case SDLK_w: m_kb_pitch = is_pressed ? 1.0f : 0.0f; break;
        case SDLK_s: m_kb_pitch = is_pressed ? -1.0f : 0.0f; break;
        // Roll
        case SDLK_a: m_kb_roll = is_pressed ? -1.0f : 0.0f; break;
        case SDLK_d: m_kb_roll = is_pressed ? 1.0f : 0.0f; break;
        // Yaw
        case SDLK_q: m_kb_yaw = is_pressed ? -1.0f : 0.0f; break;
        case SDLK_r: m_kb_yaw = is_pressed ? 1.0f : 0.0f; break;
        // Throttle
        case SDLK_LSHIFT: m_kb_throttle = is_pressed ? 1.0f : 0.0f; break;
        // Arm Switch (toggle on key down)
        case SDLK_SPACE:
            if (is_pressed) {
                m_kb_armed_toggle = !m_kb_armed_toggle;
                std::cout << "Arm switch toggled: " << (m_kb_armed_toggle ? "ARMED" : "DISARMED") << std::endl;
            }
            break;
        default:
            break;
    }
}

void ControllerInterface::buildAndPostKeyboardData() {
    RCChannelsData rc_data;
    rc_data.Timestamp = m_dataManager.getCurrentTimeUs();

    // CRSF Channel Order: 0:Roll, 1:Pitch, 2:Throttle, 3:Yaw, 4:AUX1(Arm)
    rc_data.channels.chan0 = mapToCrsf(m_kb_roll);
    rc_data.channels.chan1 = mapToCrsf(m_kb_pitch);
    rc_data.channels.chan2 = mapToCrsf(m_kb_throttle, true);
    rc_data.channels.chan3 = mapToCrsf(m_kb_yaw);


    rc_data.channels.chan4 = m_kb_armed_toggle ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;

    // Zero out other channels for safety
    rc_data.channels.chan5 = CRSF_CHANNEL_MIN;
    rc_data.channels.chan6 = CRSF_CHANNEL_MIN;
    rc_data.channels.chan7 = CRSF_CHANNEL_MIN;

    m_dataManager.post(rc_data);
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
