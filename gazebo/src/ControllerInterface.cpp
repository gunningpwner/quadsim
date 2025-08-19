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

        InputData input;
        input.roll = normalizeAxis(SDL_JoystickGetAxis(m_joystick, 0));
        input.pitch = -normalizeAxis(SDL_JoystickGetAxis(m_joystick, 1)); // Invert for standard flight controls
        input.yaw = normalizeAxis(SDL_JoystickGetAxis(m_joystick, 3));
        input.throttle = normalizeTrigger(SDL_JoystickGetAxis(m_joystick, 5)); // Use right trigger
        std::cout << "Roll: " << input.roll << ", Pitch: " << input.pitch << ", Yaw: " << input.yaw << ", Throttle: " << input.throttle << std::endl;


        m_dataManager.post(input);

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
