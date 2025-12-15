#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include "DataManager.h"
#include <SDL2/SDL.h>
#include <thread>
#include <atomic>

/**
 * @class ControllerInterface
 * @brief Manages input from an SDL2-compatible joystick (e.g., Xbox controller).
 *
 * This class initializes the SDL joystick subsystem, detects a controller,
 * and runs a polling loop in a separate thread to send input data to the DataManager.
 */
class ControllerInterface {
public:
    /**
     * @brief Constructs a ControllerInterface.
     * @param dataManager A reference to the flight software's central DataManager.
     */
    ControllerInterface(DataManager& dataManager);

    /**
     * @brief Destructor to clean up SDL resources and join the polling thread.
     */
    ~ControllerInterface();

    /**
     * @brief Initializes SDL and attempts to open the first available joystick.
     * @return True if initialization is successful and a controller is found, false otherwise.
     */
    bool initialize();

    /**
     * @brief Starts the controller polling loop in a separate thread.
     */
    void startPolling();

private:
    /**
     * @brief The main loop for polling controller input.
     *
     * This function runs in a separate thread, continuously reading the
     * controller's axis states and posting them to the DataManager.
     */
    void runPollingLoop();

    /**
     * @brief Normalizes a raw joystick axis value from [-32768, 32767] to [-1.0, 1.0].
     * @param value The raw integer value from the joystick axis.
     * @return The normalized floating-point value.
     */
    float normalizeAxis(Sint16 value);

    /**
     * @brief Maps a normalized float value [-1.0, 1.0] or [0.0, 1.0] to the CRSF channel range [172, 1811].
     * @param value The normalized input value.
     * @param is_throttle True if the input is for the throttle axis.
     * @return The corresponding value in the CRSF range.
     */
    uint16_t mapToCrsf(float value, bool is_throttle = false);
    float normalizeTrigger(Sint16 value);
    DataManager& m_dataManager;
    SDL_Window* m_window = nullptr;
    SDL_Joystick* m_joystick = nullptr;

    // --- Threading for Polling ---
    std::thread m_pollingThread;
    std::atomic<bool> m_run_polling;

    // --- Keyboard Fallback ---
    bool m_use_keyboard = false;
    float m_kb_roll = 0.0f;
    float m_kb_pitch = 0.0f;
    float m_kb_yaw = 0.0f;
    float m_kb_throttle = 0.0f;
    bool m_kb_armed_toggle = false;

    void handleKeyEvent(const SDL_KeyboardEvent& key_event);
    void buildAndPostKeyboardData();
};

#endif // CONTROLLER_INTERFACE_H
