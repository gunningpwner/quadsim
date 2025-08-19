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
    float normalizeTrigger(Sint16 value);
    DataManager& m_dataManager;
    SDL_Joystick* m_joystick = nullptr;

    // --- Threading for Polling ---
    std::thread m_pollingThread;
    std::atomic<bool> m_run_polling;
};

#endif // CONTROLLER_INTERFACE_H
