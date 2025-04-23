#ifndef DJI_HANDLER_HPP
#define DJI_HANDLER_HPP

// Base OSDK includes
#include "dji_linux_helpers.hpp" // For LinuxSetup definition
#include "dji_vehicle.hpp"       // For Vehicle
#include "dji_error.hpp"         // For ErrorCode

// Includes needed for AdvancedSensing stream (OSDK v3.x/4.x style)
#include "dji_advanced_sensing.hpp" // Include AdvancedSensing header
// Include for CameraRGBImage (likely included via dji_advanced_sensing.hpp or dji_liveview.hpp)
// #include "dji_camera_image.hpp" // Assuming this is included via headers above
// #include "dji_type.hpp" // Removed as CameraRGBImage seems to be outside DJI::OSDK

// Forward declarations (if needed)

/**
 * @brief Initializes the DJI OSDK environment and Vehicle object.
 *        Checks the 'connect_to_drone' configuration flag.
 *
 * @param argc Argument count from main().
 * @param argv Argument vector from main().
 * @param linuxEnvironment Pointer to a LinuxSetup pointer (will be allocated if successful).
 * @param vehicle Pointer to a Vehicle pointer (will point to the initialized vehicle if successful).
 * @return true if OSDK initialization is successful (or skipped intentionally), false on failure.
 */
bool initializeDJISDK(int argc, char** argv, LinuxSetup*& linuxEnvironment, DJI::OSDK::Vehicle*& vehicle);

/**
 * @brief Sets up and starts the required telemetry subscriptions (Flight Status, Display Mode, Altitude).
 *
 * @param vehicle Pointer to the initialized Vehicle object.
 * @param pkgIndex The package index to use for telemetry (output).
 * @return true if telemetry subscription is successfully started, false otherwise.
 */
bool setupTelemetry(DJI::OSDK::Vehicle* vehicle, int& pkgIndex);


/**
 * @brief The main function for the background monitoring thread.
 *        Polls telemetry data (Flight Status, Display Mode, Altitude), updates global status,
 *        checks for timeouts and unexpected status changes.
 *
 * @param vehicle Pointer to the initialized Vehicle object.
 * @param enableFlightControl Flag indicating if flight control is generally enabled (used to decide if mode warnings are relevant).
 */
void monitoringLoopFunction(DJI::OSDK::Vehicle* vehicle, bool enableFlightControl);

/**
 * @brief Attempts to enable the main camera stream using AdvancedSensing.
 *        Uses startMainCameraStream.
 *
 * @param vehicle Pointer to the initialized Vehicle object.
 * @return true if the stream was successfully started, false otherwise.
 */
bool setupCameraStream(DJI::OSDK::Vehicle* vehicle);

/**
 * @brief Attempts to disable the main camera stream using AdvancedSensing.
 *        Uses stopMainCameraStream.
 *
 * @param vehicle Pointer to the initialized Vehicle object.
 * @return true always (as the underlying API call returns void).
 */
bool stopCameraStream(DJI::OSDK::Vehicle* vehicle);


/**
 * @brief Callback function for receiving camera stream data via AdvancedSensing.
 *        Receives CameraRGBImage struct (defined outside DJI::OSDK namespace).
 *        Updates global flags to indicate data reception.
 *
 * @param image The received camera image data struct.
 * @param userData Optional user data (not used here).
 */
void cameraStreamCallback(CameraRGBImage image, void *userData); // REMOVED DJI::OSDK::


// --- OSDK Placeholder Callbacks ---
// These might be required by certain OSDK functions, even if unused.
void ObtainJoystickCtrlAuthorityCB(DJI::OSDK::ErrorCode::ErrorCodeType errorCode, DJI::OSDK::UserData userData);
void ReleaseJoystickCtrlAuthorityCB(DJI::OSDK::ErrorCode::ErrorCodeType errorCode, DJI::OSDK::UserData userData);
// --- End OSDK Placeholder Callbacks ---


#endif // DJI_HANDLER_HPP