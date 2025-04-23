#include "dji_handler.hpp"
#include "globals.hpp" // Access to global flags, status, and camera status
#include "config.hpp"  // Access to configuration flags like enable_camera_stream
#include "utils.hpp"   // Access to helper functions (getModeName)

// DJI OSDK Includes
#include "dji_telemetry.hpp" // For Telemetry topics and getValue
#include "dji_status.hpp"    // For VehicleStatus enums
#include "dji_ack.hpp"       // For ACK::getError, ACK::getErrorCodeMessage
#include "dji_advanced_sensing.hpp" // Include AdvancedSensing header
// #include "dji_type.hpp" // Removed as CameraRGBImage seems to be outside DJI::OSDK

// Standard Library Includes
#include <iostream>
#include <thread>   // For std::this_thread::sleep_for
#include <chrono>   // For std::chrono::seconds, steady_clock
#include <ctime>    // For std::time, time_t
#include <cmath>    // For std::isnan
#include <vector>   // For std::vector

// --- Add missing include for CameraRGBImage if not transitively included ---
// Check if dji_advanced_sensing.hpp includes dji_liveview.hpp which includes dji_camera_image.hpp
// If not, uncomment the line below:
#include "dji_camera_image.hpp" // Explicitly include for CameraRGBImage type

using namespace DJI::OSDK; // DJI OSDK Namespace

// Initializes the DJI OSDK environment and Vehicle object
bool initializeDJISDK(int argc, char** argv, LinuxSetup*& linuxEnvironment, Vehicle*& vehicle) {
    if (connect_to_drone) {
        std::cout << "[DJI Handler] Attempting to initialize DJI OSDK (connect_to_drone is true)..." << std::endl;
        // Dynamically allocate LinuxSetup
        linuxEnvironment = new LinuxSetup(argc, argv);
        vehicle = linuxEnvironment->getVehicle();

        // --- Explicitly check if vehicle pointer is null first ---
        if (vehicle == nullptr) {
            std::cerr << "ERROR: Failed to get Vehicle instance from LinuxSetup. OSDK initialization failed." << std::endl;
            // Do NOT delete linuxEnvironment here, let main handle it
            g_current_display_mode.store(255);
            g_camera_data_receiving.store(false);
            return false;
        }
        std::cout << "[DJI Handler Debug] Vehicle instance obtained from LinuxSetup." << std::endl;


        // --- Check CORE interfaces (Control, Subscribe) ---
        bool control_ok = (vehicle->control != nullptr);
        bool subscribe_ok = (vehicle->subscribe != nullptr);

        if (!control_ok || !subscribe_ok) {
             std::string missing = "";
             if (!control_ok) missing += " control";
             if (!subscribe_ok) missing += " subscribe";
             std::cerr << "ERROR: Vehicle initialized, but ESSENTIAL interfaces unavailable:" << missing
                       << ". Cannot proceed with OSDK flight control/telemetry. OSDK initialization failed." << std::endl;
             vehicle = nullptr; // Set vehicle to null to prevent further use
             g_current_display_mode.store(255);
             g_camera_data_receiving.store(false);
             return false; // Indicate failure
        }
        std::cout << "[DJI Handler Debug] Essential interfaces (Control, Subscribe) are present." << std::endl;

        // --- Check OPTIONAL interface (AdvancedSensing) ---
        bool advanced_sensing_ok = (vehicle->advancedSensing != nullptr);
        std::cout << "[DJI Handler Debug] AdvancedSensing interface " << (advanced_sensing_ok ? "IS" : "is NOT") << " present." << std::endl;
        std::cout << "[DJI Handler Debug] enable_camera_stream config is: " << std::boolalpha << enable_camera_stream << std::endl;

        // Fail *only* if camera is enabled AND the interface is missing
        if (enable_camera_stream && !advanced_sensing_ok) {
             std::cerr << "ERROR: Camera stream is enabled (enable_camera_stream=true), but AdvancedSensing interface is unavailable. OSDK initialization failed." << std::endl;
             // OSDK Log "[1428622.394]STATUS/1 @ init, L247: USB is not plugged or initialized successfully. Advacned-Sensing will not run." indicates why it might be null.
             vehicle = nullptr; // Set vehicle to null
             g_current_display_mode.store(255);
             g_camera_data_receiving.store(false);
             return false; // Indicate failure because camera was requested but unavailable
        }

        // --- If we reach here, all *required* interfaces are present ---
        std::cout << "[DJI Handler] OSDK Vehicle instance OK. All required interfaces present.";
        if (enable_camera_stream) {
            if (advanced_sensing_ok) {
                std::cout << " Camera stream enabled and AdvancedSensing available." << std::endl;
            } else {
                // This case should not be reached due to the check above, but log defensively
                 std::cout << " Camera stream enabled BUT AdvancedSensing unavailable (THIS SHOULD NOT HAPPEN - Check logic!)." << std::endl;
            }
        } else {
             std::cout << " Camera stream disabled by config (AdvancedSensing status: " << (advanced_sensing_ok ? "present" : "absent") << ")." << std::endl;
             g_camera_data_receiving.store(false); // Ensure camera status is false if disabled
        }
        return true; // Success!

    } else {
        std::cout << "[DJI Handler] Skipping OSDK Initialization (connect_to_drone is false). Flight control and camera disabled." << std::endl;
        linuxEnvironment = nullptr; // Ensure pointers are null if skipped
        vehicle = nullptr;
        g_current_display_mode.store(255); // Set display mode to unavailable
        g_camera_data_receiving.store(false); // Ensure camera status is false
        return true; // Indicate success (initialization skipped intentionally)
    }
}

// Sets up and starts the required telemetry subscriptions
bool setupTelemetry(DJI::OSDK::Vehicle* vehicle, int& pkgIndex) {
    if (!vehicle || !vehicle->subscribe) {
        std::cerr << "[DJI Handler] Cannot setup telemetry: Vehicle object or subscribe interface is null." << std::endl;
        g_current_display_mode.store(255);
        return false;
    }

    int functionTimeout = 1; // Timeout for OSDK calls
    int telemetrySubscriptionFrequency = 10; // Hz (can be made configurable if needed)
    pkgIndex = 0; // Use package index 0

    std::cout << "[DJI Handler] Setting up Telemetry Subscription... Freq: " << telemetrySubscriptionFrequency << " Hz" << std::endl;

    // Verify subscription capabilities
    ACK::ErrorCode subscribeAck = vehicle->subscribe->verify(functionTimeout);
    if (ACK::getError(subscribeAck)) {
         ACK::getErrorCodeMessage(subscribeAck, "[DJI Handler] verify");
         std::cerr << "Error verifying subscription package list. Monitoring will be disabled." << std::endl;
         g_current_display_mode.store(255);
         return false;
    }

    // Define the telemetry topics to subscribe to
    Telemetry::TopicName topicList[] = {
        Telemetry::TOPIC_STATUS_FLIGHT,      // Drone's current flight status (on ground, in air, etc.)
        Telemetry::TOPIC_STATUS_DISPLAYMODE, // Current flight mode (P-GPS, Attitude, SDK Control, etc.)
        Telemetry::TOPIC_HEIGHT_FUSION       // Fused altitude data (typically AGL)
    };
    int numTopic = sizeof(topicList) / sizeof(topicList[0]);
    bool enableTimestamp = false; // We don't need OSDK timestamps for these topics

    // Initialize the telemetry package
    bool topicStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList, enableTimestamp, telemetrySubscriptionFrequency);

    if (!topicStatus) {
         std::cerr << "Error initializing telemetry package " << pkgIndex << ". Monitoring disabled." << std::endl;
         g_current_display_mode.store(255);
         return false;
    }
    std::cout << "  Successfully initialized telemetry package " << pkgIndex << " with " << numTopic << " topics." << std::endl;


    // Start the telemetry package
    ACK::ErrorCode startAck = vehicle->subscribe->startPackage(pkgIndex, functionTimeout);
    if (ACK::getError(startAck)) {
         ACK::getErrorCodeMessage(startAck, "[DJI Handler] startPackage");
         std::cerr << "Error starting subscription package " << pkgIndex << ". Monitoring disabled." << std::endl;
         // Attempt to remove the package if starting failed
         vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
         g_current_display_mode.store(255);
         return false;
    }

    std::cout << "  Successfully started telemetry package " << pkgIndex << "." << std::endl;
    // Initial mode will be updated shortly by the monitoring thread
    return true; // Telemetry setup successful
}

// Setup Camera Stream using AdvancedSensing (OSDK v3.x/4.x style) - CONDITIONAL
bool setupCameraStream(DJI::OSDK::Vehicle* vehicle) {
    // --- Check if camera stream is enabled in config ---
    if (!enable_camera_stream) {
        std::cout << "[DJI Handler] Camera stream setup skipped (disabled by config)." << std::endl;
        g_camera_data_receiving.store(false);
        return false; // Indicate skipped / not enabled
    }

    if (!vehicle || !vehicle->advancedSensing) { // Check advancedSensing
        std::cerr << "[DJI Handler] Cannot setup camera stream: Vehicle object or advancedSensing interface is null." << std::endl;
        g_camera_data_receiving.store(false); // Ensure flag is false
        return false; // Indicate failure
    }

    std::cout << "[DJI Handler] Setting up Main Camera Stream (FPV) using AdvancedSensing..." << std::endl;

    // Start the main camera stream and set the callback
    vehicle->advancedSensing->startMainCameraStream(cameraStreamCallback, nullptr);
    bool startResult = true; // Assume success if the call doesn't immediately throw/fail

    std::cout << "  Called startMainCameraStream via AdvancedSensing (assuming success)." << std::endl;
    g_camera_data_receiving.store(false); // Initialize as not receiving until callback fires
    g_last_camera_data_time = std::chrono::steady_clock::time_point::min();
    return startResult;
}

// Stop Camera Stream using AdvancedSensing - CONDITIONAL
bool stopCameraStream(DJI::OSDK::Vehicle* vehicle) {
     // --- Check if camera stream was ever enabled ---
     // We only need to stop it if it might have been started.
     if (!enable_camera_stream) {
         // std::cout << "[DJI Handler] Camera stream stop skipped (disabled by config)." << std::endl; // Optional log
         g_camera_data_receiving.store(false);
         return true; // Indicate success (nothing to do)
     }

     if (!vehicle || !vehicle->advancedSensing) {
        std::cerr << "[DJI Handler] Cannot stop camera stream: Vehicle object or advancedSensing interface is null." << std::endl;
        g_camera_data_receiving.store(false); // Ensure flag is false anyway
        return false; // Indicate failure to attempt stop
    }
    std::cout << "[DJI Handler] Stopping main camera stream via AdvancedSensing..." << std::endl;

    // Stop the main camera stream - This function returns void
    vehicle->advancedSensing->stopMainCameraStream();

    // Try setting a null callback using startMainCameraStream (if it accepts it)
    vehicle->advancedSensing->startMainCameraStream(nullptr, nullptr);

    std::cout << "[DJI Handler] Main camera stream stop called via AdvancedSensing." << std::endl;

    g_camera_data_receiving.store(false); // Ensure status is false after stopping
    return true; // Return true as the stop function was called (it returns void)
}


// Camera Stream Callback (AdvancedSensing style using CameraRGBImage)
void cameraStreamCallback(CameraRGBImage image, void *userData) { // REMOVED DJI::OSDK::
    // Check if image data seems valid (using members of CameraRGBImage)
    // Common members might be rawData (std::vector<uint8_t>), width, height. Adjust if different.
    if (image.rawData.empty() || image.width <= 0 || image.height <= 0) {
        // std::cerr << "Warning: Received invalid camera frame data." << std::endl; // Can be noisy
        return;
    }

    // Update status flags
    g_camera_data_receiving.store(true);
    { // Scope for lock if needed
      // std::lock_guard<std::mutex> lock(g_cameraStatusMutex);
      g_last_camera_data_time = std::chrono::steady_clock::now();
    }

    // --- IMPORTANT ---
    // Still NOT processing or storing the image.rawData here (likely RGB or YUV, but not decoded H.264).
    // This callback just confirms data is flowing.
    // ---
}


// Background thread function for monitoring
void monitoringLoopFunction(DJI::OSDK::Vehicle* vehicle, bool enableFlightControl) {
    // Ensure vehicle pointer and subscribe interface are valid before starting loop
    if (vehicle == nullptr || vehicle->subscribe == nullptr) {
        std::cerr << "[Monitoring] Error: Invalid Vehicle object or subscribe interface provided. Thread exiting." << std::endl;
        g_current_display_mode.store(255); // Set to unavailable on error
        g_camera_data_receiving.store(false); // Ensure camera status is false
        return;
    }

    std::cout << "[Monitoring] Thread started." << std::endl; // Logged
    bool telemetry_timed_out = false;         // Flag: Is telemetry currently timed out?
    bool warned_unexpected_status = false;    // Flag: Have we warned about unexpected landing?
    uint8_t previous_flight_status = VehicleStatus::FlightStatus::STOPED; // Track previous status
    time_t last_valid_poll_time = 0;          // Timestamp of the last valid data received
    bool in_sdk_control_mode = false;         // Flag: Is the drone currently in SDK control mode?
    bool warned_not_in_sdk_mode = false;      // Flag: Have we warned about not being in SDK mode?
    const uint8_t EXPECTED_SDK_MODE = VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL; // The mode required for control

    // Allow some time for the first telemetry data to arrive
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (!stopMonitoringFlag.load()) {
        // Re-check pointers inside loop in case they somehow become invalid (unlikely but safe)
        // Note: vehicle pointer itself shouldn't become null here unless deleted elsewhere unexpectedly.
        if (vehicle == nullptr || vehicle->subscribe == nullptr) {
             if (!telemetry_timed_out) { // Avoid spamming log
                 std::cerr << "\n**** MONITORING ERROR: Vehicle/subscribe object became null. Stopping. ****" << std::endl << std::endl; // Logged
                 telemetry_timed_out = true;
             }
             g_current_display_mode.store(255); // Set to unavailable on error
             g_camera_data_receiving.store(false); // Ensure camera status is false
             break; // Exit loop
        }

        // --- Read Telemetry Data ---
        Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type      current_flight_status;
        Telemetry::TypeMap<Telemetry::TOPIC_STATUS_DISPLAYMODE>::type current_display_mode;
        Telemetry::TypeMap<Telemetry::TOPIC_HEIGHT_FUSION>::type      current_height;

        // Use non-blocking getValue calls
        current_flight_status = vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
        current_display_mode = vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();

        bool valid_poll = (current_flight_status <= VehicleStatus::FlightStatus::IN_AIR);

        // --- Update Global Display Mode ---
        g_current_display_mode.store(current_display_mode);
        // --- End Update ---

        // --- Update Global Live Data Altitude ---
        if (valid_poll && !std::isnan(current_height) && current_height >= 0) {
             if (!processingThread.joinable() || !enableFlightControl) {
                 std::lock_guard<std::mutex> lock(g_liveDataMutex);
                 g_latest_altitude = current_height;
             }
        }
        // --- End Update Live Data Altitude ---


        if (valid_poll) { // Process only if the basic poll seems valid
            time_t current_time = std::time(nullptr);
            last_valid_poll_time = current_time; // Update time of last valid data
            if (telemetry_timed_out) {
                 std::cout << "[Monitoring] Telemetry poll recovered." << std::endl; // Logged
                 telemetry_timed_out = false; // Reset timeout flag
            }

            // --- Check Flight Status Change ---
            if (current_flight_status != VehicleStatus::FlightStatus::IN_AIR) {
                if ( (previous_flight_status == VehicleStatus::FlightStatus::IN_AIR) &&
                     (current_flight_status == VehicleStatus::FlightStatus::ON_GROUND || current_flight_status == VehicleStatus::FlightStatus::STOPED) &&
                     !warned_unexpected_status) {
                     std::cerr << "\n**** MONITORING WARNING: Flight status changed unexpectedly from IN_AIR to "
                               << (current_flight_status == VehicleStatus::FlightStatus::ON_GROUND ? "ON_GROUND" : "STOPED")
                               << " (" << (int)current_flight_status << "). ****" << std::endl << std::endl; // Logged
                     warned_unexpected_status = true;
                     if (g_vertical_state.load(std::memory_order_relaxed) != VerticalAlignState::IDLE) {
                         std::cerr << "[Monitoring] Warning: Flight status changed during vertical maneuver. Reverting vertical state to IDLE." << std::endl;
                         g_vertical_state.store(VerticalAlignState::IDLE, std::memory_order_relaxed);
                     }
                }
            } else {
                 warned_unexpected_status = false; // Reset warning flag
            }
            previous_flight_status = current_flight_status; // Update previous status

            // --- Check Expected SDK Mode ---
            bool is_expected_mode = (current_display_mode == EXPECTED_SDK_MODE);
            if (is_expected_mode) {
                if (!in_sdk_control_mode) {
                    std::cout << "\n**** MONITORING INFO: Entered SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << " / " << (int)EXPECTED_SDK_MODE << ") ****" << std::endl << std::endl; // Logged
                    in_sdk_control_mode = true;
                    warned_not_in_sdk_mode = false; // Reset warning flag
                }
            } else {
                 if ( enableFlightControl && (in_sdk_control_mode || !warned_not_in_sdk_mode) && processingThread.joinable() ) {
                      std::string current_mode_name = getModeName(current_display_mode);
                      std::cerr << "\n**** MONITORING WARNING: NOT in expected SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << "). Current: "
                                << current_mode_name << " (" << (int)current_display_mode << "). External control commands may be ignored. ****" << std::endl << std::endl; // Logged
                      warned_not_in_sdk_mode = true;
                       if (g_vertical_state.load(std::memory_order_relaxed) != VerticalAlignState::IDLE) {
                           std::cerr << "[Monitoring] Warning: Display mode changed during vertical maneuver. Reverting vertical state to IDLE." << std::endl;
                           g_vertical_state.store(VerticalAlignState::IDLE, std::memory_order_relaxed);
                       }
                 }
                 in_sdk_control_mode = false; // Update status
            }
        } else { // Invalid poll data received
            if (!telemetry_timed_out) {
                 std::cerr << "\n**** MONITORING WARNING: Polling telemetry returned potentially invalid data (FlightStatus=" << (int)current_flight_status << "). ****" << std::endl << std::endl; // Logged
                 telemetry_timed_out = true;
            }
             g_current_display_mode.store(255); // Set global status to unavailable on invalid poll
        }

        // --- Check Telemetry Timeout ---
        time_t current_time_for_timeout_check = std::time(nullptr);
        if (last_valid_poll_time > 0 && (current_time_for_timeout_check - last_valid_poll_time > TELEMETRY_TIMEOUT_SECONDS)) {
            if (!telemetry_timed_out) {
                std::cerr << "\n**** MONITORING TIMEOUT: No valid telemetry for over " << TELEMETRY_TIMEOUT_SECONDS << " seconds. ****" << std::endl << std::endl; // Logged
                telemetry_timed_out = true;
                 if (g_vertical_state.load(std::memory_order_relaxed) != VerticalAlignState::IDLE) {
                     std::cerr << "[Monitoring] Warning: Telemetry timeout during vertical maneuver. Reverting vertical state to IDLE." << std::endl;
                     g_vertical_state.store(VerticalAlignState::IDLE, std::memory_order_relaxed);
                 }
            }
            g_current_display_mode.store(255); // Set global status to unavailable on timeout
        } else if (last_valid_poll_time == 0) {
             static time_t monitoring_start_time = 0;
             if (monitoring_start_time == 0) monitoring_start_time = current_time_for_timeout_check;
             if (current_time_for_timeout_check - monitoring_start_time > TELEMETRY_TIMEOUT_SECONDS * 2) {
                  if (!telemetry_timed_out) {
                       std::cerr << "\n**** MONITORING TIMEOUT: Never received valid telemetry poll after " << TELEMETRY_TIMEOUT_SECONDS * 2 << " seconds. Check connection and drone status. ****" << std::endl << std::endl; // Logged
                       telemetry_timed_out = true;
                  }
                  g_current_display_mode.store(255); // Set global status to unavailable on initial timeout
             }
        }

        // --- Check Camera Data Timeout (Only if camera stream is enabled) ---
        if (enable_camera_stream && g_camera_data_receiving.load()) {
             auto now = std::chrono::steady_clock::now();
             auto time_since_last_frame = std::chrono::duration_cast<std::chrono::seconds>(now - g_last_camera_data_time).count();
             if (g_last_camera_data_time != std::chrono::steady_clock::time_point::min() && // Ensure time was initialized
                 time_since_last_frame > CAMERA_DATA_TIMEOUT_SECONDS) {
                 std::cerr << "[Monitoring] Warning: Camera data stream timed out (callback stopped firing? Last data " << time_since_last_frame << "s ago)." << std::endl;
                 g_camera_data_receiving.store(false); // Set status to not receiving
             }
        }
        // --- End Camera Data Timeout Check ---


        // Pause before next poll cycle
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Poll frequency (adjust if needed)
    } // End while (!stopMonitoringFlag.load())

    std::cout << "[Monitoring] Thread finished." << std::endl; // Logged
    g_current_display_mode.store(255); // Set global status to unavailable when thread stops
    g_camera_data_receiving.store(false); // Ensure camera status is false when thread stops
}


// --- OSDK Placeholder Callbacks Implementation ---
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) {
    // std::cout << "Obtain Joystick Control Authority Callback Received (Unused)" << std::endl;
}

void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) {
    // std::cout << "Release Joystick Control Authority Callback Received (Unused)" << std::endl;
}
// --- End OSDK Placeholder Callbacks Implementation ---
