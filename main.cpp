#include <iostream>
#include <string>
#include <atomic>   // Included for atomic flags used in thread management
#include <thread>   // Included for thread objects (monitoringThread, processingThread)

// Core Application Modules
#include "logging.hpp"       // Must be first for RAII logging
#include "config.hpp"        // For loadPreferences() and config flags like enable_camera_stream
#include "globals.hpp"       // Access to global threads, flags, and status
#include "types.hpp"         // Basic types used across modules
#include "dji_handler.hpp"   // For OSDK init, telemetry, monitoring thread, camera setup
#include "gui.hpp"           // For GUI init, main loop, and cleanup
#include "utils.hpp"         // Potentially needed for simple helpers in main if any
#include "python_bridge.hpp" // <-- Included for stopPythonBridge

// DJI OSDK includes needed for cleanup logic in main
#include "dji_ack.hpp"
#include "dji_vehicle.hpp"
#include "dji_linux_helpers.hpp" // <-- Ensures full LinuxSetup definition is known
#include "dji_advanced_sensing.hpp" // <-- Include for stopping camera stream

// SDL includes needed for main GUI structures passed between functions
#include <SDL.h>
#include <SDL_opengl.h> // Or SDL_opengles2.h if using ES2


int main(int argc, char** argv) {
    // --- 1. Instantiate LogRedirector FIRST ---
    LogRedirector logger("run_log.txt");
    std::cout << "[Main] LogRedirector instantiated. Logging started." << std::endl;

    std::cout << "[Main] Starting application: " << (argc > 0 ? argv[0] : "djiosdk-flightcontrol-gui") << std::endl;

    // --- 2. Load Configuration ---
    std::cout << "[Main] Loading preferences..." << std::endl;
    loadPreferences(); // Loads from preferences.txt or sets defaults
    std::cout << "[Main] Preferences loaded." << std::endl;

    // --- 3. Initialize DJI OSDK (Conditional) ---
    LinuxSetup* linuxEnvironment = nullptr; // Pointer to OSDK setup object
    DJI::OSDK::Vehicle*    vehicle = nullptr;          // Pointer to OSDK vehicle object
    bool osdkInitializationAttempted = false;          // Flag: Did we attempt OSDK init?
    bool osdkInitializationSucceeded = false;          // Flag: Did OSDK init succeed?
    bool enableFlightControl = false;                  // Flag: Can we attempt flight control?
    bool monitoringEnabled = false;                    // Flag: Is the monitoring thread running?
    bool cameraStreamStarted = false;                  // Flag: Was camera stream successfully started? (Changed name for clarity)
    int telemetryPkgIndex = 0;                         // Telemetry package index

    if (connect_to_drone) {
        osdkInitializationAttempted = true; // Mark that we tried
        // initializeDJISDK now checks enable_camera_stream internally
        osdkInitializationSucceeded = initializeDJISDK(argc, argv, linuxEnvironment, vehicle);

        if (osdkInitializationSucceeded && vehicle != nullptr) {
            // Initialization successful
             enableFlightControl = true; // OSDK vehicle is ready

            // --- 4. Setup Telemetry and Start Monitoring Thread ---
            if (setupTelemetry(vehicle, telemetryPkgIndex)) {
                 std::cout << "[Main] Telemetry setup successful. Starting monitoring thread..." << std::endl;
                 stopMonitoringFlag.store(false); // Ensure flag is reset
                 monitoringThread = std::thread(monitoringLoopFunction, vehicle, enableFlightControl);
                 monitoringEnabled = true;

                 // --- 4b. Setup Camera Stream (CONDITIONAL) ---
                 if (enable_camera_stream) { // Only attempt if enabled in config
                     if (setupCameraStream(vehicle)) { // Tries to start stream via AdvancedSensing
                          std::cout << "[Main] Camera stream setup successful (started via AdvancedSensing)." << std::endl;
                          cameraStreamStarted = true;
                     } else {
                          std::cerr << "[Main] Camera stream setup failed (AdvancedSensing start failed or interface null?). Camera disabled." << std::endl;
                          cameraStreamStarted = false;
                          // Note: OSDK Initialization still succeeded overall, as camera wasn't mandatory if it failed here (unlike in initializeDJISDK)
                     }
                 } else {
                     std::cout << "[Main] Camera stream setup skipped (disabled by config)." << std::endl;
                     cameraStreamStarted = false;
                 }

            } else {
                 std::cerr << "[Main] Telemetry setup failed. Monitoring disabled. Flight control might be unreliable. Camera disabled." << std::endl;
                 std::cerr << "[Main] WARNING: Proceeding with flight control enabled but telemetry monitoring inactive!" << std::endl;
                 cameraStreamStarted = false; // Ensure camera is marked as not started if telemetry failed
            }
        } else {
             // OSDK initialization failed
             std::cerr << "[Main] Critical OSDK Initialization failure. Flight control and camera disabled." << std::endl;
             enableFlightControl = false; // Ensure flight control is disabled
             cameraStreamStarted = false;
             // vehicle should be nullptr here, linuxEnvironment might be non-null
        }
    } else {
        // OSDK initialization skipped
        std::cout << "[Main] OSDK Initialization skipped (connect_to_drone is false). Flight control and camera disabled." << std::endl;
        enableFlightControl = false; // Ensure flight control is disabled
        cameraStreamStarted = false;
        // linuxEnvironment and vehicle remain nullptr
    }


    // --- Update Status Message ---
    std::string cameraStatusText = "DISABLED";
    if (enable_camera_stream) {
        cameraStatusText = cameraStreamStarted ? "STARTED" : "FAILED/DISABLED";
    } else {
        cameraStatusText = "DISABLED (Config)";
    }
    std::cout << "[Main] Status: Flight Control " << (enableFlightControl ? "ENABLED" : "DISABLED")
              << ", Monitoring " << (monitoringEnabled ? "ENABLED" : "DISABLED")
              << ", Camera Stream " << cameraStatusText // Use updated status text
              << ", Bridge Reconnect " << std::boolalpha << enable_bridge_reconnection.load()
              << ", Local Test Script " << useLocalBridgeScript.load()
              << ", Connect to Drone Setting " << connect_to_drone << "." << std::endl;


    // --- 5. Initialize GUI ---
    SDL_Window* window = nullptr;
    SDL_GLContext gl_context = nullptr;
    bool guiInitialized = false;

    if (initializeGui(window, gl_context)) {
        guiInitialized = true;
        std::cout << "[Main] GUI Initialized successfully." << std::endl;
    } else {
        std::cerr << "[Main] GUI Initialization Failed. Exiting." << std::endl;
        // Perform partial cleanup - NO OSDK cleanup here anymore, moved to final block

        // Only stop python bridge if it was potentially started
        std::string script_to_stop = useLocalBridgeScript.load() ? localPythonBridgeScript : defaultPythonBridgeScript;
        stopPythonBridge(script_to_stop);

        // --- Perform final OSDK Environment cleanup even if GUI failed ---
        if (linuxEnvironment != nullptr) {
            std::cout << "[Main] Deleting OSDK LinuxSetup object (GUI Init Failed)..." << std::endl;
            delete linuxEnvironment;
            linuxEnvironment = nullptr;
            vehicle = nullptr; // Pointer is now invalid
        }
        // --- End final OSDK cleanup ---

        return 1; // Exit with error code
    }

    // --- 6. Run GUI Main Loop ---
    // This part is only reached if GUI initialized successfully
    if (guiInitialized) {
        runGui(window, gl_context, vehicle, enableFlightControl); // vehicle might be null if OSDK init failed
    }

    // --- 7. Final Cleanup (ALWAYS EXECUTED if GUI didn't exit early) ---
    std::cout << "[Main] Application shutdown sequence started..." << std::endl;

    // Stop Processing Thread (if running)
    std::cout << "[Main] Ensuring processing thread is stopped..." << std::endl;
    stopProcessingThreadIfNeeded(); // Includes stopping python bridge

    // Stop Monitoring Thread (if running)
    if (monitoringEnabled && monitoringThread.joinable()) {
        std::cout << "[Main] Signalling monitoring thread to stop..." << std::endl;
        stopMonitoringFlag.store(true);
        monitoringThread.join();
        std::cout << "[Main] Monitoring thread finished." << std::endl;
    }

    // OSDK Resource Cleanup (only if OSDK initialization succeeded)
    if (osdkInitializationSucceeded && vehicle != nullptr) {
        int cleanupTimeout = 1;
        std::cout << "[Main] Starting OSDK resource cleanup..." << std::endl;

        // --- Stop Camera Stream (CONDITIONAL) ---
        // Only stop if it was configured AND successfully started
        if (enable_camera_stream && cameraStreamStarted && vehicle->advancedSensing) {
             stopCameraStream(vehicle);
        } else {
             std::cout << "[Main] Skipping camera stream stop (was not enabled, not started, or advancedSensing unavailable)." << std::endl;
        }

        // Unsubscribe from telemetry (only if monitoring was enabled)
        if (monitoringEnabled && vehicle->subscribe) {
            std::cout << "[Main] Removing telemetry package " << telemetryPkgIndex << "..." << std::endl;
            DJI::OSDK::ACK::ErrorCode statusAck = vehicle->subscribe->removePackage(telemetryPkgIndex, cleanupTimeout);
            if(DJI::OSDK::ACK::getError(statusAck)) {
                DJI::OSDK::ACK::getErrorCodeMessage(statusAck, "[Main] removePackage");
            } else {
                 std::cout << "[Main] Telemetry package removed." << std::endl;
            }
        } else if (vehicle && vehicle->subscribe){ // Check vehicle still exists
             std::cout << "[Main] Skipping telemetry removal (monitoring was not enabled)." << std::endl;
        } else {
             std::cout << "[Main] Skipping telemetry removal (subscribe interface unavailable)." << std::endl;
        }


        // Release Control Authority (only if flight control was enabled)
        if (enableFlightControl && vehicle->control) {
            std::cout << "[Main] Ensuring control authority is released..." << std::endl;
            DJI::OSDK::ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(cleanupTimeout);
             if (DJI::OSDK::ACK::getError(releaseAck) && releaseAck.info.cmd_set != 0x00 && releaseAck.info.cmd_id != 0x00) {
                 DJI::OSDK::ACK::getErrorCodeMessage(releaseAck, "[Main] releaseCtrlAuthority");
                 std::cerr << "[Main] Warning: Failed to release control authority during final cleanup." << std::endl;
             } else {
                 std::cout << "[Main] Control authority released (or was not held)." << std::endl;
             }
        } else if (vehicle && vehicle->control) { // Check vehicle still exists
             std::cout << "[Main] Skipping control authority release (flight control was not enabled)." << std::endl;
        } else {
             std::cout << "[Main] Skipping control authority release (control interface unavailable)." << std::endl;
        }
         std::cout << "[Main] OSDK resource cleanup finished." << std::endl;
    } else {
         std::cout << "[Main] Skipping OSDK resource cleanup (OSDK initialization did not succeed or vehicle is null)." << std::endl;
    }

    // GUI Cleanup (only if initialized)
    if (guiInitialized) {
        cleanupGui(window, gl_context);
    }

    // Final attempt to stop python bridge (if not stopped by stopProcessingThreadIfNeeded)
    std::cout << "[Main] Final check to stop Python bridge..." << std::endl;
    std::string script_to_stop_final = useLocalBridgeScript.load() ? localPythonBridgeScript : defaultPythonBridgeScript;
    stopPythonBridge(script_to_stop_final); // Safe to call even if already stopped

    // --- Delete OSDK Linux Environment (ALWAYS, if allocated) ---
    // This is the final step before exiting.
    if (linuxEnvironment != nullptr) {
        std::cout << "[Main] Deleting OSDK LinuxSetup object (final cleanup)..." << std::endl;
        delete linuxEnvironment;
        linuxEnvironment = nullptr;
        vehicle = nullptr; // Pointer is now invalid
    }
    // --- End final OSDK cleanup ---


    std::cout << "[Main] Application finished gracefully." << std::endl;
    return 0;
}