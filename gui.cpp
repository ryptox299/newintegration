// --- GUI Includes (Order Matters) ---
// 1. GLEW must come first
#include <GL/glew.h>

// --- Now include other headers ---
#include "gui.hpp"
#include "globals.hpp"
#include "config.hpp" // Needed for enable_camera_stream flag
#include "types.hpp"
#include "dji_handler.hpp"
#include "python_bridge.hpp"
#include "radar_processor.hpp"
#include "utils.hpp"

// 2. SDL (Included via gui.hpp now)
// 3. SDL OpenGL headers (AFTER GLEW) (Included via gui.hpp now)

// 4. ImGui
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"

// Standard Library Includes
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <ctime>

// DJI OSDK Includes
#include "dji_vehicle.hpp"
#include "dji_control.hpp"
#include "dji_ack.hpp"
#include "dji_status.hpp" // Added for VehicleStatus::DisplayMode
// #include "dji_camera.hpp" // No longer needed directly in gui.cpp
#include "dji_advanced_sensing.hpp" // Needed for vehicle->advancedSensing check

using namespace DJI::OSDK;

// Helper function to stop the processing thread if it's running
void stopProcessingThreadIfNeeded() {
    std::string script_to_stop = useLocalBridgeScript.load() ? localPythonBridgeScript : defaultPythonBridgeScript; // Use load()

    if (processingThread.joinable()) {
        std::cout << "[GUI] Signalling processing thread to stop..." << std::endl;
        stopProcessingFlag.store(true);
        forceStopReconnectionFlag.store(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        processingThread.join();
        std::cout << "[GUI] Processing thread finished." << std::endl;
        stopProcessingFlag.store(false);
        forceStopReconnectionFlag.store(false);
        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
        g_vertical_state.store(VerticalAlignState::IDLE);
        stopPythonBridge(script_to_stop);

        { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
        { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
        {
            std::lock_guard<std::mutex> lockL(g_liveDataMutex);
            g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
            g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
            g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
            g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
        }

    } else {
         std::cout << "[GUI] No processing thread currently running." << std::endl;
         currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
         g_vertical_state.store(VerticalAlignState::IDLE);
         stopPythonBridge(script_to_stop); // Attempt cleanup anyway

         { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
         { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
         {
             std::lock_guard<std::mutex> lockL(g_liveDataMutex);
             g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
             g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
             g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
             g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
         }
    }
}


// Initializes SDL, GLEW, and ImGui
bool initializeGui(SDL_Window*& window, SDL_GLContext& gl_context) {
    std::cout << "[GUI] Initializing SDL..." << std::endl;
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return false;
    }

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    const char* glsl_version = "#version 100";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#elif defined(__APPLE__)
    const char* glsl_version = "#version 150";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif
    std::cout << "[GUI] Requesting GLSL version: " << glsl_version << std::endl;

    // Create window
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    window = SDL_CreateWindow("DJI OSDK Control GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
    if (!window) { /* ... error handling ... */ std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl; SDL_Quit(); return false; }

    gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) { /* ... error handling ... */ std::cerr << "SDL_GL_CreateContext Error: " << SDL_GetError() << std::endl; SDL_DestroyWindow(window); SDL_Quit(); return false; }
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Initialize GLEW
    std::cout << "[GUI] Initializing OpenGL Loader (GLEW)..." << std::endl;
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) { /* ... error handling ... */ std::cerr << "Failed to initialize GLEW: " << glewGetErrorString(err) << std::endl; SDL_GL_DeleteContext(gl_context); SDL_DestroyWindow(window); SDL_Quit(); return false; }
    std::cout << "[GUI] Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    const GLubyte* glVersion = glGetString(GL_VERSION);
    if (glVersion) { std::cout << "[GUI] OpenGL Version: " << glVersion << std::endl; }
    else { std::cerr << "[GUI] Warning: Could not retrieve OpenGL version string." << std::endl; }

    // Setup ImGui context
    std::cout << "[GUI] Initializing ImGui..." << std::endl;
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    if (!ImGui_ImplSDL2_InitForOpenGL(window, gl_context)) { /* ... error handling ... */ std::cerr << "Failed to initialize ImGui SDL2 backend" << std::endl; ImGui::DestroyContext(); SDL_GL_DeleteContext(gl_context); SDL_DestroyWindow(window); SDL_Quit(); return false; }
    if (!ImGui_ImplOpenGL3_Init(glsl_version)) { /* ... error handling ... */ std::cerr << "Failed to initialize ImGui OpenGL3 backend" << std::endl; ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext(); SDL_GL_DeleteContext(gl_context); SDL_DestroyWindow(window); SDL_Quit(); return false; }

    std::cout << "[GUI] ImGui Initialized Successfully." << std::endl;
    return true;
}

// Runs the main GUI loop
void runGui(SDL_Window* window, SDL_GLContext gl_context, Vehicle* vehicle, bool enableFlightControl) {
    std::cout << "[GUI] Entering main GUI loop..." << std::endl;

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    bool guiKeepRunning = true;
    int functionTimeout = 1;

    char target_beacon_id_buffer[128];
    strncpy(target_beacon_id_buffer, TARGET_BEACON_ID.c_str(), sizeof(target_beacon_id_buffer) - 1);
    target_beacon_id_buffer[sizeof(target_beacon_id_buffer) - 1] = '\0';

    const float side_window_width = 400.0f;
    const float bottom_window_height = 240.0f;
    const ImVec4 COLOR_GREEN = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
    const ImVec4 COLOR_RED = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    const ImVec4 COLOR_ORANGE = ImVec4(1.0f, 0.65f, 0.0f, 1.0f);
    const ImVec4 COLOR_GREY = ImVec4(0.7f, 0.7f, 0.7f, 1.0f);


    while (guiKeepRunning) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) { guiKeepRunning = false; }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window)) { guiKeepRunning = false; }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        const float padding = 10.0f;
        ImVec2 work_pos = viewport->WorkPos;
        ImVec2 work_size = viewport->WorkSize;

        // Prepare Status Info (Radar, Drone Mode)
        const char* radarStatusText = "";
        ImVec4 radarStatusColor = COLOR_RED;
        BridgeConnectionStatus status = currentBridgeStatus.load(std::memory_order_relaxed);
        switch(status) { /* ... status text/color ... */
            case BridgeConnectionStatus::CONNECTED: radarStatusText = "Radar: Connected"; radarStatusColor = COLOR_GREEN; break;
            case BridgeConnectionStatus::DISCONNECTED: radarStatusText = "Radar: Disconnected"; radarStatusColor = COLOR_RED; break;
            case BridgeConnectionStatus::RECONNECTING: radarStatusText = "Radar: Attempting Reconnect..."; radarStatusColor = COLOR_ORANGE; break;
        }
        std::string droneModeText = "Mode: N/A";
        ImVec4 droneModeColor = COLOR_GREY;
        uint8_t mode_val = g_current_display_mode.load(std::memory_order_relaxed);
        if (connect_to_drone && vehicle != nullptr) {
             droneModeText = "Mode: " + getModeName(mode_val);
             if (mode_val == VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL) { droneModeColor = COLOR_GREEN; }
             else if (mode_val == 255) { droneModeColor = COLOR_GREY; }
             else { droneModeColor = COLOR_RED; }
        }


        // 1. Main Menu Window
        ImVec2 mainMenuPos = ImVec2(work_pos.x + padding, work_pos.y + padding);
        ImGui::SetNextWindowPos(mainMenuPos, ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(side_window_width, 0), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.65f);
        ImGuiWindowFlags menu_window_flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        ImGui::Begin("Main Menu", nullptr, menu_window_flags);
        {
            bool is_processing_active = processingThread.joinable();
            std::string script_to_use = useLocalBridgeScript.load() ? localPythonBridgeScript : defaultPythonBridgeScript; // Use load()
            if (is_processing_active) ImGui::BeginDisabled();
            auto startProcessing = [&](ProcessingMode mode, const char* logName, bool requireControl, bool requireTelemetry) {
                /* ... lambda implementation as before (calls runPythonBridge, starts thread) ... */
                std::cout << "[GUI] '" << logName << "' button clicked." << std::endl;
                stopProcessingThreadIfNeeded();
                forceStopReconnectionFlag.store(false);
                { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
                { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
                {
                    std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                    g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                    g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                    g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                    g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                }
                 if (mode == ProcessingMode::WALL_BEACON_VERTICAL || mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) { g_vertical_state.store(VerticalAlignState::ALIGN_HORIZONTAL); }
                 else { g_vertical_state.store(VerticalAlignState::IDLE); }
                runPythonBridge(script_to_use);
                bool canStartWithControl = enableFlightControl && vehicle != nullptr && vehicle->control != nullptr;
                if (requireTelemetry && (!vehicle || !vehicle->subscribe)) { canStartWithControl = false; std::cerr << "[GUI] Cannot start '" << logName << "': Telemetry subscription unavailable." << std::endl; }
                if (requireControl && canStartWithControl) {
                    std::cout << "[GUI] Attempting to obtain Control Authority for " << logName << "..." << std::endl;
                    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                    if (ACK::getError(ctrlAuthAck)) { ACK::getErrorCodeMessage(ctrlAuthAck, std::string("[GUI] " + std::string(logName) + " obtainCtrlAuthority").c_str()); std::cerr << "[GUI] Failed to obtain control authority. Cannot start with control." << std::endl; currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); stopPythonBridge(script_to_use); }
                    else { std::cout << "[GUI] Obtained Control Authority." << std::endl; stopProcessingFlag.store(false); processingThread = std::thread(processingLoopFunction, script_to_use, vehicle, true, mode); }
                } else if (requireControl && !canStartWithControl) { std::cerr << "[GUI] Cannot start '" << logName << "' with control: Flight control disabled or OSDK not ready." << std::endl; currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); stopPythonBridge(script_to_use); }
                else { std::cout << "[GUI] Starting '" << logName << "' (No Control)..." << std::endl; stopProcessingFlag.store(false); processingThread = std::thread(processingLoopFunction, script_to_use, vehicle, false, mode); }
            };
            /* ... Buttons calling startProcessing ... */
            if (!enableFlightControl) ImGui::BeginDisabled();
            if (ImGui::Button("Wall+Lateral Beacon", ImVec2(-1, 0))) { startProcessing(ProcessingMode::WALL_FOLLOW, "Wall+Lateral Beacon", true, false); }
            if (!enableFlightControl) ImGui::EndDisabled();
            if (!enableFlightControl) ImGui::BeginDisabled();
            if (ImGui::Button("Wall+Lateral Beacon+Vertical", ImVec2(-1, 0))) { startProcessing(ProcessingMode::WALL_BEACON_VERTICAL, "Wall+Lateral Beacon+Vertical", true, true); }
            if (!enableFlightControl) ImGui::EndDisabled();
            if (!enableFlightControl) ImGui::BeginDisabled();
            if (ImGui::Button("Wall+Lateral Beacon+Yaw", ImVec2(-1, 0))) { startProcessing(ProcessingMode::WALL_BEACON_YAW, "Wall+Lateral Beacon+Yaw", true, false); }
            if (!enableFlightControl) ImGui::EndDisabled();
            if (!enableFlightControl) ImGui::BeginDisabled();
            if (ImGui::Button("Wall+Lateral Beacon+Yaw+Vertical", ImVec2(-1, 0))) { startProcessing(ProcessingMode::WALL_BEACON_YAW_VERTICAL, "Wall+Lateral Beacon+Yaw+Vertical", true, true); }
             if (!enableFlightControl) { ImGui::EndDisabled(); ImGui::SameLine(); ImGui::TextDisabled("(Flight Control Disabled)"); }
             else { ImGui::NewLine(); }
            if (ImGui::Button("Process Full Radar [e]", ImVec2(-1, 0))) { startProcessing(ProcessingMode::PROCESS_FULL, "Process Full Radar", false, false); }
            if (is_processing_active) ImGui::EndDisabled();
            ImGui::Separator();
            /* ... Stop Buttons ... */
             bool should_enable_stop_reconnect = is_processing_active && enable_bridge_reconnection.load() && (currentBridgeStatus.load(std::memory_order_relaxed) == BridgeConnectionStatus::RECONNECTING); // Use load()
            if (!should_enable_stop_reconnect) ImGui::BeginDisabled();
            if (ImGui::Button("Stop Reconnection Attempt")) { std::cout << "[GUI] 'Stop Reconnection Attempt' button clicked." << std::endl; forceStopReconnectionFlag.store(true); }
            if (!should_enable_stop_reconnect) ImGui::EndDisabled();
            ImGui::SameLine();
            if (!is_processing_active) ImGui::BeginDisabled();
            if (ImGui::Button("Stop Current Action")) { std::cout << "[GUI] 'Stop Current Action' button clicked." << std::endl; stopProcessingThreadIfNeeded(); }
            if (!is_processing_active) ImGui::EndDisabled();
        }
        ImGui::End(); // End Main Menu

        // 2. Combined Status Window
        ImVec2 statusWindowPos; ImVec2 statusWindowSize;
        ImGuiWindowFlags status_window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
        {
            statusWindowPos = ImVec2(mainMenuPos.x + side_window_width + padding, mainMenuPos.y);
            ImGui::SetNextWindowPos(statusWindowPos, ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.65f);
            ImGui::Begin("CombinedStatusWindow", nullptr, status_window_flags);
            /* ... status text rendering ... */
            if (strlen(radarStatusText) > 0) { ImGui::TextColored(radarStatusColor, "%s", radarStatusText); } else { ImGui::Text("Radar Status Unknown"); }
            ImGui::TextColored(droneModeColor, "%s", droneModeText.c_str());
            VerticalAlignState current_v_state = g_vertical_state.load(std::memory_order_relaxed);
            if (current_v_state != VerticalAlignState::IDLE) { ImGui::Text("Vertical State: %s", getVerticalStateName(current_v_state).c_str()); }
            statusWindowSize = ImGui::GetWindowSize();
            ImGui::End();
        }

        // 3. Live Data Window
        ImGuiWindowFlags live_data_flags = status_window_flags;
        {
             ImVec2 liveDataPos = ImVec2(statusWindowPos.x, statusWindowPos.y + statusWindowSize.y + padding);
             ImGui::SetNextWindowPos(liveDataPos, ImGuiCond_Always);
             ImGui::SetNextWindowBgAlpha(0.65f);
             ImGui::Begin("LiveDataWindow", nullptr, live_data_flags);
             /* ... live data rendering ... */
             float latest_beacon_range_disp, latest_wall_dist_disp, latest_alt_disp; std::chrono::steady_clock::time_point last_beacon_time_disp;
             { std::lock_guard<std::mutex> lock(g_liveDataMutex); latest_beacon_range_disp = g_latest_beacon_range; latest_wall_dist_disp = g_latest_wall_horizontal_distance; latest_alt_disp = g_latest_altitude; last_beacon_time_disp = g_last_beacon_seen_time; }
             std::string time_since_beacon_str = "N/A";
             if (last_beacon_time_disp != std::chrono::steady_clock::time_point::min()) { auto time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - last_beacon_time_disp); std::stringstream ss_timediff; ss_timediff << std::fixed << std::setprecision(1) << time_diff.count() << " s"; time_since_beacon_str = ss_timediff.str(); }
             ImGui::Text("Time since Beacon: %s", time_since_beacon_str.c_str());
             if (std::isnan(latest_beacon_range_disp)) ImGui::Text("Dist to Beacon: N/A"); else ImGui::Text("Dist to Beacon: %.3f m", latest_beacon_range_disp);
             if (std::isnan(latest_wall_dist_disp)) ImGui::Text("Dist to Wall HZ: N/A"); else ImGui::Text("Dist to Wall HZ: %.3f m", latest_wall_dist_disp);
             if (std::isnan(latest_alt_disp)) ImGui::Text("Altitude: N/A"); else ImGui::Text("Altitude: %.3f m", latest_alt_disp);
             ImGui::End();
        }

        // --- 4. Camera Status Window ---
        ImVec2 cameraPos = ImVec2(statusWindowPos.x, work_pos.y + work_size.y - padding); // Bottom center-right area
        ImVec2 cameraSize = ImVec2(work_size.x - side_window_width - (padding * 3), 0); // Auto height, fill available width
        ImGui::SetNextWindowPos(cameraPos, ImGuiCond_Always, ImVec2(0.0f, 1.0f)); // Pivot bottom-left
        ImGui::SetNextWindowSizeConstraints(ImVec2(200, 50), ImVec2(FLT_MAX, 100)); // Min size, limit max height
        ImGui::SetNextWindowBgAlpha(0.65f);
        ImGui::Begin("Camera Status", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize); // Auto-resize vertically
        {
            // --- Check config flag first ---
            if (!enable_camera_stream) {
                 ImGui::TextColored(COLOR_GREY, "Camera Stream: DISABLED (Config)");
            }
            // --- If enabled in config, show actual status ---
            else {
                bool is_receiving = g_camera_data_receiving.load(); // Check if callback is firing
                std::chrono::steady_clock::time_point last_rx_time;
                last_rx_time = g_last_camera_data_time; // Read timestamp

                if (is_receiving) {
                    ImGui::TextColored(COLOR_GREEN, "Camera Stream: Receiving Data");
                    auto now = std::chrono::steady_clock::now();
                    // Check if timestamp is valid before calculating diff
                    if (last_rx_time != std::chrono::steady_clock::time_point::min()) {
                        auto time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_rx_time);
                        ImGui::Text("Last data received: %.1f s ago", time_diff.count());
                    } else {
                        ImGui::Text("Last data received: Just now / Initializing...");
                    }
                    ImGui::Text("Note: Live view requires H.264/H.265 decoder.");
                } else if (connect_to_drone && vehicle != nullptr && vehicle->advancedSensing != nullptr) { // Check if advancedSensing *should* be available
                    ImGui::TextColored(COLOR_ORANGE, "Camera Stream: No Data Received");
                    ImGui::Text("Check drone connection and camera status.");
                    ImGui::Text("Ensure stream started successfully.");
                } else if (connect_to_drone && vehicle != nullptr) { // OSDK connected, but advancedSensing missing
                     ImGui::TextColored(COLOR_RED, "Camera Stream: Unavailable");
                     ImGui::Text("(AdvancedSensing interface failed to init)");
                } else { // OSDK not connected
                    ImGui::TextColored(COLOR_GREY, "Camera Stream: Unavailable");
                    ImGui::Text("(Connect to drone first)");
                }
            }
        }
        ImGui::End(); // End Camera Status Window


        // 5. Parameters Window (Top Right)
        ImVec2 paramsPos = ImVec2(work_pos.x + work_size.x - padding, work_pos.y + padding);
        ImGui::SetNextWindowPos(paramsPos, ImGuiCond_Always, ImVec2(1.0f, 0.0f)); // Pin to top-right
        ImGui::SetNextWindowSize(ImVec2(660, 0), ImGuiCond_Appearing); // Initial width, auto height
        ImGui::SetNextWindowBgAlpha(0.65f);
        ImGuiWindowFlags params_window_flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoCollapse; // Allow resize/move
        ImGui::Begin("Parameters", nullptr, params_window_flags);
        {
            // --- Status Section ---
            ImGui::Text("Status"); ImGui::Separator();
            ImGui::Text("Connect to Drone: %s", connect_to_drone ? "True" : "False");
            ImGui::Text("Flight Control: %s", enableFlightControl ? "Enabled" : "Disabled");

            // --- FIX Checkbox for std::atomic<bool> ---
            bool temp_reconnect = enable_bridge_reconnection.load();
            if (ImGui::Checkbox("Bridge Reconnect", &temp_reconnect)) {
                 enable_bridge_reconnection.store(temp_reconnect);
                 std::cout << "[GUI Param Update] Bridge Reconnect set to: " << std::boolalpha << temp_reconnect << std::endl;
            }
            ImGui::SameLine(0, 20);
            bool temp_local_script = useLocalBridgeScript.load();
            if (ImGui::Checkbox("Local Test Script", &temp_local_script)) {
                 useLocalBridgeScript.store(temp_local_script);
                 std::cout << "[GUI Param Update] Local Test Script set to: " << std::boolalpha << temp_local_script << std::endl;
            }
            ImGui::Separator();
            // --- END FIX Checkbox ---


            float availableWidth = ImGui::GetContentRegionAvail().x; float columnWidth = (availableWidth - ImGui::GetStyle().ItemSpacing.x) * 0.5f; float inputWidth = columnWidth * 0.45f;
            ImGui::BeginGroup(); ImGui::PushID("LeftColumn");
            ImGui::Text("Sensor Control (Wall Dist)"); ImGui::Separator();

            // --- FIX Checkbox for std::atomic<bool> ---
            bool temp_useWallSensorRAz = useWallSensorRAz.load();
            if (ImGui::Checkbox("R_Az##W", &temp_useWallSensorRAz)) { useWallSensorRAz.store(temp_useWallSensorRAz); std::cout << "[GUI Param Update] Use Wall R_Az set to: " << std::boolalpha << temp_useWallSensorRAz << std::endl; }
            ImGui::SameLine();
            bool temp_useWallSensorREl = useWallSensorREl.load();
            if (ImGui::Checkbox("R_El##W", &temp_useWallSensorREl)) { useWallSensorREl.store(temp_useWallSensorREl); std::cout << "[GUI Param Update] Use Wall R_El set to: " << std::boolalpha << temp_useWallSensorREl << std::endl; }
            ImGui::SameLine();
            bool temp_useWallSensorRAzREl = useWallSensorRAzREl.load();
            if (ImGui::Checkbox("R_Az_R_El##W", &temp_useWallSensorRAzREl)) { useWallSensorRAzREl.store(temp_useWallSensorRAzREl); std::cout << "[GUI Param Update] Use Wall R_Az_R_El set to: " << std::boolalpha << temp_useWallSensorRAzREl << std::endl; }
            ImGui::Spacing();

            ImGui::Text("Sensor Control (Beacon Az)"); ImGui::Separator();
            bool temp_useBeaconSensorRAz = useBeaconSensorRAz.load();
            if (ImGui::Checkbox("R_Az##BA", &temp_useBeaconSensorRAz)) { useBeaconSensorRAz.store(temp_useBeaconSensorRAz); std::cout << "[GUI Param Update] Use Beacon Az R_Az set to: " << std::boolalpha << temp_useBeaconSensorRAz << std::endl; }
            ImGui::SameLine();
            bool temp_useBeaconSensorREl = useBeaconSensorREl.load();
            if (ImGui::Checkbox("R_El##BA", &temp_useBeaconSensorREl)) { useBeaconSensorREl.store(temp_useBeaconSensorREl); std::cout << "[GUI Param Update] Use Beacon Az R_El set to: " << std::boolalpha << temp_useBeaconSensorREl << std::endl; }
            ImGui::SameLine();
            bool temp_useBeaconSensorRAzREl = useBeaconSensorRAzREl.load();
            if (ImGui::Checkbox("R_Az_R_El##BA", &temp_useBeaconSensorRAzREl)) { useBeaconSensorRAzREl.store(temp_useBeaconSensorRAzREl); std::cout << "[GUI Param Update] Use Beacon Az R_Az_R_El set to: " << std::boolalpha << temp_useBeaconSensorRAzREl << std::endl; }
            ImGui::Spacing();

            ImGui::Text("Sensor Control (Beacon Rg)"); ImGui::Separator();
             bool temp_useBeaconRangeSensorRAz = useBeaconRangeSensorRAz.load();
            if (ImGui::Checkbox("R_Az##BR", &temp_useBeaconRangeSensorRAz)) { useBeaconRangeSensorRAz.store(temp_useBeaconRangeSensorRAz); std::cout << "[GUI Param Update] Use Beacon Rg R_Az set to: " << std::boolalpha << temp_useBeaconRangeSensorRAz << std::endl; }
            ImGui::SameLine();
            bool temp_useBeaconRangeSensorREl = useBeaconRangeSensorREl.load();
            if (ImGui::Checkbox("R_El##BR", &temp_useBeaconRangeSensorREl)) { useBeaconRangeSensorREl.store(temp_useBeaconRangeSensorREl); std::cout << "[GUI Param Update] Use Beacon Rg R_El set to: " << std::boolalpha << temp_useBeaconRangeSensorREl << std::endl; }
            ImGui::SameLine();
            bool temp_useBeaconRangeSensorRAzREl = useBeaconRangeSensorRAzREl.load();
            if (ImGui::Checkbox("R_Az_R_El##BR", &temp_useBeaconRangeSensorRAzREl)) { useBeaconRangeSensorRAzREl.store(temp_useBeaconRangeSensorRAzREl); std::cout << "[GUI Param Update] Use Beacon Rg R_Az_R_El set to: " << std::boolalpha << temp_useBeaconRangeSensorRAzREl << std::endl; }
            ImGui::Spacing();

            ImGui::Text("Sensor Control (Yaw)"); ImGui::Separator();
            bool temp_useYawSensorRAz = useYawSensorRAz.load();
            if (ImGui::Checkbox("R_Az##Y", &temp_useYawSensorRAz)) { useYawSensorRAz.store(temp_useYawSensorRAz); std::cout << "[GUI Param Update] Use Yaw R_Az set to: " << std::boolalpha << temp_useYawSensorRAz << std::endl; }
            ImGui::SameLine();
            bool temp_useYawSensorREl = useYawSensorREl.load();
            if (ImGui::Checkbox("R_El##Y", &temp_useYawSensorREl)) { useYawSensorREl.store(temp_useYawSensorREl); std::cout << "[GUI Param Update] Use Yaw R_El set to: " << std::boolalpha << temp_useYawSensorREl << std::endl; }
            ImGui::SameLine();
            bool temp_useYawSensorRAzREl = useYawSensorRAzREl.load();
            if (ImGui::Checkbox("R_Az_R_El##Y", &temp_useYawSensorRAzREl)) { useYawSensorRAzREl.store(temp_useYawSensorRAzREl); std::cout << "[GUI Param Update] Use Yaw R_Az_R_El set to: " << std::boolalpha << temp_useYawSensorRAzREl << std::endl; }
            ImGui::Spacing();
            // --- END FIX Checkbox ---

            ImGui::Text("Editable Parameters"); ImGui::Separator(); ImGui::PushItemWidth(inputWidth); auto InputTextWithLabel = [&](const char* label, const char* hint, char* buf, size_t buf_size, ImGuiInputTextFlags flags = 0) { bool changed = ImGui::InputText(hint, buf, buf_size, flags); ImGui::SameLine(); ImGui::TextUnformatted(label); return changed; }; auto InputFloatWithLabel = [&](const char* label, const char* hint, float* v, float step = 0.0f, float step_fast = 0.0f, const char* format = "%.3f", ImGuiInputTextFlags flags = 0) { bool changed = ImGui::InputFloat(hint, v, step, step_fast, format, flags); ImGui::SameLine(); ImGui::TextUnformatted(label); return changed; };
            if (InputTextWithLabel("Target Beacon ID", "##TgtBeaconID", target_beacon_id_buffer, sizeof(target_beacon_id_buffer))) { TARGET_BEACON_ID = target_beacon_id_buffer; std::cout << "[GUI Param Update] Target Beacon ID set to: " << TARGET_BEACON_ID << std::endl; } if (InputFloatWithLabel("Tgt Horiz Dist (m)", "##TgtHorizDist", &targetDistance, 0.1f, 1.0f, "%.2f")) { std::cout << "[GUI Param Update] Target Horiz Distance set to: " << targetDistance << std::endl; } if (InputFloatWithLabel("Tgt Azimuth (deg)", "##TgtAzimuth", &targetAzimuth, 1.0f, 10.0f, "%.2f")) { std::cout << "[GUI Param Update] Target Azimuth set to: " << targetAzimuth << std::endl; } if (InputFloatWithLabel("Radar Mount Ang (deg)", "##RadarMountAng", &radarMountAngleDegrees, 1.0f, 5.0f, "%.1f")) { std::cout << "[GUI Param Update] Radar Mount Angle set to: " << radarMountAngleDegrees << std::endl; } ImGui::PopItemWidth();
            ImGui::PopID(); ImGui::EndGroup();
            ImGui::SameLine(); ImGui::BeginGroup(); ImGui::PushID("RightColumn");
            ImGui::Text("Forward Control"); ImGui::Separator(); ImGui::PushItemWidth(inputWidth); if (InputFloatWithLabel("Kp Forward", "##KpFwd", &Kp_forward, 0.01f, 0.1f, "%.3f")) std::cout << "[GUI Param Update] Kp Forward set to: " << Kp_forward << std::endl; if (InputFloatWithLabel("Max Fwd Spd (m/s)", "##MaxFwdSpd", &max_forward_speed, 0.05f, 0.2f, "%.2f")) std::cout << "[GUI Param Update] Max Fwd Speed set to: " << max_forward_speed << std::endl; if (InputFloatWithLabel("Fwd Dead Zone (m)", "##FwdDeadZone", &forward_dead_zone, 0.01f, 0.1f, "%.2f")) std::cout << "[GUI Param Update] Fwd Dead Zone set to: " << forward_dead_zone << std::endl; ImGui::PopItemWidth(); ImGui::Spacing();
            ImGui::Text("Lateral Control"); ImGui::Separator(); ImGui::PushItemWidth(inputWidth); if (InputFloatWithLabel("Kp Lateral", "##KpLat", &Kp_lateral, 0.001f, 0.01f, "%.4f")) std::cout << "[GUI Param Update] Kp Lateral set to: " << Kp_lateral << std::endl; if (InputFloatWithLabel("Max Lat Spd (m/s)", "##MaxLatSpd", &max_lateral_speed, 0.05f, 0.2f, "%.2f")) std::cout << "[GUI Param Update] Max Lat Speed set to: " << max_lateral_speed << std::endl; if (InputFloatWithLabel("Az Dead Zone (deg)", "##AzDeadZone", &azimuth_dead_zone, 0.1f, 1.0f, "%.2f")) std::cout << "[GUI Param Update] Azimuth Dead Zone set to: " << azimuth_dead_zone << std::endl; ImGui::PopItemWidth();

            // --- FIX Checkbox for std::atomic<bool> ---
            bool temp_invertLateral = invertLateralControl.load();
            if (ImGui::Checkbox("Invert Lateral", &temp_invertLateral)) { invertLateralControl.store(temp_invertLateral); std::cout << "[GUI Param Update] Invert Lateral Control set to: " << std::boolalpha << temp_invertLateral << std::endl; }
            ImGui::Spacing();

            ImGui::Text("Yaw Control"); ImGui::Separator(); ImGui::PushItemWidth(inputWidth); if (InputFloatWithLabel("Kp Yaw", "##KpYaw", &Kp_yaw, 0.001f, 0.01f, "%.4f")) std::cout << "[GUI Param Update] Kp Yaw set to: " << Kp_yaw << std::endl; if (InputFloatWithLabel("Max Yaw Rate (deg/s)", "##MaxYawRate", &max_yaw_rate, 0.5f, 2.0f, "%.1f")) std::cout << "[GUI Param Update] Max Yaw Rate set to: " << max_yaw_rate << std::endl; if (InputFloatWithLabel("Yaw Bal Dead Zone (deg)", "##YawBalDeadZone", &yaw_azimuth_balance_dead_zone, 0.1f, 0.5f, "%.1f")) std::cout << "[GUI Param Update] Yaw Balance Dead Zone set to: " << yaw_azimuth_balance_dead_zone << std::endl; ImGui::PopItemWidth();

            bool temp_invertYaw = invertYawControl.load();
            if (ImGui::Checkbox("Invert Yaw", &temp_invertYaw)) { invertYawControl.store(temp_invertYaw); std::cout << "[GUI Param Update] Invert Yaw Control set to: " << std::boolalpha << temp_invertYaw << std::endl; }
            ImGui::Spacing();
             // --- END FIX Checkbox ---

            ImGui::Text("Vertical Control"); ImGui::Separator(); ImGui::PushItemWidth(inputWidth); if (InputFloatWithLabel("Target Alt (m AGL)", "##TgtAlt", &TARGET_ALTITUDE, 0.1f, 0.5f, "%.2f")) std::cout << "[GUI Param Update] Target Altitude set to: " << TARGET_ALTITUDE << std::endl; if (InputFloatWithLabel("Vertical Spd (m/s)", "##VertSpd", &VERTICAL_SPEED, 0.05f, 0.2f, "%.2f")) std::cout << "[GUI Param Update] Vertical Speed set to: " << VERTICAL_SPEED << std::endl; if (InputFloatWithLabel("Hold Duration (s)", "##HoldDur", &HOLD_DURATION_SECONDS, 0.5f, 1.0f, "%.1f")) std::cout << "[GUI Param Update] Hold Duration set to: " << HOLD_DURATION_SECONDS << std::endl; ImGui::PopItemWidth();
            ImGui::PopID(); ImGui::EndGroup();
        }
        ImGui::End(); // End Parameters Window


        // 6. Anon Detections Window
        ImVec2 anonPos = ImVec2(work_pos.x + padding, work_pos.y + work_size.y - padding - bottom_window_height - padding);
        ImGui::SetNextWindowPos(anonPos, ImGuiCond_Always, ImVec2(0.0f, 1.0f));
        ImGui::SetNextWindowSize(ImVec2(side_window_width, bottom_window_height), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.65f);
        ImGuiWindowFlags detect_window_flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        ImGui::Begin("Anon Detections", nullptr, detect_window_flags);
        {
            ImGui::Text("Latest Anonymous Objects (ID contains 'anon')"); ImGui::Separator();
            if (ImGui::BeginChild("AnonScrollRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar)) {
                std::lock_guard<std::mutex> lock(g_anonMutex);
                if (g_anonObjects.empty()) { ImGui::TextUnformatted("No anon data received yet or action stopped."); }
                else {
                    for (size_t i = 0; i < g_anonObjects.size(); ++i) {
                        const auto& obj = g_anonObjects[i];
                        /* ... Timestamp Formatting ... */
                        std::string readable_ts = "Invalid Time"; try { double unix_ts_double = std::stod(obj.timestamp); double integral_part; double fractional_part = std::modf(unix_ts_double, &integral_part); std::time_t time_sec = static_cast<std::time_t>(integral_part); int milliseconds = static_cast<int>(fractional_part * 1000.0); std::tm* ptm = std::gmtime(&time_sec); if (ptm) { std::stringstream ss; ss << std::put_time(ptm, "%H:%M:%S"); ss << "." << std::setfill('0') << std::setw(3) << milliseconds; readable_ts = ss.str(); } } catch (const std::exception& e) { readable_ts = obj.timestamp; }
                        ImGui::Text("Anon #%zu:", i + 1); ImGui::Indent(); ImGui::Text("Time: %s", readable_ts.c_str()); ImGui::Text("Sensor: %s, Src: %s, ID: %s", obj.sensor.c_str(), obj.src.c_str(), obj.ID.c_str()); ImGui::Text("Pos(XYZ): %.3f, %.3f, %.3f", obj.X, obj.Y, obj.Z); ImGui::Text("Rng: %.3f, Rate: %.3f, Pwr: %.1f", obj.Range, obj.RangeRate, obj.Pwr); ImGui::Text("Az: %.2f, El: %.2f, Conf: %.2f", obj.Az, obj.El, obj.Conf); ImGui::Unindent(); ImGui::Separator();
                    }
                }
            }
            ImGui::EndChild();
        }
        ImGui::End(); // End Anon Detections

        // 7. Beacon Detections Window
        ImVec2 beaconPos = ImVec2(work_pos.x + padding, work_pos.y + work_size.y - padding);
        ImGui::SetNextWindowPos(beaconPos, ImGuiCond_Always, ImVec2(0.0f, 1.0f));
        ImGui::SetNextWindowSize(ImVec2(side_window_width, bottom_window_height), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.65f);
        ImGui::Begin("Beacon Detections", nullptr, detect_window_flags);
        {
            ImGui::Text("Latest Beacon Objects (ID contains 'BEACON')"); ImGui::Separator();
            if (ImGui::BeginChild("BeaconScrollRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar)) {
                std::lock_guard<std::mutex> lock(g_beaconMutex);
                if (g_beaconObjects.empty()) { ImGui::TextUnformatted("No beacon data received yet or action stopped."); }
                else {
                     for (size_t i = 0; i < g_beaconObjects.size(); ++i) {
                        const auto& obj = g_beaconObjects[i];
                        /* ... Timestamp Formatting ... */
                        std::string readable_ts = "Invalid Time"; try { double unix_ts_double = std::stod(obj.timestamp); double integral_part; double fractional_part = std::modf(unix_ts_double, &integral_part); std::time_t time_sec = static_cast<std::time_t>(integral_part); int milliseconds = static_cast<int>(fractional_part * 1000.0); std::tm* ptm = std::gmtime(&time_sec); if (ptm) { std::stringstream ss; ss << std::put_time(ptm, "%H:%M:%S"); ss << "." << std::setfill('0') << std::setw(3) << milliseconds; readable_ts = ss.str(); } } catch (const std::exception& e) { readable_ts = obj.timestamp; }
                        ImGui::Text("Beacon #%zu:", i + 1); ImGui::Indent(); ImGui::Text("Time: %s", readable_ts.c_str()); ImGui::Text("Sensor: %s, Src: %s, ID: %s", obj.sensor.c_str(), obj.src.c_str(), obj.ID.c_str()); ImGui::Text("Pos(XYZ): %.3f, %.3f, %.3f", obj.X, obj.Y, obj.Z); ImGui::Text("Rng: %.3f, Rate: %.3f, Pwr: %.1f", obj.Range, obj.RangeRate, obj.Pwr); ImGui::Text("Az: %.2f, El: %.2f, Conf: %.2f", obj.Az, obj.El, obj.Conf); ImGui::Unindent(); ImGui::Separator();
                    }
                }
            }
             ImGui::EndChild();
        }
        ImGui::End(); // End Beacon Detections


        // --- Rendering ---
        ImGui::Render();
        int display_w, display_h;
        SDL_GetWindowSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);

    } // --- End Main GUI Loop ---

    std::cout << "[GUI] Exited main GUI loop." << std::endl;
}

// Cleans up and shuts down ImGui, SDL, and GLEW
void cleanupGui(SDL_Window* window, SDL_GLContext gl_context) {
    std::cout << "[GUI] Cleaning up ImGui..." << std::endl;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    std::cout << "[GUI] Cleaning up SDL..." << std::endl;
    if (gl_context) { SDL_GL_DeleteContext(gl_context); }
    if (window) { SDL_DestroyWindow(window); }
    SDL_Quit();
    std::cout << "[GUI] Cleanup complete." << std::endl;
}