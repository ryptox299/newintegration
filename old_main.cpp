#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono> // Required for sleep_for, steady_clock
#include <cstdlib>  // For std::system
#include <json.hpp> // Include the JSON library
#include <boost/asio.hpp>    // Include Boost ASIO for TCP communication
#include "dji_linux_helpers.hpp"
#include <limits> // For numeric limits
#include <fstream> // For file reading AND LOGGING
#include <cmath> // For std::abs, std::isnan, std::cos, std::fabs, M_PI
#include <atomic> // For thread-safe stop flag AND GLOBAL DISPLAY MODE and Vertical State
#include <cstring> // For strchr, strncpy
#include <stdexcept> // For standard exceptions
#include <array> // For std::array used in read_some buffer
#include <ctime>  // For checking polling timestamp
#include <streambuf> // For TeeBuf
#include <mutex>     // For TeeBuf thread safety AND GLOBAL DATA AND LIVE DATA
#include <memory>    // For unique_ptr
#include <iomanip>   // For std::put_time in timestamp, std::setw, std::left, std::fixed, std::setprecision
#include <algorithm> // For std::transform

// Include the headers that define Control flags, CtrlData, FlightController, and Vehicle
#include "dji_control.hpp"           // Defines Control class, CtrlData, enums
#include "dji_flight_controller.hpp" // Defines FlightController
#include "dji_vehicle.hpp"           // Defines Vehicle class which contains Control*
#include "dji_telemetry.hpp"         // For Telemetry types
#include "dji_status.hpp"            // For VehicleStatus enums/constants
#include "dji_ack.hpp"               // For ACK::getError

// --- GUI Includes (Correct Order) ---
// 1. GLEW must come first
// #define GLEW_STATIC // Define this if linking GLEW statically, remove/comment if dynamic
#include <GL/glew.h>

// 2. SDL
#include <SDL.h>

// 3. SDL OpenGL headers (AFTER GLEW)
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

// 4. ImGui
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
// Add internal header for font size access if needed, though GetFontSize should work
#include "imgui_internal.h"


// Define M_PI if not already defined (likely in cmath)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace DJI::OSDK;
using json = nlohmann::json;
using boost::asio::ip::tcp;


// --- Radar Object Definition ---
struct RadarObject {
    std::string timestamp;
    std::string sensor;
    std::string src;
    float X, Y, Z;
    float Xdir, Ydir, Zdir;
    float Range, RangeRate, Pwr, Az, El;
    std::string ID;
    float Xsize, Ysize, Zsize;
    float Conf;
};
// --- End Radar Object Definition ---


// --- Global Data for GUI ---
std::vector<RadarObject> g_beaconObjects;
std::mutex               g_beaconMutex;
std::vector<RadarObject> g_anonObjects;
std::mutex               g_anonMutex;
std::atomic<uint8_t>     g_current_display_mode(255); // Global atomic for display mode (255 = Unknown/Unavailable)
// --- End Global Data ---

// --- Global Live Data for GUI ---
std::mutex g_liveDataMutex;
std::chrono::steady_clock::time_point g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min(); // Initialize to indicate never seen
float g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
float g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
float g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
// --- End Global Live Data ---


// --- Logging Setup ---
// (TeeBuf and LogRedirector classes remain exactly the same as before)
// TeeBuf writes output to two streambufs (e.g., console and file)
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* sb1, std::streambuf* sb2) : sb1_(sb1), sb2_(sb2) {}

protected:
    // Called when buffer is full or on explicit flush/endl
    virtual int sync() override {
        std::lock_guard<std::mutex> lock(mutex_);
        int r1 = sb1_->pubsync();
        int r2 = sb2_->pubsync();
        return (r1 == 0 && r2 == 0) ? 0 : -1;
    }

    // Called when a character is written
    virtual int_type overflow(int_type c = traits_type::eof()) override {
        if (traits_type::eq_int_type(c, traits_type::eof())) {
            return sync() == -1 ? traits_type::eof() : traits_type::not_eof(c);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        int_type const r1 = sb1_->sputc(c);
        int_type const r2 = sb2_->sputc(c);

        if (traits_type::eq_int_type(r1, traits_type::eof()) ||
            traits_type::eq_int_type(r2, traits_type::eof())) {
            return traits_type::eof(); // Indicate error if either fails
        }
        return traits_type::not_eof(c); // Indicate success
    }

private:
    std::streambuf* sb1_;
    std::streambuf* sb2_;
    std::mutex mutex_; // Protect concurrent writes from different threads
};

// RAII class to manage redirection and restoration of streams
class LogRedirector {
public:
    LogRedirector(const std::string& log_filename)
        : log_file_(log_filename, std::ios::app), // Open in append mode
          original_cout_buf_(nullptr),
          original_cerr_buf_(nullptr)
    {
        if (!log_file_.is_open()) {
            // Use original cerr because redirection hasn't happened yet
            if (original_cerr_buf_) std::cerr.rdbuf(original_cerr_buf_);
            std::cerr << "FATAL ERROR: Could not open log file: " << log_filename << std::endl;
             // Restore original cerr buffer in case it was temporarily changed above
            if (original_cerr_buf_) std::cerr.rdbuf(original_cerr_buf_);
            // Log file couldn't be opened, so don't redirect
            return;
        }

        original_cout_buf_ = std::cout.rdbuf(); // Save original cout buffer
        original_cerr_buf_ = std::cerr.rdbuf(); // Save original cerr buffer

        // Use reset(new ...) instead of std::make_unique for C++11 compatibility
        cout_tee_buf_.reset(new TeeBuf(original_cout_buf_, log_file_.rdbuf()));
        cerr_tee_buf_.reset(new TeeBuf(original_cerr_buf_, log_file_.rdbuf())); // Also log cerr to the same file


        std::cout.rdbuf(cout_tee_buf_.get()); // Redirect cout
        std::cerr.rdbuf(cerr_tee_buf_.get()); // Redirect cerr

        std::cout << "\n--- Log Start [" << getCurrentTimestamp() << "] ---" << std::endl; // Added newline for separation
    }

    ~LogRedirector() {
         std::cout << "--- Log End [" << getCurrentTimestamp() << "] ---\n" << std::endl; // Added newline for separation

        // Flush streams before restoring
        std::cout.flush();
        std::cerr.flush();

        // Restore original buffers only if redirection actually happened
        if (cout_tee_buf_ && original_cout_buf_) { // Check if unique_ptr holds a buffer
            std::cout.rdbuf(original_cout_buf_);
        }
        if (cerr_tee_buf_ && original_cerr_buf_) { // Check if unique_ptr holds a buffer
            std::cerr.rdbuf(original_cerr_buf_);
        }

        // log_file_ is closed automatically by its destructor
        // unique_ptrs clean up TeeBuf instances automatically
    }

    // Disable copy/move semantics
    LogRedirector(const LogRedirector&) = delete;
    LogRedirector& operator=(const LogRedirector&) = delete;
    LogRedirector(LogRedirector&&) = delete;
    LogRedirector& operator=(LogRedirector&&) = delete;

private:
    std::ofstream log_file_;
    std::streambuf* original_cout_buf_;
    std::streambuf* original_cerr_buf_;
    std::unique_ptr<TeeBuf> cout_tee_buf_;
    std::unique_ptr<TeeBuf> cerr_tee_buf_;

    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        #ifdef _MSC_VER // Use secure version on Windows if available
        struct tm buf;
        localtime_s(&buf, &now_c);
        ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S");
        #else
        // Use thread-safe version if available (POSIX standard)
        struct tm buf;
        localtime_r(&now_c, &buf); // Use localtime_r for thread safety
        ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S");
        #endif
        return ss.str();
    }
};
// --- End Logging Setup ---


// --- Configurable Parameters (with defaults relevant to W and E) ---
std::string TARGET_BEACON_ID = "BEACON-TX-ID:00005555"; // Still used for control logic target
float targetDistance = 8.0f;        // Target HORIZONTAL distance from the wall (meters)
float targetAzimuth = 0.0f;         // *Initial* target azimuth relative to the beacon (degrees)
float radarMountAngleDegrees = 45.0f; // Radar mounting angle (degrees, relative to horizontal)
// Forward Control (X-velocity)
float Kp_forward = 0.5;             // Proportional gain for forward movement
float max_forward_speed = 0.8;      // Max speed towards/away from the wall
float forward_dead_zone = 0.2;      // Dead zone for forward movement (meters)
// Lateral Control (Y-velocity)
float Kp_lateral = 0.02;            // Proportional gain for lateral movement
float max_lateral_speed = 0.5;      // Max speed sideways
float azimuth_dead_zone = 1.5;      // Dead zone for *initial* lateral movement (degrees)
bool invertLateralControl = false;  // Flag to invert lateral control based on beacon azimuth
// Yaw Control
float Kp_yaw = 0.01;                // Proportional gain for yaw rate based on azimuth balance
float max_yaw_rate = 10.0;          // Max yaw rate (degrees/second)
float yaw_azimuth_balance_dead_zone = 1.0; // Dead zone for yaw control based on sum of avg pos/neg azimuth (degrees)
bool invertYawControl = false;      // Flag to invert yaw control direction
// Vertical Control (NEW)
float TARGET_ALTITUDE = 2.0f;       // Target altitude for descent phase (meters AGL)
float VERTICAL_SPEED = 0.5f;        // Ascent/Descent speed (m/s)
float HOLD_DURATION_SECONDS = 3.0f; // Duration to hold position
// Sensor Selection (Independent Sets)
// For Wall Distance Calculation
bool useWallSensorRAz = true;       // Use "R_Az" sensor data for wall distance
bool useWallSensorREl = true;       // Use "R_El" sensor data for wall distance
bool useWallSensorRAzREl = true;    // Use "R_Az_R_El" sensor data for wall distance
// For Beacon Azimuth Detection
bool useBeaconSensorRAz = true;     // Use "R_Az" sensor data for beacon azimuth
bool useBeaconSensorREl = true;     // Use "R_El" sensor data for beacon azimuth
bool useBeaconSensorRAzREl = true;  // Use "R_Az_R_El" sensor data for beacon azimuth
// For Beacon Range Detection (NEW)
bool useBeaconRangeSensorRAz = true;    // Use "R_Az" sensor data for beacon range
bool useBeaconRangeSensorREl = true;    // Use "R_El" sensor data for beacon range
bool useBeaconRangeSensorRAzREl = true; // Use "R_Az_R_El" sensor data for beacon range
// For Yaw Control Calculation
bool useYawSensorRAz = true;        // Use "R_Az" sensor data for yaw control
bool useYawSensorREl = true;        // Use "R_El" sensor data for yaw control
bool useYawSensorRAzREl = true;     // Use "R_Az_R_El" sensor data for yaw control
// Bridge Reconnection Parameter
bool enable_bridge_reconnection = false; // Default to false
// Local Test Script Parameter (NEW)
bool useLocalBridgeScript = false; // Default to false
// NEW PARAMETER for Drone Connection
bool connect_to_drone = true;       // Default to true (attempt connection)
// --- End Configurable Parameters ---


// Persistent variables for tracking the current second and wall/beacon candidate data (relevant to W and E)
std::string currentSecond = "";
// General Wall/Beacon Data (Used by wall+beacon modes)
float lowestRange = std::numeric_limits<float>::max(); // Closest wall candidate DIRECT range (from selected wall sensors)
bool hasAnonData = false;                              // Flag if any *selected wall* anon data seen this second
float targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN(); // Sensed beacon azimuth (from selected beacon AZIMUTH sensors)
float targetBeaconRange = std::numeric_limits<float>::quiet_NaN();   // Sensed beacon DIRECT range (from selected beacon RANGE sensors) - NEW
bool foundTargetBeacon = false;                        // Flag if target beacon AZIMUTH seen this second (from selected beacon AZIMUTH sensors)
// Data for Yaw Control (Used by wall+beacon+yaw mode)
float sumPositiveAz = 0.0f;
int countPositiveAz = 0;
float sumNegativeAz = 0.0f;
int countNegativeAz = 0;
bool hasYawAnonData = false;                           // Flag if any *selected yaw* anon data seen this second


// Global variable for default Python bridge script (Hardcoded)
std::string defaultPythonBridgeScript = "python_bridge.py";
std::string localPythonBridgeScript = "python_bridge_LOCAL.py"; // NEW Local script name

// Flags to control the processing loop and reconnection
std::atomic<bool> stopProcessingFlag(false);
std::atomic<bool> forceStopReconnectionFlag(false); // Flag to manually stop reconnect attempts
std::thread processingThread;

// Monitoring Thread Globals
std::atomic<bool> stopMonitoringFlag(false);
std::thread monitoringThread;
const int TELEMETRY_TIMEOUT_SECONDS = 5; // Timeout for telemetry data
const int RECONNECT_DELAY_SECONDS = 5; // Delay between reconnection attempts (used in processing loop)

// --- NEW: Radar Bridge Connection Status ---
enum class BridgeConnectionStatus {
    DISCONNECTED,
    CONNECTED,
    RECONNECTING
};
std::atomic<BridgeConnectionStatus> currentBridgeStatus(BridgeConnectionStatus::DISCONNECTED); // Global atomic variable
// --- End NEW ---

// --- NEW: Vertical Alignment State ---
enum class VerticalAlignState {
    IDLE,               // Not active
    ALIGN_HORIZONTAL,   // Initial horizontal/yaw alignment
    ASCEND_TO_RANGE,    // Ascending to meet beacon range requirement
    HOLDING,            // Holding position for duration
    DESCEND_TO_ALT,     // Descending to target altitude
    ASCEND_BACK_TO_RANGE // Ascending back to meet beacon range requirement
};
std::atomic<VerticalAlignState> g_vertical_state(VerticalAlignState::IDLE);
std::chrono::steady_clock::time_point g_hold_start_time;
float g_target_beacon_range_threshold = 0.0f; // Calculated target range based on horizontal dist and angle
// --- End NEW ---


// Helper function to parse boolean string
bool parseBool(const std::string& value, bool defaultValue) {
    std::string lower_value = value;
    std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
    if (lower_value == "true" || lower_value == "1") {
        return true;
    } else if (lower_value == "false" || lower_value == "0") {
        return false;
    } else {
        std::cerr << "Warning: Invalid boolean value: '" << value << "'. Using default (" << std::boolalpha << defaultValue << ")." << std::endl;
        return defaultValue;
    }
}

// Helper function to get VerticalAlignState name string
std::string getVerticalStateName(VerticalAlignState state) {
    switch (state) {
        case VerticalAlignState::IDLE: return "IDLE";
        case VerticalAlignState::ALIGN_HORIZONTAL: return "ALIGN_HORIZONTAL";
        case VerticalAlignState::ASCEND_TO_RANGE: return "ASCEND_TO_RANGE";
        case VerticalAlignState::HOLDING: return "HOLDING";
        case VerticalAlignState::DESCEND_TO_ALT: return "DESCEND_TO_ALT";
        case VerticalAlignState::ASCEND_BACK_TO_RANGE: return "ASCEND_BACK_TO_RANGE";
        default: return "UNKNOWN";
    }
}


// Function to load preferences (Simplified, added vertical & beacon range params)
void loadPreferences() {
    std::cout << "Loading preferences..." << std::endl; // Logged
    std::ifstream preferencesFile("preferences.txt");
    if (preferencesFile.is_open()) {
        std::string line;
        while (std::getline(preferencesFile, line)) {
            // Trim leading/trailing whitespace
            line.erase(0, line.find_first_not_of(" \t\n\r\f\v"));
            line.erase(line.find_last_not_of(" \t\n\r\f\v") + 1);

            // Skip empty lines or comments
            if (line.empty() || line[0] == '#') continue;

            size_t equalsPos = line.find('=');
            if (equalsPos == std::string::npos) {
                std::cerr << "Warning: Skipping invalid line in preferences file: " << line << std::endl; // Logged
                continue;
            }

            std::string key = line.substr(0, equalsPos);
            std::string value = line.substr(equalsPos + 1);

            try {
                if (key == "target_beacon_id") {
                    TARGET_BEACON_ID = value;
                    std::cout << "  TARGET_BEACON_ID set to: " << TARGET_BEACON_ID << " (from preferences file)" << std::endl; // Logged
                } else if (key == "targetdistance") {
                    targetDistance = std::stof(value);
                    std::cout << "  targetDistance set to: " << targetDistance << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "target_azimuth") {
                    targetAzimuth = std::stof(value);
                    std::cout << "  targetAzimuth (Initial) set to: " << targetAzimuth << " degrees (from preferences file)" << std::endl; // Logged
                } else if (key == "radar_mount_angle_degrees") {
                    radarMountAngleDegrees = std::stof(value);
                    std::cout << "  radarMountAngleDegrees set to: " << radarMountAngleDegrees << " degrees (from preferences file)" << std::endl; // Logged
                } else if (key == "kp_forward") {
                    Kp_forward = std::stof(value);
                    std::cout << "  Kp_forward set to: " << Kp_forward << " (from preferences file)" << std::endl; // Logged
                } else if (key == "max_forward_speed") {
                    max_forward_speed = std::stof(value);
                    std::cout << "  max_forward_speed set to: " << max_forward_speed << " m/s (from preferences file)" << std::endl; // Logged
                } else if (key == "forward_dead_zone") {
                    forward_dead_zone = std::stof(value);
                    std::cout << "  forward_dead_zone set to: " << forward_dead_zone << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "kp_lateral") {
                    Kp_lateral = std::stof(value);
                    std::cout << "  Kp_lateral set to: " << Kp_lateral << " (from preferences file)" << std::endl; // Logged
                } else if (key == "max_lateral_speed") {
                    max_lateral_speed = std::stof(value);
                    std::cout << "  max_lateral_speed set to: " << max_lateral_speed << " m/s (from preferences file)" << std::endl; // Logged
                } else if (key == "azimuth_dead_zone") {
                    azimuth_dead_zone = std::stof(value);
                    std::cout << "  azimuth_dead_zone set to: " << azimuth_dead_zone << " degrees (from preferences file)" << std::endl; // Logged
                } else if (key == "invert_lateral_control") {
                    invertLateralControl = parseBool(value, invertLateralControl); // Use helper
                    std::cout << "  invertLateralControl set to: " << std::boolalpha << invertLateralControl << " (from preferences file)" << std::endl; // Logged
                // Yaw Control Keys
                } else if (key == "kp_yaw") {
                    Kp_yaw = std::stof(value);
                    std::cout << "  Kp_yaw set to: " << Kp_yaw << " (from preferences file)" << std::endl;
                } else if (key == "max_yaw_rate") {
                    max_yaw_rate = std::stof(value);
                    std::cout << "  max_yaw_rate set to: " << max_yaw_rate << " deg/s (from preferences file)" << std::endl;
                } else if (key == "yaw_azimuth_balance_dead_zone") {
                    yaw_azimuth_balance_dead_zone = std::stof(value);
                    std::cout << "  yaw_azimuth_balance_dead_zone set to: " << yaw_azimuth_balance_dead_zone << " degrees (from preferences file)" << std::endl;
                } else if (key == "invert_yaw_control") {
                    invertYawControl = parseBool(value, invertYawControl);
                    std::cout << "  invertYawControl set to: " << std::boolalpha << invertYawControl << " (from preferences file)" << std::endl;
                // Vertical Control Keys (NEW)
                 } else if (key == "target_altitude") {
                    TARGET_ALTITUDE = std::stof(value);
                    std::cout << "  TARGET_ALTITUDE set to: " << TARGET_ALTITUDE << " m (from preferences file)" << std::endl;
                } else if (key == "vertical_speed") {
                    VERTICAL_SPEED = std::stof(value);
                    std::cout << "  VERTICAL_SPEED set to: " << VERTICAL_SPEED << " m/s (from preferences file)" << std::endl;
                } else if (key == "hold_duration_seconds") {
                    HOLD_DURATION_SECONDS = std::stof(value);
                    std::cout << "  HOLD_DURATION_SECONDS set to: " << HOLD_DURATION_SECONDS << " s (from preferences file)" << std::endl;
                // Wall Sensor Keys
                } else if (key == "use_wall_sensor_r_az") {
                    useWallSensorRAz = parseBool(value, useWallSensorRAz);
                    std::cout << "  useWallSensorRAz set to: " << std::boolalpha << useWallSensorRAz << " (from preferences file)" << std::endl; // Logged
                } else if (key == "use_wall_sensor_r_el") {
                    useWallSensorREl = parseBool(value, useWallSensorREl);
                    std::cout << "  useWallSensorREl set to: " << std::boolalpha << useWallSensorREl << " (from preferences file)" << std::endl; // Logged
                } else if (key == "use_wall_sensor_r_az_r_el") {
                    useWallSensorRAzREl = parseBool(value, useWallSensorRAzREl);
                    std::cout << "  useWallSensorRAzREl set to: " << std::boolalpha << useWallSensorRAzREl << " (from preferences file)" << std::endl; // Logged
                // Beacon AZIMUTH Sensor Keys
                } else if (key == "use_beacon_sensor_r_az") {
                    useBeaconSensorRAz = parseBool(value, useBeaconSensorRAz);
                    std::cout << "  useBeaconSensorRAz (Azimuth) set to: " << std::boolalpha << useBeaconSensorRAz << " (from preferences file)" << std::endl; // Logged
                } else if (key == "use_beacon_sensor_r_el") {
                    useBeaconSensorREl = parseBool(value, useBeaconSensorREl);
                    std::cout << "  useBeaconSensorREl (Azimuth) set to: " << std::boolalpha << useBeaconSensorREl << " (from preferences file)" << std::endl; // Logged
                } else if (key == "use_beacon_sensor_r_az_r_el") {
                    useBeaconSensorRAzREl = parseBool(value, useBeaconSensorRAzREl);
                    std::cout << "  useBeaconSensorRAzREl (Azimuth) set to: " << std::boolalpha << useBeaconSensorRAzREl << " (from preferences file)" << std::endl; // Logged
                // Beacon RANGE Sensor Keys (NEW)
                } else if (key == "use_beacon_range_sensor_r_az") {
                    useBeaconRangeSensorRAz = parseBool(value, useBeaconRangeSensorRAz);
                    std::cout << "  useBeaconRangeSensorRAz (Range) set to: " << std::boolalpha << useBeaconRangeSensorRAz << " (from preferences file)" << std::endl;
                } else if (key == "use_beacon_range_sensor_r_el") {
                    useBeaconRangeSensorREl = parseBool(value, useBeaconRangeSensorREl);
                    std::cout << "  useBeaconRangeSensorREl (Range) set to: " << std::boolalpha << useBeaconRangeSensorREl << " (from preferences file)" << std::endl;
                } else if (key == "use_beacon_range_sensor_r_az_r_el") {
                    useBeaconRangeSensorRAzREl = parseBool(value, useBeaconRangeSensorRAzREl);
                    std::cout << "  useBeaconRangeSensorRAzREl (Range) set to: " << std::boolalpha << useBeaconRangeSensorRAzREl << " (from preferences file)" << std::endl;
                // Yaw Sensor Keys
                } else if (key == "use_yaw_sensor_r_az") {
                    useYawSensorRAz = parseBool(value, useYawSensorRAz);
                    std::cout << "  useYawSensorRAz set to: " << std::boolalpha << useYawSensorRAz << " (from preferences file)" << std::endl;
                } else if (key == "use_yaw_sensor_r_el") {
                    useYawSensorREl = parseBool(value, useYawSensorREl);
                    std::cout << "  useYawSensorREl set to: " << std::boolalpha << useYawSensorREl << " (from preferences file)" << std::endl;
                } else if (key == "use_yaw_sensor_r_az_r_el") {
                    useYawSensorRAzREl = parseBool(value, useYawSensorRAzREl);
                    std::cout << "  useYawSensorRAzREl set to: " << std::boolalpha << useYawSensorRAzREl << " (from preferences file)" << std::endl;
                // --- End NEW ---
                } else if (key == "enable_bridge_reconnection") {
                    enable_bridge_reconnection = parseBool(value, enable_bridge_reconnection);
                    std::cout << "  enable_bridge_reconnection set to: " << std::boolalpha << enable_bridge_reconnection << " (from preferences file)" << std::endl; // Logged
                } else if (key == "use_local_bridge_script") { // NEW Key for Local Test
                    useLocalBridgeScript = parseBool(value, useLocalBridgeScript);
                    std::cout << "  useLocalBridgeScript set to: " << std::boolalpha << useLocalBridgeScript << " (from preferences file)" << std::endl; // Logged
                } else if (key == "connect_to_drone") {
                    connect_to_drone = parseBool(value, connect_to_drone);
                    std::cout << "  connect_to_drone set to: " << std::boolalpha << connect_to_drone << " (from preferences file)" << std::endl; // Logged
                }
                 else {
                     // Optionally log unrecognized keys
                     std::cout << "  Ignoring unrecognized key in preferences: " << key << std::endl; // Logged
                 }

            } catch (const std::invalid_argument& ia) {
                std::cerr << "Warning: Invalid number format for key '" << key << "' in preferences file: " << value << std::endl; // Logged
            } catch (const std::out_of_range& oor) {
                std::cerr << "Warning: Value out of range for key '" << key << "' in preferences file: " << value << std::endl; // Logged
            } catch (...) {
                 std::cerr << "Warning: Unknown error parsing line for key '" << key << "' in preferences file: " << value << std::endl; // Logged
            }
        }
        preferencesFile.close();
        std::cout << "Finished loading preferences." << std::endl; // Logged
    } else {
        std::cout << "Preferences file ('preferences.txt') not found. Using default values:" << std::endl; // Logged
        std::cout << "  Default TARGET_BEACON_ID: " << TARGET_BEACON_ID << std::endl; // Logged
        std::cout << "  Default targetDistance: " << targetDistance << " meters" << std::endl; // Logged
        std::cout << "  Default targetAzimuth (Initial): " << targetAzimuth << " degrees" << std::endl; // Logged
        std::cout << "  Default radarMountAngleDegrees: " << radarMountAngleDegrees << " degrees" << std::endl; // Logged default
        std::cout << "  Default Kp_forward: " << Kp_forward << std::endl; // Logged
        std::cout << "  Default max_forward_speed: " << max_forward_speed << " m/s" << std::endl; // Logged
        std::cout << "  Default forward_dead_zone: " << forward_dead_zone << " meters" << std::endl; // Logged
        std::cout << "  Default Kp_lateral: " << Kp_lateral << std::endl; // Logged
        std::cout << "  Default max_lateral_speed: " << max_lateral_speed << " m/s" << std::endl; // Logged
        std::cout << "  Default azimuth_dead_zone: " << azimuth_dead_zone << " degrees" << std::endl; // Logged
        std::cout << "  Default invertLateralControl: " << std::boolalpha << invertLateralControl << std::endl; // Logged default
        // Yaw Control Defaults
        std::cout << "  Default Kp_yaw: " << Kp_yaw << std::endl;
        std::cout << "  Default max_yaw_rate: " << max_yaw_rate << " deg/s" << std::endl;
        std::cout << "  Default yaw_azimuth_balance_dead_zone: " << yaw_azimuth_balance_dead_zone << " degrees" << std::endl;
        std::cout << "  Default invertYawControl: " << std::boolalpha << invertYawControl << std::endl;
         // Vertical Control Defaults (NEW)
        std::cout << "  Default TARGET_ALTITUDE: " << TARGET_ALTITUDE << " m" << std::endl;
        std::cout << "  Default VERTICAL_SPEED: " << VERTICAL_SPEED << " m/s" << std::endl;
        std::cout << "  Default HOLD_DURATION_SECONDS: " << HOLD_DURATION_SECONDS << " s" << std::endl;
        // Wall Sensor Defaults
        std::cout << "  Default useWallSensorRAz: " << std::boolalpha << useWallSensorRAz << std::endl;
        std::cout << "  Default useWallSensorREl: " << std::boolalpha << useWallSensorREl << std::endl;
        std::cout << "  Default useWallSensorRAzREl: " << std::boolalpha << useWallSensorRAzREl << std::endl;
        // Beacon Azimuth Sensor Defaults
        std::cout << "  Default useBeaconSensorRAz (Azimuth): " << std::boolalpha << useBeaconSensorRAz << std::endl;
        std::cout << "  Default useBeaconSensorREl (Azimuth): " << std::boolalpha << useBeaconSensorREl << std::endl;
        std::cout << "  Default useBeaconSensorRAzREl (Azimuth): " << std::boolalpha << useBeaconSensorRAzREl << std::endl;
        // Beacon Range Sensor Defaults (NEW)
        std::cout << "  Default useBeaconRangeSensorRAz (Range): " << std::boolalpha << useBeaconRangeSensorRAz << std::endl;
        std::cout << "  Default useBeaconRangeSensorREl (Range): " << std::boolalpha << useBeaconRangeSensorREl << std::endl;
        std::cout << "  Default useBeaconRangeSensorRAzREl (Range): " << std::boolalpha << useBeaconRangeSensorRAzREl << std::endl;
        // Yaw Sensor Defaults
        std::cout << "  Default useYawSensorRAz: " << std::boolalpha << useYawSensorRAz << std::endl;
        std::cout << "  Default useYawSensorREl: " << std::boolalpha << useYawSensorREl << std::endl;
        std::cout << "  Default useYawSensorRAzREl: " << std::boolalpha << useYawSensorRAzREl << std::endl;
        // --- End NEW ---
        std::cout << "  Default enable_bridge_reconnection: " << std::boolalpha << enable_bridge_reconnection << std::endl; // Logged default
        std::cout << "  Default useLocalBridgeScript: " << std::boolalpha << useLocalBridgeScript << std::endl; // Logged default (NEW)
        std::cout << "  Default connect_to_drone: " << std::boolalpha << connect_to_drone << std::endl; // Logged default
    }
}


// Display full radar object details (Needed for 'e' and debug in 'w')
void displayRadarObjects(const std::vector<RadarObject>& objects) {
    // Check if there are any objects to display to avoid printing header unnecessarily
    if (objects.empty()) return;

    std::cout << "--- Begin Radar Objects for Second: " << currentSecond << " ---" << std::endl; // Logged - Header
    for (const auto& obj : objects) {
        std::cout << "Radar Object:\n" // Logged
                  << "  Timestamp: " << obj.timestamp << "\n" // Logged
                  << "  Sensor: " << obj.sensor << "\n" // Logged
                  << "  Source: " << obj.src << "\n" // Logged
                  << "  ID: " << obj.ID << "\n" // Logged
                  << "  X: " << obj.X << " Y: " << obj.Y << " Z: " << obj.Z << "\n" // Logged
                  << "  Xdir: " << obj.Xdir << " Ydir: " << obj.Ydir << " Zdir: " << obj.Zdir << "\n" // Logged
                  << "  Range: " << obj.Range << " Range Rate: " << obj.RangeRate << "\n" // Logged
                  << "  Power: " << obj.Pwr << " Azimuth: " << obj.Az << " Elevation: " << obj.El << "\n" // Logged
                  << "  Xsize: " << obj.Xsize << " Ysize: " << obj.Ysize << " Zsize: " << obj.Zsize << "\n" // Logged
                  << "  Confidence: " << obj.Conf << "\n" // Logged
                  << "----------------------------------------" << std::endl; // Logged
    }
     std::cout << "--- End Radar Objects for Second: " << currentSecond << " ---" << std::endl; // Logged - Footer
}

// --- REMOVED displayRadarObjectsMinimal ---

// Helper lambda to create sensor string for logging
auto getSensorString = [](bool useAz, bool useEl, bool useAzEl) -> std::string {
    std::string usedSensorsStr = "";
    if (useAz) usedSensorsStr += "R_Az ";
    if (useEl) usedSensorsStr += "R_El ";
    if (useAzEl) usedSensorsStr += "R_Az_R_El";
    if (usedSensorsStr.empty()) usedSensorsStr = "NONE";
    // Remove trailing space if any
    if (!usedSensorsStr.empty() && usedSensorsStr.back() == ' ') {
        usedSensorsStr.pop_back();
    }
    return usedSensorsStr;
};

// Enum for processing modes (Added Vertical Modes)
enum class ProcessingMode {
    WALL_FOLLOW,         // Mode for Wall+Lateral Beacon
    PROCESS_FULL,        // Mode for [e]
    WALL_BEACON_YAW,     // Mode for Wall+Lateral Beacon+Yaw
    WALL_BEACON_VERTICAL, // NEW: Wall+Lateral Beacon+Vertical
    WALL_BEACON_YAW_VERTICAL // NEW: Wall+Lateral Beacon+Yaw+Vertical
};

// --- Unified Processing Function (Handles all modes including Vertical) ---
void processRadarDataAndControl(ProcessingMode current_mode, const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    // This function now contains the logic previously split between extractBeaconAndWallData and extractWallBeaconYawData,
    // plus the new vertical state machine logic.

    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return;
        std::string ts_cleaned = obj.timestamp;
        // Timestamp cleaning already done in parseRadarData
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;

        // --- State Reset and Control Logic Trigger on New Second ---
        if (objSecond != currentSecond) {
            // Process control logic for the *previous* second if data was available
            if (!currentSecond.empty() && (hasAnonData || foundTargetBeacon || hasYawAnonData)) {

                // Get sensor strings for logging based on current settings
                std::string wallSensorLogStr = getSensorString(useWallSensorRAz, useWallSensorREl, useWallSensorRAzREl);
                std::string beaconAzSensorLogStr = getSensorString(useBeaconSensorRAz, useBeaconSensorREl, useBeaconSensorRAzREl); // Renamed
                std::string beaconRgSensorLogStr = getSensorString(useBeaconRangeSensorRAz, useBeaconRangeSensorREl, useBeaconRangeSensorRAzREl); // New
                std::string yawSensorLogStr = getSensorString(useYawSensorRAz, useYawSensorREl, useYawSensorRAzREl);

                if (enableControl && vehicle != nullptr && vehicle->control != nullptr && vehicle->subscribe != nullptr) { // Added subscribe check for altitude
                    float velocity_x = 0.0f;
                    float velocity_y = 0.0f;
                    float velocity_z = 0.0f; // Initialize vertical velocity
                    float yawRate = 0.0f;
                    float actualHorizontalDistance = std::numeric_limits<float>::quiet_NaN();
                    float current_altitude = std::numeric_limits<float>::quiet_NaN(); // Initialize altitude

                    // Get current altitude (AGL)
                    current_altitude = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_HEIGHT_FUSION>();
                    // Basic validity check for altitude (e.g., not negative, maybe within reasonable bounds)
                    if (current_altitude < 0 || std::isnan(current_altitude)) {
                        std::cerr << "Warning: Invalid altitude data (" << current_altitude << "). Holding vertical position." << std::endl;
                        current_altitude = std::numeric_limits<float>::quiet_NaN(); // Mark as invalid
                        // Potentially force state back to ALIGN_HORIZONTAL or IDLE if critical? For now, just hold Z.
                        if (g_vertical_state.load() != VerticalAlignState::IDLE) { // Only revert if in a vertical mode
                             std::cerr << "Reverting to ALIGN_HORIZONTAL due to invalid altitude." << std::endl;
                             g_vertical_state.store(VerticalAlignState::ALIGN_HORIZONTAL);
                        }
                    }


                    // --- Horizontal Distance Calculation (Common) ---
                    if (hasAnonData && lowestRange != std::numeric_limits<float>::max()) {
                        float angleRadians = radarMountAngleDegrees * M_PI / 180.0;
                        actualHorizontalDistance = lowestRange * std::cos(angleRadians);
                    }

                    // --- Forward Control (Common) ---
                    if (!std::isnan(actualHorizontalDistance)) {
                        float difference = actualHorizontalDistance - targetDistance;
                        if (std::abs(difference) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * difference, max_forward_speed));
                        }
                    }

                    // --- Lateral Control (Common) ---
                    float azimuth_error = std::numeric_limits<float>::quiet_NaN();
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) { // foundTargetBeacon is now tied to finding Azimuth
                        azimuth_error = targetBeaconAzimuth - targetAzimuth;
                        if (std::abs(azimuth_error) > azimuth_dead_zone) {
                            float lateral_gain = invertLateralControl ? Kp_lateral : -Kp_lateral;
                            velocity_y = std::max(-max_lateral_speed, std::min(lateral_gain * azimuth_error, max_lateral_speed));
                        }
                    } else if (!foundTargetBeacon && enableControl) {
                         std::cout << "*** TARGET BEACON AZIMUTH '" << TARGET_BEACON_ID << "' NOT FOUND FROM SENSORS [" << beaconAzSensorLogStr << "] ***" << std::endl; // Log beacon AZIMUTH loss if relevant
                         // If beacon azimuth lost during vertical maneuver, revert state
                         if (g_vertical_state.load() != VerticalAlignState::IDLE && g_vertical_state.load() != VerticalAlignState::ALIGN_HORIZONTAL) {
                             std::cerr << "Warning: Beacon Azimuth lost during vertical maneuver. Reverting to ALIGN_HORIZONTAL." << std::endl;
                             g_vertical_state.store(VerticalAlignState::ALIGN_HORIZONTAL);
                         }
                    }


                    // --- Yaw Control (Only for Yaw modes) ---
                    float avgPositiveAz = 0.0f;
                    float avgNegativeAz = 0.0f;
                    float yaw_balance_error = 0.0f;
                    if (current_mode == ProcessingMode::WALL_BEACON_YAW || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {
                        if (hasYawAnonData) {
                            if (countPositiveAz > 0) avgPositiveAz = sumPositiveAz / countPositiveAz;
                            if (countNegativeAz > 0) avgNegativeAz = sumNegativeAz / countNegativeAz;
                            yaw_balance_error = avgPositiveAz + avgNegativeAz; // Sum of averages
                            if (std::fabs(yaw_balance_error) > yaw_azimuth_balance_dead_zone) {
                                float yaw_gain = invertYawControl ? Kp_yaw : -Kp_yaw;
                                yawRate = std::max(-max_yaw_rate, std::min(yaw_gain * yaw_balance_error, max_yaw_rate));
                            }
                        }
                    }

                    // --- Vertical Control State Machine (Only for Vertical modes) ---
                    bool is_horizontally_aligned = (!std::isnan(actualHorizontalDistance) && std::abs(actualHorizontalDistance - targetDistance) <= forward_dead_zone);
                    // Lateral alignment now depends on foundTargetBeacon (Azimuth)
                    bool is_laterally_aligned = (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) && std::abs(targetBeaconAzimuth - targetAzimuth) <= azimuth_dead_zone);
                    bool is_yaw_aligned = true; // Default true for non-yaw modes
                    if (current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {
                        is_yaw_aligned = (!hasYawAnonData || std::fabs(yaw_balance_error) <= yaw_azimuth_balance_dead_zone);
                    }
                    // Alignment requires valid data (don't consider aligned if data is missing)
                    bool fully_aligned = is_horizontally_aligned && is_laterally_aligned && is_yaw_aligned;

                    VerticalAlignState current_v_state = g_vertical_state.load(); // Load atomic state once per cycle

                    // Range validity check for vertical state transitions
                    bool is_range_valid = !std::isnan(targetBeaconRange);
                    if (!is_range_valid && current_v_state != VerticalAlignState::IDLE && current_v_state != VerticalAlignState::ALIGN_HORIZONTAL) {
                        std::cerr << "Warning: Beacon Range data lost/invalid during vertical maneuver. Reverting to ALIGN_HORIZONTAL." << std::endl;
                        g_vertical_state.store(VerticalAlignState::ALIGN_HORIZONTAL);
                    }


                    if (current_mode == ProcessingMode::WALL_BEACON_VERTICAL || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {

                        // Calculate the target beacon range threshold (hypotenuse)
                        float angleRadians = radarMountAngleDegrees * M_PI / 180.0;
                        if (std::cos(angleRadians) > 1e-6) { // Avoid division by zero if angle is 90 deg
                            g_target_beacon_range_threshold = targetDistance / std::cos(angleRadians);
                        } else {
                            g_target_beacon_range_threshold = targetDistance; // Or handle error appropriately
                            std::cerr << "Warning: Radar mount angle near 90 degrees, using targetDistance as range threshold." << std::endl;
                        }


                        switch (current_v_state) {
                            case VerticalAlignState::ALIGN_HORIZONTAL:
                                velocity_z = 0; // Ensure no vertical movement
                                // Transition requires alignment, valid AZIMUTH, valid RANGE, and valid altitude
                                if (fully_aligned && foundTargetBeacon && is_range_valid && !std::isnan(current_altitude)) {
                                    std::cout << "[Vertical State] Horizontally Aligned. Transitioning to ASCEND_TO_RANGE." << std::endl;
                                    g_vertical_state.store(VerticalAlignState::ASCEND_TO_RANGE);
                                } else {
                                    // Keep applying horizontal/yaw corrections
                                }
                                break;

                            case VerticalAlignState::ASCEND_TO_RANGE:
                                // Maintain horizontal/yaw alignment while ascending
                                velocity_z = VERTICAL_SPEED;
                                // Check if beacon range is valid and within threshold
                                if (is_range_valid && targetBeaconRange <= g_target_beacon_range_threshold) {
                                     std::cout << "[Vertical State] Reached Target Beacon Range (" << targetBeaconRange << " <= " << g_target_beacon_range_threshold << "). Transitioning to HOLDING." << std::endl;
                                     velocity_z = 0; // Stop ascent immediately
                                     g_hold_start_time = std::chrono::steady_clock::now();
                                     g_vertical_state.store(VerticalAlignState::HOLDING);
                                }
                                // If beacon azimuth, range, or altitude invalid, revert state (handled above)
                                break;

                            case VerticalAlignState::HOLDING:
                                // Maintain horizontal/yaw alignment while holding
                                velocity_z = 0; // Hold Z
                                { // Use block scope for duration calculation
                                    auto time_elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::steady_clock::now() - g_hold_start_time);
                                    if (time_elapsed.count() >= HOLD_DURATION_SECONDS) {
                                        std::cout << "[Vertical State] Hold complete (" << time_elapsed.count() << "s). Transitioning to DESCEND_TO_ALT." << std::endl;
                                        g_vertical_state.store(VerticalAlignState::DESCEND_TO_ALT);
                                    }
                                }
                                // If beacon azimuth, range, or altitude invalid, revert state (handled above)
                                break;

                            case VerticalAlignState::DESCEND_TO_ALT:
                                // Maintain horizontal/yaw alignment while descending
                                velocity_z = -VERTICAL_SPEED;
                                // Check if altitude is valid and at or below target
                                if (!std::isnan(current_altitude) && current_altitude <= TARGET_ALTITUDE) {
                                     std::cout << "[Vertical State] Reached Target Altitude (" << current_altitude << " <= " << TARGET_ALTITUDE << "). Transitioning to ASCEND_BACK_TO_RANGE." << std::endl;
                                     velocity_z = 0; // Stop descent immediately
                                     g_vertical_state.store(VerticalAlignState::ASCEND_BACK_TO_RANGE);
                                }
                                // If beacon azimuth, range, or altitude invalid, revert state (handled above)
                                break;

                            case VerticalAlignState::ASCEND_BACK_TO_RANGE:
                                // Maintain horizontal/yaw alignment while ascending
                                velocity_z = VERTICAL_SPEED;
                                // Check if beacon range is valid and within threshold
                                if (is_range_valid && targetBeaconRange <= g_target_beacon_range_threshold) {
                                     std::cout << "[Vertical State] Reached Target Beacon Range Again (" << targetBeaconRange << " <= " << g_target_beacon_range_threshold << "). Reverting to ALIGN_HORIZONTAL." << std::endl;
                                     velocity_z = 0; // Stop ascent
                                     g_vertical_state.store(VerticalAlignState::ALIGN_HORIZONTAL); // Go back to alignment check
                                }
                                // If beacon azimuth, range, or altitude invalid, revert state (handled above)
                                break;

                             case VerticalAlignState::IDLE:
                             default:
                                // Should not happen if mode is vertical, but safety default
                                velocity_z = 0;
                                g_vertical_state.store(VerticalAlignState::ALIGN_HORIZONTAL); // Force back to alignment
                                break;
                        }
                    } else {
                        // For non-vertical modes, ensure state is IDLE and Z velocity is 0
                        g_vertical_state.store(VerticalAlignState::IDLE);
                        velocity_z = 0;
                    }


                    // --- Send Command ---
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, velocity_z, yawRate); // Use calculated velocity_z

                    // --- Logging (Combined & Updated) ---
                    std::cout << "[" << (current_mode == ProcessingMode::WALL_BEACON_YAW || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL ? "Wall+Lateral Beacon+Yaw" : "Wall+Lateral Beacon") // Updated Log Names
                              << (current_mode == ProcessingMode::WALL_BEACON_VERTICAL || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL ? "+Vertical" : "")
                              << "] Ctrl Status (Second: " << currentSecond << "):\n"
                              << "  VState: " << getVerticalStateName(g_vertical_state.load()) << "\n"
                              << "  TargetHorizDist = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n"
                              << "  RadarAngle      = " << std::fixed << std::setprecision(1) << std::setw(3) << radarMountAngleDegrees << "\n"
                              << "  Wall Sensors    = " << wallSensorLogStr << "\n"
                              << "  MeasuredRange   = " << (hasAnonData && lowestRange != std::numeric_limits<float>::max() ? std::to_string(lowestRange) : "N/A") << "\n"
                              << "  CalculatedDist  = " << (!std::isnan(actualHorizontalDistance) ? std::to_string(actualHorizontalDistance) : "N/A") << "\n"
                              << "  BeaconAz Sensors= " << beaconAzSensorLogStr << "\n" // Updated Label
                              << "  BeaconRg Sensors= " << beaconRgSensorLogStr << "\n" // New Label
                              << "  TargetBeaconAz  = " << targetAzimuth << "\n"
                              << "  CurrentBeaconAz = " << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "  CurrentBeaconRg = " << (!std::isnan(targetBeaconRange) ? std::to_string(targetBeaconRange) : "N/A") << "\n" // Log beacon range (check only NaN)
                              << "  TargetBeaconRgThresh = " << g_target_beacon_range_threshold << "\n" // Log threshold
                              << "  LateralInverted = " << std::boolalpha << invertLateralControl << "\n"
                              << "  CurrentAltitude = " << (!std::isnan(current_altitude) ? std::to_string(current_altitude) : "N/A") << "\n" // Log altitude
                              << "  TargetAltitude  = " << TARGET_ALTITUDE << "\n"; // Log target altitude

                    if (current_mode == ProcessingMode::WALL_BEACON_YAW || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {
                        std::cout << "  Yaw Sensors     = " << yawSensorLogStr << "\n"
                                  << "  Avg Pos Az      = " << avgPositiveAz << " (Count: " << countPositiveAz << ")\n"
                                  << "  Avg Neg Az      = " << avgNegativeAz << " (Count: " << countNegativeAz << ")\n"
                                  << "  Az Balance Err  = " << yaw_balance_error << "\n"
                                  << "  Yaw Inverted    = " << std::boolalpha << invertYawControl << "\n";
                    }

                    std::cout << "\n"
                              << "  Computed Vel(X=" << velocity_x << ", Y=" << velocity_y << ", Z=" << velocity_z << ", YawRate=" << yawRate << ")" << std::endl;
                    std::cout << "--------------------------------------" << std::endl;

                    vehicle->control->flightCtrl(ctrlData);

                    // --- Update Global Live Data (Under Lock) ---
                    {
                        std::lock_guard<std::mutex> lock(g_liveDataMutex);
                        g_latest_beacon_range = targetBeaconRange; // Update range (might be NaN)
                        g_latest_wall_horizontal_distance = actualHorizontalDistance; // Update distance (might be NaN)
                        g_latest_altitude = current_altitude; // Update altitude (might be NaN)
                        // Beacon seen time is updated in accumulation phase
                    }
                    // --- End Update Global Live Data ---

                } else if (hasAnonData || foundTargetBeacon || hasYawAnonData) { // Logging when control disabled or unavailable
                     float actualHorizontalDistance = std::numeric_limits<float>::quiet_NaN();
                      if (hasAnonData && lowestRange != std::numeric_limits<float>::max()) {
                         float angleRadians = radarMountAngleDegrees * M_PI / 180.0;
                         actualHorizontalDistance = lowestRange * std::cos(angleRadians);
                      }
                      if (!foundTargetBeacon) {
                          std::cout << "*** TARGET BEACON AZIMUTH '" << TARGET_BEACON_ID << "' NOT FOUND FROM SENSORS [" << beaconAzSensorLogStr << "] ***" << std::endl;
                      }
                     float avgPositiveAz = 0.0f, avgNegativeAz = 0.0f, yaw_balance_error = 0.0f;
                      if (current_mode == ProcessingMode::WALL_BEACON_YAW || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {
                         if (hasYawAnonData) {
                             avgPositiveAz = (countPositiveAz > 0) ? (sumPositiveAz / countPositiveAz) : 0.0f;
                             avgNegativeAz = (countNegativeAz > 0) ? (sumNegativeAz / countNegativeAz) : 0.0f;
                             yaw_balance_error = avgPositiveAz + avgNegativeAz;
                         }
                      }
                    float current_altitude = std::numeric_limits<float>::quiet_NaN();
                    if (vehicle != nullptr && vehicle->subscribe != nullptr) { // Try to get altitude even if control disabled
                         current_altitude = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_HEIGHT_FUSION>();
                         if (current_altitude < 0 || std::isnan(current_altitude)) current_altitude = std::numeric_limits<float>::quiet_NaN();
                    }

                     // --- Logging (No Control) ---
                    std::cout << "[" << (current_mode == ProcessingMode::WALL_BEACON_YAW || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL ? "Wall+Lateral Beacon+Yaw" : "Wall+Lateral Beacon") // Updated Log Names
                              << (current_mode == ProcessingMode::WALL_BEACON_VERTICAL || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL ? "+Vertical" : "")
                              << "] (No Control) Status (Second: " << currentSecond << "):\n"
                              << "  VState: " << getVerticalStateName(g_vertical_state.load()) << "\n"
                              << "  TargetHorizDist = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n"
                              << "  RadarAngle      = " << std::fixed << std::setprecision(1) << std::setw(3) << radarMountAngleDegrees << "\n"
                              << "  Wall Sensors    = " << wallSensorLogStr << "\n"
                              << "  MeasuredRange   = " << (hasAnonData && lowestRange != std::numeric_limits<float>::max() ? std::to_string(lowestRange) : "N/A") << "\n"
                              << "  CalculatedDist  = " << (!std::isnan(actualHorizontalDistance) ? std::to_string(actualHorizontalDistance) : "N/A") << "\n"
                              << "  BeaconAz Sensors= " << beaconAzSensorLogStr << "\n" // Updated Label
                              << "  BeaconRg Sensors= " << beaconRgSensorLogStr << "\n" // New Label
                              << "  TargetBeaconAz  = " << targetAzimuth << "\n"
                              << "  CurrentBeaconAz = " << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "  CurrentBeaconRg = " << (!std::isnan(targetBeaconRange) ? std::to_string(targetBeaconRange) : "N/A") << "\n"
                              << "  LateralInverted = " << std::boolalpha << invertLateralControl << "\n"
                              << "  CurrentAltitude = " << (!std::isnan(current_altitude) ? std::to_string(current_altitude) : "N/A") << "\n";

                    if (current_mode == ProcessingMode::WALL_BEACON_YAW || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {
                        std::cout << "  Yaw Sensors     = " << yawSensorLogStr << "\n"
                                  << "  Avg Pos Az      = " << avgPositiveAz << " (Count: " << countPositiveAz << ")\n"
                                  << "  Avg Neg Az      = " << avgNegativeAz << " (Count: " << countNegativeAz << ")\n"
                                  << "  Az Balance Err  = " << yaw_balance_error << "\n"
                                  << "  Yaw Inverted    = " << std::boolalpha << invertYawControl << "\n";
                    }
                    std::cout << "--------------------------------------" << std::endl;

                     // --- Update Global Live Data (Under Lock) - Even if no control ---
                    {
                        std::lock_guard<std::mutex> lock(g_liveDataMutex);
                        g_latest_beacon_range = targetBeaconRange;
                        g_latest_wall_horizontal_distance = actualHorizontalDistance;
                        g_latest_altitude = current_altitude;
                        // Beacon seen time updated in accumulation
                    }
                    // --- End Update Global Live Data ---
                }
            }

            // Reset state variables for the new second (Common for all modes)
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            targetBeaconRange = std::numeric_limits<float>::quiet_NaN(); // Reset beacon range
            foundTargetBeacon = false; // Reset beacon AZIMUTH found flag
            sumPositiveAz = 0.0f;
            countPositiveAz = 0;
            sumNegativeAz = 0.0f;
            countNegativeAz = 0;
            hasYawAnonData = false;
        }
        // --- END State Reset and Control Logic on New Second ---


        // --- Data Accumulation within the Current Second (Common) ---
        // Check sensor type independently for Beacon Az, Beacon Rg, Wall Anon, Yaw Anon

        // Check if sensor matches criteria for BEACON AZIMUTH detection
        bool beaconAzSensorMatch = false;
        if (useBeaconSensorRAz && obj.sensor == "R_Az") beaconAzSensorMatch = true;
        else if (useBeaconSensorREl && obj.sensor == "R_El") beaconAzSensorMatch = true;
        else if (useBeaconSensorRAzREl && obj.sensor == "R_Az_R_El") beaconAzSensorMatch = true;

        // Check if sensor matches criteria for BEACON RANGE detection (NEW)
        bool beaconRgSensorMatch = false;
        if (useBeaconRangeSensorRAz && obj.sensor == "R_Az") beaconRgSensorMatch = true;
        else if (useBeaconRangeSensorREl && obj.sensor == "R_El") beaconRgSensorMatch = true;
        else if (useBeaconRangeSensorRAzREl && obj.sensor == "R_Az_R_El") beaconRgSensorMatch = true;

        // Check if sensor matches criteria for WALL distance calculation
        bool wallSensorMatch = false;
        if (useWallSensorRAz && obj.sensor == "R_Az") wallSensorMatch = true;
        else if (useWallSensorREl && obj.sensor == "R_El") wallSensorMatch = true;
        else if (useWallSensorRAzREl && obj.sensor == "R_Az_R_El") wallSensorMatch = true;

        // Check if sensor matches criteria for YAW calculation
        bool yawSensorMatch = false;
        if (useYawSensorRAz && obj.sensor == "R_Az") yawSensorMatch = true;
        else if (useYawSensorREl && obj.sensor == "R_El") yawSensorMatch = true;
        else if (useYawSensorRAzREl && obj.sensor == "R_Az_R_El") yawSensorMatch = true;


        // Process data based on ID and matching sensor criteria
        if (obj.ID == TARGET_BEACON_ID) {
            bool beacon_processed_this_obj = false; // Flag to update time only once per object
            // Process Azimuth (only store first match per second)
            if (beaconAzSensorMatch && !foundTargetBeacon) { // Check Azimuth sensor match
                targetBeaconAzimuth = obj.Az;
                foundTargetBeacon = true; // Set flag based on finding Azimuth
                beacon_processed_this_obj = true;
            }
            // Process Range (only store first match per second, independent of Azimuth)
            if (beaconRgSensorMatch && std::isnan(targetBeaconRange)) { // Check Range sensor match and if range is not already set
                targetBeaconRange = obj.Range; // Store range
                beacon_processed_this_obj = true;
            }

            // Update last seen time if either Az or Rg was processed from selected sensors
            if (beacon_processed_this_obj) {
                std::lock_guard<std::mutex> lock(g_liveDataMutex);
                g_last_beacon_seen_time = std::chrono::steady_clock::now();
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            // Check for WALL distance calculation
            if (wallSensorMatch) {
                hasAnonData = true;
                if (obj.Range < lowestRange) {
                    lowestRange = obj.Range;
                }
            }
            // Check for YAW calculation (only if yaw mode active)
            if ( (current_mode == ProcessingMode::WALL_BEACON_YAW || current_mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) && yawSensorMatch) {
                 hasYawAnonData = true;
                 if (obj.Az > 0) {
                     sumPositiveAz += obj.Az;
                     countPositiveAz++;
                 } else if (obj.Az < 0) {
                     sumNegativeAz += obj.Az; // Add the negative value
                     countNegativeAz++;
                 }
            }
        }
        // --- END Data Accumulation ---
         if (stopProcessingFlag.load()) return;
    }
}
// --- END Unified Processing Function ---


// Parses JSON radar data (Keep)
std::vector<RadarObject> parseRadarData(const std::string& jsonData) {
    std::vector<RadarObject> radarObjects;
    if (jsonData.empty() || jsonData == "{}") return radarObjects;

    try {
        auto jsonFrame = json::parse(jsonData);
        if (!jsonFrame.contains("objects") || !jsonFrame["objects"].is_array()) return radarObjects;

        for (const auto& obj : jsonFrame["objects"]) {
            if (!obj.is_object()) {
                std::cerr << "Skipping non-object item in 'objects' array." << std::endl; // Logged
                continue;
            }
            RadarObject radarObj;
            // Clean timestamp string (remove quotes if present)
            if (obj.contains("timestamp") && obj["timestamp"].is_string()) {
                radarObj.timestamp = obj.value("timestamp", "");
                if (!radarObj.timestamp.empty() && radarObj.timestamp.front() == '"') radarObj.timestamp.erase(0, 1);
                if (!radarObj.timestamp.empty() && radarObj.timestamp.back() == '"') radarObj.timestamp.pop_back();
            } else {
                radarObj.timestamp = obj.value("timestamp", json(nullptr)).dump(); // Fallback for non-string or null
            }
            radarObj.sensor = obj.value("sensor", "N/A");
            radarObj.src = obj.value("src", "N/A");
            radarObj.X = obj.value("X", 0.0f); radarObj.Y = obj.value("Y", 0.0f); radarObj.Z = obj.value("Z", 0.0f);
            radarObj.Xdir = obj.value("Xdir", 0.0f); radarObj.Ydir = obj.value("Ydir", 0.0f); radarObj.Zdir = obj.value("Zdir", 0.0f);
            radarObj.Range = obj.value("Range", 0.0f); radarObj.RangeRate = obj.value("RangeRate", 0.0f);
            radarObj.Pwr = obj.value("Pwr", 0.0f); radarObj.Az = obj.value("Az", 0.0f); radarObj.El = obj.value("El", 0.0f);
            if (obj.contains("ID") && obj["ID"].is_string()) radarObj.ID = obj.value("ID", "N/A");
            else if (obj.contains("ID")) radarObj.ID = obj["ID"].dump(); else radarObj.ID = "N/A";
            radarObj.Xsize = obj.value("Xsize", 0.0f); radarObj.Ysize = obj.value("Ysize", 0.0f); radarObj.Zsize = obj.value("Zsize", 0.0f);
            radarObj.Conf = obj.value("Conf", 0.0f);
            radarObjects.push_back(radarObj);
        }
    } catch (const json::parse_error& e) {
        std::cerr << "JSON Parsing Error: " << e.what() << " at offset " << e.byte << ". Data: [" << jsonData.substr(0, 200) << "...]" << std::endl; // Logged
        return {};
    } catch (const json::type_error& e) {
        std::cerr << "JSON Type Error: " << e.what() << ". Data: [" << jsonData.substr(0, 200) << "...]" << std::endl; // Logged
        return {};
    }
    return radarObjects;
}

// Runs the python bridge script (Keep)
void runPythonBridge(const std::string& scriptName) {
    std::cout << "Starting Python bridge (" << scriptName << ")..." << std::endl; // Logged
    if (std::system(("python3 " + scriptName + " &").c_str()) != 0) {
        std::cerr << "Failed to start Python bridge script '" << scriptName << "'." << std::endl; // Logged
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Python bridge potentially started." << std::endl; // Logged
}

// Stops the python bridge script (Keep)
void stopPythonBridge(const std::string& scriptName) {
    std::cout << "Stopping Python bridge (" << scriptName << ")..." << std::endl; // Logged
    std::system(("pkill -f " + scriptName).c_str()); // Ignore result
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Sent SIGTERM to " << scriptName << "." << std::endl; // Logged
}

// Connects to the python bridge via TCP (Modified for Reconnection Logic and Forced Stop)
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket) {
    tcp::resolver resolver(io_context);
    int initial_retries = 5; // Number of quick initial retries
    int current_retry = 0;
    const int initial_delay_seconds = 2;
    // const int persistent_delay_seconds = 5; // Delay now handled in processingLoopFunction

    // Reset force stop flag before attempting connection
    forceStopReconnectionFlag.store(false);

    // --- Initial Connection Attempts ---
    while (current_retry < initial_retries) {
        // Check both stop flags
        if (stopProcessingFlag.load() || forceStopReconnectionFlag.load()) {
             std::cout << "[Connect Bridge] Stop requested during initial connection attempts." << std::endl; // Logged
             forceStopReconnectionFlag.store(false); // Reset flag
             currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status on stop
             return false;
        }
        try {
            if(socket.is_open()) { socket.close(); } // Close previous attempt if any
            auto endpoints = resolver.resolve("127.0.0.1", "5000");
            boost::system::error_code ec;
            boost::asio::connect(socket, endpoints, ec);
            if (!ec) {
                std::cout << "Connected to Python bridge." << std::endl; // Logged
                currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED); // Update status on success
                return true; // Success!
            }
            else {
                std::cerr << "Initial connection attempt " << (current_retry + 1) << "/" << initial_retries << " failed: " << ec.message() << std::endl; // Logged
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception during initial connection attempt " << (current_retry + 1) << "/" << initial_retries << ": " << e.what() << std::endl; // Logged
        }

        current_retry++;
        // Check flags again before sleeping
        if (current_retry < initial_retries && !stopProcessingFlag.load() && !forceStopReconnectionFlag.load()) {
             std::cout << "Retrying connection in " << initial_delay_seconds << " seconds..." << std::endl; // Logged
             // Sleep in smaller chunks to check flags more often
             for (int i = 0; i < initial_delay_seconds * 10 && !stopProcessingFlag.load() && !forceStopReconnectionFlag.load(); ++i) {
                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
             }
        }
    } // End initial retries loop

    // --- Initial Attempts Failed ---
    std::cerr << "Failed initial connection attempts to Python bridge." << std::endl; // Logged

    // Check if stop was requested during initial attempts
    if (stopProcessingFlag.load() || forceStopReconnectionFlag.load()) {
        std::cout << "[Connect Bridge] Stop requested after initial attempts failed." << std::endl;
        forceStopReconnectionFlag.store(false); // Reset flag
        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status on stop
        return false;
    }

    // --- Persistent Reconnection Attempts (Try once immediately after initial failure) ---
    // Use the CURRENT value of enable_bridge_reconnection
    if (!enable_bridge_reconnection) {
        std::cout << "Reconnection disabled. Giving up." << std::endl; // Logged
        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status on give up
        return false; // Give up if reconnection is disabled
    }

    // Try one more time immediately after initial failures before returning (delay handled by caller)
    std::cout << "Attempting final connection before handing back to caller..." << std::endl;
    try {
        if(socket.is_open()) { socket.close(); }
        auto endpoints = resolver.resolve("127.0.0.1", "5000");
        boost::system::error_code ec;
        boost::asio::connect(socket, endpoints, ec);
        if (!ec) {
            std::cout << "Final connection attempt successful." << std::endl; // Logged
            currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED); // Update status on success
            return true; // Success!
        } else {
            std::cerr << "Final connection attempt failed: " << ec.message() << std::endl;
        }
    } catch (const std::exception& e) {
         std::cerr << "Exception during final connection attempt: " << e.what() << std::endl;
    }

    // If we reach here, all initial attempts + one final attempt failed
    std::cout << "[Connect Bridge] All connection attempts failed." << std::endl;
    forceStopReconnectionFlag.store(false); // Reset flag just in case
    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status on final failure
    return false; // Indicate connection failure
}


// Placeholder Callbacks (Keep, needed by OSDK headers)
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }
void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }


// Processing loop function (Modified for Persistent GUI Data)
void processingLoopFunction(const std::string bridgeScriptName, Vehicle* vehicle, bool enableControlCmd, ProcessingMode mode) {
    // Log the specific mode being started
    std::string modeStr;
    switch(mode) {
        case ProcessingMode::WALL_FOLLOW: modeStr = "WALL_FOLLOW"; break;
        case ProcessingMode::PROCESS_FULL: modeStr = "PROCESS_FULL"; break;
        case ProcessingMode::WALL_BEACON_YAW: modeStr = "WALL_BEACON_YAW"; break;
        case ProcessingMode::WALL_BEACON_VERTICAL: modeStr = "WALL_BEACON_VERTICAL"; break;
        case ProcessingMode::WALL_BEACON_YAW_VERTICAL: modeStr = "WALL_BEACON_YAW_VERTICAL"; break;
        default: modeStr = "UNKNOWN"; break;
    }
    std::cout << "Processing thread started. Bridge Script: " << bridgeScriptName // Log the script name used
              << ", Control Enabled: " << std::boolalpha << enableControlCmd
              << ", Mode: " << modeStr
              << ", Reconnect Enabled: " << enable_bridge_reconnection << std::endl; // Logged

    int functionTimeout = 1; // Timeout for OSDK control calls

    runPythonBridge(bridgeScriptName);
    boost::asio::io_context io_context;
    tcp::socket socket(io_context);

    // Initial connection attempt (will handle retries/persistence based on flag)
    currentBridgeStatus.store(BridgeConnectionStatus::RECONNECTING); // Set status before attempting connection
    if (!connectToPythonBridge(io_context, socket)) {
        // connectToPythonBridge now handles the forceStop flag internally and logs appropriately
        // It also sets the status to DISCONNECTED on failure/stop
        std::cerr << "Processing thread: Initial connection failed or was stopped. Exiting thread." << std::endl; // Logged (Simplified message)
        stopPythonBridge(bridgeScriptName); // Stop the correct script
        // Release control if it was meant to be used
        if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
            std::cout << "[Processing Thread] Releasing control authority (connection fail/stop)..." << std::endl; // Logged
            ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
            if (ACK::getError(releaseAck)) ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority"); // Logged via ACK
            else std::cout << "[Processing Thread] Control authority released." << std::endl; // Logged
        }
        // Status is already set to DISCONNECTED by connectToPythonBridge on failure
        // Clear global GUI data and reset vertical state on exit (Already done in stopProcessingThreadIfNeeded called by main)
        // Ensure data is cleared even if stopProcessingThreadIfNeeded wasn't called (e.g., initial connect fail)
        { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
        { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
        { // Clear live data
            std::lock_guard<std::mutex> lockL(g_liveDataMutex);
            g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
            g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
            g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
            g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
        }
        g_vertical_state.store(VerticalAlignState::IDLE); // Reset state on thread exit
        return; // Exit thread
    }
    // If we reach here, connectToPythonBridge succeeded and set status to CONNECTED.
    std::cout << "Processing thread: Connection successful. Reading data stream..." << std::endl; // Logged
    std::string received_data_buffer;
    std::array<char, 4096> read_buffer;
    bool connection_error_occurred = false; // Track if we are in an error state

    // Reset state variables used by control modes before the loop starts
    currentSecond = "";
    // Wall/Beacon vars
    lowestRange = std::numeric_limits<float>::max();
    hasAnonData = false;
    targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
    targetBeaconRange = std::numeric_limits<float>::quiet_NaN(); // Reset range
    foundTargetBeacon = false; // Reset Azimuth found flag
    // Yaw vars
    sumPositiveAz = 0.0f;
    countPositiveAz = 0;
    sumNegativeAz = 0.0f;
    countNegativeAz = 0;
    hasYawAnonData = false;
    // Vertical state reset
    if (mode == ProcessingMode::WALL_BEACON_VERTICAL || mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {
         g_vertical_state.store(VerticalAlignState::ALIGN_HORIZONTAL); // Start in alignment phase
         std::cout << "[Processing Thread] Initializing Vertical State to ALIGN_HORIZONTAL." << std::endl;
    } else {
         g_vertical_state.store(VerticalAlignState::IDLE); // Ensure IDLE for other modes
    }
     // Reset Live Data on thread start (Already done in main before thread creation)


    while (!stopProcessingFlag.load()) {
        boost::system::error_code error;
        size_t len = socket.read_some(boost::asio::buffer(read_buffer), error);

        if (error) { // Handle read error
            if (error == boost::asio::error::eof) std::cerr << "Processing thread: Connection closed by Python bridge (EOF)." << std::endl; // Logged
            else std::cerr << "Processing thread: Error reading from socket: " << error.message() << std::endl; // Logged

            connection_error_occurred = true; // Mark error state

            // If control was active, send a stop command before attempting reconnect
            if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
                 std::cout << "[Processing Thread] Connection error. Sending zero velocity command..." << std::endl; // Logged
                 uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
                 DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
                 vehicle->control->flightCtrl(stopData); // Send stop command
            }

            // Use CURRENT value of enable_bridge_reconnection for decision
            if (enable_bridge_reconnection) {
                std::cout << "Connection lost. Attempting to reconnect..." << std::endl; // Logged
                currentBridgeStatus.store(BridgeConnectionStatus::RECONNECTING); // Set status to RECONNECTING
                if (socket.is_open()) socket.close(); // Ensure socket is closed before reconnecting

                // --- ADD DELAY HERE ---
                std::cout << "Waiting " << RECONNECT_DELAY_SECONDS << " seconds before next connection attempt..." << std::endl;
                for (int i = 0; i < RECONNECT_DELAY_SECONDS * 10 && !stopProcessingFlag.load() && !forceStopReconnectionFlag.load(); ++i) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                // Check if stop was requested during the delay
                if (stopProcessingFlag.load() || forceStopReconnectionFlag.load()) {
                    std::cout << "[Processing Thread] Stop requested during reconnection delay." << std::endl;
                    forceStopReconnectionFlag.store(false); // Reset flag
                    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Set status
                    // Clear global GUI data and reset vertical state on stop during reconnect
                    { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
                    { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
                    { // Clear live data
                         std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                         g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                         g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                         g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                         g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                     }
                    g_vertical_state.store(VerticalAlignState::IDLE);
                    break; // Exit the main while loop
                }
                // --- END DELAY ---

                // Attempt persistent reconnection (this will now respect forceStopReconnectionFlag and update status internally)
                if (connectToPythonBridge(io_context, socket)) {
                    std::cout << "Reconnection successful. Resuming data processing." << std::endl; // Logged
                    // connectToPythonBridge set status to CONNECTED
                    connection_error_occurred = false; // Clear error state
                    received_data_buffer.clear(); // Clear buffer as we might have partial data from before disconnect
                    // Re-initialize vertical state upon successful reconnect if applicable
                    if (mode == ProcessingMode::WALL_BEACON_VERTICAL || mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {
                        g_vertical_state.store(VerticalAlignState::ALIGN_HORIZONTAL);
                        std::cout << "[Processing Thread] Reconnected. Resetting Vertical State to ALIGN_HORIZONTAL." << std::endl;
                    }
                    continue; // Go back to the start of the while loop to try reading again
                } else {
                    // Connection failed OR was force stopped
                    // connectToPythonBridge set status to DISCONNECTED
                    std::cerr << "Processing thread: Persistent reconnection failed or was stopped. Exiting." << std::endl; // Logged
                    // Clear global GUI data and reset vertical state on failed reconnect
                    { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
                    { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
                     { // Clear live data
                         std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                         g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                         g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                         g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                         g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                     }
                    g_vertical_state.store(VerticalAlignState::IDLE);
                    break; // Exit the main while loop
                }
            } else {
                std::cerr << "Reconnection disabled. Stopping processing." << std::endl; // Logged
                currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Set status
                // Clear global GUI data and reset vertical state when stopping due to disabled reconnect
                { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
                { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
                 { // Clear live data
                     std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                     g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                     g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                     g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                     g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                 }
                g_vertical_state.store(VerticalAlignState::IDLE);
                break; // Exit the main while loop if reconnection is disabled
            }
        }

        // If no error, process data
        connection_error_occurred = false; // Clear error state if read was successful
        // Ensure status is CONNECTED if we are successfully reading data
        if (currentBridgeStatus.load() != BridgeConnectionStatus::CONNECTED) {
            currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED);
        }

        if (len > 0) { // Process received data
            received_data_buffer.append(read_buffer.data(), len);
            size_t newline_pos;
            while ((newline_pos = received_data_buffer.find('\n')) != std::string::npos) {
                std::string jsonData = received_data_buffer.substr(0, newline_pos);
                received_data_buffer.erase(0, newline_pos + 1);
                if (stopProcessingFlag.load()) break;
                if (!jsonData.empty()) {
                    try {
                        auto radarObjects = parseRadarData(jsonData);

                        // --- Update Global Data for GUI (Persistent Frame) ---
                        // Create temporary lists for the current frame
                        std::vector<RadarObject> tempBeaconObjects;
                        std::vector<RadarObject> tempAnonObjects;

                        for (const auto& obj : radarObjects) {
                            // Check if ID contains "BEACON" (case-sensitive) for the GUI display
                            if (obj.ID.find("BEACON") != std::string::npos) {
                                tempBeaconObjects.push_back(obj);
                            }
                            // Check if ID contains "anon" for the GUI display
                            else if (obj.ID.find("anon") != std::string::npos) {
                                tempAnonObjects.push_back(obj);
                            }
                        }

                        // Assign the temporary lists to the global lists under mutex protection
                        // Only update if the temp lists are not empty, to keep the last valid frame
                        if (!tempBeaconObjects.empty()) {
                            std::lock_guard<std::mutex> lockB(g_beaconMutex);
                            g_beaconObjects = std::move(tempBeaconObjects); // Move ownership
                        }
                        if (!tempAnonObjects.empty()) {
                            std::lock_guard<std::mutex> lockA(g_anonMutex);
                            g_anonObjects = std::move(tempAnonObjects); // Move ownership
                        }
                        // --- End Update Global Data ---


                        // Call appropriate processing function based on mode
                        switch (mode) {
                             // All wall/beacon/yaw/vertical modes now use the unified function
                            case ProcessingMode::WALL_FOLLOW:
                            case ProcessingMode::WALL_BEACON_YAW:
                            case ProcessingMode::WALL_BEACON_VERTICAL:
                            case ProcessingMode::WALL_BEACON_YAW_VERTICAL:
                                processRadarDataAndControl(mode, radarObjects, vehicle, enableControlCmd);
                                break;

                            case ProcessingMode::PROCESS_FULL:
                                // Need to update currentSecond for the display header
                                if (!radarObjects.empty()) {
                                    std::string ts_cleaned = radarObjects[0].timestamp;
                                    size_t dotPos = ts_cleaned.find('.');
                                    std::string objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;
                                    if (objSecond != currentSecond) {
                                        currentSecond = objSecond; // Update global second tracker
                                    }
                                }
                                displayRadarObjects(radarObjects); // Direct call for this mode
                                break;
                        }

                    } catch (const std::exception& e) {
                         std::cerr << "Error processing data: " << e.what() << "\nSnippet: [" << jsonData.substr(0, 100) << "...]" << std::endl; // Logged
                    }
                }
            }
            if (stopProcessingFlag.load()) break;
        } else {
             // If len is 0 and no error, it might just be a brief pause in data.
             // Add a small sleep to prevent busy-waiting in such cases.
             std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } // End main data reading loop

    // --- Cleanup (Keep) ---
    if (stopProcessingFlag.load()) std::cout << "[Processing Thread] Stop requested manually." << std::endl; // Logged
    else if (connection_error_occurred) std::cout << "[Processing Thread] Exiting due to unrecoverable connection error or forced stop." << std::endl; // Logged (Updated message)
    else std::cout << "[Processing Thread] Data stream ended gracefully (or loop exited)." << std::endl; // Logged (Updated message)

    if (socket.is_open()) { socket.close(); }
    stopPythonBridge(bridgeScriptName); // Stop the correct script

    // Clear global GUI data and reset vertical state on thread exit
    { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
    { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
     { // Clear live data
         std::lock_guard<std::mutex> lockL(g_liveDataMutex);
         g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
         g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
         g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
         g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
     }
    g_vertical_state.store(VerticalAlignState::IDLE); // Reset state

    // Release Control Authority if enabled AND vehicle objects are valid
    if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
        std::cout << "[Processing Thread] Sending final zero velocity command..." << std::endl; // Logged
        uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
        DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
        vehicle->control->flightCtrl(stopData);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "[Processing Thread] Releasing control authority..." << std::endl; // Logged
        ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
        // Only log error if it's a real error, not just "not obtained"
        if (ACK::getError(releaseAck) && releaseAck.info.cmd_set != 0x00 && releaseAck.info.cmd_id != 0x00) {
             ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority (on exit)"); // Logged via ACK
             std::cerr << "[Processing Thread] Warning: Failed to release control authority on exit." << std::endl; // Logged
        } else {
             std::cout << "[Processing Thread] Control authority released (or was not held)." << std::endl; // Logged
        }
    }
    std::cout << "Processing thread finished." << std::endl; // Logged
    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Ensure status is DISCONNECTED when thread ends
}


// Helper function to get mode name string (Keep for monitoring and GUI)
std::string getModeName(uint8_t mode) {
    switch(mode) {
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL: return "MANUAL_CTRL";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE: return "ATTITUDE";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS: return "P_GPS";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL: return "NAVI_SDK_CTRL"; // Value is 17
        case 31: return "Mode 31"; // Example of an unknown/other mode
        case 255: return "N/A"; // Handle our initial/unavailable state
        default: return "Other (" + std::to_string(mode) + ")";
    }
}

// Background thread function for monitoring (Corrected Scope Error)
void monitoringLoopFunction(Vehicle* vehicle, bool enableFlightControl) { // Added enableFlightControl parameter
    // Ensure vehicle pointer is valid before starting loop
    if (vehicle == nullptr || vehicle->subscribe == nullptr) {
        std::cerr << "[Monitoring] Error: Invalid Vehicle object provided. Thread exiting." << std::endl;
        g_current_display_mode.store(255); // Set to unavailable on error
        return;
    }

    std::cout << "[Monitoring] Thread started." << std::endl; // Logged
    bool telemetry_timed_out = false;
    bool warned_unexpected_status = false;
    uint8_t previous_flight_status = DJI::OSDK::VehicleStatus::FlightStatus::STOPED;
    time_t last_valid_poll_time = 0;
    bool in_sdk_control_mode = false;
    bool warned_not_in_sdk_mode = false;
    const uint8_t EXPECTED_SDK_MODE = DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL;

    while (!stopMonitoringFlag.load()) {
        // Re-check pointers inside loop in case they become invalid (though unlikely with current structure)
        if (vehicle == nullptr || vehicle->subscribe == nullptr) {
             if (!telemetry_timed_out) { // Avoid spamming log
                 std::cerr << "\n**** MONITORING ERROR: Vehicle/subscribe object became null. Stopping. ****" << std::endl << std::endl; // Logged
                 telemetry_timed_out = true;
             }
             g_current_display_mode.store(255); // Set to unavailable on error
             break; // Exit if vehicle objects become invalid
        }

        // Read telemetry data
        uint8_t current_flight_status = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
        uint8_t current_display_mode = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        float current_height = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_HEIGHT_FUSION>(); // Read height here too if needed for other monitoring
        bool valid_poll = (current_flight_status <= DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR); // Basic validity check

        // --- UPDATE GLOBAL DISPLAY MODE ---
        g_current_display_mode.store(current_display_mode);
        // --- END UPDATE ---

        // --- Update Live Data Altitude (if valid and processing thread NOT active OR control disabled) ---
        // Only update altitude from here if the processing thread isn't doing it
        if (valid_poll && !std::isnan(current_height) && current_height >= 0) {
             // Use the passed enableFlightControl parameter
             if (!processingThread.joinable() || !enableFlightControl) { // Check if processing thread is NOT running or control is disabled
                 std::lock_guard<std::mutex> lock(g_liveDataMutex);
                 g_latest_altitude = current_height;
             }
        }
        // --- End Update Live Data Altitude ---


        if (valid_poll) {
            time_t current_time = std::time(nullptr);
            last_valid_poll_time = current_time;
            if (telemetry_timed_out) {
                 std::cout << "[Monitoring] Telemetry poll recovered." << std::endl; // Logged
                 telemetry_timed_out = false;
            }

            // Check Flight Status Change (e.g., unexpected landing)
            if (current_flight_status != DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) {
                if ( (previous_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
                     (current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND || current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::STOPED) &&
                     !warned_unexpected_status) {
                     std::cerr << "\n**** MONITORING WARNING: Flight status changed unexpectedly from IN_AIR to " << (int)current_flight_status << ". ****" << std::endl << std::endl; // Logged
                     warned_unexpected_status = true;
                     // If status changes unexpectedly while in a vertical mode, revert state
                     if (g_vertical_state.load() != VerticalAlignState::IDLE) {
                         std::cerr << "Warning: Flight status changed during vertical maneuver. Reverting to IDLE." << std::endl;
                         g_vertical_state.store(VerticalAlignState::IDLE);
                         // Optionally signal the processing thread to stop? Or just let it handle the IDLE state.
                     }
                }
            } else {
                 warned_unexpected_status = false; // Reset once back in air
            }
            previous_flight_status = current_flight_status;

            // Check Expected SDK Mode (only warn if control is supposed to be active)
            bool is_expected_mode = (current_display_mode == EXPECTED_SDK_MODE);
            if (is_expected_mode) {
                if (!in_sdk_control_mode) {
                    std::cout << "\n**** MONITORING INFO: Entered SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << " / " << (int)EXPECTED_SDK_MODE << ") ****" << std::endl << std::endl; // Logged
                    in_sdk_control_mode = true;
                    warned_not_in_sdk_mode = false;
                }
            } else { // Not in expected mode
                 // Only warn if we were previously in SDK mode or haven't warned yet, AND the processing thread is active (implying control *should* be active)
                 if ((in_sdk_control_mode || !warned_not_in_sdk_mode) && !stopProcessingFlag.load() && processingThread.joinable()) {
                      std::string current_mode_name = getModeName(current_display_mode);
                      std::cerr << "\n**** MONITORING WARNING: NOT in expected SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << "). Current: " << current_mode_name << " (" << (int)current_display_mode << ") ****" << std::endl << std::endl; // Logged
                      warned_not_in_sdk_mode = true;
                      // If mode changes unexpectedly while in a vertical mode, revert state
                       if (g_vertical_state.load() != VerticalAlignState::IDLE) {
                           std::cerr << "Warning: Display mode changed during vertical maneuver. Reverting to IDLE." << std::endl;
                           g_vertical_state.store(VerticalAlignState::IDLE);
                           // Optionally signal the processing thread to stop?
                       }
                 }
                 in_sdk_control_mode = false;
            }
        } else { // Invalid poll data received
            if (!telemetry_timed_out) { // Avoid spamming log
                 std::cerr << "\n**** MONITORING WARNING: Polling telemetry returned potentially invalid data (FlightStatus=" << (int)current_flight_status << "). ****" << std::endl << std::endl; // Logged
                 telemetry_timed_out = true;
            }
             g_current_display_mode.store(255); // Set to unavailable on invalid poll
        }

        // Check Telemetry Timeout
        time_t current_time_for_timeout_check = std::time(nullptr);
        if (last_valid_poll_time > 0 && (current_time_for_timeout_check - last_valid_poll_time > TELEMETRY_TIMEOUT_SECONDS)) {
            if (!telemetry_timed_out) { // Avoid spamming log
                std::cerr << "\n**** MONITORING TIMEOUT: No valid telemetry for over " << TELEMETRY_TIMEOUT_SECONDS << " seconds. ****" << std::endl << std::endl; // Logged
                telemetry_timed_out = true;
                 // If telemetry times out during vertical maneuver, revert state
                 if (g_vertical_state.load() != VerticalAlignState::IDLE) {
                     std::cerr << "Warning: Telemetry timeout during vertical maneuver. Reverting to IDLE." << std::endl;
                     g_vertical_state.store(VerticalAlignState::IDLE);
                     // Optionally signal the processing thread to stop?
                 }
            }
            g_current_display_mode.store(255); // Set to unavailable on timeout
        } else if (last_valid_poll_time == 0) { // Check if never received first poll
             // Use a static variable to track start time only once
             static time_t start_time = 0; if (start_time == 0) start_time = current_time_for_timeout_check;
             if (current_time_for_timeout_check - start_time > TELEMETRY_TIMEOUT_SECONDS * 2) { // Allow double timeout initially
                  if (!telemetry_timed_out) { // Avoid spamming log
                       std::cerr << "\n**** MONITORING TIMEOUT: Never received valid telemetry poll after " << TELEMETRY_TIMEOUT_SECONDS * 2 << " seconds. ****" << std::endl << std::endl; // Logged
                       telemetry_timed_out = true;
                  }
                  g_current_display_mode.store(255); // Set to unavailable on initial timeout
             }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1)); // Poll frequency
    }
    std::cout << "[Monitoring] Thread finished." << std::endl; // Logged
    g_current_display_mode.store(255); // Set to unavailable when thread stops
}


// Helper function to stop the processing thread if it's running
void stopProcessingThreadIfNeeded() {
    if (processingThread.joinable()) {
        std::cout << "[GUI] Signalling processing thread to stop..." << std::endl; // Logged
        stopProcessingFlag.store(true); // Signal normal stop first
        forceStopReconnectionFlag.store(true); // Also signal forced stop for reconnection loop
        processingThread.join();
        std::cout << "[GUI] Processing thread finished." << std::endl; // Logged
        stopProcessingFlag.store(false); // Reset flag for next potential thread start
        forceStopReconnectionFlag.store(false); // Reset flag
        // Status is set to DISCONNECTED inside the thread function upon exit
        // Global data is cleared inside thread function upon exit (including live data)
        // Vertical state is reset inside thread function upon exit
    } else {
         std::cout << "[GUI] No processing thread currently running." << std::endl; // Logged
         // Ensure status is DISCONNECTED if no thread is running
         currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
         // Clear global data if no thread was running (e.g., on startup quit)
         { std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear(); }
         { std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear(); }
          { // Clear live data
             std::lock_guard<std::mutex> lockL(g_liveDataMutex);
             g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
             g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
             g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
             g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
         }
         g_vertical_state.store(VerticalAlignState::IDLE); // Ensure reset even if no thread was running
    }
}


int main(int argc, char** argv) {
    // --- Instantiate LogRedirector FIRST ---
    LogRedirector logger("run_log.txt");
    std::cout << "[Main] LogRedirector instantiated." << std::endl;

    std::cout << "Starting application: " << (argc > 0 ? argv[0] : "djiosdk-flightcontrol-gui") << std::endl; // Logged (Updated name)

    std::cout << "[Main] Calling loadPreferences()..." << std::endl;
    loadPreferences(); // Load preferences next (including independent sensor selections)
    std::cout << "[Main] Returned from loadPreferences()." << std::endl;

    // --- OSDK Initialization (Now Conditional) ---
    bool enableFlightControl = false; // Default to false, set true only if connection succeeds
    int functionTimeout = 1;
    Vehicle* vehicle = nullptr;
    LinuxSetup* linuxEnvironment = nullptr;
    int telemetrySubscriptionFrequency = 10; // Hz
    int pkgIndex = 0;
    bool monitoringEnabled = false;

    if (connect_to_drone) {
        std::cout << "[Main] Attempting to initialize DJI OSDK (connect_to_drone is true)..." << std::endl;
        linuxEnvironment = new LinuxSetup(argc, argv);
        vehicle = linuxEnvironment->getVehicle();

        if (vehicle == nullptr || vehicle->control == nullptr || vehicle->subscribe == nullptr) {
            std::cerr << "ERROR: Vehicle not initialized or interfaces unavailable. Flight control disabled." << std::endl;
            if (linuxEnvironment) { delete linuxEnvironment; linuxEnvironment = nullptr; }
            vehicle = nullptr;
            g_current_display_mode.store(255); // Ensure unavailable status if OSDK fails
            // enableFlightControl remains false
        } else {
            std::cout << "[Main] OSDK Vehicle instance OK." << std::endl;
            enableFlightControl = true; // Connection successful, enable flight control flag

            // Setup Telemetry Subscription for Monitoring (ADDED HEIGHT_FUSION)
            std::cout << "Setting up Telemetry Subscription for Monitoring... Freq: " << telemetrySubscriptionFrequency << " Hz" << std::endl;
            ACK::ErrorCode subscribeAck = vehicle->subscribe->verify(functionTimeout);
            if (ACK::getError(subscribeAck)) {
                 ACK::getErrorCodeMessage(subscribeAck, __func__);
                 std::cerr << "Error verifying subscription package list. Monitoring will be disabled." << std::endl;
                 g_current_display_mode.store(255); // Ensure unavailable status
            } else {
                 Telemetry::TopicName topicList[] = {
                     Telemetry::TOPIC_STATUS_FLIGHT,
                     Telemetry::TOPIC_STATUS_DISPLAYMODE,
                     Telemetry::TOPIC_HEIGHT_FUSION // ADDED ALTITUDE DATA
                 };
                 int numTopic = sizeof(topicList) / sizeof(topicList[0]); // Now 3 topics
                 bool topicStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList, false, telemetrySubscriptionFrequency);

                 if (topicStatus) {
                       std::cout << "Successfully initialized telemetry package " << pkgIndex << " with " << numTopic << " topics." << std::endl;
                       ACK::ErrorCode startAck = vehicle->subscribe->startPackage(pkgIndex, functionTimeout);
                       if (ACK::getError(startAck)) {
                            ACK::getErrorCodeMessage(startAck, "startPackage");
                            std::cerr << "Error starting subscription package " << pkgIndex << ". Monitoring disabled." << std::endl;
                            vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
                            g_current_display_mode.store(255); // Ensure unavailable status
                       } else {
                            std::cout << "Successfully started telemetry package " << pkgIndex << "." << std::endl;
                            std::cout << "Starting monitoring thread..." << std::endl;
                            stopMonitoringFlag.store(false);
                            // Pass enableFlightControl to the monitoring thread
                            monitoringThread = std::thread(monitoringLoopFunction, vehicle, enableFlightControl);
                            monitoringEnabled = true;
                            // g_current_display_mode will be updated by the thread
                       }
                 } else {
                      std::cerr << "Error initializing telemetry package " << pkgIndex << ". Monitoring disabled." << std::endl;
                      g_current_display_mode.store(255); // Ensure unavailable status
                 }
            }
        }
    } else {
        std::cout << "[Main] Skipping OSDK Initialization (connect_to_drone is false). Flight control disabled." << std::endl;
        g_current_display_mode.store(255); // Ensure unavailable status
        // vehicle remains nullptr, enableFlightControl remains false
    }

    std::cout << "INFO: Flight control is " << (enableFlightControl ? "ENABLED" : "DISABLED")
              << ". Bridge Reconnection: " << std::boolalpha << enable_bridge_reconnection
              << ". Local Test Script: " << useLocalBridgeScript // NEW Log
              << ". Connect to Drone: " << connect_to_drone << "." << std::endl; // Added connect_to_drone status

    // --- GUI Initialization ---
    std::cout << "[GUI] Initializing SDL..." << std::endl;
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        // Cleanup OSDK if partially initialized
        if (linuxEnvironment) delete linuxEnvironment;
        return 1;
    }

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#elif defined(__APPLE__)
    // GL 3.2 Core + GLSL 150
    const char* glsl_version = "#version 150";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif

    // Create window with graphics context
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("DJI OSDK Control GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
    if (!window) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
         if (linuxEnvironment) delete linuxEnvironment;
        return 1;
    }
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) {
        std::cerr << "SDL_GL_CreateContext Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
         if (linuxEnvironment) delete linuxEnvironment;
        return 1;
    }
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Initialize OpenGL loader (GLEW)
    std::cout << "[GUI] Initializing OpenGL Loader (GLEW)..." << std::endl;
    glewExperimental = GL_TRUE; // Needed for core profile
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW: " << glewGetErrorString(err) << std::endl;
        SDL_GL_DeleteContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();
         if (linuxEnvironment) delete linuxEnvironment;
        return 1;
    }
    std::cout << "[GUI] Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    std::cout << "[GUI] OpenGL Version: " << glGetString(GL_VERSION) << std::endl; // Check version after loader init


    // Setup Dear ImGui context
    std::cout << "[GUI] Initializing ImGui..." << std::endl;
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    // --- Load a slightly larger font if needed, or rely on scaling ---
    // io.Fonts->AddFontFromFileTTF("path/to/your/font.ttf", 16.0f); // Example

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context); // Use correct function for OpenGL
    ImGui_ImplOpenGL3_Init(glsl_version);

    // GUI state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    bool guiKeepRunning = true;

    // Buffer for Target Beacon ID Input
    char target_beacon_id_buffer[128]; // Adjust size as needed
    strncpy(target_beacon_id_buffer, TARGET_BEACON_ID.c_str(), sizeof(target_beacon_id_buffer) - 1);
    target_beacon_id_buffer[sizeof(target_beacon_id_buffer) - 1] = '\0'; // Ensure null termination

    // --- Window Size Variables ---
    const float side_window_width = 400.0f;    // Define width for side windows (Menu, Detections)
    const float bottom_window_height = 240.0f; // Define height for bottom windows

    // --- Color Constants ---
    const ImVec4 COLOR_GREEN = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
    const ImVec4 COLOR_RED = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    const ImVec4 COLOR_ORANGE = ImVec4(1.0f, 0.65f, 0.0f, 1.0f);
    const ImVec4 COLOR_GREY = ImVec4(0.7f, 0.7f, 0.7f, 1.0f); // For N/A status


    std::cout << "[GUI] Entering main GUI loop..." << std::endl;
    // --- Main GUI Loop ---
    while (guiKeepRunning) {
        // Poll and handle events (inputs, window resize, etc.)
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event); // Process SDL events for ImGui
            if (event.type == SDL_QUIT)
                guiKeepRunning = false; // Catch window close event
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
                guiKeepRunning = false; // Also catch explicit window close event
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(); // Start ImGui frame for SDL
        ImGui::NewFrame();

        // --- GUI Windows ---
        ImGuiViewport* viewport = ImGui::GetMainViewport(); // Get main viewport once for all windows
        const float padding = 10.0f; // Padding from window edges
        ImVec2 work_pos = viewport->WorkPos; // Top-left of usable area
        ImVec2 work_size = viewport->WorkSize; // Size of usable area

        // --- Status Text Variables (Used for the new status window) ---
        const char* radarStatusText = "";
        ImVec4 radarStatusColor = COLOR_RED; // Default Red

        // --- Determine Radar Status Text and Color ---
        {
            BridgeConnectionStatus status = currentBridgeStatus.load();
            switch(status) {
                case BridgeConnectionStatus::CONNECTED:
                    radarStatusText = "Radar: Connected";
                    radarStatusColor = COLOR_GREEN;
                    break;
                case BridgeConnectionStatus::DISCONNECTED:
                    radarStatusText = "Radar: Disconnected";
                    radarStatusColor = COLOR_RED;
                    break;
                case BridgeConnectionStatus::RECONNECTING:
                    radarStatusText = "Radar: Attempting Reconnect...";
                    radarStatusColor = COLOR_ORANGE;
                    break;
            }
        }
        // --- END Radar Status Text Determination ---

        // --- Drone Mode Status Variables ---
        std::string droneModeText = "Mode: N/A";
        ImVec4 droneModeColor = COLOR_GREY;
        // --- Determine Drone Mode Text and Color ---
        {
            uint8_t mode_val = g_current_display_mode.load();
            if (connect_to_drone && vehicle != nullptr) { // Only show real mode if drone connection was attempted and successful
                 droneModeText = "Mode: " + getModeName(mode_val);
                 if (mode_val == DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL) { // Check for SDK control mode (value 17)
                     droneModeColor = COLOR_GREEN;
                 } else if (mode_val == 255) { // Unknown/Unavailable
                      droneModeColor = COLOR_GREY;
                 } else { // Any other mode
                     droneModeColor = COLOR_RED;
                 }
            } else {
                // Keep default "Mode: N/A" and grey color if not connected to drone
            }
        }
        // --- END Drone Mode Status Determination ---


        // --- REMOVED StatusIndicator Window Block ---


        // 1. Main Menu Window (Top Left - Fixed Width) - REORDERED BUTTONS
        {
            ImVec2 window_pos = ImVec2(work_pos.x + padding, work_pos.y + padding);
            ImVec2 window_pos_pivot = ImVec2(0.0f, 0.0f); // Pivot at top-left
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot); // Pin to top-left
            // SET FIXED WIDTH
            ImGui::SetNextWindowSize(ImVec2(side_window_width, 0), ImGuiCond_Always); // Fixed width, auto height
            ImGui::SetNextWindowBgAlpha(0.65f);
            // REMOVED AlwaysAutoResize Flag
            ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

            ImGui::Begin("Main Menu", nullptr, window_flags);

            // Check if a processing thread is active
            bool is_processing_active = processingThread.joinable();
            std::string script_to_use = useLocalBridgeScript ? localPythonBridgeScript : defaultPythonBridgeScript; // Determine script based on flag

            // --- Action Buttons (Disable if processing is active) ---
            if (is_processing_active) ImGui::BeginDisabled();

            // --- Wall+Lateral Beacon Button --- (WAS [w])
            if (!enableFlightControl) ImGui::BeginDisabled(); // Also disable if flight control not available
            if (ImGui::Button("Wall+Lateral Beacon")) {
                std::cout << "[GUI] 'Wall+Lateral Beacon' button clicked." << std::endl;
                stopProcessingThreadIfNeeded(); // Stop previous thread if running
                forceStopReconnectionFlag.store(false);
                 // Clear lists and live data before starting
                {
                    std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear();
                    std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear();
                    std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                    g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                    g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                    g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                    g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                }
                if (enableFlightControl && vehicle != nullptr && vehicle->control != nullptr) {
                    std::cout << "[GUI] Attempting to obtain Control Authority for Wall+Lateral Beacon..." << std::endl;
                    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                    if (ACK::getError(ctrlAuthAck)) {
                        ACK::getErrorCodeMessage(ctrlAuthAck, "[GUI] Wall+Lateral Beacon obtainCtrlAuthority");
                        std::cerr << "[GUI] Failed to obtain control authority. Cannot start with control." << std::endl;
                        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                    } else {
                        std::cout << "[GUI] Obtained Control Authority." << std::endl;
                        stopProcessingFlag.store(false); // Ensure stop flag is false before starting
                        processingThread = std::thread(processingLoopFunction, script_to_use, vehicle, true, ProcessingMode::WALL_FOLLOW); // Pass chosen script
                    }
                } else {
                     std::cerr << "[GUI] Cannot start with control: Flight control disabled or OSDK not ready." << std::endl;
                     currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                }
            }
            if (!enableFlightControl) ImGui::EndDisabled();

            // --- Wall+Lateral Beacon+Vertical Button --- (NEW Name)
            if (!enableFlightControl) ImGui::BeginDisabled(); // Also disable if flight control not available
            if (ImGui::Button("Wall+Lateral Beacon+Vertical")) {
                std::cout << "[GUI] 'Wall+Lateral Beacon+Vertical' button clicked." << std::endl;
                stopProcessingThreadIfNeeded(); // Stop previous thread if running
                forceStopReconnectionFlag.store(false);
                // Clear lists and live data before starting
                {
                    std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear();
                    std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear();
                    std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                    g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                    g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                    g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                    g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                }
                if (enableFlightControl && vehicle != nullptr && vehicle->control != nullptr && vehicle->subscribe != nullptr) { // Check subscribe needed for altitude
                    std::cout << "[GUI] Attempting to obtain Control Authority for Wall+Lateral Beacon+Vertical..." << std::endl;
                    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                    if (ACK::getError(ctrlAuthAck)) {
                        ACK::getErrorCodeMessage(ctrlAuthAck, "[GUI] Wall+Lateral Beacon+Vertical obtainCtrlAuthority");
                        std::cerr << "[GUI] Failed to obtain control authority. Cannot start with control." << std::endl;
                        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                    } else {
                        std::cout << "[GUI] Obtained Control Authority." << std::endl;
                        stopProcessingFlag.store(false); // Ensure stop flag is false before starting
                        processingThread = std::thread(processingLoopFunction, script_to_use, vehicle, true, ProcessingMode::WALL_BEACON_VERTICAL); // Pass chosen script
                    }
                } else {
                     std::cerr << "[GUI] Cannot start with control: Flight control disabled, OSDK not ready, or Telemetry unavailable." << std::endl;
                     currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                }
            }
            if (!enableFlightControl) ImGui::EndDisabled();

            // --- Wall+Lateral Beacon+Yaw Button --- (NEW Name)
            if (!enableFlightControl) ImGui::BeginDisabled(); // Also disable if flight control not available
            if (ImGui::Button("Wall+Lateral Beacon+Yaw")) {
                std::cout << "[GUI] 'Wall+Lateral Beacon+Yaw' button clicked." << std::endl;
                stopProcessingThreadIfNeeded(); // Stop previous thread if running
                forceStopReconnectionFlag.store(false);
                 // Clear lists and live data before starting
                {
                    std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear();
                    std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear();
                    std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                    g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                    g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                    g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                    g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                }
                if (enableFlightControl && vehicle != nullptr && vehicle->control != nullptr) {
                    std::cout << "[GUI] Attempting to obtain Control Authority for Wall+Lateral Beacon+Yaw..." << std::endl;
                    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                    if (ACK::getError(ctrlAuthAck)) {
                        ACK::getErrorCodeMessage(ctrlAuthAck, "[GUI] Wall+Lateral Beacon+Yaw obtainCtrlAuthority");
                        std::cerr << "[GUI] Failed to obtain control authority. Cannot start with control." << std::endl;
                        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                    } else {
                        std::cout << "[GUI] Obtained Control Authority." << std::endl;
                        stopProcessingFlag.store(false); // Ensure stop flag is false before starting
                        // Start the thread with the WALL_BEACON_YAW mode
                        processingThread = std::thread(processingLoopFunction, script_to_use, vehicle, true, ProcessingMode::WALL_BEACON_YAW); // Pass chosen script
                    }
                } else {
                     std::cerr << "[GUI] Cannot start with control: Flight control disabled or OSDK not ready." << std::endl;
                     currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                }
            }
             if (!enableFlightControl) {
                ImGui::EndDisabled();
                // ImGui::SameLine(); // Remove sameline if buttons stack vertically
                // ImGui::TextDisabled("(Flight Control Disabled)"); // comment removed
            }

            // --- Wall+Lateral Beacon+Yaw+Vertical Button --- (NEW Name)
            if (!enableFlightControl) ImGui::BeginDisabled(); // Also disable if flight control not available
            if (ImGui::Button("Wall+Lateral Beacon+Yaw+Vertical")) {
                std::cout << "[GUI] 'Wall+Lateral Beacon+Yaw+Vertical' button clicked." << std::endl;
                stopProcessingThreadIfNeeded(); // Stop previous thread if running
                forceStopReconnectionFlag.store(false);
                 // Clear lists and live data before starting
                {
                    std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear();
                    std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear();
                    std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                    g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                    g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                    g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                    g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                }
                if (enableFlightControl && vehicle != nullptr && vehicle->control != nullptr && vehicle->subscribe != nullptr) { // Check subscribe needed for altitude
                    std::cout << "[GUI] Attempting to obtain Control Authority for Wall+Lateral Beacon+Yaw+Vertical..." << std::endl;
                    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                    if (ACK::getError(ctrlAuthAck)) {
                        ACK::getErrorCodeMessage(ctrlAuthAck, "[GUI] Wall+Lateral Beacon+Yaw+Vertical obtainCtrlAuthority");
                        std::cerr << "[GUI] Failed to obtain control authority. Cannot start with control." << std::endl;
                        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                    } else {
                        std::cout << "[GUI] Obtained Control Authority." << std::endl;
                        stopProcessingFlag.store(false); // Ensure stop flag is false before starting
                        processingThread = std::thread(processingLoopFunction, script_to_use, vehicle, true, ProcessingMode::WALL_BEACON_YAW_VERTICAL); // Pass chosen script
                    }
                } else {
                     std::cerr << "[GUI] Cannot start with control: Flight control disabled, OSDK not ready, or Telemetry unavailable." << std::endl;
                     currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                }
            }
            if (!enableFlightControl) {
                ImGui::EndDisabled();
                ImGui::SameLine(); // Keep text on same line for the last button pair
                ImGui::TextDisabled("(Flight Control Disabled)");
            }

            // --- Process Full Radar Button ---
            if (ImGui::Button("Process Full Radar [e]")) {
                 std::cout << "[GUI] 'e' button clicked." << std::endl;
                 stopProcessingThreadIfNeeded(); // Stop previous thread if running
                 forceStopReconnectionFlag.store(false);
                  // Clear lists and live data before starting
                 {
                     std::lock_guard<std::mutex> lockB(g_beaconMutex); g_beaconObjects.clear();
                     std::lock_guard<std::mutex> lockA(g_anonMutex); g_anonObjects.clear();
                     std::lock_guard<std::mutex> lockL(g_liveDataMutex);
                     g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
                     g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
                     g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
                     g_latest_altitude = std::numeric_limits<float>::quiet_NaN();
                 }
                 std::cout << "[GUI] Starting Radar Data processing (Full, No Control)..." << std::endl;
                 stopProcessingFlag.store(false); // Ensure stop flag is false before starting
                 processingThread = std::thread(processingLoopFunction, script_to_use, nullptr, false, ProcessingMode::PROCESS_FULL); // Pass chosen script
            }

            // End disable block for action buttons
            if (is_processing_active) ImGui::EndDisabled();

            ImGui::Separator(); // Add separator before stop buttons

            // --- Stop Reconnect Button ---
            if (enable_bridge_reconnection) {
                // Only enable if a thread is running AND it's in the RECONNECTING state
                bool should_enable_stop_reconnect = is_processing_active && (currentBridgeStatus.load() == BridgeConnectionStatus::RECONNECTING);
                if (!should_enable_stop_reconnect) ImGui::BeginDisabled();
                if (ImGui::Button("Stop Reconnection Attempt")) {
                    std::cout << "[GUI] 'Stop Reconnection Attempt' button clicked." << std::endl;
                    forceStopReconnectionFlag.store(true); // Signal the reconnect loop to stop trying
                }
                if (!should_enable_stop_reconnect) ImGui::EndDisabled();
                ImGui::SameLine(); // Keep Stop button on the same line if possible
            }

            // --- General Stop Button ---
            // Only enable if a processing thread is currently running
            if (!is_processing_active) ImGui::BeginDisabled();
            if (ImGui::Button("Stop Current Action")) {
                std::cout << "[GUI] 'Stop Current Action' button clicked." << std::endl;
                stopProcessingThreadIfNeeded(); // Call the function to stop the thread and handle cleanup
            }
            if (!is_processing_active) ImGui::EndDisabled();


            ImGui::End(); // End Main Menu
        }

        // --- NEW: Combined Status Window (Right of Main Menu) ---
        // Store its position and size for the Live Data window below
        ImVec2 status_window_pos;
        ImVec2 status_window_size;
        {
            // Calculate position: Top-left X is Main Menu's X + Width + Padding. Top-left Y is Main Menu's Y.
            status_window_pos = ImVec2(work_pos.x + padding + side_window_width + padding, work_pos.y + padding);
            ImVec2 window_pos_pivot = ImVec2(0.0f, 0.0f); // Pivot at top-left

            ImGui::SetNextWindowPos(status_window_pos, ImGuiCond_Always, window_pos_pivot); // Pin to the calculated position
            ImGui::SetNextWindowSize(ImVec2(0, 0)); // Auto-resize based on content
            ImGui::SetNextWindowBgAlpha(0.65f); // Match other windows' transparency

            // Flags for a simple status display: No title, no moving/resizing, auto-fit content
            ImGuiWindowFlags status_window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
                                                   ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

            ImGui::Begin("CombinedStatusWindow", nullptr, status_window_flags); // Use a unique name

            // Display Radar Status
            if (strlen(radarStatusText) > 0) { // Only display if there's text
                ImGui::TextColored(radarStatusColor, "%s", radarStatusText);
            } else {
                ImGui::Text("Radar Status Unknown"); // Fallback
            }

            // Display Drone Mode Status (on the next line)
            ImGui::TextColored(droneModeColor, "%s", droneModeText.c_str());

            // Display Vertical Align State (if active)
            VerticalAlignState current_v_state = g_vertical_state.load();
            if (current_v_state != VerticalAlignState::IDLE) {
                 ImGui::Text("Vertical State: %s", getVerticalStateName(current_v_state).c_str());
            }

            status_window_size = ImGui::GetWindowSize(); // Get the size after content is drawn
            ImGui::End();
        }
        // --- END NEW Combined Status Window ---

        // --- NEW: Live Data Window (Below Combined Status) ---
        {
             // Calculate position: X is same as Status window, Y is Status window Y + Status window Height + Padding
             ImVec2 live_data_pos = ImVec2(status_window_pos.x, status_window_pos.y + status_window_size.y + padding);
             ImVec2 window_pos_pivot = ImVec2(0.0f, 0.0f); // Pivot top-left

             ImGui::SetNextWindowPos(live_data_pos, ImGuiCond_Always, window_pos_pivot);
             ImGui::SetNextWindowSize(ImVec2(0, 0)); // Auto-size
             ImGui::SetNextWindowBgAlpha(0.65f);

             ImGuiWindowFlags live_data_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
                                                ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

             ImGui::Begin("LiveDataWindow", nullptr, live_data_flags);

             // Read data under lock
             float latest_beacon_range, latest_wall_dist, latest_alt;
             std::chrono::steady_clock::time_point last_beacon_time;
             {
                 std::lock_guard<std::mutex> lock(g_liveDataMutex);
                 latest_beacon_range = g_latest_beacon_range;
                 latest_wall_dist = g_latest_wall_horizontal_distance;
                 latest_alt = g_latest_altitude;
                 last_beacon_time = g_last_beacon_seen_time;
             }

             // Calculate time since beacon seen
             std::string time_since_beacon_str = "N/A";
             if (last_beacon_time != std::chrono::steady_clock::time_point::min()) {
                  auto time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - last_beacon_time);
                  std::stringstream ss;
                  ss << std::fixed << std::setprecision(1) << time_diff.count() << " s";
                  time_since_beacon_str = ss.str();
             }

             // Display data
             ImGui::Text("Time since Beacon: %s", time_since_beacon_str.c_str());

             if (std::isnan(latest_beacon_range)) {
                 ImGui::Text("Dist to Beacon: N/A");
             } else {
                 ImGui::Text("Dist to Beacon: %.3f m", latest_beacon_range);
             }

             if (std::isnan(latest_wall_dist)) {
                 ImGui::Text("Dist to Wall HZ: N/A");
             } else {
                 ImGui::Text("Dist to Wall HZ: %.3f m", latest_wall_dist);
             }

             if (std::isnan(latest_alt)) {
                  ImGui::Text("Altitude: N/A");
             } else {
                  ImGui::Text("Altitude: %.3f m", latest_alt);
             }

             ImGui::End();
        }
        // --- END NEW Live Data Window ---


        // 2. Parameters Window (Top Right - Final Layout Adjustments)
        {
            ImVec2 window_pos = ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - padding, viewport->WorkPos.y + padding); // Position from top-right corner of work area
            ImVec2 window_pos_pivot = ImVec2(1.0f, 0.0f); // Pivot at top-right
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot); // Use Always condition to pin
            ImGui::SetNextWindowSize(ImVec2(660, 0), ImGuiCond_Appearing); // Set initial width (INCREASED), height adjusts automatically
            ImGui::SetNextWindowBgAlpha(0.65f);
            ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav; // Removed AlwaysAutoResize

            ImGui::Begin("Parameters", nullptr, window_flags); // BEGIN Parameters Window

            // --- Status Section (Full Width) ---
            ImGui::Text("Status");
            ImGui::Separator();
            ImGui::Text("Connect to Drone: %s", connect_to_drone ? "True" : "False");
            ImGui::Text("Flight Control: %s", enableFlightControl ? "Enabled" : "Disabled");
            // Moved Bridge Reconnect Checkbox here
            if (ImGui::Checkbox("Bridge Reconnect", &enable_bridge_reconnection)) {
                 std::cout << "[GUI Param Update] Bridge Reconnect set to: " << std::boolalpha << enable_bridge_reconnection << std::endl;
            }
            // Added Local Test Checkbox here (NEW)
            if (ImGui::Checkbox("Local Test", &useLocalBridgeScript)) {
                 std::cout << "[GUI Param Update] Local Test set to: " << std::boolalpha << useLocalBridgeScript << std::endl;
            }
            ImGui::Separator();

            // --- Paired Sections using BeginGroup / SameLine ---
            float groupWidth = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f; // Recalculate group width dynamically
            float inputWidth = groupWidth * 0.45f; // Reduced input width

            // --- LEFT COLUMN ---
            ImGui::BeginGroup(); // Group 1 (Left Column) Start

            // Sensor Control (Wall Dist) - Simplified Labels
            ImGui::Text("Sensor Control (Wall Dist)");
             ImGui::Separator();
             if (ImGui::Checkbox("R_Az##W", &useWallSensorRAz)) { // Use ##W for unique ID
                  std::cout << "[GUI Param Update] Use Wall R_Az set to: " << std::boolalpha << useWallSensorRAz << std::endl;
             }
             ImGui::SameLine();
             if (ImGui::Checkbox("R_El##W", &useWallSensorREl)) {
                  std::cout << "[GUI Param Update] Use Wall R_El set to: " << std::boolalpha << useWallSensorREl << std::endl;
             }
             ImGui::SameLine();
             if (ImGui::Checkbox("R_Az_R_El##W", &useWallSensorRAzREl)) {
                  std::cout << "[GUI Param Update] Use Wall R_Az_R_El set to: " << std::boolalpha << useWallSensorRAzREl << std::endl;
             }
             ImGui::Spacing();

             // Sensor Control (Beacon Az) - Simplified Labels (NEW Section)
             ImGui::Text("Sensor Control (Beacon Az)");
             ImGui::Separator();
             if (ImGui::Checkbox("R_Az##BA", &useBeaconSensorRAz)) { // Use ##BA for unique ID
                 std::cout << "[GUI Param Update] Use Beacon Az R_Az set to: " << std::boolalpha << useBeaconSensorRAz << std::endl;
             }
             ImGui::SameLine();
             if (ImGui::Checkbox("R_El##BA", &useBeaconSensorREl)) {
                 std::cout << "[GUI Param Update] Use Beacon Az R_El set to: " << std::boolalpha << useBeaconSensorREl << std::endl;
             }
             ImGui::SameLine();
             if (ImGui::Checkbox("R_Az_R_El##BA", &useBeaconSensorRAzREl)) {
                 std::cout << "[GUI Param Update] Use Beacon Az R_Az_R_El set to: " << std::boolalpha << useBeaconSensorRAzREl << std::endl;
             }
             ImGui::Spacing();

             // Sensor Control (Beacon Rg) - Simplified Labels (NEW Section)
             ImGui::Text("Sensor Control (Beacon Rg)");
             ImGui::Separator();
             if (ImGui::Checkbox("R_Az##BR", &useBeaconRangeSensorRAz)) { // Use ##BR for unique ID
                 std::cout << "[GUI Param Update] Use Beacon Rg R_Az set to: " << std::boolalpha << useBeaconRangeSensorRAz << std::endl;
             }
             ImGui::SameLine();
             if (ImGui::Checkbox("R_El##BR", &useBeaconRangeSensorREl)) {
                 std::cout << "[GUI Param Update] Use Beacon Rg R_El set to: " << std::boolalpha << useBeaconRangeSensorREl << std::endl;
             }
             ImGui::SameLine();
             if (ImGui::Checkbox("R_Az_R_El##BR", &useBeaconRangeSensorRAzREl)) {
                 std::cout << "[GUI Param Update] Use Beacon Rg R_Az_R_El set to: " << std::boolalpha << useBeaconRangeSensorRAzREl << std::endl;
             }
             ImGui::Spacing();

             // Sensor Control (Yaw) - Simplified Labels
             ImGui::Text("Sensor Control (Yaw)");
             ImGui::Separator();
             if (ImGui::Checkbox("R_Az##Y", &useYawSensorRAz)) { // Add ##Y to make label unique
                 std::cout << "[GUI Param Update] Use Yaw R_Az set to: " << std::boolalpha << useYawSensorRAz << std::endl;
             }
             ImGui::SameLine();
             if (ImGui::Checkbox("R_El##Y", &useYawSensorREl)) {
                 std::cout << "[GUI Param Update] Use Yaw R_El set to: " << std::boolalpha << useYawSensorREl << std::endl;
             }
             ImGui::SameLine();
             if (ImGui::Checkbox("R_Az_R_El##Y", &useYawSensorRAzREl)) {
                 std::cout << "[GUI Param Update] Use Yaw R_Az_R_El set to: " << std::boolalpha << useYawSensorRAzREl << std::endl;
             }
             ImGui::Spacing();

            // Editable Parameters (Moved to bottom of left column) - Removed (deg)
            ImGui::Text("Editable Parameters");
            ImGui::Separator();
            ImGui::PushItemWidth(inputWidth); // Use reduced width
            if (ImGui::InputText("##TargetBeaconID", target_beacon_id_buffer, sizeof(target_beacon_id_buffer))) { // Use ## for hidden label
                TARGET_BEACON_ID = target_beacon_id_buffer;
                std::cout << "[GUI Param Update] Target Beacon ID set to: " << TARGET_BEACON_ID << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Target Beacon ID"); // Label after
            if (ImGui::InputFloat("##TargetHorizDist", &targetDistance, 0.1f, 1.0f, "%.3f")) {
                 std::cout << "[GUI Param Update] Target Horiz Distance set to: " << targetDistance << std::endl;
            }
             ImGui::SameLine(); ImGui::Text("Target Horiz Dist (m)");
            if (ImGui::InputFloat("##TargetAzimuth", &targetAzimuth, 1.0f, 10.0f, "%.3f")) {
                 std::cout << "[GUI Param Update] Target Azimuth set to: " << targetAzimuth << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Target Azimuth"); // Removed (deg)
            if (ImGui::InputFloat("##RadarMountAngle", &radarMountAngleDegrees, 1.0f, 5.0f, "%.2f")) {
                 std::cout << "[GUI Param Update] Radar Mount Angle set to: " << radarMountAngleDegrees << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Radar Mount Angle"); // Removed (deg)
            ImGui::PopItemWidth();

            ImGui::EndGroup();   // Group 1 (Left Column) End

            // --- RIGHT COLUMN ---
            ImGui::SameLine(); // Move cursor to the right for the next group
            ImGui::BeginGroup(); // Group 2 (Right Column) Start

            // REMOVED Connection Section Header/Separator

            // Forward Control
            ImGui::Text("Forward Control");
            ImGui::Separator();
            ImGui::PushItemWidth(inputWidth); // Use reduced width
            if (ImGui::InputFloat("##KpForward", &Kp_forward, 0.01f, 0.1f, "%.4f")) {
                 std::cout << "[GUI Param Update] Kp Forward set to: " << Kp_forward << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Kp Forward");
            if (ImGui::InputFloat("##MaxFwdSpeed", &max_forward_speed, 0.05f, 0.2f, "%.3f")) {
                 std::cout << "[GUI Param Update] Max Fwd Speed set to: " << max_forward_speed << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Max Fwd Speed (m/s)");
            if (ImGui::InputFloat("##FwdDeadZone", &forward_dead_zone, 0.01f, 0.1f, "%.3f")) {
                 std::cout << "[GUI Param Update] Fwd Dead Zone set to: " << forward_dead_zone << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Fwd Dead Zone (m)");
            ImGui::PopItemWidth();
            ImGui::Spacing();

            // Lateral Control - Removed (deg)
            ImGui::Text("Lateral Control");
            ImGui::Separator();
            ImGui::PushItemWidth(inputWidth); // Use reduced width
            if (ImGui::InputFloat("##KpLateral", &Kp_lateral, 0.001f, 0.01f, "%.5f")) {
                 std::cout << "[GUI Param Update] Kp Lateral set to: " << Kp_lateral << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Kp Lateral");
            if (ImGui::InputFloat("##MaxLatSpeed", &max_lateral_speed, 0.05f, 0.2f, "%.3f")) {
                 std::cout << "[GUI Param Update] Max Lat Speed set to: " << max_lateral_speed << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Max Lat Speed (m/s)");
            if (ImGui::InputFloat("##AzimuthDeadZone", &azimuth_dead_zone, 0.1f, 1.0f, "%.3f")) {
                 std::cout << "[GUI Param Update] Azimuth Dead Zone set to: " << azimuth_dead_zone << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Azimuth Dead Zone"); // Removed (deg)
            if (ImGui::Checkbox("##InvertLateral", &invertLateralControl)) {
                 std::cout << "[GUI Param Update] Invert Lateral Control set to: " << std::boolalpha << invertLateralControl << std::endl;
            }
            ImGui::SameLine(); ImGui::Text("Invert Lateral Control");
            ImGui::PopItemWidth();
            ImGui::Spacing();

            // Yaw Control - Removed (deg)
            ImGui::Text("Yaw Control");
             ImGui::Separator();
             ImGui::PushItemWidth(inputWidth); // Use reduced width
             if (ImGui::InputFloat("##KpYaw", &Kp_yaw, 0.001f, 0.01f, "%.5f")) {
                  std::cout << "[GUI Param Update] Kp Yaw set to: " << Kp_yaw << std::endl;
             }
             ImGui::SameLine(); ImGui::Text("Kp Yaw");
             if (ImGui::InputFloat("##MaxYawRate", &max_yaw_rate, 0.5f, 2.0f, "%.2f")) {
                  std::cout << "[GUI Param Update] Max Yaw Rate set to: " << max_yaw_rate << std::endl;
             }
              ImGui::SameLine(); ImGui::Text("Max Yaw Rate"); // Removed (deg/s) -> Simplified
             if (ImGui::InputFloat("##YawBalanceDeadZone", &yaw_azimuth_balance_dead_zone, 0.1f, 0.5f, "%.2f")) {
                  std::cout << "[GUI Param Update] Yaw Balance Dead Zone set to: " << yaw_azimuth_balance_dead_zone << std::endl;
             }
              ImGui::SameLine(); ImGui::Text("Yaw Balance Dead Zone"); // Removed (deg)
             if (ImGui::Checkbox("##InvertYaw", &invertYawControl)) {
                  std::cout << "[GUI Param Update] Invert Yaw Control set to: " << std::boolalpha << invertYawControl << std::endl;
             }
              ImGui::SameLine(); ImGui::Text("Invert Yaw Control");
             ImGui::PopItemWidth();
             ImGui::Spacing();

            // Vertical Control
             ImGui::Text("Vertical Control");
             ImGui::Separator();
             ImGui::PushItemWidth(inputWidth); // Use reduced width
             if (ImGui::InputFloat("##TargetAlt", &TARGET_ALTITUDE, 0.1f, 0.5f, "%.2f")) {
                  std::cout << "[GUI Param Update] Target Altitude set to: " << TARGET_ALTITUDE << std::endl;
             }
             ImGui::SameLine(); ImGui::Text("Target Alt (m AGL)");
             if (ImGui::InputFloat("##VertSpeed", &VERTICAL_SPEED, 0.05f, 0.2f, "%.2f")) {
                  std::cout << "[GUI Param Update] Vertical Speed set to: " << VERTICAL_SPEED << std::endl;
             }
             ImGui::SameLine(); ImGui::Text("Vertical Speed (m/s)");
             if (ImGui::InputFloat("##HoldDur", &HOLD_DURATION_SECONDS, 0.5f, 1.0f, "%.1f")) {
                  std::cout << "[GUI Param Update] Hold Duration set to: " << HOLD_DURATION_SECONDS << std::endl;
             }
             ImGui::SameLine(); ImGui::Text("Hold Duration (s)");
             ImGui::PopItemWidth();

            ImGui::EndGroup();   // Group 2 (Right Column) End

            ImGui::End(); // END Parameters Window
        }

        // 3. Anon Detections Window (Above Beacon Detections)
        {
            // Position near bottom-left, leaving space *below* it for the Beacon window
            ImVec2 window_pos = ImVec2(work_pos.x + padding, work_pos.y + work_size.y - padding - bottom_window_height - ImGui::GetStyle().WindowPadding.y * 2); // Adjusted Y
            ImVec2 window_pos_pivot = ImVec2(0.0f, 1.0f); // Pivot remains bottom-left of this window
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
            ImGui::SetNextWindowSize(ImVec2(side_window_width, bottom_window_height), ImGuiCond_Always); // Use fixed width & height
            ImGui::SetNextWindowBgAlpha(0.65f);
            ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

            ImGui::Begin("Anon Detections", nullptr, window_flags);
            ImGui::Text("Latest Anonymous Objects (ID contains 'anon')");
            ImGui::Separator();
            if (ImGui::BeginChild("AnonScrollRegion", ImVec2(0, 0), true)) { // Auto-size child region
                std::lock_guard<std::mutex> lock(g_anonMutex);
                if (g_anonObjects.empty()) {
                    ImGui::TextUnformatted("No anon data received yet or action stopped."); // Updated text
                } else {
                    for (size_t i = 0; i < g_anonObjects.size(); ++i) {
                        const auto& obj = g_anonObjects[i];
                        ImGui::Text("Anon #%zu:", i + 1);
                        ImGui::Indent();
                        ImGui::Text("TS: %s", obj.timestamp.c_str());
                        ImGui::Text("Sensor: %s, Src: %s", obj.sensor.c_str(), obj.src.c_str());
                        ImGui::Text("ID: %s", obj.ID.c_str());
                        ImGui::Text("Pos (X,Y,Z): %.3f, %.3f, %.3f", obj.X, obj.Y, obj.Z);
                        ImGui::Text("Dir (X,Y,Z): %.3f, %.3f, %.3f", obj.Xdir, obj.Ydir, obj.Zdir);
                        ImGui::Text("Range: %.3f, Rate: %.3f, Pwr: %.1f", obj.Range, obj.RangeRate, obj.Pwr);
                        ImGui::Text("Az: %.2f, El: %.2f, Conf: %.2f", obj.Az, obj.El, obj.Conf);
                        ImGui::Text("Size (X,Y,Z): %.3f, %.3f, %.3f", obj.Xsize, obj.Ysize, obj.Zsize);
                        ImGui::Unindent();
                        ImGui::Separator();
                    }
                }
            }
            ImGui::EndChild();
            ImGui::End();
        }

        // 4. Beacon Detections Window (Bottom Left)
        {
            // Position at the very bottom-left corner
            ImVec2 window_pos = ImVec2(work_pos.x + padding, work_pos.y + work_size.y - padding);
            ImVec2 window_pos_pivot = ImVec2(0.0f, 1.0f); // Pivot at bottom-left
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
            ImGui::SetNextWindowSize(ImVec2(side_window_width, bottom_window_height), ImGuiCond_Always); // Use fixed width & height
            ImGui::SetNextWindowBgAlpha(0.65f);
            ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

            ImGui::Begin("Beacon Detections", nullptr, window_flags);
            ImGui::Text("Latest Beacon Objects (ID contains 'BEACON')");
            ImGui::Separator();
            if (ImGui::BeginChild("BeaconScrollRegion", ImVec2(0, 0), true)) { // Auto-size child region
                std::lock_guard<std::mutex> lock(g_beaconMutex);
                if (g_beaconObjects.empty()) {
                    ImGui::TextUnformatted("No beacon data received yet or action stopped."); // Updated text
                } else {
                    for (size_t i = 0; i < g_beaconObjects.size(); ++i) {
                        const auto& obj = g_beaconObjects[i];
                        ImGui::Text("Beacon #%zu:", i + 1);
                        ImGui::Indent();
                        ImGui::Text("TS: %s", obj.timestamp.c_str());
                        ImGui::Text("Sensor: %s, Src: %s", obj.sensor.c_str(), obj.src.c_str());
                        ImGui::Text("ID: %s", obj.ID.c_str());
                        ImGui::Text("Pos (X,Y,Z): %.3f, %.3f, %.3f", obj.X, obj.Y, obj.Z);
                        ImGui::Text("Dir (X,Y,Z): %.3f, %.3f, %.3f", obj.Xdir, obj.Ydir, obj.Zdir);
                        ImGui::Text("Range: %.3f, Rate: %.3f, Pwr: %.1f", obj.Range, obj.RangeRate, obj.Pwr);
                        ImGui::Text("Az: %.2f, El: %.2f, Conf: %.2f", obj.Az, obj.El, obj.Conf);
                        ImGui::Text("Size (X,Y,Z): %.3f, %.3f, %.3f", obj.Xsize, obj.Ysize, obj.Zsize);
                        ImGui::Unindent();
                        ImGui::Separator();
                    }
                }
            }
            ImGui::EndChild();
            ImGui::End();
        }


        // --- Rendering ---

        // --- REMOVED direct drawing of status text to foreground ---

        ImGui::Render(); // Render all ImGui windows and draw lists
        int display_w, display_h;
        SDL_GetWindowSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    } // End GUI loop


    // --- Cleanup ---
    std::cout << "[GUI] Exited main GUI loop. Cleaning up..." << std::endl;

    // Stop threads cleanly
    stopProcessingThreadIfNeeded(); // Ensure processing thread is stopped (also resets vertical state)

    if (monitoringThread.joinable()) {
        std::cout << "[GUI] Signalling monitoring thread to stop..." << std::endl;
        stopMonitoringFlag.store(true);
        monitoringThread.join();
        std::cout << "[GUI] Monitoring thread finished." << std::endl;
        // monitoringEnabled = false; // Flag not strictly needed anymore
    }

    // OSDK Cleanup (Conditional)
    if (connect_to_drone && vehicle != nullptr) { // Only cleanup if connection was attempted and vehicle exists
        if (monitoringEnabled && vehicle->subscribe != nullptr) { // Check subscribe pointer too
            std::cout << "[GUI] Unsubscribing from telemetry..." << std::endl;
            ACK::ErrorCode statusAck = vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
            if(ACK::getError(statusAck)) {
                ACK::getErrorCodeMessage(statusAck, "[GUI] Cleanup");
            } else {
                std::cout << "[GUI] Telemetry unsubscribed." << std::endl;
            }
        }
        if (enableFlightControl && vehicle->control != nullptr) { // Check control pointer too
            std::cout << "[GUI] Releasing Control Authority on Quit (if obtained)..." << std::endl;
            ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
             if (ACK::getError(releaseAck) && releaseAck.info.cmd_set != 0x00 && releaseAck.info.cmd_id != 0x00) {
                 ACK::getErrorCodeMessage(releaseAck, "[GUI] Quit releaseCtrlAuthority");
             } else {
                 std::cout << "[GUI] Control Authority Released on Quit (or was not held)." << std::endl;
             }
        }
    }
    // Delete linuxEnvironment only if it was created
    if (linuxEnvironment != nullptr) {
        std::cout << "[GUI] Deleting linuxEnvironment..." << std::endl;
        delete linuxEnvironment; linuxEnvironment = nullptr;
        vehicle = nullptr; // Pointer now invalid
    }

    // ImGui Cleanup
    std::cout << "[GUI] Cleaning up ImGui..." << std::endl;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown(); // Use correct shutdown for SDL backend
    ImGui::DestroyContext();

    // SDL Cleanup
    std::cout << "[GUI] Cleaning up SDL..." << std::endl;
    SDL_GL_DeleteContext(gl_context); // Corrected variable name
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::cout << "Application finished with exit code: 0" << std::endl;
    return 0;
    // LogRedirector destructor runs here
}
