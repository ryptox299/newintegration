#include "config.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits> // Required for numeric_limits
#include <string> // Required for std::string::npos

// --- Default Values ---
bool connect_to_drone = false; // Default to false (safer)
bool enable_camera_stream = true; // Default to true to maintain current behavior
std::string defaultPythonBridgeScript = "python_bridge.py";
std::string localPythonBridgeScript = "python_bridge_local_test.py";
std::atomic<bool> useLocalBridgeScript{false};
std::atomic<bool> enable_bridge_reconnection{true}; // Default to true

float targetDistance = 2.0f;
float targetAzimuth = 0.0f;
float radarMountAngleDegrees = 0.0f; // Default to horizontal mount
float Kp_forward = 0.3f;
float Kp_lateral = -0.008f; // Default might need tuning
float Kp_yaw = -0.01f;      // Default might need tuning
float max_forward_speed = 0.5f;
float max_lateral_speed = 0.3f;
float max_yaw_rate = 10.0f; // Degrees per second
float forward_dead_zone = 0.1f; // +/- 10cm
float azimuth_dead_zone = 1.0f; // +/- 1 degree
float yaw_azimuth_balance_dead_zone = 2.0f; // +/- 2 degrees
std::atomic<bool> invertLateralControl{false};
std::atomic<bool> invertYawControl{false};

float TARGET_ALTITUDE = 1.0f; // Default target descend altitude AGL
float VERTICAL_SPEED = 0.2f; // Default vertical speed
float HOLD_DURATION_SECONDS = 3.0f; // Default hold duration

// Default Sensor Selections (Example: Using only R_Az_R_El for everything)
std::atomic<bool> useWallSensorRAz{false};
std::atomic<bool> useWallSensorREl{false};
std::atomic<bool> useWallSensorRAzREl{true};
std::atomic<bool> useBeaconSensorRAz{false};
std::atomic<bool> useBeaconSensorREl{false};
std::atomic<bool> useBeaconSensorRAzREl{true};
std::atomic<bool> useBeaconRangeSensorRAz{false};
std::atomic<bool> useBeaconRangeSensorREl{false};
std::atomic<bool> useBeaconRangeSensorRAzREl{true};
std::atomic<bool> useYawSensorRAz{false};
std::atomic<bool> useYawSensorREl{false};
std::atomic<bool> useYawSensorRAzREl{true};


std::string TARGET_BEACON_ID = "BEACON_1"; // Default target beacon ID


// --- Helper Function ---
// Trim leading/trailing whitespace
std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r\f\v");
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(" \t\n\r\f\v");
    return str.substr(first, (last - first + 1));
}

// --- Load Preferences ---
void loadPreferences() {
    std::ifstream infile("preferences.txt");
    std::string line;
    int line_num = 0; // Line counter for debugging
    std::cout << "[Config] Loading preferences from preferences.txt..." << std::endl;

    if (!infile.is_open()) {
        std::cerr << "[Config] Warning: Could not open preferences.txt. Using default values." << std::endl;
        return;
    }

    while (std::getline(infile, line)) {
        line_num++; // Increment line counter
        // --- VERY BASIC DEBUG: Print raw line ---
        std::cout << "[Config Raw Line " << line_num << "] " << line << std::endl;
        // --- END BASIC DEBUG ---

        // --- First, strip comments from the whole line ---
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        // --- Then, trim whitespace from the potentially comment-stripped line ---
        line = trim(line);
        if (line.empty()) { // Skip empty lines (or lines that were only comments)
             std::cout << "[Config Line " << line_num << "] Skipped (empty or comment)." << std::endl;
            continue;
        }

        std::stringstream ss(line);
        std::string key;
        std::string value_str;
        if (std::getline(ss, key, '=') && std::getline(ss, value_str)) {
            key = trim(key);
            value_str = trim(value_str); // Trim value *after* splitting

            // --- Added check: ensure value_str is not empty after trimming ---
            if (value_str.empty()) {
                 std::cerr << "[Config Line " << line_num << "] Warning: Empty value for key '" << key << "'." << std::endl;
                 continue; // Skip processing this line
            }

            // --- DEBUG: Print parsed key and value ---
            std::cout << "[Config Parsed Line " << line_num << "] Key: \"" << key << "\", Value: \"" << value_str << "\"" << std::endl;
            // --- END DEBUG ---

            try {
                // --- Parse Values (Using keys from warnings/preferences.txt) ---
                if (key == "connect_to_drone") {
                    bool parsed_value = (value_str == "true");
                    connect_to_drone = parsed_value;
                    // --- DEBUGGING ---
                    std::cout << "[Config Detail] Found key 'connect_to_drone'. Value string: \"" << value_str << "\". Parsed as: " << std::boolalpha << connect_to_drone << std::endl;
                    // --- END DEBUGGING ---
                }
                // --- Check for enable_camera_stream ---
                else if (key == "enable_camera_stream") {
                     bool parsed_value = (value_str == "true");
                     enable_camera_stream = parsed_value;
                     // --- DEBUGGING ---
                     std::cout << "[Config Detail] Found key 'enable_camera_stream'. Value string: \"" << value_str << "\". Parsed as: " << std::boolalpha << enable_camera_stream << std::endl;
                     // --- END DEBUGGING ---
                }
                // --- End Check ---
                else if (key == "use_local_bridge_script") useLocalBridgeScript.store(value_str == "true");
                else if (key == "enable_bridge_reconnection") enable_bridge_reconnection.store(value_str == "true");
                else if (key == "targetdistance") targetDistance = std::stof(value_str);
                else if (key == "target_azimuth") targetAzimuth = std::stof(value_str);
                else if (key == "radar_mount_angle_degrees") radarMountAngleDegrees = std::stof(value_str);
                else if (key == "kp_forward") Kp_forward = std::stof(value_str);
                else if (key == "kp_lateral") Kp_lateral = std::stof(value_str);
                else if (key == "kp_yaw") Kp_yaw = std::stof(value_str);
                else if (key == "max_forward_speed") max_forward_speed = std::stof(value_str);
                else if (key == "max_lateral_speed") max_lateral_speed = std::stof(value_str);
                else if (key == "max_yaw_rate") max_yaw_rate = std::stof(value_str);
                else if (key == "forward_dead_zone") forward_dead_zone = std::stof(value_str);
                else if (key == "azimuth_dead_zone") azimuth_dead_zone = std::stof(value_str);
                else if (key == "yaw_azimuth_balance_dead_zone") yaw_azimuth_balance_dead_zone = std::stof(value_str);
                else if (key == "invert_lateral_control") invertLateralControl.store(value_str == "true");
                else if (key == "invert_yaw_control") invertYawControl.store(value_str == "true");
                else if (key == "target_altitude") TARGET_ALTITUDE = std::stof(value_str);
                else if (key == "vertical_speed") VERTICAL_SPEED = std::stof(value_str);
                else if (key == "hold_duration_seconds") HOLD_DURATION_SECONDS = std::stof(value_str);
                else if (key == "use_wall_sensor_r_az") useWallSensorRAz.store(value_str == "true");
                else if (key == "use_wall_sensor_r_el") useWallSensorREl.store(value_str == "true");
                else if (key == "use_wall_sensor_r_az_r_el") useWallSensorRAzREl.store(value_str == "true");
                else if (key == "use_beacon_sensor_r_az") useBeaconSensorRAz.store(value_str == "true");
                else if (key == "use_beacon_sensor_r_el") useBeaconSensorREl.store(value_str == "true");
                else if (key == "use_beacon_sensor_r_az_r_el") useBeaconSensorRAzREl.store(value_str == "true");
                else if (key == "use_beacon_range_sensor_r_az") useBeaconRangeSensorRAz.store(value_str == "true");
                else if (key == "use_beacon_range_sensor_r_el") useBeaconRangeSensorREl.store(value_str == "true");
                else if (key == "use_beacon_range_sensor_r_az_r_el") useBeaconRangeSensorRAzREl.store(value_str == "true");
                else if (key == "use_yaw_sensor_r_az") useYawSensorRAz.store(value_str == "true");
                else if (key == "use_yaw_sensor_r_el") useYawSensorREl.store(value_str == "true");
                else if (key == "use_yaw_sensor_r_az_r_el") useYawSensorRAzREl.store(value_str == "true");
                else if (key == "target_beacon_id") TARGET_BEACON_ID = value_str;
                // Add more keys as needed
                else {
                    // Keep the warning for actually unknown keys
                     std::cerr << "[Config Line " << line_num << "] Warning: Unknown key '" << key << "'." << std::endl;
                }
            } catch (const std::invalid_argument& e) {
                std::cerr << "[Config Line " << line_num << "] Warning: Invalid value format for key '" << key << "': \"" << value_str << "\"." << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "[Config Line " << line_num << "] Warning: Value out of range for key '" << key << "': \"" << value_str << "\"." << std::endl;
            }
        } else if (!key.empty()) { // Handle keys with no value after '=' (or lines that were only comments)
             // Only warn if the key wasn't empty before the comment was stripped
             if (line.find('=') == std::string::npos) { // If there was no '=' sign at all
                std::cerr << "[Config Line " << line_num << "] Warning: Malformed line (missing '='?): \"" << line << "\"." << std::endl;
             } else {
                // Key was present, but value was empty after trimming (already handled above)
             }
        }
    }
    infile.close();
    std::cout << "[Config] Preferences loaded." << std::endl;

    // --- Post-load Debugging ---
    std::cout << "[Config Final Check] Final value of connect_to_drone: " << std::boolalpha << connect_to_drone << std::endl;
    std::cout << "[Config Final Check] Final value of enable_camera_stream: " << std::boolalpha << enable_camera_stream << std::endl; // Debug new flag
    // --- End Post-load Debugging ---
}

// Optional: Implement savePreferences() if needed
void savePreferences() {
    // ... implementation to write current config values back to preferences.txt ...
    // Consider saving with the same keys used for loading to maintain consistency
    std::cerr << "[Config] Warning: savePreferences() not implemented." << std::endl;
}