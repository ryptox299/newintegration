#include "radar_processor.hpp"
#include "globals.hpp"
#include "config.hpp"
#include "types.hpp"             // For RadarObject, ProcessingMode, VerticalAlignState
#include "utils.hpp"             // For getProcessingModeName, getVerticalStateName
#include "python_bridge.hpp"     // For runPythonBridge, readLineFromBridge, stopPythonBridge

// DJI OSDK Includes
#include "dji_vehicle.hpp"
#include "dji_control.hpp"
#include "dji_ack.hpp"
#include "dji_status.hpp" // For VehicleStatus::DisplayMode

// Standard Library Includes
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>    // For std::isnan, std::cos, std::sin, M_PI
#include <limits> // For numeric_limits
#include <sstream> // For stringstream used in status printing

// JSON library (assuming nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

using namespace DJI::OSDK;

// Helper to calculate horizontal distance from range and elevation/mount angle
float calculateHorizontalDistance(float range, float el_degrees, float mount_degrees) {
    // Convert angles from degrees to radians
    float el_rad = el_degrees * M_PI / 180.0f;
    float mount_rad = mount_degrees * M_PI / 180.0f;
    // Calculate the effective angle relative to the horizon
    float effective_angle_rad = el_rad + mount_rad;
    // Calculate horizontal distance using cosine
    float horizontal_distance = range * std::cos(effective_angle_rad);
    return horizontal_distance;
}


// Main processing loop function (runs in a separate thread)
void processingLoopFunction(const std::string& scriptPath, Vehicle* vehicle, bool enableControl, ProcessingMode mode) {
    // Use function from utils.hpp
    std::cout << "[" << getProcessingModeName(mode) << "] Processing thread started. Control: " << (enableControl ? "Enabled" : "Disabled") << "." << std::endl;

    // --- Variables for Control Logic ---
    float current_range_for_wall = std::numeric_limits<float>::quiet_NaN();
    float current_azimuth_for_beacon = std::numeric_limits<float>::quiet_NaN();
    float current_elevation_for_beacon = std::numeric_limits<float>::quiet_NaN(); // For potential future use
    float current_range_for_beacon = std::numeric_limits<float>::quiet_NaN();
    float current_azimuth_for_yaw = std::numeric_limits<float>::quiet_NaN();
    float current_elevation_for_yaw = std::numeric_limits<float>::quiet_NaN(); // For potential future use

    float last_valid_wall_range = std::numeric_limits<float>::quiet_NaN();
    float last_valid_beacon_azimuth = std::numeric_limits<float>::quiet_NaN();
    float last_valid_beacon_range = std::numeric_limits<float>::quiet_NaN();
    float last_valid_yaw_azimuth = std::numeric_limits<float>::quiet_NaN();

    auto last_wall_update_time = std::chrono::steady_clock::time_point::min();
    auto last_beacon_update_time = std::chrono::steady_clock::time_point::min();
    auto last_yaw_update_time = std::chrono::steady_clock::time_point::min();

    const std::chrono::milliseconds data_timeout(2000); // Timeout for considering data stale (e.g., 2 seconds)

    int functionTimeout = 1; // Timeout for OSDK control calls

    // State for vertical alignment
    auto last_vertical_state_change_time = std::chrono::steady_clock::now();

    // --- Main Loop ---
    while (!stopProcessingFlag.load()) {
        // Check bridge connection status
        BridgeConnectionStatus status = currentBridgeStatus.load(std::memory_order_relaxed);
        if (status == BridgeConnectionStatus::DISCONNECTED) {
            if (enable_bridge_reconnection.load() && !forceStopReconnectionFlag.load()) {
                currentBridgeStatus.store(BridgeConnectionStatus::RECONNECTING);
                 std::cout << "[" << getProcessingModeName(mode) << "] Bridge disconnected. Attempting reconnect..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_DELAY_SECONDS));
                // Use function from python_bridge.hpp
                runPythonBridge(scriptPath); // Attempt to restart
                continue; // Skip processing until connection re-established
            } else {
                 std::cout << "[" << getProcessingModeName(mode) << "] Bridge disconnected. Reconnection disabled or stopped. Exiting processing thread." << std::endl;
                 break; // Exit thread
            }
        } else if (status == BridgeConnectionStatus::RECONNECTING) {
             // Still waiting for reconnection attempt in runPythonBridge or main loop
             std::this_thread::sleep_for(std::chrono::milliseconds(500));
             continue;
        }

        // --- Read Data from Python Bridge ---
        std::string line;
        bool dataRead = false;
        // Use function from python_bridge.hpp
        if (readLineFromBridge(line, 100)) { // Read with a short timeout (e.g., 100ms)
            dataRead = true;

            // --- DEBUG: Print raw line received ---
            std::cout << "[" << getProcessingModeName(mode) << " Raw Data] " << line << std::endl;
            // --- END DEBUG ---

            try {
                json j = json::parse(line);

                // --- Data Validation and Update ---
                if (j.contains("timestamp") && j.contains("sensor") && j.contains("src") && j.contains("ID")) {
                    RadarObject obj;
                    obj.timestamp = j["timestamp"].get<std::string>();
                    obj.sensor = j["sensor"].get<std::string>();
                    obj.src = j["src"].get<std::string>();
                    obj.ID = j["ID"].get<std::string>();
                    obj.X = j.value("X", std::numeric_limits<float>::quiet_NaN());
                    obj.Y = j.value("Y", std::numeric_limits<float>::quiet_NaN());
                    obj.Z = j.value("Z", std::numeric_limits<float>::quiet_NaN());
                    obj.Range = j.value("Range", std::numeric_limits<float>::quiet_NaN());
                    obj.RangeRate = j.value("RangeRate", std::numeric_limits<float>::quiet_NaN());
                    obj.Az = j.value("Az", std::numeric_limits<float>::quiet_NaN());
                    obj.El = j.value("El", std::numeric_limits<float>::quiet_NaN());
                    obj.Pwr = j.value("Pwr", std::numeric_limits<float>::quiet_NaN());
                    obj.Conf = j.value("Conf", std::numeric_limits<float>::quiet_NaN());

                    auto now = std::chrono::steady_clock::now();

                    // Check if it's a beacon or anonymous object based on ID
                    if (obj.ID.find("BEACON") != std::string::npos) {
                        { // Scope for mutex lock
                            std::lock_guard<std::mutex> lock(g_beaconMutex);
                            // Keep only the latest N beacons (e.g., 5)
                            const size_t MAX_BEACONS = 5;
                            if (g_beaconObjects.size() >= MAX_BEACONS) {
                                g_beaconObjects.erase(g_beaconObjects.begin());
                            }
                            g_beaconObjects.push_back(obj);
                        } // Mutex released here

                        // Update live data for the target beacon
                        if (obj.ID == TARGET_BEACON_ID) {
                            { // Scope for live data mutex
                                std::lock_guard<std::mutex> lock(g_liveDataMutex);
                                g_latest_beacon_range = obj.Range;
                                g_last_beacon_seen_time = now;
                            } // Mutex released

                            // Update control variables based on sensor selection
                            if (useBeaconSensorRAz.load()) current_azimuth_for_beacon = obj.Az;
                            if (useBeaconSensorREl.load()) current_elevation_for_beacon = obj.El; // Store elevation if selected
                            if (useBeaconSensorRAzREl.load()) { current_azimuth_for_beacon = obj.Az; current_elevation_for_beacon = obj.El; } // Store both if selected

                            if (useBeaconRangeSensorRAz.load()) current_range_for_beacon = obj.Range; // Assuming Range is relevant if R_Az is used for range trigger? Check logic.
                            if (useBeaconRangeSensorREl.load()) current_range_for_beacon = obj.Range; // Assuming Range is relevant if R_El is used for range trigger? Check logic.
                            if (useBeaconRangeSensorRAzREl.load()) current_range_for_beacon = obj.Range; // Use range if combined sensor selected

                            last_valid_beacon_azimuth = current_azimuth_for_beacon;
                            last_valid_beacon_range = current_range_for_beacon;
                            last_beacon_update_time = now;
                        }
                    } else { // Assume anonymous object (wall, obstacle, etc.)
                         { // Scope for mutex lock
                            std::lock_guard<std::mutex> lock(g_anonMutex);
                            // Keep only the latest N anon objects (e.g., 10)
                            const size_t MAX_ANON = 10;
                            if (g_anonObjects.size() >= MAX_ANON) {
                                g_anonObjects.erase(g_anonObjects.begin());
                            }
                             g_anonObjects.push_back(obj);
                         } // Mutex released

                         // --- Update Control Variables based on Sensor Selection ---
                         // Wall Distance (Forward Control)
                         if (useWallSensorRAz.load() && !std::isnan(obj.Az) && std::abs(obj.Az) < 5) { // Example: Use if Azimuth is near 0
                              current_range_for_wall = obj.Range; last_valid_wall_range = current_range_for_wall; last_wall_update_time = now;
                         }
                         if (useWallSensorREl.load() && !std::isnan(obj.El)) { // Example: Use if Elevation is valid (might need specific criteria)
                              current_range_for_wall = obj.Range; last_valid_wall_range = current_range_for_wall; last_wall_update_time = now;
                         }
                         if (useWallSensorRAzREl.load() && !std::isnan(obj.Az) && std::abs(obj.Az) < 5 && !std::isnan(obj.El)) { // Example: Use if both Az near 0 and El valid
                              current_range_for_wall = obj.Range; last_valid_wall_range = current_range_for_wall; last_wall_update_time = now;
                         }

                         // Yaw Balancing (Yaw Control) - Using Anon object azimuth
                         if (useYawSensorRAz.load() && !std::isnan(obj.Az)) {
                              current_azimuth_for_yaw = obj.Az; last_valid_yaw_azimuth = current_azimuth_for_yaw; last_yaw_update_time = now;
                         }
                         if (useYawSensorREl.load() && !std::isnan(obj.El)) { // Example: Use El if selected for Yaw? Check logic.
                              current_azimuth_for_yaw = obj.El; last_valid_yaw_azimuth = current_azimuth_for_yaw; last_yaw_update_time = now; // Might need different logic
                         }
                         if (useYawSensorRAzREl.load() && !std::isnan(obj.Az) && !std::isnan(obj.El)) {
                              current_azimuth_for_yaw = obj.Az; last_valid_yaw_azimuth = current_azimuth_for_yaw; last_yaw_update_time = now; // Using Az if combined selected
                         }

                         // Update live data for wall distance (using the calculated horizontal distance)
                         if (!std::isnan(current_range_for_wall)) {
                              float wall_el = useWallSensorREl.load() || useWallSensorRAzREl.load() ? obj.El : 0.0f; // Use object El if selected, else assume 0
                              float wall_hz_dist = calculateHorizontalDistance(current_range_for_wall, wall_el, radarMountAngleDegrees);
                              {
                                   std::lock_guard<std::mutex> lock(g_liveDataMutex);
                                   g_latest_wall_horizontal_distance = wall_hz_dist;
                              }
                         }
                    }
                } else {
                     std::cerr << "[" << getProcessingModeName(mode) << "] Warning: Received JSON missing required fields (timestamp, sensor, src, ID)." << std::endl;
                }

            } catch (json::parse_error& e) {
                std::cerr << "[" << getProcessingModeName(mode) << "] JSON Parsing Error: " << e.what() << ". Data: " << line.substr(0, 100) << (line.length() > 100 ? "...]" : "") << std::endl; // Log first 100 chars
            } catch (json::exception& e) {
                 std::cerr << "[" << getProcessingModeName(mode) << "] JSON Exception: " << e.what() << ". Data: " << line.substr(0, 100) << (line.length() > 100 ? "...]" : "") << std::endl;
            } catch (std::exception& e) {
                 std::cerr << "[" << getProcessingModeName(mode) << "] Standard Exception during processing: " << e.what() << ". Data: " << line.substr(0, 100) << (line.length() > 100 ? "...]" : "") << std::endl;
            }
        } // End if(readLineFromBridge)

        // --- Apply Control Logic (if enabled) ---
        if (enableControl && vehicle != nullptr && vehicle->control != nullptr) {
            auto now = std::chrono::steady_clock::now();

            // Check data staleness
            if (now - last_wall_update_time > data_timeout) current_range_for_wall = std::numeric_limits<float>::quiet_NaN(); else current_range_for_wall = last_valid_wall_range;
            if (now - last_beacon_update_time > data_timeout) { current_azimuth_for_beacon = std::numeric_limits<float>::quiet_NaN(); current_range_for_beacon = std::numeric_limits<float>::quiet_NaN(); } else { current_azimuth_for_beacon = last_valid_beacon_azimuth; current_range_for_beacon = last_valid_beacon_range; }
            if (now - last_yaw_update_time > data_timeout) current_azimuth_for_yaw = std::numeric_limits<float>::quiet_NaN(); else current_azimuth_for_yaw = last_valid_yaw_azimuth;


            // --- Calculate Control Commands ---
            float cmd_vx = 0.0f; // Forward/Backward (X)
            float cmd_vy = 0.0f; // Left/Right (Y)
            float cmd_vz = 0.0f; // Up/Down (Z)
            float cmd_yaw_rate = 0.0f; // Yaw rate

            VerticalAlignState current_vertical_state = g_vertical_state.load(std::memory_order_relaxed);

            // --- Vertical State Machine ---
            // Use enum members from types.hpp
            if (mode == ProcessingMode::WALL_BEACON_VERTICAL || mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) {
                float current_altitude = std::numeric_limits<float>::quiet_NaN();
                { std::lock_guard<std::mutex> lock(g_liveDataMutex); current_altitude = g_latest_altitude; } // Get current altitude

                switch (current_vertical_state) {
                    case VerticalAlignState::ALIGN_HORIZONTAL:
                        // Maintain current altitude while aligning horizontally
                        cmd_vz = 0.0f; // Or potentially a small value to hold altitude if needed
                        // Check if horizontally aligned (using beacon range as proxy)
                        if (!std::isnan(current_range_for_beacon) && current_range_for_beacon <= targetDistance + forward_dead_zone) { // Check if close enough
                             // Use enum member from types.hpp
                             g_vertical_state.store(VerticalAlignState::ASCENDING_HOLD);
                             last_vertical_state_change_time = now;
                             std::cout << "[" << getProcessingModeName(mode) << "] Vertical State -> ASCENDING_HOLD" << std::endl;
                        }
                        break;

                    // Use enum member from types.hpp
                    case VerticalAlignState::ASCENDING_HOLD:
                        // Ascend until hold duration expires
                        cmd_vz = VERTICAL_SPEED;
                        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_vertical_state_change_time).count() >= HOLD_DURATION_SECONDS) {
                             // Use enum member from types.hpp
                             g_vertical_state.store(VerticalAlignState::DESCENDING);
                             last_vertical_state_change_time = now;
                             std::cout << "[" << getProcessingModeName(mode) << "] Vertical State -> DESCENDING" << std::endl;
                        }
                        break;

                    // Use enum member from types.hpp
                    case VerticalAlignState::DESCENDING:
                        // Descend until target altitude is reached
                        if (!std::isnan(current_altitude) && current_altitude > TARGET_ALTITUDE) {
                             cmd_vz = -VERTICAL_SPEED; // Negative for descent
                        } else {
                             cmd_vz = 0.0f; // Reached target altitude, stop vertical movement
                             // Use enum member from types.hpp
                             g_vertical_state.store(VerticalAlignState::FINAL_HOLD); // Transition to final hold or IDLE?
                             last_vertical_state_change_time = now;
                             std::cout << "[" << getProcessingModeName(mode) << "] Vertical State -> FINAL_HOLD (Reached Target Altitude)" << std::endl;
                        }
                        break;

                    // Use enum member from types.hpp
                    case VerticalAlignState::FINAL_HOLD:
                         // Hold position indefinitely (or for a duration, then IDLE?)
                         cmd_vz = 0.0f;
                         // Could potentially add logic to transition back to IDLE after some time
                         break;

                    case VerticalAlignState::IDLE: // Should not happen if mode requires vertical, but handle anyway
                    default:
                        cmd_vz = 0.0f;
                        break;
                }
            } else {
                 // If not a vertical mode, ensure state is IDLE and vz is 0
                 g_vertical_state.store(VerticalAlignState::IDLE);
                 cmd_vz = 0.0f;
            }


            // --- Horizontal Control Calculations (Run regardless of vertical state) ---
            // Forward/Backward (Wall Following) - Only if not doing vertical maneuvers or in horizontal align phase
            if (!std::isnan(current_range_for_wall) &&
                (current_vertical_state == VerticalAlignState::IDLE || current_vertical_state == VerticalAlignState::ALIGN_HORIZONTAL))
            {
                float wall_el = 0.0f; // Determine elevation angle used for wall distance calculation
                if (useWallSensorREl.load() || useWallSensorRAzREl.load()) {
                     // Need to retrieve the El value associated with the current_range_for_wall
                     // This requires better state management or accessing the last anon object
                     // For now, approximate using 0 or require R_Az_R_El
                     // wall_el = ... retrieve last anon El if needed ...
                }
                 float current_horizontal_distance = calculateHorizontalDistance(current_range_for_wall, wall_el, radarMountAngleDegrees);
                 float distance_error = current_horizontal_distance - targetDistance;

                 if (std::abs(distance_error) > forward_dead_zone) {
                      cmd_vx = -Kp_forward * distance_error; // Move forward if too far, backward if too close
                      // Clamp speed
                      cmd_vx = std::max(-max_forward_speed, std::min(max_forward_speed, cmd_vx));
                 }
            }

            // Left/Right (Beacon Azimuth Tracking) - Only if not doing vertical maneuvers or in horizontal align phase
            if (!std::isnan(current_azimuth_for_beacon) &&
                (current_vertical_state == VerticalAlignState::IDLE || current_vertical_state == VerticalAlignState::ALIGN_HORIZONTAL))
            {
                 float azimuth_error = current_azimuth_for_beacon - targetAzimuth; // Target is usually 0

                 if (std::abs(azimuth_error) > azimuth_dead_zone) {
                      cmd_vy = -Kp_lateral * azimuth_error; // Negative Kp typically needed for right-positive azimuth
                      if (invertLateralControl.load()) { cmd_vy = -cmd_vy; } // Apply inversion if flag is set
                      // Clamp speed
                      cmd_vy = std::max(-max_lateral_speed, std::min(max_lateral_speed, cmd_vy));
                 }
            }

            // Yaw Control (Beacon Azimuth or Anon Object Azimuth) - Only if not doing vertical maneuvers or in horizontal align phase
            if ( (mode == ProcessingMode::WALL_BEACON_YAW || mode == ProcessingMode::WALL_BEACON_YAW_VERTICAL) &&
                 (current_vertical_state == VerticalAlignState::IDLE || current_vertical_state == VerticalAlignState::ALIGN_HORIZONTAL) )
            {
                  float yaw_source_azimuth = std::numeric_limits<float>::quiet_NaN();
                  // Prioritize beacon azimuth if available and within reasonable limits for yaw
                  if (!std::isnan(current_azimuth_for_beacon) && std::abs(current_azimuth_for_beacon) < 45.0f) { // Example limit
                       yaw_source_azimuth = current_azimuth_for_beacon;
                  }
                  // Otherwise, use anon object azimuth if selected and available
                  else if (!std::isnan(current_azimuth_for_yaw) && (useYawSensorRAz.load() || useYawSensorREl.load() || useYawSensorRAzREl.load())) {
                       yaw_source_azimuth = current_azimuth_for_yaw;
                  }

                  if (!std::isnan(yaw_source_azimuth)) {
                       float yaw_error = yaw_source_azimuth - targetAzimuth; // Target is usually 0

                       // Only apply yaw correction if error is significant enough
                       if (std::abs(yaw_error) > yaw_azimuth_balance_dead_zone) {
                            cmd_yaw_rate = -Kp_yaw * yaw_error; // Negative Kp typically needed for right-positive azimuth
                            if (invertYawControl.load()) { cmd_yaw_rate = -cmd_yaw_rate; } // Apply inversion
                            // Clamp yaw rate
                            cmd_yaw_rate = std::max(-max_yaw_rate, std::min(max_yaw_rate, cmd_yaw_rate));
                       }
                  }
            }


            // --- Send Control Command ---
            // Define the control flag
            uint8_t control_flag = Control::VERTICAL_VELOCITY | // Control::VERTICAL_POSITION | // Use velocity or position control?
                                   Control::HORIZONTAL_VELOCITY |
                                   Control::YAW_RATE | // Control::YAW_ANGLE | // Use rate or angle control?
                                   Control::HORIZONTAL_BODY | // HORIZONTAL_GROUND or HORIZONTAL_BODY?
                                   Control::STABLE_ENABLE; // STABLE_DISABLE or STABLE_ENABLE?

            // Construct CtrlData using the correct constructor and member names
            // Provide initial values (0.0f) as required by the constructor
            Control::CtrlData ctrlData(control_flag, 0.0f, 0.0f, 0.0f, 0.0f);

            // Assign calculated values using correct member names (x, y, z, yaw)
            ctrlData.x = cmd_vx;
            ctrlData.y = cmd_vy;
            ctrlData.z = cmd_vz; // Use calculated vertical velocity
            // ctrlData.z = target_altitude; // If using VERTICAL_POSITION
            ctrlData.yaw = cmd_yaw_rate; // Use calculated yaw rate
            // ctrlData.yaw = target_yaw_angle; // If using YAW_ANGLE

            vehicle->control->flightCtrl(ctrlData);

             // --- Print Status Periodically ---
             static auto last_status_print_time = std::chrono::steady_clock::now();
             auto now_print = std::chrono::steady_clock::now();
             if (std::chrono::duration_cast<std::chrono::seconds>(now_print - last_status_print_time).count() >= 1) {
                 last_status_print_time = now_print;
                 std::stringstream ss;
                 ss << "[" << getProcessingModeName(mode) << "] Ctrl Status (Sec: " << std::chrono::duration_cast<std::chrono::seconds>(now_print.time_since_epoch()).count() << "):";
                 // Use function from utils.hpp
                 ss << " VState: " << getVerticalStateName(current_vertical_state) << "\n";
                 ss << "  TgtHoriz(m): " << std::fixed << std::setprecision(3) << targetDistance;
                 ss << ", RdrAng(deg): " << std::fixed << std::setprecision(3) << radarMountAngleDegrees << "\n";

                 float wall_hz_dist_print = std::numeric_limits<float>::quiet_NaN();
                 { std::lock_guard<std::mutex> lock(g_liveDataMutex); wall_hz_dist_print = g_latest_wall_horizontal_distance; }
                 ss << "  Wall Sens: [" << (useWallSensorRAz.load()?"R_Az ":"") << (useWallSensorREl.load()?"R_El ":"") << (useWallSensorRAzREl.load()?"R_Az_R_El":"") << "]";
                 ss << ", MeasRng(m): " << std::fixed << std::setprecision(6) << current_range_for_wall;
                 ss << ", CalcDist(m): " << std::fixed << std::setprecision(6) << wall_hz_dist_print << "\n";

                 ss << "  BcnAz Sens: [" << (useBeaconSensorRAz.load()?"R_Az ":"") << (useBeaconSensorREl.load()?"R_El ":"") << (useBeaconSensorRAzREl.load()?"R_Az_R_El":"") << "]";
                 ss << ", TgtAz(deg): " << std::fixed << std::setprecision(3) << targetAzimuth;
                 ss << ", CurAz(deg): " << std::fixed << std::setprecision(6) << current_azimuth_for_beacon << "\n";

                 ss << "  BcnRg Sens: [" << (useBeaconRangeSensorRAz.load()?"R_Az ":"") << (useBeaconRangeSensorREl.load()?"R_El ":"") << (useBeaconRangeSensorRAzREl.load()?"R_Az_R_El":"") << "]";
                 ss << ", CurRg(m): " << std::fixed << std::setprecision(6) << current_range_for_beacon;
                 ss << ", TgtRgThr(m): " << std::fixed << std::setprecision(3) << (targetDistance + forward_dead_zone) << "\n"; // Show threshold for vertical transition

                 float alt_print = std::numeric_limits<float>::quiet_NaN();
                 { std::lock_guard<std::mutex> lock(g_liveDataMutex); alt_print = g_latest_altitude; }
                 ss << "  LatInv: " << std::boolalpha << invertLateralControl.load();
                 ss << ", CurAlt(m): " << std::fixed << std::setprecision(6) << alt_print;
                 ss << ", TgtAlt(m): " << std::fixed << std::setprecision(3) << TARGET_ALTITUDE << "\n";

                 ss << "  => Cmd Vel(X:" << std::fixed << std::setprecision(3) << cmd_vx
                    << ", Y:" << std::fixed << std::setprecision(3) << cmd_vy
                    << ", Z:" << std::fixed << std::setprecision(3) << cmd_vz
                    << "), YawRate:" << std::fixed << std::setprecision(3) << cmd_yaw_rate;
                 std::cout << ss.str() << "\n--------------------------------------" << std::endl;
             }


        } // End if(enableControl)


        // --- Small delay ---
        // Avoid busy-waiting if no data was read
        if (!dataRead) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Short sleep if no data
        }

    } // End while(!stopProcessingFlag.load())


    // --- Cleanup before exiting thread ---
    std::cout << "[" << getProcessingModeName(mode) << "] Processing thread stopping..." << std::endl;

    // Send zero velocity command if control was enabled
    if (enableControl && vehicle != nullptr && vehicle->control != nullptr) {
        std::cout << "[" << getProcessingModeName(mode) << "] Sending zero velocity command..." << std::endl;
        // Define the control flag for stopping
        uint8_t stop_flag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_VELOCITY | Control::YAW_RATE | Control::HORIZONTAL_BODY | Control::STABLE_ENABLE;
        // Construct stop command correctly
        Control::CtrlData stopCmd(stop_flag, 0.0f, 0.0f, 0.0f, 0.0f);
        // Assign zero values (redundant given constructor, but explicit)
        stopCmd.x = 0.0f;
        stopCmd.y = 0.0f;
        stopCmd.z = 0.0f;
        stopCmd.yaw = 0.0f;
        vehicle->control->flightCtrl(stopCmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Allow time for command to be sent

        // Attempt to release control authority
        std::cout << "[" << getProcessingModeName(mode) << "] Releasing control authority..." << std::endl;
        ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
        if (ACK::getError(releaseAck) && releaseAck.info.cmd_set != 0x00 && releaseAck.info.cmd_id != 0x00) { // Check if it's a real error
             ACK::getErrorCodeMessage(releaseAck, std::string("["+getProcessingModeName(mode)+"] releaseCtrlAuthority").c_str());
             std::cerr << "[" << getProcessingModeName(mode) << "] Warning: Failed to release control authority on exit." << std::endl;
        } else {
             std::cout << "[" << getProcessingModeName(mode) << "] Control authority released (or was not held)." << std::endl;
        }
    }
    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status
    // Use enum member from types.hpp
    g_vertical_state.store(VerticalAlignState::IDLE); // Reset vertical state
    std::cout << "[" << getProcessingModeName(mode) << "] Processing thread finished." << std::endl;
}