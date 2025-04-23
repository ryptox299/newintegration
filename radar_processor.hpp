#ifndef RADAR_PROCESSOR_HPP
#define RADAR_PROCESSOR_HPP

#include <vector>
#include <string>
#include "types.hpp" // Includes RadarObject, ProcessingMode, VerticalAlignState

// Forward declare Vehicle class to avoid including the full dji_vehicle.hpp here
namespace DJI {
    namespace OSDK {
        class Vehicle; // Forward declaration
    }
}

/**
 * @brief Parses a JSON string containing radar data into a vector of RadarObject structs.
 *
 * @param jsonData The raw JSON string received from the radar bridge.
 * @return std::vector<RadarObject> A vector containing the parsed radar objects. Returns an empty vector on error or if no objects are found.
 */
std::vector<RadarObject> parseRadarData(const std::string& jsonData);

/**
 * @brief Processes a vector of radar objects based on the current processing mode,
 *        calculates control commands, and sends them to the vehicle if enabled.
 *        Also updates global live data for the GUI.
 *
 * @param current_mode The active ProcessingMode (e.g., WALL_FOLLOW, WALL_BEACON_YAW_VERTICAL).
 * @param objects The vector of RadarObject structs for the current timeframe (usually one second).
 * @param vehicle Pointer to the DJI OSDK Vehicle object (can be nullptr if control is disabled).
 * @param enableControl Boolean flag indicating whether flight control commands should be sent.
 */
void processRadarDataAndControl(ProcessingMode current_mode, const std::vector<RadarObject>& objects, DJI::OSDK::Vehicle* vehicle, bool enableControl);

/**
 * @brief Displays the full details of all radar objects in a given vector to the console (and log file).
 *        Used primarily for the PROCESS_FULL mode.
 *
 * @param objects The vector of RadarObject structs to display.
 */
void displayRadarObjects(const std::vector<RadarObject>& objects);

// --- ADDED Declaration ---
/**
 * @brief The main function for the background processing thread.
 *        Handles connection to the Python bridge, reads data, calls parsing and processing functions.
 *        Manages bridge connection status and reconnection logic.
 *
 * @param scriptName The name of the Python bridge script to run/connect to.
 * @param vehicle Pointer to the initialized DJI Vehicle object (can be nullptr).
 * @param enableFlightControl Flag indicating if flight control commands should be attempted.
 * @param mode The ProcessingMode to use for radar data.
 */
void processingLoopFunction(const std::string& scriptName, DJI::OSDK::Vehicle* vehicle, bool enableFlightControl, ProcessingMode mode);
// --- End Added Declaration ---


#endif // RADAR_PROCESSOR_HPP