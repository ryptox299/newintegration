#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono> // For time_point
#include <limits> // For numeric_limits
#include <string>
#include "types.hpp" // Includes RadarObject, ProcessingMode, etc.

// --- Thread Management ---
extern std::thread monitoringThread;
extern std::thread processingThread;
extern std::atomic<bool> stopMonitoringFlag;
extern std::atomic<bool> stopProcessingFlag;
extern std::atomic<bool> forceStopReconnectionFlag; // Flag to force stop reconnection attempts

// --- Status ---
extern std::atomic<uint8_t> g_current_display_mode; // Stores raw display mode value from telemetry
extern std::atomic<BridgeConnectionStatus> currentBridgeStatus;
extern std::atomic<VerticalAlignState> g_vertical_state; // State for vertical alignment maneuver

// --- Radar Processing State (used within processRadarDataAndControl) ---
// These are reset each second within the processing function
extern std::string currentSecond;
extern float lowestRange;
extern bool hasAnonData;
extern float targetBeaconAzimuth;
extern float targetBeaconRange; // Added for vertical control logic
extern bool foundTargetBeacon;
extern float sumPositiveAz;
extern int countPositiveAz;
extern float sumNegativeAz;
extern int countNegativeAz;
extern bool hasYawAnonData;
extern float g_target_beacon_range_threshold; // Calculated target range based on horizontal dist and angle
extern std::chrono::steady_clock::time_point g_hold_start_time; // Timestamp for vertical hold state

// --- GUI Data (Protected by Mutexes) ---
extern std::mutex g_beaconMutex;
extern std::vector<RadarObject> g_beaconObjects; // Latest BEACON objects for GUI

extern std::mutex g_anonMutex;
extern std::vector<RadarObject> g_anonObjects;   // Latest anon objects for GUI

extern std::mutex g_liveDataMutex;
extern std::chrono::steady_clock::time_point g_last_beacon_seen_time;
extern float g_latest_beacon_range;
extern float g_latest_wall_horizontal_distance;
extern float g_latest_altitude;

// --- Camera Status Data (Protected by Mutex) ---
// No longer storing frame data, just status
extern std::mutex g_cameraStatusMutex;
extern std::atomic<bool> g_camera_data_receiving; // Flag indicating if callback is getting data
extern std::chrono::steady_clock::time_point g_last_camera_data_time; // Timestamp of last received frame

#endif // GLOBALS_HPP