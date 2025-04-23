#include "globals.hpp"

// --- Thread Management ---
std::thread monitoringThread;
std::thread processingThread;
std::atomic<bool> stopMonitoringFlag{false};
std::atomic<bool> stopProcessingFlag{false};
std::atomic<bool> forceStopReconnectionFlag{false};

// --- Status ---
std::atomic<uint8_t> g_current_display_mode{255}; // Default to unavailable
std::atomic<BridgeConnectionStatus> currentBridgeStatus{BridgeConnectionStatus::DISCONNECTED};
std::atomic<VerticalAlignState> g_vertical_state{VerticalAlignState::IDLE};

// --- Radar Processing State ---
// Initialized properly before use or reset each second in processing function
std::string currentSecond = "";
float lowestRange = std::numeric_limits<float>::max();
bool hasAnonData = false;
float targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
float targetBeaconRange = std::numeric_limits<float>::quiet_NaN();
bool foundTargetBeacon = false;
float sumPositiveAz = 0.0f;
int countPositiveAz = 0;
float sumNegativeAz = 0.0f;
int countNegativeAz = 0;
bool hasYawAnonData = false;
float g_target_beacon_range_threshold = std::numeric_limits<float>::quiet_NaN();
std::chrono::steady_clock::time_point g_hold_start_time;

// --- GUI Data ---
std::mutex g_beaconMutex;
std::vector<RadarObject> g_beaconObjects;

std::mutex g_anonMutex;
std::vector<RadarObject> g_anonObjects;

std::mutex g_liveDataMutex;
std::chrono::steady_clock::time_point g_last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
float g_latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
float g_latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
float g_latest_altitude = std::numeric_limits<float>::quiet_NaN();

// --- Camera Status Data ---
std::mutex g_cameraStatusMutex; // Mutex might not be strictly needed for atomics/timestamps, but good practice if adding more later
std::atomic<bool> g_camera_data_receiving{false};
std::chrono::steady_clock::time_point g_last_camera_data_time = std::chrono::steady_clock::time_point::min();