#ifndef TYPES_HPP
#define TYPES_HPP

#include <string>
#include <vector>
#include <chrono> // Included for time_point

// Structure to hold radar object data
struct RadarObject {
    std::string timestamp;
    std::string sensor;
    std::string src;
    std::string ID;
    float X = std::numeric_limits<float>::quiet_NaN();
    float Y = std::numeric_limits<float>::quiet_NaN();
    float Z = std::numeric_limits<float>::quiet_NaN();
    float Range = std::numeric_limits<float>::quiet_NaN();
    float RangeRate = std::numeric_limits<float>::quiet_NaN();
    float Az = std::numeric_limits<float>::quiet_NaN();
    float El = std::numeric_limits<float>::quiet_NaN();
    float Pwr = std::numeric_limits<float>::quiet_NaN();
    float Conf = std::numeric_limits<float>::quiet_NaN();
};

// Enum for different processing modes
enum class ProcessingMode {
    PROCESS_FULL,              // Process all data, no control
    WALL_FOLLOW,               // Wall following + Lateral Beacon (No Yaw/Vertical)
    WALL_BEACON_VERTICAL,      // Wall following + Lateral Beacon + Vertical Control
    WALL_BEACON_YAW,           // Wall following + Lateral Beacon + Yaw Control (No Vertical)
    WALL_BEACON_YAW_VERTICAL   // Wall following + Lateral Beacon + Yaw + Vertical Control
};

// Enum for Python bridge connection status
enum class BridgeConnectionStatus {
    DISCONNECTED,
    CONNECTING, // Optional: If initial connection takes time
    CONNECTED,
    RECONNECTING
};

// Enum for Vertical Alignment State Machine
enum class VerticalAlignState {
    IDLE,
    ALIGN_HORIZONTAL, // Align horizontally first
    ASCENDING_HOLD,   // Ascend for a duration
    DESCENDING,       // Descend to target altitude
    FINAL_HOLD        // Hold at final altitude
};


// Structure for live data shared with GUI
struct LiveData {
    float latest_beacon_range = std::numeric_limits<float>::quiet_NaN();
    float latest_wall_horizontal_distance = std::numeric_limits<float>::quiet_NaN();
    float latest_altitude = std::numeric_limits<float>::quiet_NaN();
    std::chrono::steady_clock::time_point last_beacon_seen_time = std::chrono::steady_clock::time_point::min();
};


#endif // TYPES_HPP