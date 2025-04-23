#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <vector>
#include <atomic>

// --- Connection Settings ---
extern bool connect_to_drone; // Whether to attempt OSDK connection
extern bool enable_camera_stream; // NEW: Whether to attempt to initialize and use the camera stream
extern std::string defaultPythonBridgeScript; // Default Python script name
extern std::string localPythonBridgeScript; // Local Python script name for testing
extern std::atomic<bool> useLocalBridgeScript; // Flag to switch between default and local script
extern std::atomic<bool> enable_bridge_reconnection; // Flag to enable/disable bridge reconnection attempts
const int RECONNECT_DELAY_SECONDS = 5; // Delay between reconnection attempts
const int BRIDGE_CONNECTION_ATTEMPTS = 5; // Number of initial connection attempts
const int BRIDGE_CONNECT_TIMEOUT_SECONDS = 2; // Timeout for each connection attempt

// --- Control Parameters ---
extern float targetDistance;       // Target horizontal distance to maintain from the wall/objects (meters)
extern float targetAzimuth;        // Target azimuth angle for the beacon (degrees)
extern float radarMountAngleDegrees; // Mounting angle of the radar relative to horizontal (degrees)
extern float Kp_forward;           // Proportional gain for forward velocity control
extern float Kp_lateral;           // Proportional gain for lateral velocity control
extern float Kp_yaw;               // Proportional gain for yaw rate control
extern float max_forward_speed;    // Maximum allowed forward speed (m/s)
extern float max_lateral_speed;    // Maximum allowed lateral speed (m/s)
extern float max_yaw_rate;         // Maximum allowed yaw rate (deg/s)
extern float forward_dead_zone;    // Dead zone for forward distance error (meters)
extern float azimuth_dead_zone;    // Dead zone for azimuth angle error (degrees)
extern float yaw_azimuth_balance_dead_zone; // Dead zone for yaw balancing error (degrees)
extern std::atomic<bool> invertLateralControl; // Flag to invert lateral control direction
extern std::atomic<bool> invertYawControl;     // Flag to invert yaw control direction

// --- Vertical Control Parameters ---
extern float TARGET_ALTITUDE;      // Target altitude AGL for the descending phase (meters)
extern float VERTICAL_SPEED;       // Speed for ascending/descending (m/s, always positive)
extern float HOLD_DURATION_SECONDS;// Duration to hold position after first ascent (seconds)


// --- Sensor Selection for Control ---
// For Wall Distance (Forward Control)
extern std::atomic<bool> useWallSensorRAz;
extern std::atomic<bool> useWallSensorREl;
extern std::atomic<bool> useWallSensorRAzREl;
// For Beacon Azimuth (Lateral Control)
extern std::atomic<bool> useBeaconSensorRAz;
extern std::atomic<bool> useBeaconSensorREl;
extern std::atomic<bool> useBeaconSensorRAzREl;
// For Beacon Range (Vertical Control Trigger)
extern std::atomic<bool> useBeaconRangeSensorRAz;
extern std::atomic<bool> useBeaconRangeSensorREl;
extern std::atomic<bool> useBeaconRangeSensorRAzREl;
// For Yaw Balancing (Yaw Control)
extern std::atomic<bool> useYawSensorRAz;
extern std::atomic<bool> useYawSensorREl;
extern std::atomic<bool> useYawSensorRAzREl;


// --- Target Identification ---
extern std::string TARGET_BEACON_ID; // ID string of the target beacon

// --- Telemetry/Monitoring ---
const int TELEMETRY_TIMEOUT_SECONDS = 5; // Seconds before telemetry is considered timed out
const int CAMERA_DATA_TIMEOUT_SECONDS = 5; // Seconds before camera data is considered timed out

// --- Function Declarations ---
void loadPreferences();
void savePreferences(); // Optional: If you want to save changes made in GUI

#endif // CONFIG_HPP