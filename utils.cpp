#include "utils.hpp"
#include "dji_status.hpp" // For VehicleStatus enums

// Function to get a string representation of the flight mode
std::string getModeName(uint8_t mode) {
    switch(mode) {
        // --- Modes likely present in OSDK 3.x ---
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL:       return "MANUAL_CTRL";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE:          return "ATTITUDE";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS:             return "P_GPS"; // Re-add common older modes
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF:  return "ASSISTED_TAKEOFF";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF:      return "AUTO_TAKEOFF";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING:      return "AUTO_LANDING";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME:      return "NAVI_GO_HOME";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL:     return "NAVI_SDK_CTRL"; // The mode we want for control
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING: return "FORCE_AUTO_LANDING";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_SEARCH_MODE:       return "SEARCH_MODE";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START:      return "ENGINE_START";

        // --- Remove modes likely NOT present in OSDK 3.x ---
        // case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_POS_HOLD:     return "POS_HOLD";
        // case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_FENCE:        return "FENCE";
        // case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_AVOID_OBSTACLE: return "AVOID_OBSTACLE";
        // case DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_ASSISTANT_SET: return "FORCE_ASSISTANT_SET";
        // case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_VEL_CTRL:     return "VEL_CTRL";

        // --- Special/Unknown Cases ---
        case 255: return "TIMEOUT/INVALID"; // Special value used in our code
        default: return "UNKNOWN (" + std::to_string(mode) + ")";
    }
}

// Function to get a string representation of the processing mode
std::string getProcessingModeName(ProcessingMode mode) {
    switch(mode) {
        case ProcessingMode::PROCESS_FULL: return "Process Full";
        case ProcessingMode::WALL_FOLLOW: return "Wall+LatBeacon";
        case ProcessingMode::WALL_BEACON_VERTICAL: return "Wall+LatBeacon+Vert";
        case ProcessingMode::WALL_BEACON_YAW: return "Wall+LatBeacon+Yaw";
        case ProcessingMode::WALL_BEACON_YAW_VERTICAL: return "Wall+LatBeacon+Yaw+Vert";
        default: return "Unknown Mode";
    }
}

// Function to get a string representation of the vertical alignment state
std::string getVerticalStateName(VerticalAlignState state) {
    switch(state) {
        case VerticalAlignState::IDLE: return "IDLE";
        case VerticalAlignState::ALIGN_HORIZONTAL: return "ALIGN_HORIZONTAL";
        case VerticalAlignState::ASCENDING_HOLD: return "ASCENDING_HOLD";
        case VerticalAlignState::DESCENDING: return "DESCENDING";
        case VerticalAlignState::FINAL_HOLD: return "FINAL_HOLD";
        default: return "Unknown State";
    }
}