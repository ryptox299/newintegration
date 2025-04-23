#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include "types.hpp" // Include for ProcessingMode and VerticalAlignState enums

// Function to get a string representation of the flight mode
std::string getModeName(uint8_t mode);

// Function to get a string representation of the processing mode
std::string getProcessingModeName(ProcessingMode mode);

// Function to get a string representation of the vertical alignment state
std::string getVerticalStateName(VerticalAlignState state);


#endif // UTILS_HPP