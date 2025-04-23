#ifndef PYTHON_BRIDGE_HPP
#define PYTHON_BRIDGE_HPP

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

// Function to start the Python bridge script
bool runPythonBridge(const std::string& scriptPath);

// Function to stop the Python bridge script
void stopPythonBridge(const std::string& scriptPath);

// Function to read a line from the Python bridge stdout
// Returns true if a line was read, false otherwise (e.g., timeout, no data)
// timeout_ms: Maximum time to wait for a line in milliseconds
bool readLineFromBridge(std::string& line, int timeout_ms = 100);


#endif // PYTHON_BRIDGE_HPP