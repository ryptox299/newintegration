#include "python_bridge.hpp"
#include "globals.hpp" // For currentBridgeStatus
#include "config.hpp"  // For BRIDGE_CONNECT_TIMEOUT_SECONDS

#include <iostream>
#include <cstdio>   // For popen, pclose, fgets
#include <thread>   // For sleep_for
#include <chrono>   // For milliseconds
#include <sys/wait.h> // For waitpid
#include <signal.h>   // For kill
#include <unistd.h>   // For pipe, fork, dup2, execvp, close, read
#include <fcntl.h>    // For fcntl, F_SETFL, O_NONBLOCK
#include <errno.h>    // For errno
#include <cstring>  // For strerror
#include <vector>   // For std::vector in readLineFromBridge

// --- Globals specific to the bridge ---
namespace { // Use anonymous namespace for file-local globals
    pid_t python_pid = -1;
    int   pipe_fd[2] = {-1, -1}; // File descriptors for the pipe [0]=read, [1]=write
    std::mutex bridge_mutex; // Mutex to protect access to python_pid and pipe_fd
}
// --- End Globals ---

// Function to start the Python bridge script
// Returns true if process started successfully, false otherwise
bool runPythonBridge(const std::string& scriptName) {
    std::lock_guard<std::mutex> lock(bridge_mutex); // Protect access

    // Ensure any previous instance is stopped
    if (python_pid != -1) {
         std::cout << "[Bridge] Warning: Attempting to start bridge while a process (PID: " << python_pid << ") might still exist. Trying to stop it first." << std::endl;
         stopPythonBridge(scriptName); // Call internal stop which also locks
         // Re-acquire lock is implicitly handled by lock_guard destructor/constructor
    }

    std::cout << "[Bridge] Creating pipe..." << std::endl;
    if (pipe(pipe_fd) == -1) {
        std::cerr << "[Bridge] Error creating pipe: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "[Bridge] Forking process for script: " << scriptName << "..." << std::endl;
    python_pid = fork();

    if (python_pid < 0) {
        // Fork failed
        std::cerr << "[Bridge] Error forking process: " << strerror(errno) << std::endl;
        close(pipe_fd[0]); // Clean up pipe
        close(pipe_fd[1]);
        pipe_fd[0] = pipe_fd[1] = -1;
        python_pid = -1;
        return false;
    } else if (python_pid == 0) {
        // --- Child process ---
        // Redirect stdout to the write end of the pipe
        close(pipe_fd[0]); // Close read end in child
        if (dup2(pipe_fd[1], STDOUT_FILENO) == -1) {
             perror("[Bridge Child] Error redirecting stdout");
             close(pipe_fd[1]);
             _exit(EXIT_FAILURE); // Use _exit in child after fork
        }
        // Optionally redirect stderr as well (can be useful for debugging Python errors)
        // dup2(pipe_fd[1], STDERR_FILENO); // Uncomment to redirect stderr
        close(pipe_fd[1]); // Close original write end after duplication

        // Prepare arguments for execvp
        // Assumes python3 is in PATH and scriptName is relative/absolute path
        char* const argv[] = { (char*)"python3", (char*)scriptName.c_str(), nullptr };

        // Execute the python script
        std::cout << "[Bridge Child] Executing python3 " << scriptName << "..." << std::endl; // This might not appear if stdout is fully redirected
        execvp("python3", argv);

        // execvp only returns if an error occurred
        perror("[Bridge Child] Error executing script");
        _exit(EXIT_FAILURE); // Exit child process on error
    } else {
        // --- Parent process ---
        close(pipe_fd[1]); // Close write end in parent
        pipe_fd[1] = -1;   // Mark write end as closed

        // Set the read end of the pipe to non-blocking
        int flags = fcntl(pipe_fd[0], F_GETFL, 0);
        if (flags == -1) {
            std::cerr << "[Bridge] Error getting pipe flags: " << strerror(errno) << std::endl;
            // Consider stopping the child process here if flags can't be set
            kill(python_pid, SIGTERM);
            waitpid(python_pid, nullptr, 0);
            close(pipe_fd[0]);
            pipe_fd[0] = -1;
            python_pid = -1;
            return false;
        }
        if (fcntl(pipe_fd[0], F_SETFL, flags | O_NONBLOCK) == -1) {
            std::cerr << "[Bridge] Error setting pipe non-blocking: " << strerror(errno) << std::endl;
            // Consider stopping the child process here
            kill(python_pid, SIGTERM);
            waitpid(python_pid, nullptr, 0);
            close(pipe_fd[0]);
            pipe_fd[0] = -1;
            python_pid = -1;
            return false;
        }

        std::cout << "[Bridge] Python script started (PID: " << python_pid << "). Read pipe FD: " << pipe_fd[0] << std::endl;

        // --- Connection Check ---
        std::cout << "[Bridge] Waiting for initial connection confirmation..." << std::endl;
        std::string connect_line;
        bool connected = false;
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < BRIDGE_CONNECT_TIMEOUT_SECONDS * 2) // Double timeout for initial connect
        {
            if (readLineFromBridge(connect_line, 100)) { // Check for output
                 if (connect_line.find("PYTHON_BRIDGE_CONNECTED") != std::string::npos) {
                      std::cout << "[Bridge] Connection confirmed by Python script." << std::endl;
                      connected = true;
                      currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED);
                      break;
                 } else {
                      // Print other initial output for debugging
                      std::cout << "[Bridge Init Output] " << connect_line << std::endl;
                 }
            } else {
                 // Check if the process exited prematurely
                 int status;
                 pid_t result = waitpid(python_pid, &status, WNOHANG);
                 if (result == python_pid) {
                     std::cerr << "[Bridge] Error: Python process exited prematurely during connection check." << std::endl;
                     if (WIFEXITED(status)) { std::cerr << "  Exit code: " << WEXITSTATUS(status) << std::endl; }
                     else if (WIFSIGNALED(status)) { std::cerr << "  Terminated by signal: " << WTERMSIG(status) << std::endl; }
                     close(pipe_fd[0]);
                     pipe_fd[0] = -1;
                     python_pid = -1;
                     currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                     return false; // Indicate failure
                 } else if (result == -1) {
                      std::cerr << "[Bridge] Error checking child process status with waitpid: " << strerror(errno) << std::endl;
                      // Continue trying to connect, but log the error
                 }
                 // If result is 0, process is still running, continue waiting
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Short sleep between checks
        }

        if (!connected) {
            std::cerr << "[Bridge] Error: Timed out waiting for connection confirmation from Python script." << std::endl;
            stopPythonBridge(scriptName); // Clean up the process
            currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
            return false; // Indicate failure
        }

        return true; // Indicate success
    }
}

// Function to stop the Python bridge script
void stopPythonBridge(const std::string& scriptName /* unused for now */) {
    std::lock_guard<std::mutex> lock(bridge_mutex); // Protect access

    if (python_pid != -1) {
        std::cout << "[Bridge] Stopping Python script (PID: " << python_pid << ")..." << std::endl;

        // Close the read end of the pipe first
        if (pipe_fd[0] != -1) {
            close(pipe_fd[0]);
            pipe_fd[0] = -1;
             std::cout << "[Bridge] Closed read pipe." << std::endl;
        }
         // Write end should already be closed in parent (pipe_fd[1] == -1)

        // Send SIGTERM first for graceful shutdown
        if (kill(python_pid, SIGTERM) == 0) {
            std::cout << "[Bridge] Sent SIGTERM to PID " << python_pid << ". Waiting..." << std::endl;
            // Wait for a short period for the process to exit
            bool exited = false;
            for (int i = 0; i < 50; ++i) { // Wait up to 5 seconds (50 * 100ms)
                int status;
                pid_t result = waitpid(python_pid, &status, WNOHANG);
                if (result == python_pid) {
                    std::cout << "[Bridge] Process " << python_pid << " exited gracefully." << std::endl;
                    exited = true;
                    break;
                } else if (result == -1) {
                    std::cerr << "[Bridge] Error waiting for process " << python_pid << ": " << strerror(errno) << std::endl;
                    break; // Stop waiting if waitpid fails
                }
                // If result is 0, process still running
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (!exited) {
                std::cout << "[Bridge] Process " << python_pid << " did not exit after SIGTERM. Sending SIGKILL." << std::endl;
                kill(python_pid, SIGKILL); // Force kill
                waitpid(python_pid, nullptr, 0); // Wait for it to be killed
            }
        } else {
            std::cerr << "[Bridge] Error sending SIGTERM to PID " << python_pid << ": " << strerror(errno) << ". Process might already be dead." << std::endl;
             // Attempt to wait anyway, in case it's a zombie
             waitpid(python_pid, nullptr, WNOHANG);
        }

        python_pid = -1; // Mark as stopped
    } else {
        // std::cout << "[Bridge] No active Python script process to stop." << std::endl;
    }

    // Ensure pipe is fully closed if stop is called without start succeeding fully
    if (pipe_fd[0] != -1) { close(pipe_fd[0]); pipe_fd[0] = -1; }
    if (pipe_fd[1] != -1) { close(pipe_fd[1]); pipe_fd[1] = -1; } // Should be -1 already

    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Ensure status is updated
}


// Function to read a line from the Python bridge stdout
bool readLineFromBridge(std::string& line, int timeout_ms) {
    // This function needs to read from pipe_fd[0] which is non-blocking
    // It should accumulate characters until a newline is found or timeout occurs.

    std::lock_guard<std::mutex> lock(bridge_mutex); // Protect pipe_fd access

    if (pipe_fd[0] == -1) {
        // Pipe is not open (process not running or failed to start)
        // currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Maybe update status here?
        return false;
    }

    static std::vector<char> buffer; // Static buffer to handle partial lines across calls
    line.clear();
    char read_buf[256];
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // Check if a complete line is already in the buffer
        bool newline_found = false;
        size_t newline_pos = 0;
        for (size_t i = 0; i < buffer.size(); ++i) {
            if (buffer[i] == '\n') {
                newline_found = true;
                newline_pos = i;
                break;
            }
        }

        if (newline_found) {
            // Copy line from buffer (excluding newline)
            line.assign(buffer.begin(), buffer.begin() + newline_pos);
            // Remove line (including newline) from buffer
            buffer.erase(buffer.begin(), buffer.begin() + newline_pos + 1);
            return true;
        }

        // If no newline, try reading more data
        ssize_t bytes_read = read(pipe_fd[0], read_buf, sizeof(read_buf) - 1);

        if (bytes_read > 0) {
            // Append new data to buffer
            buffer.insert(buffer.end(), read_buf, read_buf + bytes_read);
            // Reset timeout start time as we received data
            start_time = std::chrono::steady_clock::now();
            // Loop again to check if the new data completed a line
            continue;
        } else if (bytes_read == 0) {
            // Pipe closed (EOF) - Python script likely exited
            std::cerr << "[Bridge] Read returned 0 (EOF). Python script may have exited." << std::endl;
            close(pipe_fd[0]);
            pipe_fd[0] = -1;
            python_pid = -1; // Assume process is gone
            currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
            // Return any remaining data in the buffer as the last line
            if (!buffer.empty()) {
                 line.assign(buffer.begin(), buffer.end());
                 buffer.clear();
                 return true;
            }
            return false; // No more data, pipe closed
        } else { // bytes_read < 0
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available right now (non-blocking read)
                // Check timeout
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() >= timeout_ms) {
                    // Timeout reached, return false (no complete line read)
                     // Do NOT clear buffer here, might contain partial line for next call
                    return false;
                }
                // Wait briefly before trying to read again
                std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Small sleep to avoid busy-waiting
            } else {
                // Actual read error
                std::cerr << "[Bridge] Error reading from pipe: " << strerror(errno) << std::endl;
                close(pipe_fd[0]);
                pipe_fd[0] = -1;
                python_pid = -1; // Assume process is gone or pipe broken
                currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                // Return any remaining data in the buffer
                 if (!buffer.empty()) {
                     line.assign(buffer.begin(), buffer.end());
                     buffer.clear();
                     return true;
                 }
                return false;
            }
        }
    } // end while(true)
}
