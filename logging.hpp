#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <streambuf>
#include <fstream>
#include <iostream>
#include <string>
#include <mutex>
#include <memory> // For unique_ptr

// TeeBuf writes output to two streambufs (e.g., console and file)
class TeeBuf : public std::streambuf {
public:
    // Constructor takes pointers to the two streambufs to write to
    TeeBuf(std::streambuf* sb1, std::streambuf* sb2);

protected:
    // Called when buffer is full or on explicit flush/endl
    virtual int sync() override;

    // Called when a character needs to be written to the buffer
    virtual int_type overflow(int_type c = traits_type::eof()) override;

private:
    std::streambuf* sb1_;
    std::streambuf* sb2_;
    std::mutex mutex_; // Protect concurrent writes from different threads
};

// RAII class to manage redirection and restoration of streams (cout, cerr)
class LogRedirector {
public:
    // Constructor opens the log file and redirects cout and cerr
    LogRedirector(const std::string& log_filename);

    // Destructor restores original stream buffers and closes the log file
    ~LogRedirector();

    // Disable copy/move semantics to prevent issues with stream buffer management
    LogRedirector(const LogRedirector&) = delete;
    LogRedirector& operator=(const LogRedirector&) = delete;
    LogRedirector(LogRedirector&&) = delete;
    LogRedirector& operator=(LogRedirector&&) = delete;

private:
    std::ofstream log_file_;
    std::streambuf* original_cout_buf_;
    std::streambuf* original_cerr_buf_;
    std::unique_ptr<TeeBuf> cout_tee_buf_;
    std::unique_ptr<TeeBuf> cerr_tee_buf_;

    // Helper function to get a formatted timestamp string
    std::string getCurrentTimestamp();
};

#endif // LOGGING_HPP