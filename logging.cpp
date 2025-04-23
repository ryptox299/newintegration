#include "logging.hpp"
#include <chrono>    // For system_clock
#include <iomanip>   // For std::put_time
#include <sstream>   // For stringstream
#include <ctime>     // For time_t, tm, localtime_r/localtime_s

// --- TeeBuf Implementation ---

TeeBuf::TeeBuf(std::streambuf* sb1, std::streambuf* sb2) : sb1_(sb1), sb2_(sb2) {}

int TeeBuf::sync() {
    std::lock_guard<std::mutex> lock(mutex_);
    int r1 = sb1_->pubsync();
    int r2 = sb2_->pubsync();
    return (r1 == 0 && r2 == 0) ? 0 : -1; // Success if both sync successfully
}

std::streambuf::int_type TeeBuf::overflow(int_type c) {
    if (traits_type::eq_int_type(c, traits_type::eof())) {
        // If EOF is passed, just sync the buffers
        return sync() == -1 ? traits_type::eof() : traits_type::not_eof(c);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    // Write the character to both underlying buffers
    int_type const r1 = sb1_->sputc(c);
    int_type const r2 = sb2_->sputc(c);

    // Check if either write failed
    if (traits_type::eq_int_type(r1, traits_type::eof()) ||
        traits_type::eq_int_type(r2, traits_type::eof())) {
        return traits_type::eof(); // Indicate error
    }
    return traits_type::not_eof(c); // Indicate success
}

// --- LogRedirector Implementation ---

LogRedirector::LogRedirector(const std::string& log_filename)
    : log_file_(log_filename, std::ios::app), // Open in append mode
      original_cout_buf_(nullptr),
      original_cerr_buf_(nullptr)
{
    if (!log_file_.is_open()) {
        // Use original cerr because redirection hasn't happened yet
        // Save original cerr temporarily in case we need to restore it
        original_cerr_buf_ = std::cerr.rdbuf();
        std::cerr << "FATAL ERROR: Could not open log file: " << log_filename << std::endl;
        // Restore original cerr buffer if it was saved
        if (original_cerr_buf_) std::cerr.rdbuf(original_cerr_buf_);
        // Reset pointers as redirection won't happen
        original_cerr_buf_ = nullptr;
        return; // Don't redirect if file couldn't be opened
    }

    original_cout_buf_ = std::cout.rdbuf(); // Save original cout buffer
    original_cerr_buf_ = std::cerr.rdbuf(); // Save original cerr buffer

    // Create TeeBuf instances using unique_ptr
    // Use reset(new ...) for C++11 compatibility if std::make_unique isn't available/desired
    cout_tee_buf_.reset(new TeeBuf(original_cout_buf_, log_file_.rdbuf()));
    cerr_tee_buf_.reset(new TeeBuf(original_cerr_buf_, log_file_.rdbuf())); // Also log cerr to the same file

    // Redirect cout and cerr to the TeeBuf instances
    std::cout.rdbuf(cout_tee_buf_.get());
    std::cerr.rdbuf(cerr_tee_buf_.get());

    std::cout << "\n--- Log Start [" << getCurrentTimestamp() << "] ---" << std::endl; // Added newline for separation
}

LogRedirector::~LogRedirector() {
     // Ensure logs end with a clear marker and newline
     std::cout << "--- Log End [" << getCurrentTimestamp() << "] ---\n" << std::endl;

    // Flush streams before restoring original buffers
    std::cout.flush();
    std::cerr.flush();

    // Restore original buffers only if redirection actually happened
    if (cout_tee_buf_ && original_cout_buf_) {
        std::cout.rdbuf(original_cout_buf_);
    }
    if (cerr_tee_buf_ && original_cerr_buf_) {
        std::cerr.rdbuf(original_cerr_buf_);
    }

    // log_file_ is closed automatically by its ofstream destructor
    // unique_ptrs clean up TeeBuf instances automatically
}

std::string LogRedirector::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    struct tm buf;

    #ifdef _WIN32 // Check for Windows (_WIN32 is usually defined by MSVC, MinGW, etc.)
        // Use Microsoft's secure version
        localtime_s(&buf, &now_c);
    #else
        // Use POSIX thread-safe version
        localtime_r(&now_c, &buf);
    #endif

    ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}