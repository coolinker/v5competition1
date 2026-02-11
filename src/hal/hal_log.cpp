#include "hal/hal_log.h"
#include "vex.h"
#include <fstream>
#include <ctime>
#include <cstring>
#include <sstream>

using namespace vex;

// Extern reference to the Brain object defined in main.cpp
extern brain Brain;

// Log file path on VEX V5 Brain SD card (adjust if needed)
#define HAL_LOG_FILE "/usd/hal_log.txt"

void hal_log(const std::string& message, bool printToScreen) {
    // Get current time
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // Format timestamp using strftime (compatible with GCC 4.9.3 libstdc++)
    char timebuf[32];
    std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", now_tm);
    std::string timestamp(timebuf);

    // Compose log entry
    std::string log_entry = "[" + timestamp + "] " + message + "\n";

    // Append to log file
    std::ofstream log_file(HAL_LOG_FILE, std::ios::app);
    if (log_file.is_open()) {
        log_file << log_entry;
        log_file.close();
    }

    // Print to VEX Brain screen if requested
    if (printToScreen) {
        Brain.Screen.print("%s", log_entry.c_str());
        Brain.Screen.newLine();
    }
}
