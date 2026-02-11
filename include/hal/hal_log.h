#pragma once
#include <string>
#include <stdio.h>

// Workaround: std::to_string is not available in VEX V5 GCC 4.9.3 libstdc++
template<typename T>
inline std::string to_str(T value) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%g", static_cast<double>(value));
    return std::string(buf);
}

void hal_log(const std::string& message, bool printToScreen = true);
