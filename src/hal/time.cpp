// ============================================================================
//  hal/time.cpp — Timing utilities
// ============================================================================
#include "hal/time.h"
#include "vex.h"
#include "hal/hal_log.h"

double get_time_sec() {
    double t = vex::timer::system() / 1000.0;
    hal_log("Get time (sec): " + to_str(t));
    return t;
}

unsigned long get_time_ms() {
    unsigned long t = (unsigned long)vex::timer::system();
    hal_log("Get time (ms): " + to_str(t));
    return t;
}

void wait_ms(int ms) {
    hal_log("Wait ms: " + to_str(ms));
    vex::task::sleep(ms);
}
