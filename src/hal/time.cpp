// ============================================================================
//  hal/time.cpp — Timing utilities
// ============================================================================
#include "hal/time.h"
#include "vex.h"

double get_time_sec() {
    return vex::timer::system() / 1000.0;
}

unsigned long get_time_ms() {
    return (unsigned long)vex::timer::system();
}

void wait_ms(int ms) {
    vex::task::sleep(ms);
}
