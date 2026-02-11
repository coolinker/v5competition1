#pragma once
// ============================================================================
//  hal/time.h — Hardware abstraction for timing
// ============================================================================
//  Provides a clean interface for time measurement and delays.
//  In host-side tests, these are replaced by mock implementations.
// ============================================================================

#include "hal/hal_log.h"

/// Get elapsed time since program start, in seconds.
double get_time_sec();

/// Get elapsed time since program start, in milliseconds.
unsigned long get_time_ms();

/// Sleep for the given number of milliseconds.
/// Yields CPU to other tasks (important on V5's RTOS).
void wait_ms(int ms);
