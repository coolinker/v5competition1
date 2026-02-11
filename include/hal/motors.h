#pragma once
// ============================================================================
//  hal/motors.h — Hardware abstraction for drivetrain motors
// ============================================================================
//  Why this exists:
//    All motor access goes through these functions. When you change hardware
//    (e.g. switch from 2-motor to 6-motor drive), you only edit motors.cpp.
//    The rest of the codebase stays untouched.
//
//  ARCHITECTURE HIGHLIGHT:
//    The API below is IDENTICAL for 2-motor and 6-motor configurations.
//    set_drive_motors() sends the same voltage to ALL motors on a side.
//    Odometry reads from a single "primary" encoder per side.
//    Upper layers (PID, motion profile, drive_to_pose, turn_to_heading)
//    are completely unaware of how many physical motors exist.
//
//  FUTURE: Add functions for intake, catapult, flywheel, etc.
// ============================================================================

#include "hal/hal_log.h"

/// Send voltage to left and right drive motors.
/// In multi-motor configs, all motors on a side receive the same voltage.
/// @param left_voltage  -12.0 to +12.0 (volts, positive = forward)
/// @param right_voltage -12.0 to +12.0
void set_drive_motors(double left_voltage, double right_voltage);

/// Stop all drive motors immediately (coast / brake depending on config).
void stop_drive_motors();

/// Read cumulative encoder position (ticks) for left side.
/// In multi-motor configs, reads from the designated primary encoder motor.
double get_left_encoder_ticks();

/// Read cumulative encoder position (ticks) for right side.
/// In multi-motor configs, reads from the designated primary encoder motor.
double get_right_encoder_ticks();

/// Zero both drive encoders.
void reset_encoders();
