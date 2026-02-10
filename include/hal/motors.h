#pragma once
// ============================================================================
//  hal/motors.h — Hardware abstraction for drivetrain motors
// ============================================================================
//  Why this exists:
//    All motor access goes through these functions. When you change hardware
//    (e.g. switch from 2-motor to 6-motor drive), you only edit motors.cpp.
//    The rest of the codebase stays untouched.
//
//  FUTURE: Add functions for intake, catapult, flywheel, etc.
// ============================================================================

/// Send voltage to left and right drive motors.
/// @param left_voltage  -12.0 to +12.0 (volts, positive = forward)
/// @param right_voltage -12.0 to +12.0
void set_drive_motors(double left_voltage, double right_voltage);

/// Stop all drive motors immediately (coast / brake depending on config).
void stop_drive_motors();

/// Read cumulative encoder position (ticks) for left side.
double get_left_encoder_ticks();

/// Read cumulative encoder position (ticks) for right side.
double get_right_encoder_ticks();

/// Zero both drive encoders.
void reset_encoders();
