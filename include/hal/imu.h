#pragma once
// ============================================================================
//  hal/imu.h — Hardware abstraction for the inertial sensor (IMU)
// ============================================================================
//  The V5 inertial sensor provides heading (yaw) which is far more stable
//  than encoder-only heading. We fuse both in odometry.cpp.
//
//  FUTURE: Add get_imu_pitch(), get_imu_roll() for tilt detection.
// ============================================================================

/// Get current heading from the IMU in radians [0, 2π).
/// Automatically converts from VEX degrees to radians.
double get_imu_heading_rad();

/// Get total rotation (can exceed 360°) in radians.
/// Useful for tracking cumulative turns.
double get_imu_rotation_rad();

/// Reset IMU heading to zero.
void reset_imu();

/// Calibrate the IMU. Blocks until calibration is complete.
/// Call once during pre_auton().
void calibrate_imu();
