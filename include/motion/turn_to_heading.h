#pragma once
// ============================================================================
//  motion/turn_to_heading.h — Point-turn to an absolute heading
// ============================================================================
//  Rotates the robot in place to face a given heading using PID control.
//  Uses settle-time and timeout to decide when the turn is "done".
//
//  FUTURE:
//    • Motion-profiled angular velocity (S-curve)
//    • Cascaded velocity → voltage PID
// ============================================================================

/// Turn the robot in place to face `target_heading_rad`.
/// Blocks until settled or timed out.
void turn_to_heading(double target_heading_rad);

/// Compute angular correction for a given heading error.
/// Exposed so drive_to_pose() can reuse the turn PID for heading correction.
double turn_to_heading_pid_calculate(double error);
