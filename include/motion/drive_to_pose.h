#pragma once
// ============================================================================
//  motion/drive_to_pose.h — Drive to an (x, y) position on the field
// ============================================================================
//  Two-phase approach:
//    Phase 1: Turn in place toward the target (turn_to_heading)
//    Phase 2: Drive forward with motion-profiled velocity + heading correction
//
//  FUTURE:
//    • Pure-pursuit or Ramsete controller for curved paths
//    • Reverse driving when target is behind the robot
//    • Final heading alignment after reaching the target
// ============================================================================
#include "localization/odometry.h"

/// Drive to the given (x, y) coordinate.  theta in target_pose is used
/// only for the initial approach heading; final heading is NOT enforced.
/// Blocks until settled or timed out.
void drive_to_pose(const Pose& target_pose);
