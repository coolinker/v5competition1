#pragma once
// ============================================================================
//  motion/drive_to_pose.h — Drive to an (x, y) position on the field
// ============================================================================
//  6-motor (Boomerang controller):
//    Drives a smooth curved path toward (x, y) arriving at heading θ.
//    A "carrot" point offset from the target guides the curve.
//    Supports reverse driving.
//
//  2-motor (turn-then-drive, unchanged):
//    Phase 1: Turn in place toward the target
//    Phase 2: Drive forward with profiled velocity + heading correction
//
//  FUTURE:
//    • Pure-pursuit for multi-waypoint path following
// ============================================================================
#include "localization/odometry.h"

/// Drive to the given (x, y, θ) pose on the field.
///
/// 6-motor (Boomerang): drives a smooth curved path, arrives at the target
/// with the desired final heading θ.  Set reverse=true to drive backward.
///
/// 2-motor (turn-then-drive): turns to face target, drives straight.
/// theta is used for approach heading; final heading is NOT enforced.
///
/// Blocks until settled or timed out.
void drive_to_pose(const Pose& target_pose, bool reverse = false);
