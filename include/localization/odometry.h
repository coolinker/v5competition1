#pragma once
// ============================================================================
//  localization/odometry.h — Robot position tracking
// ============================================================================
//  Odometry estimates the robot's (x, y, θ) position on the field by
//  integrating wheel encoder deltas and fusing with the IMU heading.
//
//  Coordinate system:
//    • x = forward (meters)
//    • y = left    (meters)
//    • θ = counter-clockwise from +x axis (radians)
//
//  Call odometry_update() every 10ms in your main loop.
//
//  FUTURE:
//    • Add tracking wheels (passive encoders) for higher accuracy
//    • Add Kalman filter for multi-sensor fusion
//    • Add field-relative coordinate transforms
// ============================================================================

/// Robot pose: position + heading on the field
struct Pose {
    double x;      ///< forward position (meters)
    double y;      ///< lateral position (meters)
    double theta;  ///< heading (radians, CCW positive)
};

/// Call every loop iteration (~10ms) to update the pose estimate.
void odometry_update();

/// Get the current estimated pose.
Pose get_pose();

/// Manually set the pose (e.g. at the start of autonomous).
void set_pose(const Pose& new_pose);
