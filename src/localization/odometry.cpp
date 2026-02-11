// ============================================================================
//  localization/odometry.cpp — Differential drive odometry with IMU fusion
// ============================================================================
//
//  Math overview (discrete approximation):
//
//    Δs_L = (ticks_L / TICKS_PER_REV) × π × D   (left wheel distance)
//    Δs_R = (ticks_R / TICKS_PER_REV) × π × D   (right wheel distance)
//    Δs   = (Δs_L + Δs_R) / 2                    (center distance)
//    Δθ   = (Δs_R − Δs_L) / W                    (heading change)
//
//    We FUSE θ with IMU for drift correction:
//    θ_fused = α × θ_imu + (1−α) × θ_enc
//
//    x += Δs × cos(θ + Δθ/2)     (midpoint approximation)
//    y += Δs × sin(θ + Δθ/2)
//
// ============================================================================
#include "localization/odometry.h"
#include "config.h"
#include "hal/motors.h"
#include "hal/imu.h"
#include <cmath>

// ── Internal state ──────────────────────────────────────────────────────────
static Pose current_pose = {0.0, 0.0, 0.0};
static double prev_left_ticks  = 0.0;
static double prev_right_ticks = 0.0;
#ifdef ROBOT_6MOTOR
static double prev_imu_rotation = 0.0;
#endif

void odometry_update() {
    // 1. Read current encoder positions
    double left_ticks  = get_left_encoder_ticks();
    double right_ticks = get_right_encoder_ticks();

    // 2. Compute delta ticks since last update
    double dL = left_ticks  - prev_left_ticks;
    double dR = right_ticks - prev_right_ticks;

    // 3. Convert ticks → meters
    double dist_L = (dL / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
    double dist_R = (dR / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;

    // 4. Center displacement and encoder heading change
    double ds         = (dist_L + dist_R) / 2.0;
    double dtheta_enc = (dist_R - dist_L) / WHEEL_TRACK;

    // 5. Fuse heading with IMU
#ifdef ROBOT_6MOTOR
    // Competition: fuse DELTAS to avoid heading() wrap-around at 0°/360°
    double imu_rotation = get_imu_rotation_rad();
    double dtheta_imu   = imu_rotation - prev_imu_rotation;
    prev_imu_rotation   = imu_rotation;
    double dtheta = IMU_FUSION_ALPHA * dtheta_imu
                  + (1.0 - IMU_FUSION_ALPHA) * dtheta_enc;
#else
    // Entry-level: simple absolute fusion (kept for 2-motor prototype)
    double theta_imu = get_imu_heading_rad();
    double theta_enc = current_pose.theta + dtheta_enc;
    double theta_fused = IMU_FUSION_ALPHA * theta_imu
                       + (1.0 - IMU_FUSION_ALPHA) * theta_enc;
    double dtheta = theta_fused - current_pose.theta;
#endif

    // 6. Update pose (midpoint approximation for curved paths)
    double mid_theta = current_pose.theta + dtheta / 2.0;
    current_pose.x     += ds * cos(mid_theta);
    current_pose.y     += ds * sin(mid_theta);
    current_pose.theta += dtheta;

    // 7. Save for next iteration
    prev_left_ticks  = left_ticks;
    prev_right_ticks = right_ticks;
}

Pose get_pose() {
    return current_pose;
}

void set_pose(const Pose& new_pose) {
    current_pose = new_pose;
    reset_encoders();
    reset_imu();
    prev_left_ticks  = 0;
    prev_right_ticks = 0;
#ifdef ROBOT_6MOTOR
    prev_imu_rotation = 0.0;
#endif
}
