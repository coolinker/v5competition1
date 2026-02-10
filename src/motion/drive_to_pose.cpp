// ============================================================================
//  motion/drive_to_pose.cpp — Turn-then-drive implementation
// ============================================================================
//
//  Phase 1  –  Turn in place toward (target.x, target.y)
//  Phase 2  –  Drive forward:
//    • MotionProfile supplies a smooth target velocity (trapezoidal)
//    • PID computes a linear correction from distance error
//    • Heading error is corrected with a proportional term so the
//      robot tracks a straight line even if bumped or wheels slip
//
//  Exit conditions:
//    • Distance within DRIVE_SETTLE_M for DRIVE_SETTLE_TIME_MS → success
//    • DRIVE_TIMEOUT_MS exceeded → abort
//
// ============================================================================
#include "motion/drive_to_pose.h"
#include "config.h"
#include "motion/turn_to_heading.h"
#include "control/pid.h"
#include "control/motion_profile.h"
#include "hal/motors.h"
#include "hal/time.h"
#include "localization/odometry.h"
#include <cmath>

void drive_to_pose(const Pose& target_pose) {
    // --- Compute approach heading & total distance ---
    Pose start = get_pose();
    double dx = target_pose.x - start.x;
    double dy = target_pose.y - start.y;
    double target_heading = atan2(dy, dx);

    // Phase 1: rotate to face the target
    turn_to_heading(target_heading);

    // Phase 2: drive forward with profiled velocity + heading correction
    PIDController drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD);
    MotionProfile profile(MAX_VELOCITY, MAX_ACCELERATION);
    drive_pid.reset();

    unsigned long drive_start = get_time_ms();
    unsigned long settle_start = 0;
    bool settling = false;

    while (true) {
        // --- Timeout ---
        unsigned long elapsed = get_time_ms() - drive_start;
        if (elapsed > DRIVE_TIMEOUT_MS) break;

        // --- Distance remaining ---
        Pose cur = get_pose();
        dx = target_pose.x - cur.x;
        dy = target_pose.y - cur.y;
        double dist_to_go = sqrt(dx * dx + dy * dy);

        // --- Settle detection ---
        if (dist_to_go < DRIVE_SETTLE_M) {
            if (!settling) {
                settling = true;
                settle_start = get_time_ms();
            } else if (get_time_ms() - settle_start >= DRIVE_SETTLE_TIME_MS) {
                break;  // close enough for long enough → done
            }
        } else {
            settling = false;
        }

        // --- Motion profile → target velocity ---
        double time_sec = (get_time_ms() - drive_start) / 1000.0;
        double target_v = profile.get_target_velocity(time_sec, dist_to_go);

        // --- Heading correction (keeps robot driving straight) ---
        double heading_error = target_heading - cur.theta;
        heading_error = atan2(sin(heading_error), cos(heading_error));
        double angular_correction = HEADING_CORRECTION_KP * heading_error;

        // --- Differential drive kinematics ---
        double left_v  = target_v - angular_correction * WHEEL_TRACK / 2.0;
        double right_v = target_v + angular_correction * WHEEL_TRACK / 2.0;
        set_drive_motors(left_v, right_v);

        wait_ms(LOOP_INTERVAL_MS);
    }

    stop_drive_motors();
}
