// ============================================================================
//  motion/drive_to_pose.cpp — Drive to an (x, y, θ) pose
// ============================================================================
//
//  6-MOTOR — Boomerang controller:
//    A "carrot" point is placed behind the target along its heading.
//    The robot aims at the carrot; as distance shrinks, the carrot
//    converges to the target, producing a smooth curved approach
//    that arrives at the correct final heading.  Supports reverse.
//
//  2-MOTOR — Turn-then-drive (entry-level, unchanged):
//    Phase 1: turn in place toward the target
//    Phase 2: drive forward with profiled velocity + heading correction
//
//  Exit conditions (both):
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

void drive_to_pose(const Pose& target_pose, bool reverse) {

#ifdef ROBOT_6MOTOR
    // ════════════════════════════════════════════════════════════════════════
    //  Boomerang controller — smooth curved approach with final heading
    // ════════════════════════════════════════════════════════════════════════
    //
    //  Instead of stopping to turn then driving straight, we aim at a
    //  "carrot" point offset from the target along its heading vector:
    //
    //           carrot <── lead ──── Target
    //             ╱                     ↑ θ_final
    //          Robot
    //
    //  As distance shrinks, the carrot converges to the target, curving
    //  the path so the robot arrives at the correct heading.
    //
    PIDController angular_pid(TURN_KP, TURN_KI, TURN_KD);
    angular_pid.set_integral_limit(TURN_INTEGRAL_LIMIT);
    angular_pid.set_d_filter(TURN_D_FILTER);
    angular_pid.set_output_limit(12.0);
    angular_pid.reset();

    unsigned long start_time = get_time_ms();
    unsigned long settle_start = 0;
    bool settling = false;
    double prev_cmd_v = 0.0;

    while (true) {
        if (get_time_ms() - start_time > DRIVE_TIMEOUT_MS) break;

        Pose cur = get_pose();
        double dx = target_pose.x - cur.x;
        double dy = target_pose.y - cur.y;
        double dist = sqrt(dx * dx + dy * dy);

        // ── Settle detection ──
        if (dist < DRIVE_SETTLE_M) {
            if (!settling) {
                settling = true;
                settle_start = get_time_ms();
            } else if (get_time_ms() - settle_start >= DRIVE_SETTLE_TIME_MS) {
                break;
            }
        } else {
            settling = false;
        }

        // ── Boomerang carrot point ──
        // Offset from target backwards along its heading, scaled by distance
        double carrot_x = target_pose.x - BOOMERANG_LEAD * dist * cos(target_pose.theta);
        double carrot_y = target_pose.y - BOOMERANG_LEAD * dist * sin(target_pose.theta);

        double target_heading = atan2(carrot_y - cur.y, carrot_x - cur.x);
        if (reverse) target_heading += M_PI;

        double heading_error = atan2(sin(target_heading - cur.theta),
                                     cos(target_heading - cur.theta));

        // ── Linear velocity: decel-limited, capped, cosine-throttled ──
        double decel_v = sqrt(2.0 * MAX_ACCELERATION * dist);
        double raw_v = (decel_v < MAX_VELOCITY) ? decel_v : MAX_VELOCITY;

        // Slow down when heading is far off (prevents driving sideways)
        double cos_err = cos(heading_error);
        if (cos_err < 0.0) cos_err = 0.0;
        raw_v *= cos_err;
        if (reverse) raw_v = -raw_v;

        // Acceleration slew rate
        double max_dv = MAX_ACCELERATION * (LOOP_INTERVAL_MS / 1000.0);
        if (raw_v - prev_cmd_v >  max_dv) raw_v = prev_cmd_v + max_dv;
        if (prev_cmd_v - raw_v >  max_dv) raw_v = prev_cmd_v - max_dv;
        prev_cmd_v = raw_v;

        // ── Angular correction (full PID) ──
        double omega = angular_pid.calculate(0.0, -heading_error);

        // ── Differential drive ──
        double left_v  = raw_v - omega * WHEEL_TRACK / 2.0;
        double right_v = raw_v + omega * WHEEL_TRACK / 2.0;
        set_drive_motors(left_v, right_v);

        wait_ms(LOOP_INTERVAL_MS);
    }

    stop_drive_motors();

#else
    // ════════════════════════════════════════════════════════════════════════
    //  Turn-then-drive (entry-level, unchanged)
    // ════════════════════════════════════════════════════════════════════════
    (void)reverse;  // not supported in entry-level

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
#endif
}
