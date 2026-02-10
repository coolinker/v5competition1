// ============================================================================
//  motion/turn_to_heading.cpp — Point-turn implementation
// ============================================================================
//
//  Algorithm:
//    1. Compute heading error, normalized to [-π, +π]
//    2. Feed error through PID → angular correction
//    3. Convert angular correction to differential wheel voltages:
//         left  = -ω × (track_width / 2)
//         right = +ω × (track_width / 2)
//    4. Exit when error stays small for TURN_SETTLE_TIME_MS,
//       or when TURN_TIMEOUT_MS is exceeded.
//
// ============================================================================
#include "motion/turn_to_heading.h"
#include "config.h"
#include "control/pid.h"
#include "hal/motors.h"
#include "hal/time.h"
#include "localization/odometry.h"
#include <cmath>

// Module-level PID — persists between calls for reuse by drive_to_pose
static PIDController turn_pid(TURN_KP, TURN_KI, TURN_KD);

double turn_to_heading_pid_calculate(double error) {
    // Trick: setpoint=0, pv=-error  →  internal_error = 0-(-error) = error
    return turn_pid.calculate(0.0, -error);
}

void turn_to_heading(double target_heading_rad) {
    turn_pid.reset();

    unsigned long settle_start = 0;  // timestamp when we first entered tolerance
    bool settling = false;
    unsigned long start_time = get_time_ms();

    while (true) {
        // --- Timeout check ---
        unsigned long elapsed = get_time_ms() - start_time;
        if (elapsed > TURN_TIMEOUT_MS) break;

        // --- Heading error (normalized to [-π, π]) ---
        Pose current = get_pose();
        double error = target_heading_rad - current.theta;
        error = atan2(sin(error), cos(error));  // shortest-path wrap

        // --- Settle detection ---
        if (std::abs(error) < TURN_SETTLE_RAD) {
            if (!settling) {
                settling = true;
                settle_start = get_time_ms();
            } else if (get_time_ms() - settle_start >= TURN_SETTLE_TIME_MS) {
                break;  // within tolerance long enough → done
            }
        } else {
            settling = false;  // left tolerance, reset settle timer
        }

        // --- PID → wheel voltages ---
        double omega = turn_to_heading_pid_calculate(error);
        double left_v  = -omega * WHEEL_TRACK / 2.0;
        double right_v =  omega * WHEEL_TRACK / 2.0;
        set_drive_motors(left_v, right_v);

        wait_ms(LOOP_INTERVAL_MS);
    }

    stop_drive_motors();
}
