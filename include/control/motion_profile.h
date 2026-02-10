#pragma once
// ============================================================================
//  control/motion_profile.h — Trapezoidal velocity profiler
// ============================================================================
//  Generates a target velocity that ramps up, cruises, and ramps down:
//
//    velocity
//    ▲
//    │    ┌────────────┐
//    │   /              \
//    │  /                \
//    │ /                  \
//    └─────────────────────── time
//      accel   cruise  decel
//
//  The PID controller tracks this target velocity, resulting in smooth,
//  controlled motion instead of jerky bang-bang control.
//
//  FUTURE:
//    • S-curve profile (jerk-limited) for even smoother motion
//    • Asymmetric accel/decel rates
// ============================================================================

class MotionProfile {
public:
    /// @param max_v  maximum cruise velocity (m/s)
    /// @param max_a  maximum acceleration and deceleration (m/s²)
    MotionProfile(double max_v, double max_a);

    /// Get the target velocity at a given moment.
    /// @param time_elapsed   time since motion started (seconds)
    /// @param distance_to_go remaining distance to target (meters, positive)
    /// @return target velocity (m/s, always ≥ 0)
    double get_target_velocity(double time_elapsed, double distance_to_go);

private:
    double _max_velocity;
    double _max_acceleration;
};
