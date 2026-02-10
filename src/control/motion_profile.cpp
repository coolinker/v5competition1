// ============================================================================
//  control/motion_profile.cpp — Trapezoidal velocity planner
// ============================================================================
//
//  At each moment, the target velocity is the MINIMUM of three constraints:
//    1. Acceleration limit : v = a × t           (can't accelerate too fast)
//    2. Max velocity cap   : v = v_max           (can't exceed top speed)
//    3. Deceleration limit : v = √(2 × a × d)   (must be able to stop in time)
//
//  This naturally creates the trapezoidal shape.
//
// ============================================================================
#include "control/motion_profile.h"
#include <cmath>
#include <algorithm>

MotionProfile::MotionProfile(double max_v, double max_a)
    : _max_velocity(max_v), _max_acceleration(max_a) {}

double MotionProfile::get_target_velocity(double time_elapsed, double distance_to_go) {
    // Constraint 1: acceleration ramp (starts from zero)
    double accel_v = _max_acceleration * time_elapsed;

    // Constraint 2: deceleration ramp (must stop at target)
    //   From v² = 2·a·d  →  v = √(2·a·d)
    double decel_v = sqrt(2.0 * _max_acceleration * std::abs(distance_to_go));

    // Constraint 3: top speed cap
    // Take the most restrictive (smallest) of all three
    return std::min({_max_velocity, accel_v, decel_v});
}
