#pragma once
// ============================================================================
//  control/pid.h — General-purpose PID controller
// ============================================================================
//  A PID controller computes a corrective output based on the difference
//  between a desired setpoint and the measured process variable:
//
//    output = Kp×error + Ki×∫error×dt + Kd×(d_error/dt)
//
//  Usage:
//    PIDController pid(2.0, 0.0, 0.1);    // create with gains
//    pid.reset();                           // call before each movement
//    double output = pid.calculate(target, current);  // call every loop
//
//  Enhancements (optional, backward-compatible — disabled by default):
//    • Anti-windup (integral clamping) — set_integral_limit()
//    • Derivative EMA low-pass filter — set_d_filter()
//    • Symmetric output clamping — set_output_limit()
//
//  FUTURE:
//    • Feed-forward term (applied externally, not inside PID)
// ============================================================================

class PIDController {
public:
    /// Construct with PID gains.
    PIDController(double kp, double ki, double kd);

    /// Compute PID output.
    /// @param setpoint  desired value
    /// @param pv        process variable (current measured value)
    /// @return corrective output
    double calculate(double setpoint, double pv);

    /// Reset integral accumulator and derivative state.
    /// Call this before starting a new movement.
    void reset();

    // ── Optional enhancements (disabled by default for backward compat) ──
    void set_integral_limit(double limit);   ///< Anti-windup: clamp |∫error·dt|
    void set_d_filter(double alpha);         ///< Derivative EMA (0=off, 0.5–0.8 typical)
    void set_output_limit(double limit);     ///< Symmetric output clamp (0=off)

private:
    double _kp, _ki, _kd;
    double _integral;
    double _prev_error;
    double _last_time;
    double _integral_limit;    ///< 0 = disabled
    double _d_filter_alpha;    ///< 0 = no filter
    double _filtered_deriv;    ///< EMA state
    double _output_limit;      ///< 0 = disabled
};
