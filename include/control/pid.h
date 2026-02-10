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
//  FUTURE:
//    • Add anti-windup (integral clamping)
//    • Add feed-forward term
//    • Add output slew rate limiting
// ============================================================================

class PIDController {
public:
    /// Construct with PID gains.
    PIDController(double kp, double ki, double kd);

    /// Compute PID output.
    /// @param setpoint  desired value
    /// @param pv        process variable (current measured value)
    /// @return corrective output (unbounded — caller should clamp if needed)
    double calculate(double setpoint, double pv);

    /// Reset integral accumulator and derivative state.
    /// Call this before starting a new movement.
    void reset();

private:
    double _kp, _ki, _kd;
    double _integral;
    double _prev_error;
    double _last_time;
};
