// ============================================================================
//  control/pid.cpp — PID controller implementation
// ============================================================================
//
//  output = Kp × error  +  Ki × ∫error·dt  +  Kd × d(error)/dt
//
//  Key details:
//    • Uses real elapsed time (dt) for accurate I and D terms
//    • Guards against dt=0 (first call or very fast loops)
//    • No output clamping here — the caller decides voltage limits
//
// ============================================================================
#include "control/pid.h"
#include "hal/time.h"

PIDController::PIDController(double kp, double ki, double kd)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0), _prev_error(0), _last_time(0) {}

double PIDController::calculate(double setpoint, double pv) {
    double now = get_time_sec();
    double dt  = now - _last_time;

    // Guard: avoid division by zero on first call
    if (dt <= 0.0) dt = 0.01;

    // Error = how far off we are
    double error = setpoint - pv;

    // Proportional: respond to current error
    double p_out = _kp * error;

    // Integral: accumulate past error (helps eliminate steady-state offset)
    _integral += error * dt;
    double i_out = _ki * _integral;

    // Derivative: react to rate of change (dampens overshoot)
    double derivative = (error - _prev_error) / dt;
    double d_out = _kd * derivative;

    // Save state for next iteration
    _prev_error = error;
    _last_time  = now;

    return p_out + i_out + d_out;
}

void PIDController::reset() {
    _integral   = 0;
    _prev_error = 0;
    _last_time  = get_time_sec();
}
