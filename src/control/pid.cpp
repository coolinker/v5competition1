// ============================================================================
//  control/pid.cpp — PID controller implementation
// ============================================================================
//
//  output = Kp × error  +  Ki × ∫error·dt  +  Kd × d(error)/dt
//
//  Key details:
//    • Uses real elapsed time (dt) for accurate I and D terms
//    • Guards against dt=0 (first call or very fast loops)
//    • Optional: anti-windup, derivative EMA filter, output clamping
//
// ============================================================================
#include "control/pid.h"
#include "hal/time.h"

PIDController::PIDController(double kp, double ki, double kd)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0), _prev_error(0), _last_time(0),
      _integral_limit(0), _d_filter_alpha(0),
      _filtered_deriv(0), _output_limit(0) {}

double PIDController::calculate(double setpoint, double pv) {
    double now = get_time_sec();
    double dt  = now - _last_time;

    // Guard: avoid division by zero on first call
    if (dt <= 0.0) dt = 0.01;

    double error = setpoint - pv;

    // ── Proportional ──
    double p_out = _kp * error;

    // ── Integral with anti-windup ──
    _integral += error * dt;
    if (_integral_limit > 0.0) {
        if (_integral >  _integral_limit) _integral =  _integral_limit;
        if (_integral < -_integral_limit) _integral = -_integral_limit;
    }
    double i_out = _ki * _integral;

    // ── Derivative with optional EMA filter ──
    double raw_deriv = (error - _prev_error) / dt;
    if (_d_filter_alpha > 0.0) {
        _filtered_deriv = _d_filter_alpha * _filtered_deriv
                        + (1.0 - _d_filter_alpha) * raw_deriv;
    } else {
        _filtered_deriv = raw_deriv;
    }
    double d_out = _kd * _filtered_deriv;

    // Save state
    _prev_error = error;
    _last_time  = now;

    double output = p_out + i_out + d_out;

    // ── Output clamping ──
    if (_output_limit > 0.0) {
        if (output >  _output_limit) output =  _output_limit;
        if (output < -_output_limit) output = -_output_limit;
    }

    return output;
}

void PIDController::reset() {
    _integral       = 0;
    _prev_error     = 0;
    _filtered_deriv = 0;
    _last_time      = get_time_sec();
}

void PIDController::set_integral_limit(double limit) { _integral_limit = limit; }
void PIDController::set_d_filter(double alpha) { _d_filter_alpha = alpha; }
void PIDController::set_output_limit(double limit) { _output_limit = limit; }
