// ============================================================================
//  hal/motors.cpp — Drivetrain motor control
// ============================================================================
//
//  ARCHITECTURE HIGHLIGHT — FLEXIBILITY:
//    This file is the ONLY implementation file that changes between 2-motor
//    and 6-motor configurations.  The public API (set_drive_motors, etc.)
//    remains identical, so every upper layer (PID, odometry, motion profile,
//    drive_to_pose, turn_to_heading) works without modification.
//
//    To switch configs: change one #define in config.h, rebuild.  Done.
//
// ============================================================================
#include "hal/motors.h"
#include "config.h"
#include "vex.h"
#include "hal/hal_log.h"

// ── Helper: clamp voltage to safe range ─────────────────────────────────────
static double clamp_voltage(double v) {
    if (v >  12.0) return  12.0;
    if (v < -12.0) return -12.0;
    return v;
}

// ============================================================================
//  2-MOTOR CONFIGURATION
// ============================================================================
#ifdef ROBOT_2MOTOR

// Hardware references (defined in main.cpp)
extern vex::motor LeftDriveSmart;
extern vex::motor RightDriveSmart;

void set_drive_motors(double left_voltage, double right_voltage) {
    left_voltage  = clamp_voltage(left_voltage);
    right_voltage = clamp_voltage(right_voltage);
    hal_log("Set drive motors: left=" + to_str(left_voltage) + ", right=" + to_str(right_voltage));
    LeftDriveSmart.spin(vex::fwd,  left_voltage  * 1000.0, vex::voltageUnits::mV);
    RightDriveSmart.spin(vex::fwd, right_voltage * 1000.0, vex::voltageUnits::mV);
}

void stop_drive_motors() {
    hal_log("Stop drive motors");
    LeftDriveSmart.stop(vex::brakeType::brake);
    RightDriveSmart.stop(vex::brakeType::brake);
}

double get_left_encoder_ticks() {
    double ticks = LeftDriveSmart.position(vex::rotationUnits::raw);
    hal_log("Left encoder ticks: " + to_str(ticks));
    return ticks;
}

double get_right_encoder_ticks() {
    double ticks = RightDriveSmart.position(vex::rotationUnits::raw);
    hal_log("Right encoder ticks: " + to_str(ticks));
    return ticks;
}

void reset_encoders() {
    hal_log("Reset encoders");
    LeftDriveSmart.resetPosition();
    RightDriveSmart.resetPosition();
}

#endif // ROBOT_2MOTOR

// ============================================================================
//  6-MOTOR CONFIGURATION
// ============================================================================
//
//  Motor layout (top-down view):
//
//    LeftFront  ────── RightFront     ← steer quickly at the front
//    LeftMid    ────── RightMid       ← primary encoders (best contact)
//    LeftRear   ────── RightRear      ← stability & traction
//
//  All 3 motors on a side receive the SAME voltage command.
//  Encoder reading comes from the MIDDLE motor (configurable via
//  ENCODER_MOTOR_INDEX in config.h).
//
// ============================================================================
#ifdef ROBOT_6MOTOR

// Hardware references (defined in main.cpp)
extern vex::motor LeftFront;
extern vex::motor LeftMid;
extern vex::motor LeftRear;
extern vex::motor RightFront;
extern vex::motor RightMid;
extern vex::motor RightRear;

// Convenient arrays for iteration
static vex::motor* left_motors[]  = { &LeftFront,  &LeftMid,  &LeftRear  };
static vex::motor* right_motors[] = { &RightFront, &RightMid, &RightRear };

void set_drive_motors(double left_voltage, double right_voltage) {
    left_voltage  = clamp_voltage(left_voltage);
    right_voltage = clamp_voltage(right_voltage);
    hal_log("Set drive motors (6M): left=" + to_str(left_voltage) + ", right=" + to_str(right_voltage));

    // Send identical voltage to all motors on each side
    for (int i = 0; i < MOTORS_PER_SIDE; ++i) {
        left_motors[i]->spin(vex::fwd,  left_voltage  * 1000.0, vex::voltageUnits::mV);
        right_motors[i]->spin(vex::fwd, right_voltage * 1000.0, vex::voltageUnits::mV);
    }
}

void stop_drive_motors() {
    hal_log("Stop drive motors (6M)");
    for (int i = 0; i < MOTORS_PER_SIDE; ++i) {
        left_motors[i]->stop(vex::brakeType::brake);
        right_motors[i]->stop(vex::brakeType::brake);
    }
}

double get_left_encoder_ticks() {
    // Read from the designated encoder motor (middle by default)
    double ticks = left_motors[ENCODER_MOTOR_INDEX]->position(vex::rotationUnits::raw);
    hal_log("Left encoder ticks (6M): " + to_str(ticks));
    return ticks;
}

double get_right_encoder_ticks() {
    double ticks = right_motors[ENCODER_MOTOR_INDEX]->position(vex::rotationUnits::raw);
    hal_log("Right encoder ticks (6M): " + to_str(ticks));
    return ticks;
}

void reset_encoders() {
    hal_log("Reset encoders (6M)");
    // Reset ALL motor encoders for consistency
    for (int i = 0; i < MOTORS_PER_SIDE; ++i) {
        left_motors[i]->resetPosition();
        right_motors[i]->resetPosition();
    }
}

#endif // ROBOT_6MOTOR
