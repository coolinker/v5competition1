// ============================================================================
//  hal/motors.cpp — Drivetrain motor control
// ============================================================================
#include "hal/motors.h"
#include "vex.h"

// Hardware references (defined in main.cpp)
extern vex::motor LeftDriveSmart;
extern vex::motor RightDriveSmart;

void set_drive_motors(double left_voltage, double right_voltage) {
    // Clamp to [-12, +12] volts for safety
    if (left_voltage  >  12.0) left_voltage  =  12.0;
    if (left_voltage  < -12.0) left_voltage  = -12.0;
    if (right_voltage >  12.0) right_voltage =  12.0;
    if (right_voltage < -12.0) right_voltage = -12.0;

    // VEX API expects millivolts: -12000 to +12000
    LeftDriveSmart.spin(vex::fwd,  left_voltage  * 1000.0, vex::voltageUnits::mV);
    RightDriveSmart.spin(vex::fwd, right_voltage * 1000.0, vex::voltageUnits::mV);
}

void stop_drive_motors() {
    LeftDriveSmart.stop(vex::brakeType::brake);
    RightDriveSmart.stop(vex::brakeType::brake);
}

double get_left_encoder_ticks() {
    return LeftDriveSmart.position(vex::rotationUnits::raw);
}

double get_right_encoder_ticks() {
    return RightDriveSmart.position(vex::rotationUnits::raw);
}

void reset_encoders() {
    LeftDriveSmart.resetPosition();
    RightDriveSmart.resetPosition();
}
