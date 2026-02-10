// ============================================================================
//  hal/imu.cpp — Inertial sensor interface
// ============================================================================
#include "hal/imu.h"
#include "vex.h"
#include <cmath>

// Hardware reference (defined in main.cpp)
extern vex::inertial DrivetrainInertial;

double get_imu_heading_rad() {
    // VEX heading() returns [0, 360). Convert to radians.
    return DrivetrainInertial.heading(vex::rotationUnits::deg) * M_PI / 180.0;
}

double get_imu_rotation_rad() {
    // rotation() returns total cumulative rotation (can be > 360°)
    return DrivetrainInertial.rotation(vex::rotationUnits::deg) * M_PI / 180.0;
}

void reset_imu() {
    DrivetrainInertial.resetRotation();
    DrivetrainInertial.resetHeading();
}

void calibrate_imu() {
    DrivetrainInertial.calibrate();
    // Block until calibration finishes (~2 seconds)
    while (DrivetrainInertial.isCalibrating()) {
        vex::task::sleep(50);
    }
}
