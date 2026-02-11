// ============================================================================
//  hal/imu.cpp — Inertial sensor interface
// ============================================================================
#include "hal/imu.h"
#include "vex.h"
#include <cmath>
#include "hal/hal_log.h"

// Hardware reference (defined in main.cpp)
extern vex::inertial DrivetrainInertial;

double get_imu_heading_rad() {
    double heading = DrivetrainInertial.heading(vex::rotationUnits::deg) * M_PI / 180.0;
    hal_log("IMU heading(rad): " + to_str(heading));
    return heading;
}

double get_imu_rotation_rad() {
    double rotation = DrivetrainInertial.rotation(vex::rotationUnits::deg) * M_PI / 180.0;
    hal_log("IMU rotation(rad): " + to_str(rotation));
    return rotation;
}

void reset_imu() {
    DrivetrainInertial.resetRotation();
    DrivetrainInertial.resetHeading();
    hal_log("IMU reset");
}

void calibrate_imu() {
    DrivetrainInertial.calibrate();
    hal_log("IMU calibration started");

    // Timeout after 3 seconds in case the IMU is not plugged in
    int elapsed = 0;
    while (DrivetrainInertial.isCalibrating() && elapsed < 3000) {
        vex::task::sleep(50);
        elapsed += 50;
    }

    if (elapsed >= 3000) {
        hal_log("IMU calibration TIMEOUT — sensor may not be connected");
    } else {
        hal_log("IMU calibration finished");
    }
}
