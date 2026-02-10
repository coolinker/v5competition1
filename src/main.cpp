// ============================================================================
//  main.cpp — VEX V5 Competition Template
// ============================================================================
//  This file wires everything together:
//    • Hardware definitions (motors, IMU, brain)
//    • Competition callbacks (pre_auton, autonomous, usercontrol)
//    • Background odometry loop
//
//  To customise your robot, edit:
//    1. config.h       — tuning constants (ports, PID gains, dimensions)
//    2. autonomous()   — your autonomous routine
//    3. usercontrol()  — your driver-control code
// ============================================================================

#include "vex.h"
#include "config.h"
#include "hal/motors.h"
#include "hal/imu.h"
#include "hal/time.h"
#include "localization/odometry.h"
#include "motion/drive_to_pose.h"
#include "motion/turn_to_heading.h"
#include <cmath>

using namespace vex;

// ============================================================================
//  Hardware Definitions
//  These global objects are referenced by the HAL layer via `extern`.
//  Change port numbers in config.h, gear ratios here.
// ============================================================================
brain  Brain;
competition Competition;
controller Controller;

motor  LeftDriveSmart  = motor(LEFT_MOTOR_PORT,  ratio18_1, false);
motor  RightDriveSmart = motor(RIGHT_MOTOR_PORT, ratio18_1, true);   // reversed
inertial DrivetrainInertial = inertial(IMU_PORT);

// ============================================================================
//  Pre-Autonomous  —  runs once on power-up
// ============================================================================
void pre_auton() {
    // Calibrate the IMU (blocks ~2 s, required for heading)
    calibrate_imu();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Ready.");
}

// ============================================================================
//  Autonomous  —  your 15-second / 1-minute routine
// ============================================================================
//  Example: drive a simple L-shaped path.
//  Replace with your actual autonomous strategy.
// ============================================================================
void autonomous() {
    set_pose({0, 0, 0});           // reset origin at current position

    drive_to_pose({1.0, 0.0, 0});  // drive 1 m forward
    turn_to_heading(M_PI / 2.0);   // turn 90° left
    drive_to_pose({1.0, 0.8, 0});  // drive 0.8 m to the side

    stop_drive_motors();
}

// ============================================================================
//  User Control  —  tank-drive by default
// ============================================================================
//  Left stick (Axis3)  → left wheels
//  Right stick (Axis2) → right wheels
//  Modify to add mechanisms (intakes, lifts, clamps, etc.)
// ============================================================================
void usercontrol() {
    while (true) {
        double left_pct  = Controller.Axis3.position();
        double right_pct = Controller.Axis2.position();

        LeftDriveSmart.spin(fwd,  left_pct,  percent);
        RightDriveSmart.spin(fwd, right_pct, percent);

        wait(20, msec);
    }
}

// ============================================================================
//  Main  —  binds callbacks and runs the background odometry loop
// ============================================================================
int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    // Background loop: keep odometry up-to-date at all times
    while (true) {
        odometry_update();
        wait(LOOP_INTERVAL_MS, msec);
    }
}
