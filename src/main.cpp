// ============================================================================
//  main.cpp — Simple motor test (no wheels, no competition switch needed)
// ============================================================================

#include "vex.h"
#include "config.h"

using namespace vex;

// ============================================================================
//  Hardware Definitions
// ============================================================================
//
//  ARCHITECTURE HIGHLIGHT — EXTENDABILITY:
//    Only this file and motors.cpp change between robot configurations.
//    All upper layers (odometry, PID, motion, drive_to_pose) are reused.
//
// ============================================================================
brain  Brain;

// ── 2-Motor Entry-Level ────────────────────────────────────────────────────
#ifdef ROBOT_2MOTOR
motor  LeftDriveSmart  = motor(LEFT_MOTOR_PORT,  ratio18_1, false);
motor  RightDriveSmart = motor(RIGHT_MOTOR_PORT, ratio18_1, false);
#endif

// ── 6-Motor Advanced ──────────────────────────────────────────────────────
//
//  Motor directions: left side reversed (motors face opposite direction)
//  Blue cartridge (ratio6_1) for maximum speed: 600 RPM
//
#ifdef ROBOT_6MOTOR
motor  LeftFront  = motor(LEFT_FRONT_MOTOR_PORT,  ratio6_1, true);   // reversed
motor  LeftMid    = motor(LEFT_MID_MOTOR_PORT,    ratio6_1, true);   // reversed
motor  LeftRear   = motor(LEFT_REAR_MOTOR_PORT,   ratio6_1, true);   // reversed
motor  RightFront = motor(RIGHT_FRONT_MOTOR_PORT, ratio6_1, false);
motor  RightMid   = motor(RIGHT_MID_MOTOR_PORT,   ratio6_1, false);
motor  RightRear  = motor(RIGHT_REAR_MOTOR_PORT,  ratio6_1, false);
#endif

// ============================================================================
//  Main — runs immediately on power-up, no competition switch needed
// ============================================================================
int main() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

#ifdef ROBOT_2MOTOR
    Brain.Screen.print("2-Motor Config");

    Brain.Screen.newLine();
    Brain.Screen.print("Testing Port 1...");

    // Test port 1 alone
    LeftDriveSmart.spin(forward, 50, percent);
    wait(3, seconds);
    LeftDriveSmart.stop();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Testing Port 2...");

    // Test port 2 alone
    RightDriveSmart.spin(forward, 50, percent);
    wait(3, seconds);
    RightDriveSmart.stop();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Testing BOTH...");

    // Test both together
    LeftDriveSmart.spin(forward, 50, percent);
    RightDriveSmart.spin(forward, 50, percent);
    wait(3, seconds);
    LeftDriveSmart.stop();
    RightDriveSmart.stop();
#endif

#ifdef ROBOT_6MOTOR
    Brain.Screen.print("6-Motor Config");

    // Test left side (3 motors)
    Brain.Screen.newLine();
    Brain.Screen.print("Testing LEFT side...");
    LeftFront.spin(forward, 50, percent);
    LeftMid.spin(forward, 50, percent);
    LeftRear.spin(forward, 50, percent);
    wait(3, seconds);
    LeftFront.stop();
    LeftMid.stop();
    LeftRear.stop();

    // Test right side (3 motors)
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Testing RIGHT side...");
    RightFront.spin(forward, 50, percent);
    RightMid.spin(forward, 50, percent);
    RightRear.spin(forward, 50, percent);
    wait(3, seconds);
    RightFront.stop();
    RightMid.stop();
    RightRear.stop();

    // Test ALL 6 together
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Testing ALL 6...");
    LeftFront.spin(forward, 50, percent);
    LeftMid.spin(forward, 50, percent);
    LeftRear.spin(forward, 50, percent);
    RightFront.spin(forward, 50, percent);
    RightMid.spin(forward, 50, percent);
    RightRear.spin(forward, 50, percent);
    wait(3, seconds);
    LeftFront.stop();
    LeftMid.stop();
    LeftRear.stop();
    RightFront.stop();
    RightMid.stop();
    RightRear.stop();
#endif

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Done.");

    // Keep program alive
    while (true) {
        wait(100, msec);
    }
}
