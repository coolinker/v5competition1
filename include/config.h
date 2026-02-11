#pragma once
// ============================================================================
//  config.h — Single source of truth for ALL tunable parameters
// ============================================================================
//  HOW TO USE:
//    1. Measure your robot and fill in the "Physical" section
//    2. Start with the default PID values below
//    3. Tune on the field: P first, then D, then I (if needed)
//    4. Never hard-code numbers anywhere else — always reference this file
//
//  ROBOT CONFIGURATIONS:
//    Uncomment exactly ONE of the defines below to select your drivetrain.
//    Each config sets its own physical dimensions, PID gains, motor ports,
//    and performance parameters.  The rest of the codebase (control,
//    localization, motion) works unchanged — this is the power of the
//    layered architecture.
//
//  FUTURE: When you add new subsystems (intake, catapult, wings, etc.)
//          add their constants here too.
// ============================================================================

// ─── Robot Configuration Selector ──────────────────────────────────────────
//
//  Uncomment ONE of the following lines:
//
//    ROBOT_2MOTOR  — Entry-level 2-motor differential drive (simple, cheap)
//    ROBOT_6MOTOR  — Advanced 6-motor drive (fast, powerful, competition)
//
//  Then rebuild.  Only config.h, motors.cpp, and main.cpp contain
//  conditional code — everything else (PID, odometry, motion profile,
//  drive_to_pose, turn_to_heading) is reused as-is.
// ────────────────────────────────────────────────────────────────────────────

#define ROBOT_2MOTOR    // ← Entry-level: 1 left + 1 right motor
// #define ROBOT_6MOTOR    // ← Advanced:    3 left + 3 right motors (600 RPM blue)

// ============================================================================
//  Guard: exactly one config must be selected
// ============================================================================
#if defined(ROBOT_2MOTOR) && defined(ROBOT_6MOTOR)
  #error "Select only ONE robot config: ROBOT_2MOTOR or ROBOT_6MOTOR"
#endif
#if !defined(ROBOT_2MOTOR) && !defined(ROBOT_6MOTOR)
  #error "Select a robot config: uncomment ROBOT_2MOTOR or ROBOT_6MOTOR above"
#endif

// ############################################################################
//  ENTRY-LEVEL — 2-Motor Differential Drive
// ############################################################################
#ifdef ROBOT_2MOTOR

// ─── Physical Constants (MUST measure your robot) ──────────────────────────
//
//  WHEEL_DIAMETER : measure with calipers across the tread (meters)
//  WHEEL_TRACK    : center-to-center distance between left & right wheels (m)
//  TICKS_PER_REV  : encoder ticks for one full wheel revolution
//                   V5 motors: 360 (18:1 green), 900 (36:1 red), 1800 (6:1 blue)
//
constexpr double WHEEL_DIAMETER   = 0.1016;   // 4-inch wheels → 0.1016 m
constexpr double WHEEL_TRACK      = 0.381;    // ~15 inches → 0.381 m
constexpr double TICKS_PER_REV    = 360.0;    // green cartridge (18:1)

// ─── Odometry ──────────────────────────────────────────────────────────────
constexpr double IMU_FUSION_ALPHA = 0.98;

// ─── Turn PID ──────────────────────────────────────────────────────────────
//
//  Tuning guide:
//    1. Set I=0, D=0. Increase P until the robot oscillates around target
//    2. Increase D to dampen oscillation
//    3. Only add I if there is persistent steady-state error
//
constexpr double TURN_KP = 2.0;
constexpr double TURN_KI = 0.0;
constexpr double TURN_KD = 0.1;
constexpr double TURN_SETTLE_RAD      = 0.035;  // ~2° — "close enough"
constexpr double TURN_SETTLE_TIME_MS  = 200;    // must stay within tolerance this long
constexpr double TURN_TIMEOUT_MS      = 2000;   // give up after this

// ─── Drive PID ─────────────────────────────────────────────────────────────
constexpr double DRIVE_KP = 5.0;
constexpr double DRIVE_KI = 0.0;
constexpr double DRIVE_KD = 0.3;
constexpr double DRIVE_SETTLE_M       = 0.02;   // 2 cm
constexpr double DRIVE_SETTLE_TIME_MS = 200;
constexpr double DRIVE_TIMEOUT_MS     = 5000;

// ─── Heading Correction ────────────────────────────────────────────────────
constexpr double HEADING_CORRECTION_KP = 3.0;

// ─── Motion Profile ────────────────────────────────────────────────────────
constexpr double MAX_VELOCITY     = 0.8;   // m/s — start slower, increase later
constexpr double MAX_ACCELERATION = 1.5;   // m/s²

// ─── Motor Ports ───────────────────────────────────────────────────────────
//  VEX ports are 0-indexed: physical port 1 = 0, port 2 = 1, etc.
constexpr int LEFT_MOTOR_PORT  = 0;   // physical port 1
constexpr int RIGHT_MOTOR_PORT = 1;   // physical port 2
constexpr int IMU_PORT         = 4;   // physical port 5

// ─── Motor Count (used by HAL) ─────────────────────────────────────────────
constexpr int MOTORS_PER_SIDE = 1;

#endif // ROBOT_2MOTOR

// ############################################################################
//  ADVANCED — 6-Motor High-Performance Drivetrain
// ############################################################################
//
//  Why 6 motors?
//    • 3× the torque of a 2-motor drive → push battles, fast acceleration
//    • 600 RPM blue cartridge → higher top speed with gearing headroom
//    • 6 wheels (front/middle/rear per side) → better traction & stability
//    • Same software architecture — only the HAL layer changes!
//
//  Wiring diagram (top-down view):
//
//     Port 1 [L-Front]   ──── [R-Front]  Port 4
//     Port 2 [L-Mid  ]   ──── [R-Mid  ]  Port 5
//     Port 3 [L-Rear ]   ──── [R-Rear ]  Port 6
//                     IMU = Port 10
//
//  Encoder strategy:
//    Only the MIDDLE motor on each side is read for odometry.
//    Middle wheels have the most consistent ground contact and
//    are least affected by turning scrub or wheelies.
//
// ############################################################################
#ifdef ROBOT_6MOTOR

// ─── Physical Constants ────────────────────────────────────────────────────
//
//  Larger wheels + wider track are common on 6-motor competition bots
//  Adjust these to match YOUR specific build
//
constexpr double WHEEL_DIAMETER   = 0.08255;  // 3.25-inch wheels → 0.08255 m
constexpr double WHEEL_TRACK      = 0.330;    // ~13 inches → 0.330 m (compact, agile)
constexpr double TICKS_PER_REV    = 300.0;    // blue cartridge (6:1) = 300 ticks/rev

// ─── Odometry ──────────────────────────────────────────────────────────────
constexpr double IMU_FUSION_ALPHA = 0.98;

// ─── Turn PID ──────────────────────────────────────────────────────────────
//
//  Higher gains because 6 motors have much more torque:
//    • P higher → responds faster with 3× the push
//    • D higher → dampens the stronger inertia
//    • Tighter settle tolerance achievable with more torque
//
constexpr double TURN_KP = 3.5;
constexpr double TURN_KI = 0.02;
constexpr double TURN_KD = 0.25;
constexpr double TURN_SETTLE_RAD      = 0.025;  // ~1.4° — tighter tolerance, more torque
constexpr double TURN_SETTLE_TIME_MS  = 150;    // settles faster with more power
constexpr double TURN_TIMEOUT_MS      = 1500;   // quicker timeout — should turn fast

// ─── Drive PID ─────────────────────────────────────────────────────────────
//
//  More aggressive than 2-motor: the extra torque allows tighter control
//
constexpr double DRIVE_KP = 8.0;
constexpr double DRIVE_KI = 0.05;
constexpr double DRIVE_KD = 0.5;
constexpr double DRIVE_SETTLE_M       = 0.015;  // 1.5 cm — more precise
constexpr double DRIVE_SETTLE_TIME_MS = 150;
constexpr double DRIVE_TIMEOUT_MS     = 4000;

// ─── Heading Correction ────────────────────────────────────────────────────
constexpr double HEADING_CORRECTION_KP = 4.5;

// ─── PID Enhancements (competition-grade) ──────────────────────────────────
//
//  These activate anti-windup and derivative filtering in PID controllers.
//  Only used by 6-motor motion code; the entry-level 2-motor config does
//  not call the setters, so PID runs in basic mode for that config.
//
constexpr double DRIVE_INTEGRAL_LIMIT  = 5.0;   // anti-windup: max |∫error·dt|
constexpr double DRIVE_D_FILTER        = 0.7;   // derivative EMA alpha (0=off)
constexpr double TURN_INTEGRAL_LIMIT   = 3.0;
constexpr double TURN_D_FILTER         = 0.5;

// ─── Boomerang Controller ──────────────────────────────────────────────────
//
//  Replaces turn-then-drive with a smooth curved approach.
//  A "carrot" point is placed behind the target along its heading vector;
//  as the robot closes in, the carrot converges to the target, producing
//  a natural arc that arrives at the correct (x, y, θ).
//
constexpr double BOOMERANG_LEAD        = 0.6;   // carrot lead factor [0, 1]

// ─── Motion Profile ────────────────────────────────────────────────────────
//
//  Much faster than 2-motor drive:
//    • 600 RPM blue cartridge + 3.25" wheels ≈ 1.3 m/s theoretical max
//    • 3× motors = 3× torque for aggressive acceleration
//
constexpr double MAX_VELOCITY     = 1.2;   // m/s — near max for blue cartridge
constexpr double MAX_ACCELERATION = 3.0;   // m/s² — twice the accel with 3× torque

// ─── Motor Ports ───────────────────────────────────────────────────────────
//
//  6 drive motors: 3 left + 3 right
//  VEX ports are 0-indexed: physical port 1 = 0, etc.
//
constexpr int LEFT_FRONT_MOTOR_PORT  = 0;   // physical port 1
constexpr int LEFT_MID_MOTOR_PORT    = 1;   // physical port 2
constexpr int LEFT_REAR_MOTOR_PORT   = 2;   // physical port 3
constexpr int RIGHT_FRONT_MOTOR_PORT = 3;   // physical port 4
constexpr int RIGHT_MID_MOTOR_PORT   = 4;   // physical port 5
constexpr int RIGHT_REAR_MOTOR_PORT  = 5;   // physical port 6
constexpr int IMU_PORT               = 9;   // physical port 10

// ─── Motor Count (used by HAL) ─────────────────────────────────────────────
constexpr int MOTORS_PER_SIDE = 3;

// ─── Encoder Source ────────────────────────────────────────────────────────
//  Which motor's encoder to read for odometry (0=front, 1=mid, 2=rear)
//  Middle motor recommended: best ground contact, least turning scrub
constexpr int ENCODER_MOTOR_INDEX = 1;  // middle motor

#endif // ROBOT_6MOTOR

// ############################################################################
//  SHARED CONSTANTS — apply to ALL robot configurations
// ############################################################################

// Derived constant — do not change manually
constexpr double WHEEL_CIRCUMFERENCE = 3.14159265358979 * WHEEL_DIAMETER;

// ─── Control Loop Timing ───────────────────────────────────────────────────
constexpr int LOOP_INTERVAL_MS = 10;   // 100 Hz control loop

