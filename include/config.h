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
//  FUTURE: When you add new subsystems (intake, catapult, wings, etc.)
//          add their constants here too.
// ============================================================================

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

// Derived constant — do not change manually
constexpr double WHEEL_CIRCUMFERENCE = 3.14159265358979 * WHEEL_DIAMETER;

// ─── Odometry ──────────────────────────────────────────────────────────────
//
//  IMU_FUSION_ALPHA : how much to trust IMU vs encoders for heading
//                     1.0 = 100% IMU, 0.0 = 100% encoders
//                     Start at 0.98 (IMU is very accurate on V5)
//
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

// ─── Heading Correction (used while driving straight) ──────────────────────
constexpr double HEADING_CORRECTION_KP = 3.0;

// ─── Motion Profile (trapezoidal velocity planning) ────────────────────────
//
//  MAX_VELOCITY     : top cruise speed (m/s). Measure by timing a 2m run.
//  MAX_ACCELERATION : how fast to ramp up/down (m/s²). Start conservative.
//
constexpr double MAX_VELOCITY     = 0.8;   // m/s — start slower, increase later
constexpr double MAX_ACCELERATION = 1.5;   // m/s²

// ─── Motor Ports (change to match YOUR wiring) ─────────────────────────────
//  Future: add intake, catapult, pneumatics ports here
constexpr int LEFT_MOTOR_PORT  = 1;
constexpr int RIGHT_MOTOR_PORT = 10;
constexpr int IMU_PORT         = 5;

// ─── Control Loop Timing ───────────────────────────────────────────────────
constexpr int LOOP_INTERVAL_MS = 10;   // 100 Hz control loop

