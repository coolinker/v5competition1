# Full Test Document — VEX V5 Robot

---

## Table of Contents

1. [Test Strategy Overview](#1-test-strategy-overview)
2. [Test Levels](#2-test-levels)
3. [Level 1: Host-Side Unit Tests](#3-level-1-host-side-unit-tests)
4. [Level 2: On-Robot Component Tests](#4-level-2-on-robot-component-tests)
5. [Level 3: Integration Tests](#5-level-3-integration-tests)
6. [Level 4: System Tests](#6-level-4-system-tests)
7. [Level 5: Competition Readiness](#7-level-5-competition-readiness)
8. [PID Tuning Guide](#8-pid-tuning-guide)
9. [Test Checklist](#9-test-checklist)
10. [Common Failure Modes](#10-common-failure-modes)

---

## 1. Test Strategy Overview

Testing a competition robot involves **5 levels**, from pure software on your laptop to full match simulations on the field:

```
Level 5: Competition Readiness    ← Full match simulation
Level 4: System Tests             ← Complete autonomous routines
Level 3: Integration Tests        ← Multiple modules working together
Level 2: On-Robot Component Tests ← Individual hardware functions
Level 1: Host-Side Unit Tests     ← Pure math, no hardware needed
```

**Rule of thumb:**
- **Level 1** catches ~60% of bugs (math errors, logic mistakes)
- **Level 2-3** catches ~30% of bugs (hardware issues, timing)
- **Level 4-5** catches ~10% of bugs (field conditions, strategy)

Start at Level 1 and work your way up. Don't skip levels!

---

## 2. Test Levels

| Level | Where it runs | What it tests | When to run |
|-------|--------------|---------------|-------------|
| 1 | Your computer | PID math, motion profile, odometry | After every code change |
| 2 | Robot (stationary) | Motors spin, IMU reads, encoders count | After hardware changes |
| 3 | Robot (moving) | Odometry tracks, PID converges | After PID gain changes |
| 4 | Robot on field | Full autonomous routes | Before competition |
| 5 | Match simulation | Full match with driver + auton | Day before / morning of |

---

## 3. Level 1: Host-Side Unit Tests

These run on your development computer with `make test`. No robot needed.

**Covered modules:**

| Module | Tests | What's verified |
|--------|-------|----------------|
| PID Controller | 6 | P/I/D terms, zero error, reset |
| Motion Profile | 5 | Accel/cruise/decel phases, bounds |
| Odometry | 6 | Forward/backward/turn/accumulation |
| **Total** | **17** | |

**How to run:**
```bash
cd v5competition1
make test
```

**Pass criteria:** All 17 tests show `[ OK ]`, exit code 0.

See [Unit Test Documentation](../en/2_unit_test_doc.md) for detailed test case descriptions.

---

## 4. Level 2: On-Robot Component Tests

These tests verify that each piece of hardware works correctly. Run them with the robot on blocks (wheels off the ground) or on a smooth surface.

### Test 2.1: Motor Direction Check

**Procedure:**
1. Lift the robot so wheels are off the ground
2. In `usercontrol()`, push the left joystick forward
3. **Observe:** Left wheel spins forward
4. Push the right joystick forward
5. **Observe:** Right wheel spins forward

**If wrong:** The motor is reversed. Flip the `true/false` parameter in the motor constructor in `main.cpp`, or swap the motor port.

**Pass criteria:** Both wheels spin forward when joysticks are pushed forward.

---

### Test 2.2: Encoder Verification

**Procedure:**
1. Add temporary debug output in `main()` loop:
   ```cpp
   Brain.Screen.clearLine(2);
   Brain.Screen.setCursor(2, 1);
   Brain.Screen.print("L: %.1f  R: %.1f",
       get_left_encoder_ticks(), get_right_encoder_ticks());
   ```
2. Manually rotate each wheel forward by one full turn
3. **Observe:** Encoder reads approximately `TICKS_PER_REV` (360)
4. Rotate backward
5. **Observe:** Encoder counts down

**Pass criteria:**
- One full forward turn ≈ 360 ticks (±5)
- Direction matches: forward = positive

---

### Test 2.3: IMU Calibration & Heading

**Procedure:**
1. Place robot on a flat, stable surface
2. Power on — wait for "Ready." on screen (IMU calibrating)
3. Add debug output:
   ```cpp
   Brain.Screen.print("IMU: %.2f rad", get_imu_heading_rad());
   ```
4. **Observe:** Reading is ≈ 0.0 after calibration
5. Rotate robot 90° clockwise by hand
6. **Observe:** Reading is ≈ π/2 (1.57)
7. Rotate 90° more (total 180°)
8. **Observe:** Reading is ≈ π (3.14)

**Pass criteria:**
- After calibration: heading ≈ 0
- After 90° turn: heading ≈ 1.57 ± 0.1
- After 180° turn: heading ≈ 3.14 ± 0.1

---

## 5. Level 3: Integration Tests

These test combinations of modules working together. Robot must move.

### Test 3.1: Odometry Accuracy — Straight Line

**Procedure:**
1. Place robot at a known starting position
2. Mark a point exactly 1 meter ahead on the floor (use tape/ruler)
3. In autonomous, run:
   ```cpp
   set_pose({0, 0, 0});
   // Manually drive forward 1 meter using usercontrol
   ```
4. Display pose on screen:
   ```cpp
   Pose p = get_pose();
   Brain.Screen.print("x=%.3f y=%.3f", p.x, p.y);
   ```
5. Push robot forward exactly 1m by hand (or drive carefully)

**Pass criteria:**
- x ≈ 1.0 ± 0.05 (within 5 cm)
- y ≈ 0.0 ± 0.03

**If failing:** Check `WHEEL_DIAMETER` and `TICKS_PER_REV` in config.h. Measure your actual wheel diameter with calipers.

---

### Test 3.2: Odometry Accuracy — Point Turn

**Procedure:**
1. Place robot on floor, mark its heading with tape
2. `set_pose({0, 0, 0})`
3. Manually rotate robot 360° (back to start)
4. Check pose: θ should be ≈ 0 (or ≈ 2π)

**Pass criteria:**
- Position (x, y) should still be ≈ (0, 0) ± 0.05m
- After 360°: θ error < 0.1 rad (≈ 6°)

**If failing:** Check `WHEEL_TRACK` in config.h. Measure the actual distance between your wheel contact patches.

---

### Test 3.3: Turn-to-Heading Accuracy

**Procedure:**
1. `set_pose({0, 0, 0})`
2. Run `turn_to_heading(M_PI / 2.0)` (90° left)
3. Measure actual heading with a protractor or by aligning with field marks

**Pass criteria:**
- Final heading within ±5° of target
- Robot stops smoothly (no violent oscillation)
- Completes within TURN_TIMEOUT_MS

**If failing:**
- Oscillating: Reduce TURN_KP or increase TURN_KD
- Doesn't reach target: Increase TURN_KP
- Very slow: Increase TURN_KP

---

### Test 3.4: Drive-to-Pose Accuracy

**Procedure:**
1. Mark start position and a point 1.0m ahead
2. `set_pose({0, 0, 0})`
3. Run `drive_to_pose({1.0, 0, 0})`
4. Measure actual stopping position

**Pass criteria:**
- Stops within 5 cm of target
- Doesn't overshoot significantly
- Drives in a reasonably straight line

**If failing:**
- Overshoots: Reduce MAX_VELOCITY or increase MAX_ACCELERATION
- Veers sideways: Check HEADING_CORRECTION_KP, check wheel diameters are equal
- Stops short: Check DRIVE_SETTLE_M tolerance

---

## 6. Level 4: System Tests

System tests run the complete autonomous routine on a real field (or a practice field). The goal is to verify the full path works end-to-end.

### Test 4.1: L-Path Autonomous

**Procedure:**
1. Set up a practice field with a clear 1m × 0.8m area
2. Place robot at the starting position
3. Run the autonomous routine:
   ```cpp
   set_pose({0, 0, 0});
   drive_to_pose({1.0, 0.0, 0});    // 1m forward
   turn_to_heading(M_PI / 2.0);     // 90° left
   drive_to_pose({1.0, 0.8, 0});    // 0.8m sideways
   ```
4. Record: Did the robot follow the expected L-shaped path?

**Pass criteria:**
- Robot drives ~1m forward, turns ~90° left, drives ~0.8m
- Final position within 10 cm of expected endpoint
- No collisions, smooth motion

---

### Test 4.2: Repeatability Test

**Procedure:**
1. Run the same autonomous routine 5 times from the exact same starting position
2. Mark the final position each time

**Pass criteria:**
- All 5 final positions within a 10 cm × 10 cm box
- Consistent timing (within ±0.5 seconds)

**Why this matters:** In competition, you need to trust that your autonomous does the same thing every time.

---

### Test 4.3: Timeout Safety Test

**Procedure:**
1. Hold the robot still (wheels can't move) during an autonomous movement
2. **Observe:** Robot should stop trying after timeout period (2-5 seconds)

**Pass criteria:**
- Robot does NOT spin motors forever
- Motors stop after TURN_TIMEOUT_MS or DRIVE_TIMEOUT_MS
- Program continues to next movement

---

## 7. Level 5: Competition Readiness

Run this checklist before every competition event:

### Pre-Competition Checklist

- [ ] **Battery:** Fully charged (> 95%)
- [ ] **Firmware:** V5 Brain firmware up to date
- [ ] **Code compiled:** `make` builds without errors
- [ ] **Unit tests pass:** `make test` → 17/17
- [ ] **IMU calibration:** Shows "Ready." on screen within 3 seconds
- [ ] **Motor check:** All motors spin correct direction (Test 2.1)
- [ ] **Encoder check:** Both encoders count correctly (Test 2.2)
- [ ] **Autonomous runs:** L-path completes correctly (Test 4.1)
- [ ] **Repeatability:** 3 consecutive successful autonomous runs (Test 4.2)
- [ ] **Timeout works:** Robot stops if held (Test 4.3)
- [ ] **Driver control:** Tank drive works smoothly
- [ ] **No loose wires:** All connections secure

### Match Simulation Procedure

1. Place robot on field at starting position
2. Run full 1-minute autonomous
3. Immediately switch to 1-minute driver control
4. Record any issues
5. Repeat 3 times

---

## 8. PID Tuning Guide

PID tuning is the most important testing activity. Bad PID gains = bad robot performance.

### Step-by-step tuning procedure

> **Golden rule:** Tune one thing at a time. Change one number, test, repeat.

#### Tuning Turn PID (TURN_KP, TURN_KI, TURN_KD):

1. **Start with P only:** Set Ki=0, Kd=0, Kp=1.0
2. Run `turn_to_heading(M_PI / 2.0)` (90° turn)
3. **Increase Kp** until the robot turns decisively but oscillates slightly
4. **Add Kd** (start at Kp/10). Increase until oscillation stops
5. **Add Ki only if needed** (start at Kp/100). Only if robot consistently stops short

| Symptom | What to change |
|---------|---------------|
| Too slow, stops short | Increase Kp |
| Oscillates (wobbles back and forth) | Increase Kd, or decrease Kp |
| Overshoots once then settles | Slightly increase Kd |
| Never quite reaches target | Add small Ki |
| Violent oscillation, never stops | Kp is WAY too high — halve it |

#### Tuning Drive PID (DRIVE_KP, DRIVE_KI, DRIVE_KD):

Same process, but test with `drive_to_pose({1.0, 0, 0})`.

#### Tuning tips:

- **Film your tests** — video lets you analyze behavior frame by frame
- **Log data** — print (time, error, output) to screen or SD card
- **Test on the actual competition surface** — tile friction differs from carpet!
- **Tune for WORST case** — add weight, low battery, sticky wheels

---

## 9. Test Checklist

Use this table to track test progress. Mark ✅ when passing, ❌ when failing, ⬜ for not yet tested.

```
=== Level 1: Host-Side Unit Tests ===
⬜ make test → 17/17 pass

=== Level 2: Component Tests ===
⬜ 2.1  Motor direction
⬜ 2.2  Encoder accuracy
⬜ 2.3  IMU calibration & heading

=== Level 3: Integration Tests ===
⬜ 3.1  Odometry straight-line accuracy
⬜ 3.2  Odometry turn accuracy
⬜ 3.3  Turn-to-heading accuracy
⬜ 3.4  Drive-to-pose accuracy

=== Level 4: System Tests ===
⬜ 4.1  L-path autonomous
⬜ 4.2  Repeatability (5 runs)
⬜ 4.3  Timeout safety

=== Level 5: Competition Readiness ===
⬜ Full pre-competition checklist
⬜ Match simulation (3 runs)
```

---

## 10. Common Failure Modes

| # | Symptom | Likely Cause | Fix |
|---|---------|-------------|-----|
| 1 | Robot drives in a curve instead of straight | Left and right wheels have different diameters or friction | Check wheels, tune HEADING_CORRECTION_KP |
| 2 | Odometry position drifts over time | Wheel slippage, inaccurate WHEEL_DIAMETER | Measure wheel diameter precisely, consider tracking wheels |
| 3 | Robot oscillates (shakes) during turn | Kp too high or Kd too low | Reduce Kp, increase Kd |
| 4 | Robot never reaches target heading | Kp too low, or friction too high | Increase Kp, check for mechanical binding |
| 5 | Autonomous overshoots target position | MAX_VELOCITY too high, motion profile decel too short | Reduce MAX_VELOCITY or increase MAX_ACCELERATION |
| 6 | IMU heading wrong after hitting wall | IMU was disturbed by impact | This is expected — consider recalibrating |
| 7 | Robot spins forever | No timeout in turn_to_heading | Already fixed with TURN_TIMEOUT_MS! |
| 8 | Different behavior on different surfaces | Friction affects wheel slip and PID response | Re-tune PID gains for competition surface |
| 9 | `make test` fails but robot works fine | Tests check math, not hardware behavior | Fix the test — math should always be correct |
| 10 | Robot works but auton is inconsistent | Battery level variation, wheels wearing down | Charge battery, check wheels, tune at worst-case |

---

## Quick Reference Card

```
╔══════════════════════════════════════════════╗
║         VEX V5 TEST QUICK REFERENCE         ║
╠══════════════════════════════════════════════╣
║                                              ║
║  Run unit tests:     make test               ║
║  Build for robot:    make                    ║
║                                              ║
║  Config file:        include/config.h        ║
║  Test file:          test/host_tests.cpp     ║
║                                              ║
║  PID Tuning Order:   P → D → I              ║
║  Start with:         Kp=1, Ki=0, Kd=0       ║
║                                              ║
║  Key measurements:                           ║
║    Wheel diameter:   Use calipers!           ║
║    Wheel track:      Center-to-center        ║
║    Ticks/rev:        Rotate 1 full turn      ║
║                                              ║
║  Emergency:  Check timeout constants         ║
║              TURN_TIMEOUT_MS = 2000          ║
║              DRIVE_TIMEOUT_MS = 5000         ║
║                                              ║
╚══════════════════════════════════════════════╝
```
