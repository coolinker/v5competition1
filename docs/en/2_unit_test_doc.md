# Unit Test Documentation — VEX V5 Robot

---

## Table of Contents

1. [What Are Unit Tests?](#1-what-are-unit-tests)
2. [Our Testing Approach](#2-our-testing-approach)
3. [How to Run Tests](#3-how-to-run-tests)
4. [Test Framework Explained](#4-test-framework-explained)
5. [Mock HAL Layer](#5-mock-hal-layer)
6. [Test Cases — PID Controller](#6-test-cases--pid-controller)
7. [Test Cases — Motion Profile](#7-test-cases--motion-profile)
8. [Test Cases — Odometry](#8-test-cases--odometry)
9. [How to Add New Tests](#9-how-to-add-new-tests)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. What Are Unit Tests?

A **unit test** tests one small piece of code (a "unit") in isolation. Think of it like testing one LEGO brick before building the whole castle.

**Why?**
- Catch bugs **before** deploying to the robot (saves time at competitions!)
- Verify math is correct without needing the physical robot
- Safely change code — if tests still pass, you didn't break anything

**Example:** Testing that a PID controller with Kp=2 and error=5 outputs 10:
```
Input:  setpoint=10, current=5 → error=5
Expect: output = Kp × error = 2 × 5 = 10  ✓
```

---

## 2. Our Testing Approach

VEX robots run on ARM processors, but tests run on your **laptop/desktop** (x86/ARM Mac). This is called **host-side testing**.

```
┌─────────────────┐          ┌─────────────────┐
│  Your Computer  │          │   VEX V5 Brain  │
│  (host tests)   │          │   (real robot)   │
│                 │          │                  │
│  test/          │          │  Same algorithm  │
│  host_tests.cpp │          │  code, but with  │
│                 │          │  REAL hardware   │
│  Mock HAL ←─────┤          ├──→ Real HAL      │
│  (fake motors)  │          │  (real motors)   │
└─────────────────┘          └─────────────────┘
```

**Key insight:** The algorithm code (PID, MotionProfile, Odometry) is the same in both environments. Only the HAL changes — tests use **mock (fake) functions**, the robot uses **real hardware functions**.

> **Multi-config note:** Unit tests are completely **configuration-independent**. Whether you use `ROBOT_2MOTOR` or `ROBOT_6MOTOR`, the same 23 tests validate the same PID, MotionProfile, and Odometry math. The tests use mock HAL functions that bypass all config-specific motor code. This demonstrates the architecture's reusability — algorithms are tested once and work for any drivetrain configuration.

### File Structure

All tests live in a single file: `test/host_tests.cpp`

```
test/host_tests.cpp
├── Minimal Test Framework (macros for assertions)
├── Mock HAL Layer (fake hardware functions)
├── #include source files under test
├── PID Controller Tests (6 tests)
├── PID Enhancement Tests (6 tests) — anti-windup, D-filter, output clamp
├── Motion Profile Tests (5 tests)
├── Odometry Tests (6 tests)
└── main() — runs all tests
```

---

## 3. How to Run Tests

Open a terminal in the project directory and run:

```bash
make test
```

**Expected output (all passing):**
```
============================================
  VEX Robot Host-Side Unit Tests
============================================

[PID Controller]
  [RUN ] PID_PositiveErrorProducesPositiveOutput
  [ OK ] PID_PositiveErrorProducesPositiveOutput
  ... (more tests)

[PID Enhancements]
  ... (6 tests)

[Motion Profile]
  ... (more tests)

[Odometry]
  ... (more tests)

[Drive Straight 1m]
  ... (3 tests)

============================================
  Results: 26 passed, 0 failed, 26 total
============================================
  ALL TESTS PASSED
```

**If a test fails:**
```
  [FAIL] PID_PositiveError (line 142): expected 10.000000, got 5.000000 (tol=0.010000)
```
This tells you: which test, which line in the test file, what was expected vs. what happened.

---

## 4. Test Framework Explained

We use a minimal custom framework (no external dependencies). Here are the key macros:

### Defining a test

```cpp
TEST(MyTestName) {
    // Your test code here
    ASSERT_NEAR(actual_value, expected_value, tolerance);
}
```

### Assertion macros

| Macro | What it checks | Example |
|-------|---------------|---------|
| `ASSERT_TRUE(cond)` | condition is true | `ASSERT_TRUE(x > 0)` |
| `ASSERT_NEAR(a, e, t)` | \|a - e\| ≤ t | `ASSERT_NEAR(output, 10.0, 0.01)` |
| `ASSERT_GT(a, b)` | a > b | `ASSERT_GT(speed, 0.0)` |
| `ASSERT_LT(a, b)` | a < b | `ASSERT_LT(error, 1.0)` |

### Running a test

```cpp
// In main():
RUN_TEST(MyTestName);
```

---

## 5. Mock HAL Layer

Since tests run on your computer (not on the robot), we need **fake** hardware functions. These are called "mocks."

```cpp
// Real HAL (on robot):                Mock HAL (in tests):
double get_left_encoder_ticks() {      double get_left_encoder_ticks() {
  return LeftMotor.position(deg);        return mock_left_ticks;  // ← we control this!
}                                      }
```

**Mock variables** let us simulate any sensor reading:

```cpp
static double mock_left_ticks = 0.0;    // Simulated left encoder
static double mock_right_ticks = 0.0;   // Simulated right encoder
static double mock_imu_heading_rad = 0; // Simulated IMU heading
static double mock_time_sec = 0.0;      // Simulated clock
```

Before each test, call `reset_all_mocks()` to start fresh.

---

## 6. Test Cases — PID Controller

### Test 1: Positive Error → Positive Output

| Item | Value |
|------|-------|
| **What it tests** | P-only controller responds correctly to positive error |
| **Setup** | Kp=2.0, Ki=0, Kd=0, setpoint=10, current=5 |
| **Expected** | output = 2.0 × 5.0 = 10.0 |
| **Why it matters** | Basic sanity — if this fails, PID is fundamentally broken |

### Test 2: Negative Error → Negative Output

| Item | Value |
|------|-------|
| **What it tests** | Controller outputs negative when current > setpoint |
| **Setup** | Kp=2.0, setpoint=5, current=10 → error=-5 |
| **Expected** | output = -10.0 |
| **Why it matters** | Ensures correctional direction is correct |

### Test 3: Zero Error → Zero Output

| Item | Value |
|------|-------|
| **What it tests** | No correction when already at target |
| **Setup** | setpoint = current = 5.0 |
| **Expected** | output ≈ 0.0 |
| **Why it matters** | Robot should stop moving when it reaches the target |

### Test 4: Integral Accumulates

| Item | Value |
|------|-------|
| **What it tests** | I-term grows over time with sustained error |
| **Setup** | Ki=1.0 (I-only), two iterations with same error |
| **Expected** | Second output > first output (integral grew) |
| **Why it matters** | Integral eliminates steady-state error over time |

### Test 5: Derivative Responds to Change

| Item | Value |
|------|-------|
| **What it tests** | D-term reacts to error change, zero when error is steady |
| **Setup** | Kd=1.0 (D-only), first call: error jumps from 0→5, second: stays at 5 |
| **Expected** | First: large positive output; Second: ≈ 0 |
| **Why it matters** | Derivative dampens overshoot by reacting to rate of change |

### Test 6: Reset Clears State

| Item | Value |
|------|-------|
| **What it tests** | `reset()` returns PID to fresh state |
| **Setup** | Run 10 iterations (building integral), reset, compare with fresh PID |
| **Expected** | Output after reset ≈ output from brand new PID |
| **Why it matters** | Must reset between autonomous movements to avoid stale integral |

---

## 6b. Test Cases — PID Enhancements

### Test 7: Anti-Windup Clamps Integral (Positive)

| Item | Value |
|------|-------|
| **What it tests** | `set_integral_limit(2.0)` prevents integral from growing beyond ±2.0 |
| **Setup** | Ki=1.0, 100 iterations with error=100, integral_limit=2.0 |
| **Expected** | Output ≈ 2.0 (not 100+ as it would be without clamping) |
| **Why it matters** | Prevents integral wind-up during motor saturation (e.g., pushing matches) |

### Test 8: Anti-Windup Clamps Integral (Negative)

| Item | Value |
|------|-------|
| **What it tests** | Anti-windup works symmetrically for negative errors |
| **Setup** | Same as Test 7 but with negative error |
| **Expected** | Output ≈ -2.0 |
| **Why it matters** | Ensures clamping works in both directions |

### Test 9: D-Filter Smooths Derivative

| Item | Value |
|------|-------|
| **What it tests** | `set_d_filter(0.7)` attenuates derivative spikes |
| **Setup** | Two D-only PIDs — one raw, one with EMA filter alpha=0.7 |
| **Expected** | Filtered output < raw output on first error spike |
| **Why it matters** | Reduces noise-induced jitter in motor commands |

### Test 10: Output Limit Clamps Output

| Item | Value |
|------|-------|
| **What it tests** | `set_output_limit(5.0)` clamps output to ±5.0 |
| **Setup** | Kp=10.0, error=100, output_limit=5.0 |
| **Expected** | Output = +5.0 (not 1000). Negative error → -5.0 |
| **Why it matters** | Prevents commanding voltages beyond hardware capability |

### Test 11: No Clamping When Disabled

| Item | Value |
|------|-------|
| **What it tests** | Default output_limit=0 means no clamping |
| **Setup** | Kp=10.0, error=100, no set_output_limit() call |
| **Expected** | Output = 1000.0 (unclamped) |
| **Why it matters** | Ensures backward compatibility — 2-motor config is unaffected |

### Test 12: Reset Clears Enhanced State

| Item | Value |
|------|-------|
| **What it tests** | `reset()` also clears EMA filtered derivative state |
| **Setup** | PID with all enhancements enabled, 20 iterations, reset, compare with fresh |
| **Expected** | Output after reset ≈ output from brand new PID with same settings |
| **Why it matters** | Ensures no stale filter state carries over between autonomous movements |

---

## 7. Test Cases — Motion Profile

### Test 1: Acceleration Phase

| Item | Value |
|------|-------|
| **What it tests** | Early in motion, velocity follows accel ramp |
| **Setup** | max_v=1.0, max_a=2.0, t=0.1s, d=2.0m |
| **Expected** | v = a×t = 2.0×0.1 = 0.2 m/s |
| **Why it matters** | Robot must start slowly, not jump to full speed |

### Test 2: Reaches Max Velocity

| Item | Value |
|------|-------|
| **What it tests** | Velocity caps at max_v after enough time |
| **Setup** | max_v=1.0, t=1.0s (accel would give 2.0 m/s) |
| **Expected** | v = 1.0 (capped) |
| **Why it matters** | Safety — never exceed the configured speed limit |

### Test 3: Deceleration Phase

| Item | Value |
|------|-------|
| **What it tests** | Slows down when approaching target |
| **Setup** | distance_to_go = 0.1m |
| **Expected** | v = √(2×2.0×0.1) ≈ 0.632 < max_v |
| **Why it matters** | Robot must stop at the target, not overshoot it |

### Test 4: Zero Distance → Zero Velocity

| Item | Value |
|------|-------|
| **What it tests** | At target: velocity should be zero |
| **Setup** | distance_to_go = 0 |
| **Expected** | v = 0.0 |
| **Why it matters** | Robot should be stopped when it has arrived |

### Test 5: Velocity Never Exceeds Max

| Item | Value |
|------|-------|
| **What it tests** | No combination of time/distance produces v > max |
| **Setup** | 50×50 = 2500 combinations of t and d |
| **Expected** | All v ≤ 1.0 |
| **Why it matters** | Ensures max_velocity cap works in all scenarios |

---

## 8. Test Cases — Odometry

### Test 1: Initial Pose Is Zero

| Item | Value |
|------|-------|
| **What it tests** | After `set_pose({0,0,0})`, pose reads back as (0,0,0) |
| **Expected** | x=0, y=0, θ=0 |
| **Why it matters** | Basic initialization sanity check |

### Test 2: Set Pose Works

| Item | Value |
|------|-------|
| **What it tests** | `set_pose({1.5, 2.5, 0.5})` stores correctly |
| **Expected** | get_pose() returns (1.5, 2.5, 0.5) |
| **Why it matters** | Must be able to set starting position |

### Test 3: Drive Straight Forward

| Item | Value |
|------|-------|
| **What it tests** | Driving both wheels 1m forward, heading=0 |
| **Setup** | left_ticks = right_ticks = 1m worth, IMU heading = 0 |
| **Expected** | x ≈ 1.0, y ≈ 0.0, θ ≈ 0.0 |
| **Why it matters** | Core odometry: straight driving must work correctly |

### Test 4: Point Turn 90°

| Item | Value |
|------|-------|
| **What it tests** | Spinning in place (left backward, right forward) |
| **Setup** | left_ticks = negative, right_ticks = positive (90° arc), IMU = π/2 |
| **Expected** | x ≈ 0, y ≈ 0, θ ≈ π/2 |
| **Why it matters** | Turning must update heading without moving position |

### Test 5: Drive Backward

| Item | Value |
|------|-------|
| **What it tests** | Negative encoder ticks = backward motion |
| **Setup** | Both wheels -0.5m of ticks, heading=0 |
| **Expected** | x ≈ -0.5, y ≈ 0 |
| **Why it matters** | Odometry must handle reverse correctly |

### Test 6: Multiple Updates Accumulate

| Item | Value |
|------|-------|
| **What it tests** | Two half-meter updates equal one full meter |
| **Setup** | Step 1: 0.5m forward. Step 2: another 0.5m forward |
| **Expected** | Final x ≈ 1.0 |
| **Why it matters** | Incremental updates must be additive — no state corruption |

---

## 9. How to Add New Tests

1. Open `test/host_tests.cpp`
2. Add a new test function using the `TEST()` macro:

```cpp
TEST(MyNewFeature_SomeCondition) {
    reset_all_mocks();       // Start clean
    mock_time_sec = 1.0;     // Set up mock state

    // Call the function you're testing
    double result = my_function(input);

    // Check the result
    ASSERT_NEAR(result, expected_value, 0.01);
}
```

3. Register it in `main()`:

```cpp
RUN_TEST(MyNewFeature_SomeCondition);
```

4. Run `make test` to verify.

**Test naming convention:** `ModuleName_WhatIsBeingTested`

Examples:
- `PID_ZeroErrorProducesZeroOutput`
- `MotionProfile_DecelerationPhase`
- `Odometry_DriveStraightForward`

---

## 10. Troubleshooting

| Problem | Solution |
|---------|----------|
| `make test` says "Nothing to be done" | Check that the `test` target exists in `makefile` |
| `g++: command not found` | Install Xcode Command Line Tools: `xcode-select --install` |
| `fatal error: 'config.h' file not found` | Run from the project root directory |
| Test compiles but gives wrong results | Add `reset_all_mocks()` at the start of each test |
| `Undefined symbol: get_imu_rotation_rad` | Add the mock function to the Mock HAL section |
| All tests pass but robot behaves wrong | Tests only verify math — PID gains need tuning on real hardware |

---

## Test Coverage Summary

| Module | # Tests | Coverage |
|--------|---------|---------|
| PID Controller | 6 | P, I, D terms individually, zero error, reset |
| PID Enhancements | 6 | Anti-windup (±), D-filter, output clamp (on/off), reset |
| Motion Profile | 5 | Accel, cruise, decel, zero, bounds |
| Odometry | 6 | Init, set, forward, turn, backward, accumulation |
| Drive Straight 1m | 3 | Target degrees, speed ramp, stop condition |
| **Total** | **26** | **Core algorithms + enhancements fully covered** |

**Not tested (by design):**
- `turn_to_heading` / `drive_to_pose` — These are integration-level (depend on control loop timing). Tested on-robot.
- Boomerang controller — Integration-level; verified through on-robot system tests.
- HAL functions — These just wrap VEX API calls; nothing to unit test. The 6-motor motor array logic is verified through on-robot component tests (see [Full Test Document](3_full_test_doc.md)).
- `main.cpp` — Competition callbacks and motor declarations are config-specific; tested by running the robot.
- Config selection (`#ifdef ROBOT_2MOTOR` / `ROBOT_6MOTOR`) — Compile-time guards; tested implicitly by building the project.
