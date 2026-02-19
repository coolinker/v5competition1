# Step-by-Step Build & Test Guide — VEX V5 Six-Wheel Competition Robot

**Progressive build, test, and tuning best practices — from zero to a complete system**

> **Core Principle: Add one module at a time. Confirm it works before moving on.**
> Never plug in all sensors, flash all the code, and pray it works.

---

## Table of Contents

- [Overall Approach](#overall-approach)
- [Phase 0: Host Unit Tests (No Hardware Needed)](#phase-0-host-unit-tests-no-hardware-needed)
- [Phase 1: Minimal Chassis — Motors Only](#phase-1-minimal-chassis--motors-only)
- [Phase 2: Add the Inertial Sensor (IMU)](#phase-2-add-the-inertial-sensor-imu)
- [Phase 3: Add Tracking Wheels](#phase-3-add-tracking-wheels)
- [Phase 4: Odometry Validation](#phase-4-odometry-validation)
- [Phase 5: Turn PID Tuning](#phase-5-turn-pid-tuning)
- [Phase 6: Drive PID Tuning](#phase-6-drive-pid-tuning)
- [Phase 7: Boomerang Path Validation](#phase-7-boomerang-path-validation)
- [Phase 8: Vision Localization (Optional)](#phase-8-vision-localization-optional)
- [Phase 9: Full Autonomous Routine](#phase-9-full-autonomous-routine)
- [Phase 10: Competition Readiness Checklist](#phase-10-competition-readiness-checklist)
- [Appendix A: Troubleshooting Table](#appendix-a-troubleshooting-table)
- [Appendix B: Tuning Quick-Reference](#appendix-b-tuning-quick-reference)

---

## Overall Approach

```
Phase 0   Host tests (no hardware)
  ↓
Phase 1   Motors → can drive
  ↓
Phase 2   +IMU → knows heading
  ↓
Phase 3   +Tracking wheels → knows position
  ↓
Phase 4   Odometry validation → position is accurate
  ↓
Phase 5   Turn PID → precise turns
  ↓
Phase 6   Drive PID → precise straight lines
  ↓
Phase 7   Boomerang → curved paths
  ↓
Phase 8   Vision localization → absolute correction (optional)
  ↓
Phase 9   Full autonomous → match strategy
  ↓
Phase 10  Competition ready → go compete
```

Each phase contains three parts:
1. **Hardware Wiring** — what to connect
2. **Software Verification** — how to confirm the code works
3. **Pass Criteria** — what metrics must be met before advancing

---

## Phase 0: Host Unit Tests (No Hardware Needed)

### Goal
Verify all math and algorithms on your computer (Mac/Linux/Windows) — no VEX hardware required.

### Steps

#### 0.1 Build and Run Tests

```bash
cd v5competition1
make test
```

#### 0.2 Confirm Output

You should see output like:

```
============================================
  VEX Robot Host-Side Unit Tests
  Config: 6-motor + perpendicular tracking wheels
============================================

[PID Controller]
  [ OK ] PID_PositiveErrorProducesPositiveOutput
  [ OK ] PID_NegativeErrorProducesNegativeOutput
  ...

[Odometry — Perpendicular Tracking Wheels]
  [ OK ] Odometry_DriveStraightForward
  [ OK ] Odometry_PointTurn90Degrees
  [ OK ] Odometry_LateralSlide
  ...

  Results: 24 passed, 0 failed, 24 total
  ALL TESTS PASSED
```

#### 0.3 Understand Test Coverage

| Category | Tests | What Is Verified |
|----------|-------|------------------|
| PID Basics | 6 | P/I/D terms independently correct, reset clears state |
| PID Advanced | 6 | Integral anti-windup, derivative filter, output clamping |
| Motion Profile | 5 | Accel / cruise / decel / zero distance / saturation |
| Odometry | 7 | Straight / turn / reverse / accumulation / arc |

### ✅ Pass Criteria
- 24/24 tests pass
- If any fail, fix the code and re-run until all pass

---

## Phase 1: Minimal Chassis — Motors Only

### Goal
Confirm all 6 motors spin, direction is correct, and speed is nominal.

### Hardware Wiring

| Device | Port | Notes |
|--------|------|-------|
| Left Front Motor | Port 1 | Blue cartridge 600 RPM, reversed |
| Left Middle Motor | Port 2 | Blue cartridge 600 RPM, reversed |
| Left Rear Motor | Port 3 | Blue cartridge 600 RPM, reversed |
| Right Front Motor | Port 4 | Blue cartridge 600 RPM, forward |
| Right Middle Motor | Port 5 | Blue cartridge 600 RPM, forward |
| Right Rear Motor | Port 6 | Blue cartridge 600 RPM, forward |

> ⚠️ **Left-side motors are set to reversed** because they are physically mounted in the opposite direction.

### Software Verification

#### 1.1 Flash the Program to the V5 Brain

In VS Code: press `Ctrl+Shift+P` → select `VEX: Build and Upload`

#### 1.2 Controller Drive Test

After uploading, connect the controller and switch to `Driver Control` mode:
- **Left stick up** → left-side wheels spin forward
- **Right stick up** → right-side wheels spin forward
- **Both sticks up** → robot drives straight forward
- **Left up, right down** → robot turns right

```
Controller mapping (tank drive):
  Left stick  (Axis3) → left 3 motors voltage
  Right stick (Axis2) → right 3 motors voltage

  Voltage = stick percent × 12 V
  Deadband: |stick| < 5% treated as 0
```

#### 1.3 Checklist

| # | Check | Expected | If Abnormal |
|---|-------|----------|-------------|
| 1 | All 6 motors spin | No dead motors | Check port wiring and motor cables |
| 2 | Left 3 motors same direction | Pushing left stick → all 3 spin forward | One is reversed → check port assignment in `config.h` or flip reversed flag in `main.cpp` |
| 3 | Right 3 motors same direction | Pushing right stick → all 3 spin forward | Same as above |
| 4 | Both sticks up → straight line | Robot drives roughly straight (slight drift OK) | Check that same-side motors have identical gear ratios |
| 5 | No unusual noises at full speed | Motor sound is normal, no clicking | Mechanical issue: gear meshing, bent shaft |

#### 1.4 Brain Screen Check

After power-on the screen should display:
```
=== 6M Tracking Odom ===
X: 0.000 m
Y: 0.000 m
Heading: 0.0 deg
```

> Pose data will not update correctly yet (no IMU or tracking wheels connected) — this is expected.

### ✅ Pass Criteria
- All 6 motors are controllable
- Same-side 3 motors spin in the same direction
- Both sticks up → robot moves forward
- Brain screen displays something (not black)

---

## Phase 2: Add the Inertial Sensor (IMU)

### Goal
Confirm the IMU calibrates successfully and heading readings are accurate.

### Hardware Wiring

On top of Phase 1, add:

| Device | Port | Notes |
|--------|------|-------|
| V5 Inertial Sensor | Port 10 | Mount as close to the chassis center as possible, away from motors |

> ⚠️ **Mounting requirement:** The IMU must be mounted level (horizontal). Any tilt will introduce reading bias.

### Software Verification

#### 2.1 Calibration Confirmation

After flashing, watch the Brain screen:
```
Initializing...
INF IMU calibration started      ← you should see this
INF IMU calibration finished     ← see this within 3 seconds = success
```

If you see:
```
INF IMU calibration TIMEOUT
```
the IMU is not plugged in correctly or the port is wrong.

#### 2.2 Heading Test

1. After flashing, place the robot on the ground. Screen should show `Heading: 0.0 deg`
2. **Manually rotate the robot 90° counter-clockwise** → should show `Heading: ~90 deg`
3. **Rotate back to the original position** → should return to `Heading: ~0 deg`
4. **Rotate 90° clockwise** → should show `Heading: ~-90 deg` or `~270 deg`

#### 2.3 Calibration Tips

- The robot **must be completely still** during calibration (~2 seconds)
- Do not calibrate while motors are running (vibration hurts accuracy)
- Do not bump the Brain after calibration — it affects the internal gyroscope

### ✅ Pass Criteria
- IMU calibration completes within 3 seconds (no TIMEOUT)
- Manually rotating 90° → screen reads between 85°–95°
- Full 360° rotation back to start → error < 5°

---

## Phase 3: Add Tracking Wheels (Perpendicular Layout)

### Goal
Confirm that the forward and lateral tracking wheels read distance correctly and direction is correct.

### Hardware Wiring

On top of Phase 2, add:

| Device | Port | Notes |
|--------|------|-------|
| Forward Tracking Wheel Rotation Sensor | Port 8 | 2.75″ omni wheel + V5 Rotation Sensor, parallel to the forward direction |
| Lateral Tracking Wheel Rotation Sensor | Port 9 | 2.75″ omni wheel + V5 Rotation Sensor, perpendicular to the forward direction |

> ⚠️ **Mounting requirements:**
> - Tracking wheels must always touch the ground (use spring-loaded floating mounts)
> - **Forward wheel** is mounted along the robot’s forward direction, as close to the centerline as possible
> - **Lateral wheel** is mounted perpendicular to the forward direction (measures sideways sliding)
> - Measure the lateral wheel’s front/back offset from the rotation center → `LATERAL_WHEEL_OFFSET` in `config.h`
> - Measure the forward wheel’s left/right offset from the rotation center → `FORWARD_WHEEL_OFFSET` in `config.h`

### Software Verification

#### 3.1 Connection Check

After flashing, the screen should NOT display `Tracking wheels NOT detected!`

If it does, check:
- That the rotation sensors are plugged into Port 8 and Port 9
- That the sensor cables are fully seated

#### 3.2 Manual Roll Test

**Method: Lift the robot off the ground (wheels not touching) and spin the tracking wheels by hand.**

Temporarily add the following code to `autonomous()` to print tracking wheel readings:

```cpp
void autonomous() {
    while (true) {
        double fwd_d = tracking_get_forward_distance_m();
        double lat_d = tracking_get_lateral_distance_m();
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Fwd dist: %.4f m", fwd_d);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Lat dist: %.4f m", lat_d);
        wait(50, msec);
    }
}
```

| # | Check | Expected | If Abnormal |
|---|-------|----------|-------------|
| 1 | Roll forward tracking wheel forward | Fwd dist increases | Set `FORWARD_TRACKING_REVERSED = true` in `config.h` |
| 2 | Roll lateral tracking wheel to the left | Lat dist increases | Set `LATERAL_TRACKING_REVERSED = true` in `config.h` |
| 3 | Roll one full revolution (2.75″ wheel) | Reading ≈ 0.2195 m | Check `TRACKING_WHEEL_DIAMETER` is correct |
| 4 | Two sensors are independent | Rolling one doesn’t affect the other | Check wiring for cross-connection |

#### 3.3 Accuracy Verification

Use a ruler to mark **exactly 1 meter** on the floor:

1. Place the robot at the start mark
2. Manually push it forward exactly 1 meter → check forward wheel reading
3. Manually push it sideways exactly 1 meter → check lateral wheel reading

**Expected: Reading 0.98–1.02 m (±2% error)**

If error > 5%:
- Re-measure the wheel diameter more carefully
- Check that the wheel maintains ground contact (no slipping)
- Check that the rotation sensor shaft is not loose

### ✅ Pass Criteria
- Both tracking wheels connect without warnings
- Direction is correct (forward/left = positive)
- Pushing 1 meter → reading error < 2%

---

## Phase 4: Odometry Validation

### Goal
Confirm the odometry system correctly fuses perpendicular tracking wheel and IMU data, and pose estimates are accurate.

### Prerequisites
Phases 1–3 all passed.

### Software Verification

Restore the normal `autonomous()` code (remove the temporary code from Phase 3).

#### 4.1 Straight-Line Test

1. Place the robot at (0, 0), facing the positive X direction
2. Manually push it in a straight line for **exactly 1 meter**
3. Read the Brain screen:

```
Expected: X ≈ 1.00 m,  Y ≈ 0.00 m,  Heading ≈ 0°
```

| Accuracy | Pass | Excellent |
|----------|------|-----------|
| X error | < 3 cm | < 1 cm |
| Y drift | < 3 cm | < 1 cm |

#### 4.2 Turn Test

1. Place at (0, 0)
2. Manually rotate 90° counter-clockwise in place (use floor markings as reference)
3. Read the Brain screen:

```
Expected: X ≈ 0.00 m,  Y ≈ 0.00 m,  Heading ≈ 90°
```

#### 4.3 Rectangle Test (Most Important)

**This is the ultimate test for odometry accuracy.**

1. Tape a 1 m × 1 m square on the floor
2. Place the robot at one corner
3. Manually push it around the full square: forward 1 m → turn left 90° → forward 1 m → turn left 90° → forward 1 m → turn left 90° → forward 1 m → turn left 90°
4. Return to the starting point

```
Expected: X ≈ 0.00 m,  Y ≈ 0.00 m,  Heading ≈ 0° (or 360°)
```

| Accuracy | Pass | Excellent |
|----------|------|-----------|
| Position error | < 10 cm | < 3 cm |
| Heading error | < 10° | < 3° |

#### 4.4 If Error Is Too Large

| Symptom | Probable Cause | Solution |
|---------|---------------|----------|
| X/Y scale is off | `TRACKING_WHEEL_DIAMETER` is inaccurate | Re-measure the wheel diameter |
| Position shifts during turns | `FORWARD_WHEEL_OFFSET` or `LATERAL_WHEEL_OFFSET` is inaccurate | Re-measure the tracking wheel offsets from the rotation center |
| Straight-line drift | Forward tracking wheel not aligned with the forward direction | Physically realign the forward tracking wheel mount |
| Heading drifts | IMU drift (normal) | Periodically recalibrate IMU, or add Vision position correction |

#### 4.5 Fine-Tuning Method

**Wheel diameter correction:**
```
Actual distance = 1.000 m
Odometry reading = 0.980 m
Correction factor = 1.000 / 0.980 = 1.0204
New diameter = TRACKING_WHEEL_DIAMETER × 1.0204
```

**Offset correction:**
```
If position shifts during in-place turns:
1. Turn 360° in place and record the final X and Y error
2. Fine-tune FORWARD_WHEEL_OFFSET and LATERAL_WHEEL_OFFSET
3. Repeat until X/Y stays near zero during pure rotation
```

### ✅ Pass Criteria
- Straight-line 1 m → position error < 3 cm
- Turn 90° → heading error < 5°
- Rectangle test → return-to-start error < 10 cm

---

## Phase 5: Turn PID Tuning

### Goal
Make the robot turn precisely to a specified heading in place.

### Prerequisites
Phase 4 passed (odometry is accurate).

### Tuning Steps

#### 5.1 Prepare Test Code

Write the following in `autonomous()`:

```cpp
void autonomous() {
    hal_log("=== Turn PID Test ===");

    // Turn to 90°
    turn_to_heading(M_PI / 2.0);
    wait(1000, msec);  // pause 1 s to observe

    // Turn back to 0°
    turn_to_heading(0.0);
    wait(1000, msec);

    // Turn to 180°
    turn_to_heading(M_PI);
    wait(1000, msec);

    // Return to 0°
    turn_to_heading(0.0);
}
```

#### 5.2 Three-Step Tuning Method

**All parameters are modified in Section 5 of `config.h`.**

##### Step 1: Tune P only (set I=0, D=0)

```cpp
constexpr double TURN_KP = 2.0;   // start at 2.0
constexpr double TURN_KI = 0.0;   // set to 0 for now
constexpr double TURN_KD = 0.0;   // set to 0 for now
```

Observe behavior:

| Behavior | KP Adjustment |
|----------|--------------|
| Doesn't reach target, far off | Increase KP (+0.5) |
| Reaches target but slowly, slightly short | Increase KP slightly (+0.2) |
| Obvious overshoot, oscillates back and forth | Decrease KP (-0.5) |
| Reaches quickly, slight overshoot | ✅ KP is about right, proceed to next step |

**Goal: When turning 90°, the robot approaches quickly with overshoot < 10°**

##### Step 2: Add D (I still at 0)

```cpp
constexpr double TURN_KD = 0.1;   // start at 0.1
```

| Behavior | KD Adjustment |
|----------|--------------|
| Still oscillates | Increase KD (+0.1) |
| Response becomes sluggish | Decrease KD (-0.05) |
| Fast and stable, no oscillation | ✅ KD is about right |

**Goal: No oscillation, fast and stable settling**

##### Step 3: Add a tiny amount of I

```cpp
constexpr double TURN_KI = 0.01;  // start at 0.01, very small
```

| Behavior | KI Adjustment |
|----------|--------------|
| Steady-state error of 1–2° remains | Increase KI (+0.01) |
| Starts oscillating | KI is too high, decrease it |
| Settles precisely on target | ✅ Done |

#### 5.3 Advanced Parameter Adjustment

```cpp
constexpr double TURN_INTEGRAL_LIMIT = 3.0;    // prevent integral windup
constexpr double TURN_D_FILTER       = 0.5;    // smooth derivative noise
```

- **INTEGRAL_LIMIT**: If the I term causes oscillation after turning, lower this value
- **D_FILTER**: If the robot jitters near the target, increase this value (0–1, higher = smoother)

#### 5.4 Validate Multiple Angles

| Target Angle | Pass Accuracy | Excellent Accuracy |
|-------------|--------------|-------------------|
| 90° | error < 3° | error < 1.5° |
| 180° | error < 5° | error < 2° |
| 45° | error < 3° | error < 1.5° |
| -90° | error < 3° | error < 1.5° |

### ✅ Pass Criteria
- 90° turn settling time < 1 second
- Steady-state error < 2° (≈ 0.035 rad)
- No sustained oscillation
- 4 consecutive turns produce consistent results

---

## Phase 6: Drive PID Tuning

### Goal
Make the robot drive a precise straight line to a specified distance while maintaining heading.

### Tuning Steps

#### 6.1 Test Code

```cpp
void autonomous() {
    hal_log("=== Drive PID Test ===");

    // Drive forward 1 m
    drive_to_pose({1.0, 0.0, 0.0});
    wait(1000, msec);

    // Return to start
    drive_to_pose({0.0, 0.0, M_PI}, true);  // reverse
    wait(1000, msec);
}
```

#### 6.2 PID Inside Boomerang

Note: `drive_to_pose` internally uses the **turn PID** to correct heading — there is **no separate linear PID to tune**.
Linear speed is governed by the motion planner (deceleration curve `sqrt(2×a×d)` + acceleration limiting).

Key parameters are in Sections 6 and 7 of `config.h`:

```cpp
// Section 6: Drive settling tolerances
constexpr double DRIVE_SETTLE_M       = 0.015;   // position tolerance 1.5 cm
constexpr double DRIVE_SETTLE_TIME_MS = 150;      // time to remain within tolerance
constexpr double DRIVE_TIMEOUT_MS     = 4000;     // timeout

// Section 7: Velocity planning
constexpr double MAX_VELOCITY     = 1.2;   // max speed m/s
constexpr double MAX_ACCELERATION = 3.0;   // max acceleration m/s²
constexpr double BOOMERANG_LEAD   = 0.6;   // Boomerang lead factor
```

#### 6.3 Tuning Method

| Parameter | Effect | How to Adjust |
|-----------|--------|---------------|
| `MAX_VELOCITY` | Maximum speed | Start at 0.5 m/s for debugging, increase once stable |
| `MAX_ACCELERATION` | Acceleration | Start at 1.5 m/s², decrease if too aggressive |
| `DRIVE_SETTLE_M` | Position tolerance | Smaller = more precise but more likely to timeout |
| `TURN_KP` (angular PID) | Heading correction during drive (`drive_to_pose` reuses the turn PID) | Increase if the robot drifts sideways |

#### 6.4 Validation

| # | Test | Pass | Excellent |
|---|------|------|-----------|
| 1 | Drive 1 m — position error | < 3 cm | < 1.5 cm |
| 2 | Lateral drift during drive | < 5 cm | < 2 cm |
| 3 | Heading deviation after drive | < 5° | < 2° |
| 4 | Smooth motion, no abrupt stops | Yes | Yes, accel/decel are smooth |

### ✅ Pass Criteria
- 1 m straight-line position error < 3 cm
- Lateral drift < 5 cm
- No sudden acceleration or deceleration during motion

---

## Phase 7: Boomerang Path Validation

### Goal
Verify the Boomerang controller can drive smooth curved paths to any target pose.

### Test Code

```cpp
void autonomous() {
    hal_log("=== Boomerang Path Test ===");

    // Test 1: Straight line + rotation
    drive_to_pose({1.0, 0.0, 0.0});       // drive straight 1 m
    wait(500, msec);

    // Test 2: Curved approach
    drive_to_pose({1.0, 1.0, M_PI / 2.0}); // arc to (1,1), facing north
    wait(500, msec);

    // Test 3: Reverse arc
    drive_to_pose({0.0, 0.0, 0.0}, true);  // reverse back to start
    wait(500, msec);

    // Test 4: Large-angle arc
    drive_to_pose({0.5, 1.0, M_PI});       // arc to (0.5, 1), facing left
}
```

### Validation Checklist

| # | Check | Expected |
|---|-------|----------|
| 1 | Path is a curve, not a broken line | Robot arcs to the target — does not turn-then-drive |
| 2 | Final heading is correct | Heading matches target angle upon arrival |
| 3 | Reverse works | With reverse=true the robot arcs backward |
| 4 | No overshoot oscillation | Robot stops quietly after reaching the target |

### BOOMERANG_LEAD Tuning

```
BOOMERANG_LEAD = 0.0  → straight line to target (ignores final heading)
BOOMERANG_LEAD = 0.3  → gentle arc
BOOMERANG_LEAD = 0.6  → moderate arc (recommended starting point)
BOOMERANG_LEAD = 1.0  → strong arc (wide sweep)
```

- If the arc is too wide and leaves the field → decrease LEAD
- If the final heading is inaccurate → increase LEAD

### ✅ Pass Criteria
- Curved path is smooth
- Arrival pose error < 5 cm (position) + < 10° (heading)
- Reverse mode works
- Completes a 1 m arc within 4 seconds

---

## Phase 8: Vision Localization (Optional)

### Goal
Use AprilTag absolute positioning to correct odometry drift.

### Hardware Wiring

On top of Phase 7, add:

| Device | Port | Notes |
|--------|------|-------|
| AI Vision Sensor | Port 12 | Mount on the front of the robot, facing forward |

### Field Preparation

Place AprilTag markers around the field perimeter and fill in each tag's information in the `FIELD_TAGS[]` array in `vision_localizer.cpp`:
- `id` — tag ID
- `x, y` — position on the field (meters)
- `z` — height above ground (meters), used for distance estimation
- `facing` — tag orientation (radians, outward normal direction)

### Verification Steps

#### 8.1 Connection Confirmation

The Brain screen `Vision tags: X` should change as tags are detected:
- Facing a tag → `Vision tags: 1`
- Facing away → `Vision tags: 0`

#### 8.2 Localization Accuracy Test

1. Place the robot at a known position (e.g., (1.0, 1.0))
2. Call `set_pose({1.0, 1.0, 0.0})`
3. Let odometry run for a while
4. Check whether the Vision-corrected pose is more accurate

### Tuning Parameters

```cpp
// config.h Section 10
constexpr double VISION_CORRECTION_ALPHA = 0.4;  // correction strength (0=none, 1=fully trust vision)
constexpr double VISION_MAX_CORRECTION_M = 0.30;  // max single correction magnitude
constexpr double VISION_MIN_CONFIDENCE   = 0.15;  // minimum confidence threshold
```

### ✅ Pass Criteria
- AprilTags are detected
- Vision correction reduces odometry drift (does not increase it)
- No visible pose jumps after correction

---

## Phase 9: Full Autonomous Routine

### Goal
Write the complete match autonomous routine and test it on the field.

### Steps

#### 9.1 Plan the Route

1. Mark all target positions on the field map
2. Determine the order of waypoints
3. Pay attention to the robot's final heading at each waypoint

#### 9.2 Write the Routine

```cpp
void autonomous() {
    hal_log("=== Match Autonomous ===");

    // Starting pose (adjust based on your start position)
    set_pose({0.3, 0.3, 0.0});

    // Waypoint 1: Drive to first target
    drive_to_pose({1.0, 0.3, 0.0});

    // Waypoint 2: Turn and advance
    drive_to_pose({1.0, 1.0, M_PI / 2.0});

    // Waypoint 3: Continue...
    turn_to_heading(M_PI);
    drive_to_pose({0.3, 1.0, M_PI});

    // ... continue based on match strategy
}
```

#### 9.3 Analyze SD Card Logs

The program automatically generates `odom_log.csv` on the SD card:

```csv
time_ms,x,y,theta,error
100,0.001,0.000,0.001,0.999
200,0.015,0.001,0.002,0.985
...
```

Open it on your computer with Excel or Python and plot the trajectory:

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('odom_log.csv')
plt.plot(df['x'], df['y'], '-o', markersize=2)
plt.axis('equal')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory')
plt.grid(True)
plt.show()
```

#### 9.4 Iterative Optimization

1. Run the autonomous once → view the CSV trajectory
2. Find the point with the largest deviation → adjust parameters or path
3. Repeat until satisfied

### ✅ Pass Criteria
- Full autonomous routine completes within 15 seconds (match time limit)
- Each waypoint position error < 5 cm
- Repeat 5 times → consistency > 80%

---

## Phase 10: Competition Readiness Checklist

### Pre-Match Check

| # | Item | Status |
|---|------|--------|
| 1 | Battery charge > 80% | ☐ |
| 2 | All motors detected by the Brain | ☐ |
| 3 | IMU calibration passed (no TIMEOUT) | ☐ |
| 4 | Both tracking wheels connected normally (forward + lateral) | ☐ |
| 5 | Controller connected, controls responsive | ☐ |
| 6 | Autonomous tested 3 times with consistent results | ☐ |
| 7 | SD card inserted (for logging) | ☐ |
| 8 | All screws tightened | ☐ |
| 9 | Tracking wheel springs pressing against ground (check both) | ☐ |
| 10 | Starting pose `set_pose()` set to correct values | ☐ |

### Mechanical Check

| # | Item | Status |
|---|------|--------|
| 1 | Wheels are not loose | ☐ |
| 2 | No unusual gear noise | ☐ |
| 3 | Cables cannot be pinched or crushed | ☐ |
| 4 | Battery is securely fastened | ☐ |
| 5 | Robot dimensions are within rules | ☐ |

---

## Appendix A: Troubleshooting Table

| # | Symptom | Probable Cause | Solution |
|---|---------|---------------|----------|
| 1 | Motor doesn't spin | Wrong port / loose cable | Check port numbers in `config.h`, reseat cables |
| 2 | One side motor reversed | Motor reversed flag incorrect | Flip `true/false` for that motor in `main.cpp` |
| 3 | IMU calibration timeout | Sensor not plugged in / wrong port | Check Port 10 wiring |
| 4 | Tracking wheel reading stays 0 | Rotation sensor not connected | Check Ports 8/9, confirm sensor LED is on |
| 5 | Tracking wheel direction reversed | reversed flag is wrong | Set `FORWARD/LATERAL_TRACKING_REVERSED` in `config.h` |
| 6 | Odometry drifts on straight line | Forward tracking wheel misaligned / diameter inaccurate | Physical adjustment + correct `TRACKING_WHEEL_DIAMETER` |
| 7 | Odometry position shifts during turns | Tracking wheel offset inaccurate | Correct `FORWARD_WHEEL_OFFSET` and `LATERAL_WHEEL_OFFSET` |
| 8 | Turn oscillation | KP too high or KD too low | Decrease KP, increase KD |
| 9 | Turn doesn't reach target | KP too low or no KI | Increase KP, add small KI |
| 10 | Drives crooked | Single-side motor issue / insufficient heading correction | Check motors, increase `TURN_KP` (`drive_to_pose` reuses the turn PID) |
| 11 | Autonomous route drifts significantly | Accumulated odometry error | Add Vision correction, shorten path segments |
| 12 | Brain goes black | Program crash | Check compiler warnings, simplify code and debug incrementally |
| 13 | No log on SD card | SD card not formatted / not inserted | Format as FAT32, reinsert |
| 14 | Motor overheating | Excessive load / gears jammed | Check mechanism, lower voltage output cap |

---

## Appendix B: Tuning Quick-Reference

### Recommended Starting Values

| Parameter | Initial Value | Range | Notes |
|-----------|--------------|-------|-------|
| **TURN_KP** | 2.0 | 1.0–8.0 | Turn P — tune first |
| **TURN_KI** | 0.0 | 0.0–0.1 | Turn I — tune last, keep very small |
| **TURN_KD** | 0.0 | 0.0–1.0 | Turn D — tune second |
| **TURN_INTEGRAL_LIMIT** | 3.0 | 1.0–10.0 | I term upper limit |
| **TURN_D_FILTER** | 0.5 | 0.0–0.9 | D filter, higher = smoother |
| **MAX_VELOCITY** | 0.5 | 0.3–1.3 | Start slow, increase later |
| **MAX_ACCELERATION** | 1.5 | 1.0–5.0 | Start gentle, increase later |
| **BOOMERANG_LEAD** | 0.6 | 0.0–1.0 | Arc intensity |
| **IMU_FUSION_ALPHA** | 0.98 | 0.8–1.0 | Higher = trust IMU more |
| **DRIVE_SETTLE_M** | 0.015 | 0.01–0.05 | Position tolerance |

### Tuning Strategy Summary

```
1. Ensure hardware is functional (Phases 1–3)
2. Ensure odometry is accurate (Phase 4)
3. Tune turn PID first (Phase 5): P → D → I
4. Then tune drive motion (Phase 6): velocity → acceleration → tolerance
5. Finally tune curved paths (Phase 7): BOOMERANG_LEAD
6. Change only one parameter at a time
7. Run 3 trials after each change to confirm consistency
```

---

> **Remember: Patience is the key virtue of tuning. Thoroughly passing each phase before moving to the next will save you enormous amounts of time on competition day.**
