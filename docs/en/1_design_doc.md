# Design Document — VEX V5 Robot Software Architecture

---

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [Robot Hardware Configuration](#3-robot-hardware-configuration)
4. [Layer-by-Layer Design](#4-layer-by-layer-design)
   - 4.1 [Configuration Layer](#41-configuration-layer)
   - 4.2 [Hardware Abstraction Layer (HAL)](#42-hardware-abstraction-layer-hal)
   - 4.3 [Localization — Odometry](#43-localization--odometry)
   - 4.4 [Control — PID & Motion Profile](#44-control--pid--motion-profile)
   - 4.5 [Motion — High-Level Commands](#45-motion--high-level-commands)
   - 4.6 [Main Program Entry](#46-main-program-entry)
5. [Data Flow](#5-data-flow)
6. [Core Algorithms](#6-core-algorithms)
7. [Architecture Strengths](#7-architecture-strengths)
8. [Future Roadmap](#8-future-roadmap)

---

## 1. Overview

This project is a modular software architecture for a VEX V5 competition robot. Design goals:

- **Easy to understand** — each file does one thing
- **Extensible** — features can be added incrementally
- **Competition-ready** — high-precision tracking wheel odometry + vision correction

The software provides:

1. **Position tracking** (odometry) — "Where am I on the field?" (perpendicular dual tracking wheels + IMU)
2. **Vision localization** — AI Vision detects AprilTag markers to correct accumulated drift
3. **PID control** — "How do I smoothly reach my target?"
4. **Motion profiling** — "How do I accelerate and decelerate smoothly?"
5. **Autonomous motion** — "Drive to this point" or "Turn to this heading"

---

## 2. System Architecture

The software is organized in **layers**, like a stack of building blocks. Upper layers use lower layers, but lower layers never depend on upper ones.

```
┌─────────────────────────────────────────────┐
│              main.cpp                       │  ← Application layer (top)
│          (competition callbacks)            │
├─────────────────────────────────────────────┤
│     motion/                                 │  ← Motion command layer
│   turn_to_heading    drive_to_pose          │     "Turn 90°", "Drive to (1,0)"
├─────────────────────────────────────────────┤
│     control/                                │  ← Algorithm layer
│   PIDController      MotionProfile          │     PID math, velocity planning
├─────────────────────────────────────────────┤
│     localization/                           │  ← State estimation layer
│   odometry (Pose: x, y, θ)                 │     "Where am I?" (tracking + IMU)
│   vision_localizer (AprilTag → abs. pos.)   │     Vision correction
├─────────────────────────────────────────────┤
│     hal/                                    │  ← Hardware abstraction layer
│   motors  imu  time  tracking_wheels        │     Talk to real hardware
│   vision  hal_log                           │
├─────────────────────────────────────────────┤
│     config.h                                │  ← Constants
│   (ports, dimensions, PID gains, offsets)   │     Single source of truth
└─────────────────────────────────────────────┘
```

**Why layers?**

- Test upper logic **without hardware** (mock HAL)
- **Swap hardware** by changing only HAL (different motors, sensors)
- Each layer can be developed and debugged independently

---

## 3. Robot Hardware Configuration

This project targets a **6-motor differential drivetrain** + **perpendicular dual tracking wheels** + **IMU** + **AI Vision**.

### Hardware Bill of Materials

| Device | Qty | Port | Notes |
|--------|-----|------|-------|
| V5 Motor (Blue 600RPM) | 6 | Port 1-6 | 3 left + 3 right |
| V5 Inertial Sensor (IMU) | 1 | Port 10 | Mounted level at chassis center |
| V5 Rotation Sensor (forward tracking) | 1 | Port 8 | 2.75" omni wheel, parallel to drive direction |
| V5 Rotation Sensor (lateral tracking) | 1 | Port 9 | 2.75" omni wheel, perpendicular to drive direction |
| V5 AI Vision Sensor | 1 | Port 12 | Front-facing |

### Wiring Diagram

```
  Port 1 [Left Front]  ──────  [Right Front] Port 4
  Port 2 [Left Mid]    ──────  [Right Mid]   Port 5
  Port 3 [Left Rear]   ──────  [Right Rear]  Port 6

  Port 10 = IMU (Inertial Sensor)
  Port 8  = Forward Tracking Wheel (Rotation Sensor)
  Port 9  = Lateral Tracking Wheel (Rotation Sensor)
  Port 12 = AI Vision Sensor
```

### Tracking Wheel Layout (Perpendicular Dual-Wheel Scheme)

```
         ↑ Forward
         |
  [Fwd wheel(↑)]    ← Measures forward/backward displacement
         |
—[Lat wheel(→)]—    ← Measures left/right sliding
         |
```

The two tracking wheels are mounted in a cross pattern. Rotation angle comes entirely from the IMU.
This scheme is superior to parallel dual wheels because it **detects lateral sliding** (accurate tracking even when pushed sideways by opponents).

---

## 4. Layer-by-Layer Design

### 4.1 Configuration Layer

**File:** `include/config.h`

A single header containing all tunable parameters, centralized for quick field-side adjustment.

**Parameter Groups:**

| Group | Key Constants | Purpose |
|-------|---------------|---------|
| 1. Physical | `WHEEL_DIAMETER` (0.08255 m), `WHEEL_TRACK` (0.33 m), `TICKS_PER_REV` (300) | Drive wheel dimensions |
| 2. Motor Ports | `LEFT_FRONT_MOTOR_PORT` … `RIGHT_REAR_MOTOR_PORT`, `MOTORS_PER_SIDE` (3) | 6-motor assignment |
| 3. IMU | `IMU_PORT` (10), `IMU_FUSION_ALPHA` (0.98, reserved) | Inertial sensor |
| 4. Tracking Wheels | `FORWARD/LATERAL_TRACKING_PORT`, `TRACKING_WHEEL_DIAMETER` (0.06985 m), `FORWARD/LATERAL_WHEEL_OFFSET` | Perpendicular dual-wheel scheme |
| 5. Turn PID | `TURN_KP` (3.5) / `KI` (0.02) / `KD` (0.25), anti-windup, D-filter | In-place turning |
| 6. Drive | `DRIVE_KP` (8.0) / `KI` (0.05) / `KD` (0.5), tolerance, timeout | Linear driving |
| 7. Motion Profile | `MAX_VELOCITY` (1.2 m/s), `MAX_ACCELERATION` (3.0 m/s²), `BOOMERANG_LEAD` (0.6) | Velocity curve + Boomerang |
| 8. Loop Interval | `LOOP_INTERVAL_MS` (10) | 100 Hz |
| 9. Logging | `LOG_VERBOSITY` (2) | Debug output level |
| 10. Vision | `VISION_PORT` (11), focal length, tag size, confidence threshold, correction gain | AprilTag localization |

**Why one file?** At competition, you only need to visit one place to change numbers — no searching through 10 files.

---

### 4.2 Hardware Abstraction Layer (HAL)

**Files:**
- `include/hal/motors.h` + `src/hal/motors.cpp` — Motor control
- `include/hal/imu.h` + `src/hal/imu.cpp` — Inertial sensor
- `include/hal/time.h` + `src/hal/time.cpp` — Time functions
- `include/hal/tracking_wheels.h` + `src/hal/tracking_wheels.cpp` — Tracking wheel sensors
- `include/hal/vision.h` + `src/hal/vision.cpp` — AI Vision sensor
- `include/hal/hal_log.h` + `src/hal/hal_log.cpp` — Logging system

The HAL is a thin wrapper around VEX hardware. It provides simple functions that hide VEX-specific APIs:

```
VEX API (complex)              HAL (simple)
──────────────                 ────────────
motor.spin(fwd, 8, volt)  →   set_drive_motors(8.0, 8.0)
motor.stop(brake)          →   stop_drive_motors()
inertial.heading(degrees)  →   get_imu_heading_rad()
rotation.position(degrees) →   tracking_get_forward_distance_m()
```

**Core Functions:**

| Module | Function | Purpose |
|--------|----------|---------|
| motors | `set_drive_motors(left_v, right_v)` | Set left/right motor voltage (-12 to +12) |
| motors | `stop_drive_motors()` | Brake-stop all motors |
| motors | `get_left_encoder_ticks()` | Read left wheel encoder |
| motors | `get_right_encoder_ticks()` | Read right wheel encoder |
| motors | `reset_encoders()` | Zero encoders |
| imu | `get_imu_heading_rad()` | Current heading (radians) [0, 2π) |
| imu | `get_imu_rotation_rad()` | Unbounded rotation (can exceed 2π) |
| imu | `calibrate_imu()` | Calibrate IMU and wait |
| imu | `reset_imu()` | Zero heading |
| tracking | `tracking_get_forward_distance_m()` | Forward tracking wheel cumulative distance (meters) |
| tracking | `tracking_get_lateral_distance_m()` | Lateral tracking wheel cumulative distance (meters) |
| tracking | `tracking_wheels_reset()` | Zero tracking wheel readings |
| tracking | `tracking_wheels_connected()` | Check if tracking wheels are connected |
| time | `get_time_sec()` / `get_time_ms()` | Current time |
| time | `wait_ms(ms)` | Sleep for milliseconds |
| vision | `vision_detect_apriltags()` | Detect AprilTag markers |
| log | `hal_log(msg)` | Output log to terminal and SD card |

**Design Decisions:**
- Motor voltage is clamped to [-12, +12] inside the HAL — callers don't worry about exceeding hardware limits
- `set_drive_motors()` sends the same voltage to all 3 motors on one side (caller doesn't need to know motor count)
- Tracking wheel functions automatically convert rotation sensor angles to meters (using `TRACKING_WHEEL_CIRCUMFERENCE`)

---

### 4.3 Localization — Odometry

**Files:** `include/localization/odometry.h` + `src/localization/odometry.cpp`

Odometry answers: **"Where is my robot on the field?"**

It tracks the robot's **Pose** — a combination of position and heading:

```
Pose = { x, y, θ }
         ↑  ↑  ↑
         │  │  └── Heading angle (radians, 0 = facing right/+X)
         │  └───── Y position (meters, left is positive)
         └──────── X position (meters, forward is positive)
```

**How It Works (Perpendicular Dual-Wheel Scheme, Step by Step):**

1. **Read sensors** — get forward wheel distance, lateral wheel distance, and IMU rotation
2. **Compute deltas** — subtract previous readings to get changes since last update:
   - `Δforward` (how far the forward wheel moved)
   - `Δlateral` (how far the lateral wheel slid)
   - `Δθ` (how much the IMU measured rotation, **100% from IMU**)
3. **Compensate rotation arcs** — tracking wheels offset from the rotation center trace arcs during rotation, not true translation:
   - `Δforward_corrected = Δforward - FORWARD_WHEEL_OFFSET × Δθ`
   - `Δlateral_corrected = Δlateral - LATERAL_WHEEL_OFFSET × Δθ`
4. **Transform to global coordinates** using midpoint approximation:
   - `mid_θ = θ + Δθ/2`
   - `x += Δforward × cos(mid_θ) - Δlateral × sin(mid_θ)`
   - `y += Δforward × sin(mid_θ) + Δlateral × cos(mid_θ)`
   - `θ += Δθ`

**Why tracking wheels instead of drive wheel encoders?** Drive wheels slip during acceleration, turning, and collisions — encoder readings become inaccurate. Tracking wheels are unpowered, don't slip, and measure more precisely.

**Why 100% IMU for heading?** The perpendicular scheme has one forward wheel and one lateral wheel — it cannot compute rotation from wheel difference like parallel dual wheels. The IMU is extremely accurate short-term and the best heading source.

#### Vision Localization — AI Vision Sensor Absolute Position Correction

**Files:**
- `include/hal/vision.h` + `src/hal/vision.cpp`
- `include/localization/vision_localizer.h` + `src/localization/vision_localizer.cpp`

The vision system uses the **V5 AI Vision Sensor** to detect **AprilTag** markers at known field positions, providing **absolute position correction** that complements the tracking wheels' relative positioning.

**Tracking Wheels vs AI Vision (complementary — we use both):**

| | Tracking Wheels | AI Vision |
|---|---|---|
| Positioning type | Relative (accumulates drift) | **Absolute (resets drift)** |
| Update rate | 100 Hz (very fast) | 20 Hz (slower) |
| Accuracy source | Wheel-to-ground contact quality | Tag visibility + distance |
| Collision resilience | Collision may shift wheels | **No physical alignment needed** |

**How It Works:**

```
AprilTag on field (known position)
        │
        ▼
   ┌────────────────┐
   │  AI Vision     │  ← Detect tag → pixel coordinates + size
   │  Sensor (HAL)  │
   └────────────────┘
        │
        ▼
   ┌────────────────┐
   │  Vision        │  ← Pinhole camera model → distance + bearing
   │  Localizer     │  ← Known tag position → robot absolute coords
   └────────────────┘
        │
        ▼
   Complementary filter fusion into odometry
   pose = (1-α) × odom + α×confidence × vision
```

**Step-by-step algorithm:**

1. **Capture snapshot** — AI Vision Sensor detects AprilTags in field of view
2. **Distance estimation** — pinhole camera model: `distance = (actual_tag_size × focal_length) / pixel_size`
3. **Bearing estimation** — from horizontal pixel offset: `bearing = atan(pixel_offset / focal_length)`
4. **Coordinate transform** — known tag field coords + distance + bearing → robot field coords
5. **Confidence scoring** — based on distance and tag size: close + large tag → high confidence
6. **Complementary filter fusion** — weighted blend into odometry estimate by confidence

**Safety mechanisms:**
- Detections below `VISION_MIN_CONFIDENCE` are discarded
- Corrections exceeding `VISION_MAX_CORRECTION_M` (30 cm) are rejected (likely misdetection)
- Heading always comes from IMU (more reliable than vision)
- Uses `set_pose_no_reset()` to adjust pose without breaking encoder delta tracking

**Example field tag layout (12 ft × 12 ft field):**

```
  ┌───────────────────────────────────┐
  │              +y wall              │
  │  Tag7                      Tag8  │
  │                                  │
  │  Tag3                      Tag4  │
  │ +x wall                  -x wall │
  │  Tag1                      Tag2  │
  │                                  │
  │  Tag5                      Tag6  │
  │              -y wall              │
  └───────────────────────────────────┘
     Origin (0, 0) = bottom-left
```

---

### 4.4 Control — PID & Motion Profile

**Files:**
- `include/control/pid.h` + `src/control/pid.cpp`
- `include/control/motion_profile.h` + `src/control/motion_profile.cpp`

#### PID Controller

A PID controller is like a smart thermostat, continuously adjusting output to reach a target:

```
error = target - current

output = Kp × error              ← Proportional: "How far off am I?"
       + Ki × ∫error × dt        ← Integral: "How long have I been off?"
       + Kd × d(error)/dt        ← Derivative: "Am I getting closer or further?"
```

**Analogy:** Imagine driving toward a traffic light:
- **P** (Proportional): The further away, the harder you press the gas → might overshoot
- **I** (Integral): If you've been off-target for a while, push harder → eliminates steady-state error
- **D** (Derivative): If approaching too fast, start braking → reduces overshoot

**Enhancements:**

| Enhancement | Method | Purpose |
|-------------|--------|---------|
| **Anti-windup** | `set_integral_limit(limit)` | Clamp integral to ±limit, preventing integral explosion during motor saturation |
| **Derivative EMA filter** | `set_d_filter(alpha)` | Low-pass: `filtered = α×prev + (1-α)×raw`. Smooths derivative noise spikes. alpha=0 disables |
| **Output clamping** | `set_output_limit(limit)` | Symmetric ±limit output clamp. Prevents commands exceeding hardware capability |

Motion code calls setters after `reset()` to activate competition-grade behavior (turn PID uses anti-windup 3.0, D-filter 0.5, output clamp 12V).

---

#### Motion Profile

Instead of jumping instantly from 0 to max speed (causing wheel slip and jerky motion), we use a **trapezoidal velocity profile**:

```
Velocity
▲
│    ┌────────────┐
│   /              \
│  /                \
│ /                  \
└──────────────────────── Time
  Accel    Cruise    Decel
```

At each moment, target velocity = min(acceleration limit, max velocity, deceleration limit):

1. **Acceleration limit**: `v = a × t` — can't accelerate too fast
2. **Max velocity cap**: `v = v_max` — can't exceed top speed
3. **Deceleration limit**: `v = √(2 × a × d)` — must be able to stop in time

The smallest of the three constraints wins.

---

### 4.5 Motion — High-Level Commands

**Files:**
- `include/motion/turn_to_heading.h` + `src/motion/turn_to_heading.cpp`
- `include/motion/drive_to_pose.h` + `src/motion/drive_to_pose.cpp`

#### In-Place Turn (turn_to_heading)

Rotates the robot in place to a specified heading. Uses PID to control heading error.

**Algorithm:**
1. Compute heading error = target − current
2. Normalize to [-π, +π] (always take the short way)
3. PID → angular velocity → differential wheel voltage
4. Exit when error stays small for `TURN_SETTLE_TIME_MS`, or on timeout

Turn PID activates anti-windup (`TURN_INTEGRAL_LIMIT`), derivative filter (`TURN_D_FILTER`), and output clamping (12V) for smoother, more precise turning on high-torque drivetrains.

**Exit conditions (why two?):**
- **Settle time**: Ensures the robot actually stopped, not just passing through the target
- **Timeout**: Safety net — don't spin forever if something goes wrong

---

#### Drive to Pose (drive_to_pose) — Boomerang Controller

Drives the robot to a specified (x, y, θ) pose on the field. Uses the **Boomerang controller** for smooth curved approach.

Instead of stopping to turn, it places a "carrot" lead point behind the target along its heading vector. The robot aims at the carrot; as distance shrinks, the carrot converges to the target, producing a smooth curved approach with correct final heading.

```
         Carrot ◀── lead ──── Target
           ╱                     ↑ θ_final
        Robot
```

**Boomerang features:**
- Smooth curved approach — no intermediate stops
- Final heading control — arrive facing the desired direction
- Reverse driving support (`reverse=true` parameter)
- Cosine scaling — slows down when heading error is large
- Slew rate limiting — prevents wheel slip
- Full heading PID with anti-windup, D-filter, and output clamping

---

### 4.6 Main Program Entry

**File:** `src/main.cpp`

The entry program glues everything together:

- **Hardware declarations**:
  - 6 drive motors → `LeftFront/Mid/Rear`, `RightFront/Mid/Rear` (blue gearbox, left side reversed)
  - IMU → `DrivetrainInertial`
  - Tracking wheels → `ForwardTrackingSensor`, `LateralTrackingSensor`
  - AI Vision → `VisionSensor`
- **Background tasks**: screen display, logging
- **Competition callbacks**: `pre_auton()`, `autonomous()`, `usercontrol()`

Note: Left-side motors have the `reverse` flag set due to physical mounting orientation — left motors face opposite to right motors.

---

## 5. Data Flow

Here is how data flows through the system during autonomous mode:

```
Tracking Wheels + IMU            AI Vision Sensor
      │                                │
      ▼                                ▼
  ┌────────────┐            ┌──────────────────┐
  │  Odometry  │──→ Pose ←──│  Vision          │
  │ (100 Hz)   │   {x,y,θ}  │  Localizer       │
  └────────────┘      │      │  (20 Hz, abs.)   │
                      │      └──────────────────┘
                      │         Complementary ↑
                      │         filter fusion
                      ▼
              ┌──────────────────┐
              │   drive_to_pose  │
              │   or             │
              │  turn_to_heading │
              └──────────────────┘
                    │         │
          ┌─────────┘         └──────────┐
          ▼                              ▼
   ┌──────────────┐              ┌──────────────┐
   │ Motion       │              │     PID      │
   │ Profile      │              │ (correction) │
   │ (target vel) │              │              │
   └──────────────┘              └──────────────┘
          │                              │
          └──────────┬───────────────────┘
                     ▼
              ┌──────────────┐
              │  HAL Motors  │──→ Wheel voltage
              └──────────────┘
```

---

## 6. Core Algorithms

### Angle Normalization

When computing the difference between two angles, the result must be in [-π, +π] to ensure the robot always takes the shortest path:

```cpp
error = atan2(sin(error), cos(error));
```

Without this, turning from 350° to 10° would go the long way (340°) instead of the short way (20°).

---

### Perpendicular Dual-Wheel Odometry

Unlike traditional differential odometry (which computes rotation from left/right wheel difference), the perpendicular scheme directly measures displacement along two independent axes:

```
Inputs:
  Δfwd  = forward wheel displacement delta
  Δlat  = lateral wheel displacement delta
  Δθ    = IMU rotation delta

Compensate rotation arcs (off-center wheels trace arcs):
  Δfwd_c = Δfwd - FORWARD_WHEEL_OFFSET × Δθ
  Δlat_c = Δlat - LATERAL_WHEEL_OFFSET × Δθ

Global coordinate update (midpoint approximation):
  mid_θ = θ + Δθ/2
  x += Δfwd_c × cos(mid_θ) - Δlat_c × sin(mid_θ)
  y += Δfwd_c × sin(mid_θ) + Δlat_c × cos(mid_θ)
  θ += Δθ
```

---

### Differential Drive Kinematics

Conversion between wheel speeds and robot motion:

```
Forward:                      Inverse:
v = (v_right + v_left) / 2    v_left  = v - ω × W/2
ω = (v_right - v_left) / W    v_right = v + ω × W/2

Where:
  v      = linear velocity (m/s)
  ω      = angular velocity (rad/s)
  W      = wheel track (m)
  v_left = left wheel speed
  v_right = right wheel speed
```

---

## 7. Architecture Strengths

### Modularity (one responsibility per file)
- HAL encapsulates hardware details — swap sensors by changing HAL only, upper code untouched
- PID and motion profile are generic algorithms — no dependency on specific robot configuration
- Odometry only cares about "where is the pose", not "who reads the sensors"

### Testability (test without hardware)
- 24 host-side unit tests covering PID, motion profile, odometry
- Mock HAL simulates all sensors — tests run on your computer in 0.1 seconds
- Run `make test` after every code change for instant verification

### Separation of Concerns
```
  What changes           Where              What stays the same
  ─────────────        ──────────────      ───────────────────
  Motor ports      →   config.h            PID algorithm
  Wheel size       →   config.h            Odometry math
  PID gains        →   config.h            Motion profiling
  Tracking offsets →   config.h            Drive-to-pose logic
  Sensor type      →   hal/*.cpp           Turn-to-heading logic
```

### Easy to Extend
- Adding subsystems (intake, lift, pneumatics) follows the same HAL pattern
- Adding new localization sources (e.g., GPS) just means implementing a new localizer module and fusing into odometry

---

## 8. Future Roadmap

| Priority | Feature | Difficulty | Status |
|----------|---------|------------|--------|
| 1 | **Tune PID on real robot** | Easy | Ongoing |
| 2 | **Add mechanisms** (intake, lift, clamp) | Easy | Next |
| ~~3~~ | ~~**PID anti-windup**~~ | ~~Medium~~ | ✅ Done — `set_integral_limit()` |
| ~~3b~~ | ~~**PID derivative filter**~~ | ~~Easy~~ | ✅ Done — `set_d_filter()` |
| ~~3c~~ | ~~**PID output clamping**~~ | ~~Easy~~ | ✅ Done — `set_output_limit()` |
| ~~4~~ | ~~**IMU heading wraparound fix**~~ | ~~Medium~~ | ✅ Done — delta-based IMU |
| ~~5~~ | ~~**Curved path following**~~ | ~~Hard~~ | ✅ Done — Boomerang controller |
| ~~6~~ | ~~**Tracking wheel localization**~~ | ~~Medium~~ | ✅ Done — perpendicular dual-wheel (forward + lateral) |
| ~~7~~ | ~~**Absolute position localization**~~ | ~~Hard~~ | ✅ Done — AI Vision AprilTag localization |
| 8 | **S-curve motion profile** | Medium | Future — jerk limiting |
| 9 | **Multi-tag triangulation** | Medium | Future — simultaneous multi-tag detection |
| 10 | **Vision heading correction** | Medium | Future — estimate heading from tag angle |
| 11 | **Kalman filter** localization | Hard | Future — optimal multi-sensor fusion |

---

## File Map

```
v5competition1/
├── include/
│   ├── config.h                    ← All tunable parameters
│   ├── vex.h                       ← VEX SDK header
│   ├── hal/
│   │   ├── motors.h                ← Motor functions
│   │   ├── imu.h                   ← IMU functions
│   │   ├── time.h                  ← Time functions
│   │   ├── tracking_wheels.h       ← Tracking wheel functions
│   │   ├── vision.h                ← AI Vision sensor functions
│   │   └── hal_log.h               ← Logging system
│   ├── localization/
│   │   ├── odometry.h              ← Position tracking (tracking wheels + IMU)
│   │   └── vision_localizer.h      ← Vision absolute localization
│   ├── control/
│   │   ├── pid.h                   ← PID controller
│   │   └── motion_profile.h        ← Velocity planning
│   └── motion/
│       ├── turn_to_heading.h       ← Turn command
│       └── drive_to_pose.h         ← Drive command (Boomerang controller)
├── src/
│   ├── main.cpp                    ← Entry program
│   ├── hal/     (*.cpp)            ← HAL implementations
│   ├── localization/ (*.cpp)       ← Odometry + vision localization
│   ├── control/ (*.cpp)            ← Algorithm implementations
│   └── motion/  (*.cpp)            ← Motion command implementations
├── test/
│   └── host_tests.cpp              ← Host-side unit tests (24 tests)
└── docs/
    ├── en/                         ← English documentation
    └── cn/                         ← Chinese documentation
```
