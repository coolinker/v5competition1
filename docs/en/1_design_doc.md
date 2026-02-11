# Design Document — VEX V5 Robot Software Architecture

---

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [Robot Configurations](#3-robot-configurations)
4. [Layer-by-Layer Design](#4-layer-by-layer-design)
   - 4.1 [Config](#41-config)
   - 4.2 [HAL — Hardware Abstraction Layer](#42-hal--hardware-abstraction-layer)
   - 4.3 [Localization — Odometry](#43-localization--odometry)
   - 4.4 [Control — PID & Motion Profile](#44-control--pid--motion-profile)
   - 4.5 [Motion — High-level Commands](#45-motion--high-level-commands)
   - 4.6 [Main — Application Entry](#46-main--application-entry)
5. [Data Flow](#5-data-flow)
6. [Key Algorithms](#6-key-algorithms)
7. [Architecture Strengths Demonstrated](#7-architecture-strengths-demonstrated)
8. [Future Roadmap](#8-future-roadmap)

---

## 1. Overview

This project is a modular software architecture for a VEX V5 competition robot. It is designed to be:

- **Simple to start** — works out of the box with a basic differential-drive robot
- **Easy to understand** — every file has one clear responsibility
- **Extensible** — ready to grow into a top-level competition codebase

The codebase supports **multiple robot configurations** — from a simple 2-motor starter bot to an advanced 6-motor competition drivetrain — all sharing the same control, localization, and motion code. Switch between them by changing one `#define` in `config.h`.

The software provides:

1. **Position tracking** (odometry) — "Where am I on the field?"
2. **PID control** — "How do I smoothly reach my target?"
3. **Motion profiling** — "How do I accelerate and decelerate smoothly?"
4. **Autonomous movement** — "Drive to this point" or "Turn to this heading"

---

## 2. System Architecture

The software is organized in **layers**, like a stack. Higher layers use lower layers, but never the reverse.

```
┌─────────────────────────────────────────────┐
│              main.cpp                       │  ← Application (top)
│         (competition callbacks)             │
├─────────────────────────────────────────────┤
│     motion/                                 │  ← High-level commands
│   turn_to_heading    drive_to_pose          │     "turn 90°", "drive to (1,0)"
├─────────────────────────────────────────────┤
│     control/                                │  ← Algorithms
│   PIDController      MotionProfile          │     PID math, velocity planning
├─────────────────────────────────────────────┤
│     localization/                           │  ← State estimation
│   odometry (Pose: x, y, θ)                 │     "Where am I?"
├─────────────────────────────────────────────┤
│     hal/                                    │  ← Hardware Abstraction
│   motors    imu    time                     │     Talks to real hardware
├─────────────────────────────────────────────┤
│     config.h                                │  ← Constants
│   (ports, dimensions, PID gains)            │     Single source of truth
└─────────────────────────────────────────────┘
```

**Why layers?**

- You can **test** upper layers without real hardware (mock the HAL)
- You can **swap hardware** (different motors, sensors) by only changing the HAL
- Each layer can be developed and debugged independently

---

## 3. Robot Configurations

The codebase ships with two ready-to-use drivetrain configurations.
Select one by uncommenting a single `#define` in `config.h`:

| | **ROBOT_2MOTOR** (entry-level) | **ROBOT_6MOTOR** (advanced) |
|---|---|---|
| **Motors** | 1 left + 1 right | 3 left + 3 right |
| **Cartridge** | Green 18:1 (200 RPM) | Blue 6:1 (600 RPM) |
| **Wheels** | 4" (0.1016 m) | 3.25" (0.08255 m) |
| **Top speed** | 0.8 m/s | 1.2 m/s |
| **Acceleration** | 1.5 m/s² | 3.0 m/s² |
| **Torque** | 1× baseline | 3× baseline |
| **Turn tolerance** | ~2° | ~1.4° |
| **Best for** | Learning, prototyping | Competition, pushing |

### What changes between configs?

```
  Files with config-specific code:     Files that stay IDENTICAL:
  ────────────────────────────         ──────────────────────────
  config.h          ← constants        control/pid.h & .cpp
  motors.cpp        ← HAL impl         control/motion_profile.h & .cpp
  main.cpp          ← hw objects       hal/motors.h   ← API unchanged!
  odometry.cpp      ← 6M delta fusion  hal/imu.h & .cpp
  drive_to_pose.cpp ← 6M Boomerang     hal/time.h & .cpp
  turn_to_heading.cpp ← 6M PID tuning  localization/odometry.h ← API unchanged!
                                        motion/drive_to_pose.h
                                        motion/turn_to_heading.h
```

**Config-specific code uses `#ifdef ROBOT_6MOTOR` blocks** — the 2-motor code path is preserved unchanged in `#else` branches. All header APIs remain identical.

### 6-Motor Wiring Diagram

```
  Port 1 [L-Front]  ──────  [R-Front]  Port 4
  Port 2 [L-Mid  ]  ──────  [R-Mid  ]  Port 5    ← encoder source
  Port 3 [L-Rear ]  ──────  [R-Rear ]  Port 6
                   IMU = Port 10
```

Middle motors are used for encoder readings — they have the most consistent ground contact and are least affected by turning scrub.

---

## 4. Layer-by-Layer Design

### 4.1 Config

**Files:** `include/config.h`

A single header file with all tunable constants, organized by robot configuration. The file uses compile-time `#ifdef` guards to select one configuration, with a build-time error if none or both are selected.

**Shared constants** (same for all configs):

| Constant | What it means |
|----------|---------------|
| `WHEEL_CIRCUMFERENCE` | Derived: π × diameter (auto-calculated) |
| `LOOP_INTERVAL_MS` | Control loop period (10 ms) |

**Per-configuration constants** (each config defines its own):

| Constant | 2-Motor Value | 6-Motor Value |
|----------|---------------|---------------|
| `WHEEL_DIAMETER` | 0.1016 m (4") | 0.08255 m (3.25") |
| `WHEEL_TRACK` | 0.381 m (15") | 0.330 m (13") |
| `TICKS_PER_REV` | 360 (green) | 300 (blue) |
| `TURN_KP/KI/KD` | 2.0 / 0.0 / 0.1 | 3.5 / 0.02 / 0.25 |
| `DRIVE_KP/KI/KD` | 5.0 / 0.0 / 0.3 | 8.0 / 0.05 / 0.5 |
| `MAX_VELOCITY` | 0.8 m/s | 1.2 m/s |
| `MAX_ACCELERATION` | 1.5 m/s² | 3.0 m/s² |
| `MOTORS_PER_SIDE` | 1 | 3 |
| `DRIVE_INTEGRAL_LIMIT` | — | 5.0 (anti-windup) |
| `DRIVE_D_FILTER` | — | 0.7 (derivative EMA) |
| `TURN_INTEGRAL_LIMIT` | — | 3.0 (anti-windup) |
| `TURN_D_FILTER` | — | 0.5 (derivative EMA) |
| `BOOMERANG_LEAD` | — | 0.6 (carrot lead factor) |

**Why one file?** When you're tuning your robot at a competition, you want ONE place to change numbers, not hunting through 10 files.

---

### 4.2 HAL — Hardware Abstraction Layer

**Files:**
- `include/hal/motors.h` + `src/hal/motors.cpp`
- `include/hal/imu.h` + `src/hal/imu.cpp`
- `include/hal/time.h` + `src/hal/time.cpp`

The HAL is a thin wrapper around VEX hardware. It provides simple functions that hide the VEX-specific API:

```
VEX API (complex)              HAL (simple)
─────────────────               ────────────
motor.spin(fwd, 8, volt)  →    set_drive_motors(8.0, 8.0)
motor.stop(brake)          →    stop_drive_motors()
motor.position(degrees)    →    get_left_encoder_ticks()
inertial.heading(degrees)  →    get_imu_heading_rad()
```

**Key functions:**

| Module | Function | What it does |
|--------|----------|-------------|
| motors | `set_drive_motors(left_v, right_v)` | Set left/right motor voltages (-12 to +12) |
| motors | `stop_drive_motors()` | Stop both motors with brake |
| motors | `get_left_encoder_ticks()` | Read left wheel encoder |
| motors | `get_right_encoder_ticks()` | Read right wheel encoder |
| motors | `reset_encoders()` | Zero both encoders |
| imu | `get_imu_heading_rad()` | Current heading in radians [0, 2π) |
| imu | `get_imu_rotation_rad()` | Unbounded rotation (can go past 2π) |
| imu | `calibrate_imu()` | Calibrate and wait until done |
| imu | `reset_imu()` | Zero the heading |
| time | `get_time_sec()` | Current time in seconds |
| time | `get_time_ms()` | Current time in milliseconds |
| time | `wait_ms(ms)` | Sleep for some milliseconds |

**Design decision:** Motor voltages are clamped to [-12, +12] inside the HAL, so upper layers don't need to worry about exceeding hardware limits.

**Multi-motor transparency:** In the 6-motor config, `set_drive_motors()` sends the same voltage to all 3 motors on a side. `get_left_encoder_ticks()` reads from the designated primary encoder (middle motor). The API signature is identical — callers never know how many physical motors exist.

---

### 4.3 Localization — Odometry

**Files:** `include/localization/odometry.h` + `src/localization/odometry.cpp`

Odometry answers the question: **"Where is my robot on the field?"**

It tracks the robot's **Pose** — a combination of position and heading:

```
Pose = { x, y, θ }
         ↑  ↑  ↑
         │  │  └── heading angle (radians, 0 = facing right / +X)
         │  └───── Y position (meters, forward from start)
         └──────── X position (meters, right from start)
```

**How it works (step by step):**

1. **Read encoders** — get the total ticks for left and right wheels
2. **Compute delta ticks** — subtract previous reading to get change since last update
3. **Convert to meters** — `delta_meters = delta_ticks × (wheel_circumference / ticks_per_rev)`
4. **Compute robot motion:**
   - Linear distance: `d = (d_left + d_right) / 2`
   - Heading change from encoders: `dθ_enc = (d_right - d_left) / wheel_track`
5. **Fuse with IMU** — blend encoder heading with IMU heading:
   - **2-motor** (absolute fusion): `θ = α × θ_imu + (1-α) × θ_encoder` where α = 0.98
   - **6-motor** (delta fusion): `Δθ = α × Δθ_imu + (1-α) × Δθ_encoder`
     Uses `get_imu_rotation_rad()` deltas instead of absolute `heading()` to avoid the catastrophic 0°/360° wrap-around bug that occurs near heading boundaries.
6. **Update position** using midpoint approximation:
   - `x += d × cos(θ_mid)`
   - `y += d × sin(θ_mid)`

**Why IMU fusion?** Encoders drift when wheels slip. The IMU doesn't slip, but it drifts slowly over time. Combining both gives the best accuracy.

**Why delta fusion for 6-motor?** The 2-motor absolute fusion (`α × heading + (1-α) × encoder`) works fine at low speeds but has a critical flaw: when heading crosses 0°/360° (≈ 0/2π radians), the weighted average produces nonsensical jumps (e.g., `0.98 × 0.01 + 0.02 × 6.27 ≈ 0.14` instead of the correct `≈ 0.01`). Delta fusion avoids this entirely by fusing heading *changes*, which are always small.

---

### 4.4 Control — PID & Motion Profile

**Files:**
- `include/control/pid.h` + `src/control/pid.cpp`
- `include/control/motion_profile.h` + `src/control/motion_profile.cpp`

#### PID Controller

A PID controller is like a smart thermostat. It continuously adjusts output to reach a target:

```
Error = Target - Current

Output = Kp × Error          ← Proportional: "How far off am I?"
       + Ki × ∫Error × dt    ← Integral:     "Have I been off for a long time?"
       + Kd × dError/dt      ← Derivative:   "Am I getting closer or further?"
```

**Analogy:** Imagine driving a car toward a traffic light:
- **P** (Proportional): The farther away, the harder you press the gas → might overshoot
- **I** (Integral): If you've been too far for too long, push harder → eliminates permanent offset
- **D** (Derivative): If you're approaching fast, start braking → reduces overshoot

**Optional enhancements** (6-motor config activates all three):

| Enhancement | Method | What it does |
|-------------|--------|-------------|
| **Anti-windup** | `set_integral_limit(limit)` | Clamps `∫error·dt` to ±limit, preventing integral explosion when motors are saturated (e.g., during pushing matches) |
| **Derivative EMA filter** | `set_d_filter(alpha)` | Low-pass filter: `filtered = α×prev + (1-α)×raw`. Smooths noisy derivative spikes without adding phase lag. alpha=0 disables |
| **Output clamping** | `set_output_limit(limit)` | Symmetric ±limit clamp on final output. Prevents commanding voltages beyond hardware capability |

All three are **disabled by default** (set to 0). The 2-motor config runs PID in basic mode for simplicity. The 6-motor motion code calls the setters after `reset()` to activate competition-grade behavior.

---

#### Motion Profile

Instead of going from 0 to max speed instantly (which causes wheel slip and jerky motion), we use a **trapezoidal velocity profile**:

```
velocity
▲
│    ┌────────────┐
│   /              \
│  /                \
│ /                  \
└──────────────────────── time
  accel   cruise  decel
```

At each moment, target velocity = min(acceleration_limit, max_velocity, deceleration_limit):

1. **Acceleration limit**: `v = a × t` — can't speed up too fast
2. **Max velocity cap**: `v = v_max` — can't go faster than top speed
3. **Deceleration limit**: `v = √(2 × a × d)` — must be able to stop in time

The smallest of these three constraints wins.

---

### 4.5 Motion — High-level Commands

**Files:**
- `include/motion/turn_to_heading.h` + `src/motion/turn_to_heading.cpp`
- `include/motion/drive_to_pose.h` + `src/motion/drive_to_pose.cpp`

#### turn_to_heading

Turns the robot in place to face a specific direction. Uses PID control on the heading error.

**Algorithm:**
1. Compute heading error = target − current
2. Normalize to [-π, +π] (always turn the short way)
3. PID → angular velocity → differential wheel voltages
4. Exit when error stays small for `TURN_SETTLE_TIME_MS`, or timeout

**6-motor enhancement:** After `reset()`, the turn PID activates anti-windup (`TURN_INTEGRAL_LIMIT`), derivative filter (`TURN_D_FILTER`), and output clamping (12V) for smoother, more precise turns with the higher-torque drivetrain.

**Exit conditions (why both?):**
- **Settle time**: Ensures the robot has actually stopped, not just passing through the target
- **Timeout**: Safety net — if something goes wrong, don't spin forever

---

#### drive_to_pose

Drives the robot to a specific (x, y, θ) pose. The strategy differs by configuration:

**2-motor — Turn-then-drive** (entry-level):
1. **Phase 1 — Turn**: Rotate in place to face the target point
2. **Phase 2 — Drive**: Move forward with motion-profiled velocity + heading correction

```
         Target (x, y)
            ●
           /
          /  ← Phase 2: drive forward
         /
        /
    ●──→    ← Phase 1: turn to face target
  Start
```

**6-motor — Boomerang controller** (competition-grade):
Instead of stopping to turn, a "carrot" point is placed behind the target along its heading vector. The robot aims at the carrot; as distance shrinks, the carrot converges to the target, producing a smooth curved approach that arrives at the correct final heading.

```
         carrot ◀── lead ──── Target
           ╱                     ↑ θ_final
        Robot
```

**Boomerang features:**
- Smooth curved approach — no intermediate stops
- Final heading control — arrives facing the desired direction
- Reverse driving support (`reverse=true` parameter)
- Cosine throttle — slows down when heading error is large
- Acceleration slew rate limiting — prevents wheel slip
- Full angular PID with anti-windup, D-filter, and output clamping

---

### 4.6 Main — Application Entry

**File:** `src/main.cpp`

The entry point glues everything together. It uses `#ifdef` to create the correct motor objects for the selected configuration:

- **2-motor**: 2 motors → `LeftDriveSmart`, `RightDriveSmart`
- **6-motor**: 6 motors → `LeftFront/Mid/Rear`, `RightFront/Mid/Rear` (blue cartridge, left side reversed)

Note: motor `reverse` flags in the 6-motor config account for physical mounting direction — left-side motors face opposite to right-side motors.

---

## 5. Data Flow

Here's how data flows through the system during autonomous:

```
Encoders + IMU
      │
      ▼
  ┌────────────┐
  │  Odometry   │──→ Pose {x, y, θ}
  └────────────┘         │
                         ▼
              ┌──────────────────┐
              │  drive_to_pose   │
              │  or              │
              │  turn_to_heading │
              └──────────────────┘
                    │         │
          ┌─────────┘         └──────────┐
          ▼                              ▼
   ┌──────────────┐              ┌──────────────┐
   │MotionProfile │              │     PID      │
   │(target vel)  │              │(correction)  │
   └──────────────┘              └──────────────┘
          │                              │
          └──────────┬───────────────────┘
                     ▼
              ┌──────────────┐
              │  HAL Motors  │──→ Wheel voltages
              └──────────────┘
```

---

## 6. Key Algorithms

### Angle Normalization

When computing the difference between two angles, the result must be in [-π, +π] so the robot always turns the shortest way:

```cpp
error = atan2(sin(error), cos(error));
```

Without this, turning from 350° to 10° would go the long way (340° turn) instead of the short way (20° turn).

---

### Differential Drive Kinematics

Converting between wheel velocities and robot motion:

```
Forward:                    Inverse:
v = (v_R + v_L) / 2        v_L = v - ω × W/2
ω = (v_R - v_L) / W        v_R = v + ω × W/2

Where:
  v   = linear velocity (m/s)
  ω   = angular velocity (rad/s)
  W   = wheel track width (m)
  v_L = left wheel velocity
  v_R = right wheel velocity
```

---

## 7. Architecture Strengths Demonstrated

The 6-motor configuration proves the architecture works as designed:

### Flexibility (adapt to different hardware)
- Switching from 2 motors to 6 motors requires changing **config-specific `#ifdef` blocks** in a handful of files
- A single `#define` in `config.h` selects the entire configuration
- Compile-time guards prevent invalid combinations
- Config-specific behaviors (Boomerang, delta fusion) are isolated in `#ifdef`/`#else` blocks — the 2-motor code path is always preserved unchanged

### Reusability (same code, different robots)
- `PIDController` class works identically for both configs — only the *gains* and optional enhancements differ
- `MotionProfile` works unchanged — only `MAX_VELOCITY` and `MAX_ACCELERATION` differ
- HAL APIs (`get_left_encoder_ticks()`, `set_drive_motors()`) hide all multi-motor complexity
- Odometry, drive_to_pose, and turn_to_heading share the same API — config-specific behavior is selected at compile time

### Extendability (easy to add more)
- Adding an 8-motor config (e.g., mecanum) follows the same pattern:
  1. Add `#define ROBOT_8MOTOR` section in `config.h`
  2. Add motor control logic in `motors.cpp`
  3. Add motor declarations in `main.cpp`
  4. All other files remain untouched
- Adding subsystems (intake, catapult, pneumatics) follows the same HAL pattern

### Separation of Concerns
```
  What changes           Where it lives         What DOESN'T change
  ─────────────          ──────────────         ─────────────────────
  Number of motors   →   motors.cpp              PID algorithm
  Motor ports        →   config.h                Odometry math
  Wheel dimensions   →   config.h                Motion profiling
  PID gains          →   config.h                Drive-to-pose logic
  Cartridge type     →   main.cpp                Turn-to-heading logic
```

---

## 8. Future Roadmap

This codebase is designed to grow. Here are natural next steps, ordered by impact:

| Priority | Feature | Difficulty | Status |
|----------|---------|------------|--------|
| 1 | **PID tuning on the real robot** | Easy | Ongoing — tune on field |
| 2 | **Add mechanisms** (intake, lift, clamp) | Easy | Next step |
| ~~3~~ | ~~**Anti-windup for PID**~~ | ~~Medium~~ | ✅ Done — `set_integral_limit()` |
| ~~3b~~ | ~~**Derivative filter for PID**~~ | ~~Easy~~ | ✅ Done — `set_d_filter()` |
| ~~3c~~ | ~~**Output clamping for PID**~~ | ~~Easy~~ | ✅ Done — `set_output_limit()` |
| ~~4~~ | ~~**IMU heading wrap-around fix**~~ | ~~Medium~~ | ✅ Done — delta-based IMU fusion (6M) |
| ~~5~~ | ~~**Curved path following**~~ | ~~Hard~~ | ✅ Done — Boomerang controller (6M) |
| 6 | **Tracking wheels** for odometry | Medium | Future — more accurate positioning |
| 7 | **S-curve motion profile** | Medium | Future — jerk-limited motion |
| 8 | **Kalman filter** for localization | Hard | Future — optimal sensor fusion |
| 9 | **Vision sensor integration** | Hard | Future — absolute position fixes |

---

## File Map

```
v5competition1/
├── include/
│   ├── config.h                    ← All tunable constants
│   ├── vex.h                       ← VEX SDK header
│   ├── hal/
│   │   ├── motors.h                ← Motor functions
│   │   ├── imu.h                   ← IMU functions
│   │   └── time.h                  ← Time functions
│   ├── localization/
│   │   └── odometry.h              ← Position tracking
│   ├── control/
│   │   ├── pid.h                   ← PID controller
│   │   └── motion_profile.h        ← Velocity planner
│   └── motion/
│       ├── turn_to_heading.h       ← Turn command
│       └── drive_to_pose.h         ← Drive command
├── src/
│   ├── main.cpp                    ← Entry point
│   ├── hal/     (*.cpp)            ← HAL implementations
│   ├── localization/ (*.cpp)       ← Odometry implementation
│   ├── control/ (*.cpp)            ← Algorithm implementations
│   └── motion/  (*.cpp)            ← Command implementations
├── test/
│   └── host_tests.cpp              ← Host-side unit tests
└── docs/
    ├── en/                         ← English documentation
    └── cn/                         ← Chinese documentation
```
