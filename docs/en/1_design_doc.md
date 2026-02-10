# Design Document — VEX V5 Robot Software Architecture

---

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [Layer-by-Layer Design](#3-layer-by-layer-design)
   - 3.1 [Config](#31-config)
   - 3.2 [HAL — Hardware Abstraction Layer](#32-hal--hardware-abstraction-layer)
   - 3.3 [Localization — Odometry](#33-localization--odometry)
   - 3.4 [Control — PID & Motion Profile](#34-control--pid--motion-profile)
   - 3.5 [Motion — High-level Commands](#35-motion--high-level-commands)
   - 3.6 [Main — Application Entry](#36-main--application-entry)
4. [Data Flow](#4-data-flow)
5. [Key Algorithms](#5-key-algorithms)
6. [Future Roadmap](#6-future-roadmap)

---

## 1. Overview

This project is a modular software architecture for a VEX V5 competition robot. It is designed to be:

- **Simple to start** — works out of the box with a basic differential-drive robot
- **Easy to understand** — every file has one clear responsibility
- **Extensible** — ready to grow into a top-level competition codebase

The robot uses a **differential drive** (two powered wheels, one on each side). The software provides:

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

## 3. Layer-by-Layer Design

### 3.1 Config

**Files:** `include/config.h`

A single header file with all tunable constants. Every number that you might want to change (wheel size, motor ports, PID gains) lives here.

| Constant | What it means | Example Value |
|----------|---------------|---------------|
| `WHEEL_DIAMETER` | Wheel diameter in meters | 0.1016 (4") |
| `WHEEL_TRACK` | Distance between left and right wheels | 0.381 (15") |
| `TICKS_PER_REV` | Encoder ticks per wheel revolution | 360 |
| `WHEEL_CIRCUMFERENCE` | Derived: π × diameter | auto-calculated |
| `TURN_KP/KI/KD` | PID gains for turning | 2.0 / 0.0 / 0.1 |
| `DRIVE_KP/KI/KD` | PID gains for driving straight | 5.0 / 0.0 / 0.3 |
| `MAX_VELOCITY` | Top speed during autonomous (m/s) | 0.8 |
| `MAX_ACCELERATION` | How fast we speed up (m/s²) | 1.5 |
| `LOOP_INTERVAL_MS` | Control loop period | 10 ms |

**Why one file?** When you're tuning your robot at a competition, you want ONE place to change numbers, not hunting through 10 files.

---

### 3.2 HAL — Hardware Abstraction Layer

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

---

### 3.3 Localization — Odometry

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
   - `θ = α × θ_imu + (1-α) × θ_encoder` where α = 0.98 (trust IMU more)
6. **Update position** using midpoint approximation:
   - `x += d × cos(θ_mid)`
   - `y += d × sin(θ_mid)`

**Why IMU fusion?** Encoders drift when wheels slip. The IMU doesn't slip, but it drifts slowly over time. Combining both gives the best accuracy.

---

### 3.4 Control — PID & Motion Profile

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

### 3.5 Motion — High-level Commands

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

**Exit conditions (why both?):**
- **Settle time**: Ensures the robot has actually stopped, not just passing through the target
- **Timeout**: Safety net — if something goes wrong, don't spin forever

---

#### drive_to_pose

Drives the robot to a specific (x, y) coordinate on the field. Two-phase approach:

1. **Phase 1 — Turn**: Rotate in place to face the target point
2. **Phase 2 — Drive**: Move forward with motion-profiled velocity
   - Motion profile provides smooth acceleration/deceleration
   - Heading correction keeps the robot driving straight (compensates for drift)

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

---

### 3.6 Main — Application Entry

**File:** `src/main.cpp`

The entry point glues everything together:

1. **Hardware definitions** — creates motor, IMU, and brain objects
2. **`pre_auton()`** — calibrates IMU on startup
3. **`autonomous()`** — your 15-second autonomous routine (example L-path included)
4. **`usercontrol()`** — tank-drive with left/right joysticks
5. **`main()`** — registers callbacks with the VEX competition system, then runs the odometry background loop

**To customize:** Only edit `autonomous()` and `usercontrol()`. Everything else is infrastructure.

---

## 4. Data Flow

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

## 5. Key Algorithms

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

## 6. Future Roadmap

This codebase is designed to grow. Here are natural next steps, ordered by impact:

| Priority | Feature | Difficulty | Benefit |
|----------|---------|------------|---------|
| 1 | **PID tuning on the real robot** | Easy | Everything else depends on this |
| 2 | **Add mechanisms** (intake, lift, clamp) | Easy | Game-specific scoring |
| 3 | **Anti-windup for PID** | Medium | Prevents integral wind-up during saturated outputs |
| 4 | **Tracking wheels** for odometry | Medium | Much more accurate position tracking |
| 5 | **Pure pursuit** path following | Hard | Smooth curved paths instead of turn-then-drive |
| 6 | **S-curve motion profile** | Medium | Jerk-limited motion for less wheel slip |
| 7 | **Kalman filter** for localization | Hard | Optimal sensor fusion under noise |
| 8 | **Vision sensor integration** | Hard | Absolute position fixes, object detection |

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
