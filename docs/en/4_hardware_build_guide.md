# Hardware Build & Deployment Guide — VEX V5 Differential Drive Robot

**Complete bill of materials, step-by-step mechanical assembly, and deployment instructions**

---

## Table of Contents

1. [Bill of Materials (BOM)](#1-bill-of-materials-bom)
   - 1.1 [Electronics](#11-electronics)
   - 1.2 [Structural — C-Channels & Beams](#12-structural--c-channels--beams)
   - 1.3 [Drivetrain — Wheels, Shafts & Gears](#13-drivetrain--wheels-shafts--gears)
   - 1.4 [Fasteners — Screws & Nuts](#14-fasteners--screws--nuts)
   - 1.5 [Spacers & Shaft Collars](#15-spacers--shaft-collars)
   - 1.6 [Bearings](#16-bearings)
   - 1.7 [Support Wheel](#17-support-wheel)
   - 1.8 [Cable Management](#18-cable-management)
   - 1.9 [Tools Required](#19-tools-required)
2. [Chassis Dimensions & Layout](#2-chassis-dimensions--layout)
3. [Gear Ratio & Shaft Spacing](#3-gear-ratio--shaft-spacing)
4. [Assembly — Step by Step](#4-assembly--step-by-step)
   - 4.1 [Step 1: Build the Base Frame](#41-step-1-build-the-base-frame)
   - 4.2 [Step 2: Install Drive Shafts & Wheels](#42-step-2-install-drive-shafts--wheels)
   - 4.3 [Step 3: Mount Motors & Gear Train](#43-step-3-mount-motors--gear-train)
   - 4.4 [Step 4: Mount Support Wheel](#44-step-4-mount-support-wheel)
   - 4.5 [Step 5: Mount the Brain & Battery](#45-step-5-mount-the-brain--battery)
   - 4.6 [Step 6: Mount the IMU](#46-step-6-mount-the-imu)
   - 4.7 [Step 7: Wiring & Cable Management](#47-step-7-wiring--cable-management)
5. [Software Configuration](#5-software-configuration)
6. [Deploying to VEX V5 Brain](#6-deploying-to-vex-v5-brain)
7. [First-Boot Verification (6 Checks)](#7-first-boot-verification-6-checks)
8. [Troubleshooting](#8-troubleshooting)
9. [Appendix: Part Number Quick Reference](#9-appendix-part-number-quick-reference)

---

## 1. Bill of Materials (BOM)

### 1.1 Electronics

| # | Part | VEX Part Number | Qty | Notes |
|---|------|----------------|-----|-------|
| 1 | V5 Robot Brain | 276-4810 | 1 | Main controller |
| 2 | V5 Robot Battery | 276-4811 | 1 | Li-Ion, charge to >50% before use |
| 3 | V5 Smart Motor (11W) | 276-4840 | 2 | **Green cartridge (18:1, 200 RPM)** pre-installed |
| 4 | V5 Inertial Sensor (IMU) | 276-4855 | 1 | Gyro + accelerometer |
| 5 | V5 Smart Cable (300 mm) | 276-4850 | 2 | For motors |
| 6 | V5 Smart Cable (600 mm) | 276-4851 | 1 | For IMU (longer reach) |
| 7 | V5 Controller | 276-4820 | 1 | Wireless or USB tethered |
| 8 | V5 Battery Charger | 276-4812 | 1 | For charging between matches |
| 9 | USB-C Cable | — | 1 | For uploading code to Brain |

### 1.2 Structural — C-Channels & Beams

All VEX V5 structural metal uses **12.7 mm (0.5 in) hole spacing**.
Material: 0.040" wall aluminum (VEX standard) unless noted.

| # | Part | Spec (holes × profile) | Length (mm) | VEX Part # | Qty | Use |
|---|------|------------------------|-------------|-----------|-----|-----|
| 1 | C-Channel 1×2×1×25 | 25 holes, 1×2×1 profile | 317.5 mm (12.5") | 276-2288 | 2 | Left & right chassis side rails (main frame) |
| 2 | C-Channel 1×2×1×15 | 15 holes, 1×2×1 profile | 190.5 mm (7.5") | 276-2286 | 2 | Front & rear cross braces |
| 3 | C-Channel 1×5×1×25 | 25 holes, 1×5×1 profile | 317.5 mm (12.5") | 276-2294 | 1 | **(Optional)** Wide center beam for Brain mounting platform |
| 4 | L-Channel (Angle) 1×1×25 | 25 holes | 317.5 mm (12.5") | 276-2291 | 2 | **(Optional)** Extra stiffening along side rails |
| 5 | Angle Bracket (1×1, 2-hole) | 2 holes per side, 90° bend | — | 276-1066 | 4 | Corner gusset for frame rigidity |
| 6 | Flat Plate (5×5) | 5×5 holes | 63.5×63.5 mm | 276-1050 | 1 | **(Optional)** IMU / sensor mounting platform |

> **Profile naming:** "1×2×1" means the C-channel flanges are each 1-hole (12.7mm) tall and the bottom is 2-holes (25.4mm) wide — the standard VEX C-channel cross-section.

> **Minimum build:** You only need items **#1** (×2) and **#2** (×2) for the simplest frame. The angle brackets (#5) are highly recommended for rigidity. Everything else is optional.

### 1.3 Drivetrain — Wheels, Shafts & Gears

| # | Part | Spec | VEX Part # | Qty | Notes |
|---|------|------|-----------|-----|-------|
| 1 | 4" Omni-Directional Wheel | 101.6 mm diameter | 276-3526 | 2 | Main drive wheels — omni rollers allow side-sliding during turns |
| 2 | High-Strength (HS) Shaft — 4" | 101.6 mm length, 1/4" square | 276-2293 | 2 | Drive axles |
| 3 | High-Strength (HS) Shaft — 2" | 50.8 mm length, 1/4" square | 276-2296 | 2 | **(Option A — geared)** Motor-to-gear intermediate shafts |
| 4 | 60-Tooth High-Strength Gear | Module 0.8, HS bore | 276-2169 | 2 | **(Option A — geared)** Driven gear on drive shaft |
| 5 | 36-Tooth High-Strength Gear | Module 0.8, HS bore | 276-2167 | 2 | **(Option A — geared)** Driving gear on motor shaft |
| 6 | HS Motor Shaft Insert | Adapts motor output to HS shaft | 276-8532 | 2 | Required whether geared or direct drive |
| 7 | Standard Shaft — 2" | 50.8 mm, standard 1/8" square | 276-2011 | 1 | For support wheel axle |

**Drivetrain options:**

| Option | Gear ratio | Effective output | Shafts needed | Gears needed |
|--------|-----------|-----------------|---------------|-------------|
| **A — Geared 60:36** | Internal 18:1 × external 60/36 = 30:1 | ~120 RPM, high torque | 2× HS 4" + 2× HS 2" | 2× 60T + 2× 36T |
| **B — Direct drive** (simplest) | Internal 18:1 only | 200 RPM, moderate torque | 2× HS 4" only | None |

> **Direct drive is recommended for initial testing** — fewer parts, simpler assembly. Switch to geared if you need more pushing power.

### 1.4 Fasteners — Screws & Nuts

VEX uses **8-32 UNC** thread standard throughout. All screws are T15 star drive.

| # | Part | Spec | VEX Part # | Qty | Use |
|---|------|------|-----------|-----|-----|
| 1 | Star Drive Screw 8-32 × 0.250" | 1/4" (6.35 mm) long | 276-6690 | 30 | General: beam-to-beam, bracket, IMU |
| 2 | Star Drive Screw 8-32 × 0.375" | 3/8" (9.53 mm) long | 276-6691 | 10 | Through single spacer + beam wall |
| 3 | Star Drive Screw 8-32 × 0.500" | 1/2" (12.7 mm) long | 276-6692 | 20 | Motor mounting, through standoff connections |
| 4 | Star Drive Screw 8-32 × 0.750" | 3/4" (19.05 mm) long | 276-6694 | 8 | Through thick stacks / Brain mounting |
| 5 | Star Drive Screw 8-32 × 1.000" | 1" (25.4 mm) long | 276-6696 | 4 | Through standoff + beam (Brain platform) |
| 6 | Keps Nut (8-32) | Integrated lock washer | 276-4796 | 40 | Quick assembly (prototyping only) |
| 7 | Nylock Nut (8-32) | Nylon insert lock nut | 276-4800 | 30 | **Competition use** — won't vibrate loose |

**Screw length selection guide:**

| Scenario | Total stack thickness | Recommended screw |
|----------|----------------------|-------------------|
| Beam to beam (two walls) | 2 × 0.040" = ~2 mm | 0.250" (1/4") |
| Beam + spacer + beam | ~2 mm + spacer | 0.375" – 0.500" |
| Motor flange to beam | Motor body + wall | 0.500" (1/2") |
| Through standoff + beam wall | 25.4 mm + wall | 1.000" (1") |
| Angle bracket + two walls | 3 × 0.040" | 0.250" (1/4") |

> **Competition rule:** Use **Nylock nuts** on the competition robot. Keps nuts are convenient for prototyping but vibrate loose during matches. Budget 1 Nylock per screw that matters.

### 1.5 Spacers & Shaft Collars

Spacers fill gaps between components on shafts. Shaft collars lock shaft position.

| # | Part | Dimension | Bore | VEX Part # | Qty | Use |
|---|------|-----------|------|-----------|-----|-----|
| 1 | Teflon Washer / Thin Spacer | 1/16" (1.6 mm) thick | Fits 1/8" shaft | 276-1212 | 12 | Fine spacing adjustments |
| 2 | Nylon Spacer (small) | 1/8" (3.2 mm) thick | Fits 1/8" shaft | 276-1200 | 20 | Standard gap fill on shafts |
| 3 | Nylon Spacer (medium) | 1/4" (6.35 mm) thick | Fits 1/8" shaft | 276-1201 | 10 | Larger gaps on shafts |
| 4 | Nylon Spacer (large) | 3/8" (9.53 mm) thick | Fits 1/8" shaft | 276-1202 | 4 | Large spacing (rare) |
| 5 | HS Spacer | 1/8" (3.2 mm) thick | Fits HS 1/4" shaft | 276-2312 | 12 | Gap fill on HS drive shafts |
| 6 | HS Spacer | 1/4" (6.35 mm) thick | Fits HS 1/4" shaft | 276-2313 | 8 | Larger gaps on HS drive shafts |
| 7 | Standard Shaft Collar | set-screw clamp | Fits 1/8" shaft | 276-2010 | 4 | Lock standard shafts (support wheel) |
| 8 | HS Clamping Shaft Collar | set-screw clamp | Fits HS 1/4" shaft | 276-2308 | 4 | Lock HS drive shafts |
| 9 | Rubber Shaft Collar | friction fit | Fits 1/8" shaft | 276-1204 | 4 | Soft stop / cable clip (optional) |

**Spacer stacking for drive shaft (geared option):**

```
Outside                                                    Inside
  ◄────────────────── HS 4" shaft ──────────────────►

  [collar] [wheel] | wall | [bearing] [1/8" HS sp] [60T gear] [1/8" HS sp] [bearing] | wall | [collar]
                   ▲                                                                  ▲
             outer C-channel wall                                                inner C-channel wall
```

**Spacer stacking for drive shaft (direct drive):**

```
Outside                                                    Inside
  ◄────────────────── HS 4" shaft ──────────────────►

  [collar] [wheel] | wall | [bearing] [spacers to fill gap] [bearing] | wall | [motor insert]
```

### 1.6 Bearings

Bearings let shafts spin freely in metal holes without friction or wobble.

| # | Part | Type | VEX Part # | Qty | Use |
|---|------|------|-----------|-----|-----|
| 1 | HS Bearing Flat | Press-fits into beam hole, HS bore | 276-2310 | 4 | Support HS drive shafts in C-channel walls (2 per shaft) |
| 2 | Standard Bearing Flat | Press-fits into beam hole, standard bore | 276-1209 | 2 | Support standard shaft for support wheel |
| 3 | Pillow-Block Bearing (HS) | Bolts to surface, HS bore | 276-2316 | 2 | **(Optional)** Extra shaft support at chassis end |

> **Always use bearings.** Metal-on-metal shaft-in-hole contact wears down the C-channel holes and creates wobble. Press bearings into every hole a shaft passes through.

### 1.7 Support Wheel

The robot needs a third contact point for stability (2 driven wheels + 1 passive support).

| # | Part | Spec | VEX Part # | Qty | Notes |
|---|------|------|-----------|-----|-------|
| 1 | 2.75" Omni-Directional Wheel | 69.85 mm diameter | 276-3525 | 1 | Free-spinning rear support, allows sliding during turns |
| 2 | Standard Shaft — 2" | 50.8 mm, 1/8" square | 276-2011 | 1 | Support wheel axle |
| 3 | Standard Shaft Collar | set-screw | 276-2010 | 2 | Lock support shaft |
| 4 | Standard Bearing Flat | standard bore | 276-1209 | 2 | Support shaft in rear cross brace |
| 5 | Nylon Spacer 1/8" | standard bore | 276-1200 | 2 | Center wheel on shaft |

> **3-wheel layout:** Two big drive wheels near rear, one small omni wheel at front-center (or vice-versa). Omni rollers allow free sideways sliding during point turns.

### 1.8 Cable Management

| # | Part | VEX Part # | Qty | Use |
|---|------|-----------|-----|-----|
| 1 | Zip Ties (small, 4") | — | 15 | Secure cables to chassis every 50–80 mm |
| 2 | Adhesive Cable Clip | — | 4 | (Optional) Route cables along beams |
| 3 | Rubber Band (size #32 or #64) | 276-1070 | 5 | Temporary cable bundle, also for mechanisms |

### 1.9 Tools Required

| Tool | Spec | Use |
|------|------|-----|
| VEX Star Driver (T15 Torx) | — | All 8-32 star drive screws |
| 5/64" Hex Allen Key | — | Standard shaft collar set screws |
| 3/32" Hex Allen Key | — | HS shaft collar set screws |
| 7/16" Box Wrench or Socket | — | Holding 8-32 nuts while tightening |
| Needle-Nose Pliers | — | Reaching tight spaces, holding nuts |
| Calipers (digital preferred) | 0–150 mm | **Critical:** measuring wheel diameter & track |
| Ruler / Tape Measure | metric + imperial | General length checks |
| Wire Cutters / Flush Cutters | — | Trimming zip ties |
| Hobby Knife | — | (Optional) Cleaning metal burrs |

---

## 2. Chassis Dimensions & Layout

### Top-down view

```
       ◄────── ~190 mm (15 holes) ──────►
   ┌═══════════════════════════════════════┐  ──┬
   ║  ● ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  ● ║    │
   ║  │    Front Cross-Brace (15-hole)  │  ║    │
   ║  ●                                 ●  ║    │
   ║  │     ┌─────────────────────┐     │  ║    │
   ║  │     │      V5 Brain       │     │  ║    │
   ║  │     │   (on standoffs)    │     │  ║    │
   ║  │     └─────────────────────┘     │  ║    │
   ║  │                                 │  ║    │  ~317 mm
   ║  │           ■ IMU (flat)          │  ║    │  (25 holes)
   ║  │                                 │  ║    │
   ║  │     ┌────────┐  ┌────────┐      │  ║    │
   ║  │     │  Left  │  │ Right  │      │  ║    │
   ║  │     │ Motor  │  │ Motor  │      │  ║    │
   ║  │     └───┬────┘  └────┬───┘      │  ║    │
   ║  │         │ gear mesh  │          │  ║    │
   ║  ● ─ ─ ─ ─┼ ─ ─ ─ ─ ─ ┼─ ─ ─ ─  ●  ║    │
   ║  │    Rear Cross-Brace  │          │  ║    │
   ║  ● ───────┼────────────┼──────────●   ║    │
   ╠══════════ ╪════════════╪══════════════╣  ──┘
     ▓▓▓▓▓  ──┤            ├──  ▓▓▓▓▓
   L Wheel    Drive Shafts      R Wheel
   (4" omni)                    (4" omni)

Left 25-hole            ◯             Right 25-hole
C-channel ═══      Support wheel      C-channel ═══
                  (2.75" omni)
```

### Side view

```
                 ┌─────────────┐
                 │   V5 Brain  │     ← screen facing up
                 └──┬──────┬───┘
                    │      │         ← 1" standoffs
   ─────────────────┴──────┴──────── ← base frame (C-channel)
   │  Battery (underneath or rear) │
   ──────────────────────────────────
   ▓▓▓▓  Drive wheels  ▓▓▓▓    ◯  ← support wheel
```

### Key measurements for `config.h`

| Measurement | How to measure | Typical value | Config constant |
|------------|---------------|---------------|-----------------|
| **Wheel diameter** | Calipers across rubber tread, through center | 101.6 mm → 0.1016 m | `WHEEL_DIAMETER` |
| **Wheel track** | Center of left tire contact patch → center of right tire contact patch on the ground | ~254-381 mm → 0.254-0.381 m | `WHEEL_TRACK` |
| **Ticks per rev** | Green cartridge = 360, Red = 900, Blue = 1800; multiply by external gear ratio if geared | 360 (direct) or 600 (60:36) | `TICKS_PER_REV` |

> **Critical:** `WHEEL_TRACK` is **NOT** the outer width of the chassis. It is the distance between where the left and right wheels contact the ground, measured at the center of each tire. Measure this carefully with calipers or a ruler!

---

## 3. Gear Ratio & Shaft Spacing

### Motor cartridge reference

| Cartridge color | Internal ratio | Free speed | Stall torque | Ticks/rev |
|----------------|---------------|-----------|-------------|-----------|
| **Green** (default) | 18:1 | 200 RPM | 1.05 N·m | 360 |
| Red | 36:1 | 100 RPM | 2.1 N·m | 900 |
| Blue | 6:1 | 600 RPM | 0.35 N·m | 1800 |

### External gear ratio (Option A)

```
Driving gear (on motor):  36 teeth
Driven gear (on wheel shaft): 60 teeth

External gear ratio = 60 / 36 = 1.667 : 1

Combined with green cartridge (18:1):
  Effective ratio  = 18 × 1.667 = 30 : 1
  Wheel speed      = 200 / 1.667 ≈ 120 RPM
  Wheel torque     = 1.05 × 1.667 ≈ 1.75 N·m

TICKS_PER_REV = 360 × (60 / 36) = 600
```

### Center-to-center shaft spacing

When two gears mesh, the distance between their shaft centers must match:

```
Shaft spacing = Module × (Teeth₁ + Teeth₂) / 2

VEX gears are Module 0.8:
  Spacing(36T + 60T) = 0.8 × (36 + 60) / 2 = 0.8 × 48 = 38.4 mm

38.4 mm ÷ 12.7 mm/hole = 3.024 holes ≈ 3 holes apart
```

> **In practice:** Mount the motor 3 holes from the drive shaft center. The slight 0.3 mm difference from ideal is absorbed by gear backlash — this is normal and intentional. If gears are too tight (grinding), shift to the nearest half-hole using a slotted mounting.

### Common VEX gear pairs & spacing

| Driving | Driven | Ratio | Ideal spacing (mm) | Holes apart |
|---------|--------|-------|--------------------:|------------:|
| 12T | 60T | 5:1 | 28.8 | 2.27 → 2 holes + spacer |
| 12T | 84T | 7:1 | 38.4 | 3 holes |
| 36T | 60T | 1.67:1 | 38.4 | **3 holes** ← our build |
| 36T | 84T | 2.33:1 | 48.0 | 3.78 → 4 holes |
| 60T | 84T | 1.4:1 | 57.6 | 4.54 → 4.5 holes |

---

## 4. Assembly — Step by Step

### 4.1 Step 1: Build the Base Frame

**Goal:** A rectangular frame from two side rails and two cross braces.

**Parts for this step:**
- 2× C-Channel 1×2×1×25 (side rails)
- 2× C-Channel 1×2×1×15 (cross braces)
- 4× Angle bracket 1×1
- 16× Screws 8-32 × 0.250"
- 16× Nylock nuts 8-32

**Procedure:**

1. Lay the two **25-hole C-channels** parallel on a flat surface, open side facing **inward**, approximately 190 mm apart (this will be set by the cross braces)
2. Place the two **15-hole C-channels** perpendicular at the front (holes 1–2) and rear (holes 24–25) of the side rails
3. Attach at each corner with **2 screws + 2 Nylock nuts** through overlapping holes
4. Place an **angle bracket** at each corner and secure with **1 screw + 1 Nylock nut** through each bracket leg (8 more fasteners total)

```
   Screw positions (top view):          ● = screw through beam joint
   ● ● ─────────────────── ● ●
   │                         │   ← 15-hole cross brace
   ▼                         ▼
   25-hole side rail         25-hole side rail
   │                         │
   │       (open space       │
   │        for motors       │
   │        and shafts)      │
   │                         │
   ▲                         ▲
   │                         │   ← 15-hole cross brace
   ● ● ─────────────────── ● ●
```

**Quality checks:**
- Frame sits flat on a table (no rocking — all 4 corners coplanar)
- Side rails are parallel (measure front and rear spacing — should be equal)
- Corners are approximately 90° (measure diagonals — they should be equal)
- All screws are snug but not over-tightened (strips aluminum threads)

### 4.2 Step 2: Install Drive Shafts & Wheels

**Goal:** Two high-strength shafts through the rear of the chassis with 4" omni wheels.

**Parts for this step:**
- 2× HS Shaft 4"
- 2× 4" Omni Wheel
- 4× HS Bearing Flat
- 4× HS Clamping Shaft Collar
- ~8× HS Spacer (1/8") for centering

**Procedure:**

1. **Choose shaft position:** Holes 21–23 on the 25-hole side rails (near the rear). Both sides must use the same hole!
2. **Press HS Bearing Flats** into the C-channel holes on both walls where the shaft passes through (4 bearings total — inner wall and outer wall of each side rail)
3. Slide the **HS 4" shaft** through from outside:

   ```
   Outside-to-inside order (one side):
   [HS Collar] → [4" Omni Wheel] → | outer wall + bearing | → [HS spacers] → | inner wall + bearing | → [HS Collar]
   ```

4. Attach the **4" omni wheel** to the outer end of each shaft — press the shaft into the wheel's square bore
5. Add HS spacers between inner and outer walls to **take up slack** — the shaft should not slide laterally
6. Clamp **HS shaft collars** on both sides of each C-channel wall to lock the shaft in position

**Quality checks:**
- Shafts spin freely with no binding (flick wheel — should spin 3+ seconds)
- No lateral play (shaft doesn't slide in/out)
- Wheels are perpendicular to the shaft (no wobble)
- Both wheels are at the same height (robot sits level on the two drive wheels)

### 4.3 Step 3: Mount Motors & Gear Train

**Goal:** Each motor connects to its drive shaft.

#### Option A — Geared (60:36)

**Additional parts:**
- 2× HS Shaft 2" (motor intermediate shafts)
- 2× 36T HS Gear
- 2× 60T HS Gear
- 2× HS Motor Shaft Insert
- 2× HS Bearing Flat (for intermediate shafts)
- 4× Screws 8-32 × 0.500" (motor mounting)

**Procedure:**

1. **Install the 60T gear** on each drive shaft (the 4" HS shaft already in the chassis), between the inner bearings. Position it **3 holes from the drive shaft center**.
2. Insert an **HS Motor Shaft Insert** into each motor's output
3. Attach a **36T gear** to the motor shaft insert
4. Mount each motor to the **inside** of the C-channel side rail using **2× 0.500" screws** through the motor's threaded mounting holes:
   - The motor's 36T gear must mesh with the 60T gear on the drive shaft
   - Motor shaft center is **3 holes** from drive shaft center (38.4 mm spacing)
5. Adjust motor position so gears mesh smoothly — a **tiny gap** (half a tooth) between faces is better than teeth jammed together

#### Option B — Direct Drive (simplest)

**Additional parts:**
- 2× HS Motor Shaft Insert
- 4× Screws 8-32 × 0.500" (motor mounting)

**Procedure:**

1. Insert an **HS Motor Shaft Insert** into each motor's output
2. Slide the motor shaft insert **directly into the HS drive shaft** (or directly into the wheel hub if it fits)
3. Mount each motor to the inside of the C-channel using **2× 0.500" screws**

**Gear mesh check (geared option):**
- Spin the wheel by hand — smooth, slight resistance, no clicking or grinding
- A sheet of paper should barely fit between meshing teeth (backlash test)
- No skipping under moderate load (hold wheel, apply force)

> **Motor orientation:** Both motors mount **mirror-symmetrically** with output shafts pointing inward (toward chassis center). This is why `RightDriveSmart` is constructed with `true` (reversed) in `main.cpp`.

### 4.4 Step 4: Mount Support Wheel

**Goal:** One free-spinning omni wheel at the front or rear center for balance.

**Parts for this step:**
- 1× 2.75" Omni Wheel
- 1× Standard Shaft 2"
- 2× Standard Bearing Flat
- 2× Standard Shaft Collar
- 2× Nylon Spacer 1/8"

**Procedure:**

1. **Choose mounting point:** Center of the front or rear cross brace (the end opposite the drive wheels works best)
2. Press **2 Standard Bearing Flats** into the cross brace holes, centered
3. Slide **Standard 2" shaft** through: bearing → 1/8" spacer → 2.75" omni wheel → 1/8" spacer → bearing
4. Lock with **Standard Shaft Collars** on both sides

**Quality check:** Support wheel spins freely. Robot sits level on all 3 contact points (2 drive + 1 support). Robot doesn't tip in any direction.

### 4.5 Step 5: Mount the Brain & Battery

**Goal:** Brain accessible for screen viewing and USB upload; battery easy to swap.

**Parts for this step:**
- 4× Standoffs (1" or 2")
- 8× Screws 8-32 × 0.500" (or 1.000" for through-standoff)
- 4× Nylock Nuts

**Procedure:**

1. Attach 4× **standoffs (1")** to the top surface of the frame using 0.500" screws through the standoff into a nut below the frame
   - Position: centered on the chassis, toward the front, matching the Brain mounting hole pattern
   - Standoffs raise the Brain above the frame — wires route underneath
2. Set the **V5 Brain** on top of the standoffs. Align its mounting holes with the standoffs
3. Secure with 0.250" screws through Brain holes into standoff tops
   - Screen faces **UP** and is easily readable
   - USB-C port is **accessible** from the side (not blocked by metal)
4. Place the **V5 Battery** underneath the chassis or behind the Brain depending on balance
   - Secure with **zip ties** or VEX rubber bands around the frame
   - Plug battery cable into Brain's battery port
   - Battery should be removable for charging between matches

### 4.6 Step 6: Mount the IMU

**Goal:** IMU flat, centered, far from motors, rigidly fixed.

**Parts for this step:**
- 1× V5 Inertial Sensor
- 2× Screws 8-32 × 0.250"
- 2× Nylock Nuts (or Keps for prototyping)
- (Optional) 1× Flat Plate for mounting

**Procedure:**

1. Find a flat area near the **center of the chassis** — ideally on a cross brace or a small flat plate bolted between the rails
2. Mount the IMU **flat** (parallel to ground) using 2× screws through its mounting holes
3. The sensor's arrow indicator should point **forward** (toward the robot's front)

**IMU mounting rules:**
- ✅ **Flat and level** — any tilt introduces heading error proportional to `sin(tilt)`
- ✅ **Centered** on chassis — minimizes vibration-induced measurement noise
- ✅ **Rigidly secured** — loose mounting = jittery readings, especially during acceleration
- ❌ **NOT near motors** — motor permanent magnets interfere with the magnetometer
- ❌ **NOT cantilevered** — must be supported from below, not hanging off an unsupported edge
- ❌ **NOT touching the battery** — battery draws current and creates magnetic field

### 4.7 Step 7: Wiring & Cable Management

**Connect V5 Smart Cables:**

| Device | Brain Port | Cable Length | Config Constant |
|--------|-----------|-------------|-----------------|
| Left Drive Motor | **Port 1** | 300 mm | `LEFT_MOTOR_PORT = 1` |
| Inertial Sensor (IMU) | **Port 5** | 600 mm | `IMU_PORT = 5` |
| Right Drive Motor | **Port 10** | 300 mm | `RIGHT_MOTOR_PORT = 10` |

**Cable routing rules:**

1. Route cables cleanly — no loops near wheels, gears, or shafts
2. Use **zip ties** every 50–80 mm along the frame to secure cables
3. Leave **slack** near each connector (strain relief) — don't pull cables taut
4. Keep cables **away from moving parts** — at least 10 mm clearance from spinning shafts and gears
5. Tuck excess cable length under the Brain or beside the battery
6. Double-check: push each connector in until it clicks (partially-seated connectors cause intermittent failures)

> **If you use different ports:** Update the three constants in `include/config.h`.

---

## 5. Software Configuration

After assembly, update `include/config.h` with your actual measurements:

```cpp
// ── Measure with calipers! ──
constexpr double WHEEL_DIAMETER   = 0.1016;   // 4" omni = 0.1016 m
constexpr double WHEEL_TRACK      = 0.280;    // MEASURE your robot! (m)
constexpr double TICKS_PER_REV    = 360.0;    // 360 direct drive
                                               // 600 if using 60:36 gears

// ── Ports — match your wiring ──
constexpr int LEFT_MOTOR_PORT  = 1;
constexpr int RIGHT_MOTOR_PORT = 10;
constexpr int IMU_PORT         = 5;
```

**Also verify `src/main.cpp`:**

```cpp
// Motor cartridge must match what's physically installed:
motor LeftDriveSmart  = motor(LEFT_MOTOR_PORT,  ratio18_1, false); // green = 18:1
motor RightDriveSmart = motor(RIGHT_MOTOR_PORT, ratio18_1, true);  // reversed

// If using red cartridge: ratio36_1   (and TICKS_PER_REV = 900)
// If using blue cartridge: ratio6_1   (and TICKS_PER_REV = 1800)
```

---

## 6. Deploying to VEX V5 Brain

### Prerequisites

- **VEX VS Code Extension** installed (or VEXcode Pro V5)
- **V5 Brain** powered on and connected
- Project builds successfully (`make` exits with code 0)

### Upload Steps

1. **Connect** the V5 Brain:
   - **Wired (recommended):** USB-C cable → Computer to Brain
   - **Wireless:** USB cable → Computer to Controller → (wireless) → Brain

2. **Build** the project:
   - Press `Cmd+Shift+B` (Mac) or `Ctrl+Shift+B` (Windows)
   - Or in terminal: `make`
   - Wait for `BUILD SUCCESSFUL`

3. **Upload** to Brain:
   - Click the **Download ↓** button in the VEX extension toolbar
   - Or open Command Palette → `VEX: Build and Download`
   - The Brain screen shows an upload progress bar

4. **Run** the program:
   - On V5 Brain: tap the program name → **Run**
   - Or connect a Competition Switch / Field Controller for autonomous + driver control

---

## 7. First-Boot Verification (6 Checks)

Run these checks in order after first upload. Each one validates a layer of the software stack.

### ✅ Check 1: Brain shows "Ready."

**Verifies:** IMU calibration, `pre_auton()` runs

- Power on → program runs → screen shows **"Ready."** within ~3 seconds
- **Fail?** IMU not connected to Port 5, or bad cable

### ✅ Check 2: Motor direction

**Verifies:** HAL motor layer, correct wiring, reversed flag

1. **Put robot on blocks** (wheels off ground — safety first!)
2. Push left joystick forward → left wheel spins forward
3. Push right joystick forward → right wheel spins forward

- **Fail?** Flip `true`↔`false` for that motor in `main.cpp`

### ✅ Check 3: Encoder counting

**Verifies:** Encoder ticks, `TICKS_PER_REV`

1. Add debug code in `main()` loop (after `odometry_update()`):
   ```cpp
   Brain.Screen.clearLine(2); Brain.Screen.setCursor(2, 1);
   Brain.Screen.print("L:%.0f R:%.0f",
       get_left_encoder_ticks(), get_right_encoder_ticks());
   ```
2. Hand-rotate each wheel one full turn → should read ~360 (±5) for direct drive, ~600 (±5) for 60:36

### ✅ Check 4: IMU heading

**Verifies:** IMU reads, heading direction

1. Add debug: `Brain.Screen.print("IMU: %.2f rad", get_imu_heading_rad());`
2. Rotate robot 90° clockwise by hand → reading ≈ 1.57

### ✅ Check 5: Odometry tracking

**Verifies:** Full localization pipeline

1. Add debug:
   ```cpp
   Pose p = get_pose(); Brain.Screen.clearLine(4); Brain.Screen.setCursor(4, 1);
   Brain.Screen.print("x:%.2f y:%.2f t:%.2f", p.x, p.y, p.theta);
   ```
2. Push robot forward 1 meter by hand → x ≈ 1.00

### ✅ Check 6: Autonomous L-path

**Verifies:** PID, motion profile, complete control stack

1. Place robot on floor with 1.5 m clear space ahead and to the left
2. Run autonomous mode → robot drives L-shaped path
3. **Hold the robot lightly** on the first run in case motors are reversed!

---

## 8. Troubleshooting

### Mechanical Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Robot veers left/right | Wheels different diameter, uneven friction | Swap wheels, clean treads, check for debris |
| Gears grind/skip | Mesh too tight or too loose | Loosen motor screws, adjust to 3-hole spacing, retighten |
| Shaft wobbles | Missing bearings, worn holes | Add bearing flats to every hole a shaft passes through |
| Robot tips forward/back | Center of gravity off-balance | Reposition battery (heaviest component) |
| Wheels don't turn freely | Spacer stack too tight, shaft binding | Loosen shaft collars, add/remove thin spacers |
| Robot rattles during driving | Loose screws/nuts | Tighten all fasteners, switch from Keps to Nylock |
| Frame flexes/twists | Not enough cross bracing | Add angle brackets, optional L-channel stiffeners |
| Wheel slips on shaft | Shaft collar not tight, worn bore | Tighten collar set screw, replace worn wheel |

### Electrical Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Brain says "No program" | Program not uploaded | Re-upload via VEX extension |
| "Ready." never shows | IMU wrong port / bad cable | Check wiring, try different cable/port |
| Motor doesn't spin | Cable not fully seated | Re-seat cable until it clicks, verify port |
| One wheel spins backward | Motor reversed flag wrong | Flip `true`↔`false` in main.cpp |
| Encoder reads ~900 | Red cartridge installed | Set `TICKS_PER_REV=900`, `ratio36_1` |
| Upload fails | Brain not connected properly | Use USB-C directly to Brain, restart Brain |
| Controller has no response | Not paired / not charged | Re-pair with Brain, charge controller |

### Software Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Odometry drifts | `WHEEL_DIAMETER` or `WHEEL_TRACK` wrong | Re-measure precisely with calipers |
| Turns overshoot | PID gains too aggressive | Reduce `TURN_KP`, increase `TURN_KD` |
| Drives past target | `MAX_VELOCITY` too high | Reduce `MAX_VELOCITY` in config.h |
| `make` fails | SDK path wrong | Ensure VEX extension installed, SDK path exists |
| `make test` fails | Tests need host compiler | Run `xcode-select --install` for g++ on Mac |

---

## 9. Appendix: Part Number Quick Reference

```
╔══════════════════════════════════════════════════════════════════════╗
║  VEX V5 DIFFERENTIAL DRIVE — COMPLETE PARTS SUMMARY                ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                      ║
║  ELECTRONICS                              Part #         Qty        ║
║    V5 Brain                               276-4810       ×1         ║
║    V5 Battery                             276-4811       ×1         ║
║    V5 Smart Motor (green cart.)           276-4840       ×2         ║
║    V5 Inertial Sensor                     276-4855       ×1         ║
║    V5 Controller                          276-4820       ×1         ║
║    Smart Cable 300mm                      276-4850       ×2         ║
║    Smart Cable 600mm                      276-4851       ×1         ║
║                                                                      ║
║  STRUCTURE                                                           ║
║    C-Channel 1×2×1×25                     276-2288       ×2         ║
║    C-Channel 1×2×1×15                     276-2286       ×2         ║
║    Angle Bracket 1×1                      276-1066       ×4         ║
║                                                                      ║
║  DRIVETRAIN (Direct Drive)                                           ║
║    4" Omni Wheel                          276-3526       ×2         ║
║    HS Shaft 4"                            276-2293       ×2         ║
║    HS Motor Shaft Insert                  276-8532       ×2         ║
║                                                                      ║
║  DRIVETRAIN (Geared — add these)                                     ║
║    60T HS Gear                            276-2169       ×2         ║
║    36T HS Gear                            276-2167       ×2         ║
║    HS Shaft 2"                            276-2296       ×2         ║
║                                                                      ║
║  SUPPORT WHEEL                                                       ║
║    2.75" Omni Wheel                       276-3525       ×1         ║
║    Standard Shaft 2"                      276-2011       ×1         ║
║                                                                      ║
║  BEARINGS                                                            ║
║    HS Bearing Flat                        276-2310       ×4         ║
║    Standard Bearing Flat                  276-1209       ×2         ║
║                                                                      ║
║  FASTENERS                                                           ║
║    8-32 × 0.250" Star Drive              276-6690       ~30        ║
║    8-32 × 0.375" Star Drive              276-6691       ~10        ║
║    8-32 × 0.500" Star Drive              276-6692       ~20        ║
║    8-32 × 0.750" Star Drive              276-6694       ~8         ║
║    8-32 × 1.000" Star Drive              276-6696       ~4         ║
║    Keps Nut 8-32                          276-4796       ~40        ║
║    Nylock Nut 8-32                        276-4800       ~30        ║
║                                                                      ║
║  SPACERS & COLLARS                                                   ║
║    Thin Spacer 1/16"                      276-1212       ~12        ║
║    Nylon Spacer 1/8"                      276-1200       ~20        ║
║    Nylon Spacer 1/4"                      276-1201       ~10        ║
║    HS Spacer 1/8"                         276-2312       ~12        ║
║    HS Spacer 1/4"                         276-2313       ~8         ║
║    Standard Shaft Collar                  276-2010       ×4         ║
║    HS Clamping Shaft Collar               276-2308       ×4         ║
║                                                                      ║
║  STANDOFFS                                                           ║
║    1" Standoff 8-32                       276-2010       ×4         ║
║                                                                      ║
║  MISC                                                                ║
║    Zip Ties (small)                       —              ~15        ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝
```

---

**End of Hardware Build & Deployment Guide**
