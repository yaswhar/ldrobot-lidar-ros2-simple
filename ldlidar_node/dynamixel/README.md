# DYNAMIXEL parallel-linkage platform nodes

Two ROS2 (Humble) nodes that actuate the 4x **DYNAMIXEL XC430-W150-T** servos of
the parallel-linkage platform. They live in this `dynamixel/` subfolder of
`ldlidar_node` and are **fully decoupled from the lidar code**.

## Architecture — two nodes, split on purpose

```
  oscillation_planner_node                 dynamixel_actuator_node
  (geometry / IK / modes)   --/joint_goal-->   (motors / SDK / ticks)
                            <--/joint_states--
```

The **only** coupling is the `/joint_goal` topic
(`sensor_msgs/JointState`, `name: [theta, alpha, beta]`, `position` in rad,
optional `velocity` in rad/s). A future `lidar_planner_node` can replace the
planner with **zero changes** to the actuator.

| Node | Owns | Never touches |
|------|------|---------------|
| `dynamixel_actuator_node` | motor IDs, DynamixelSDK, control table, calibration offsets, soft limits, current cutoff, torque-disable on fault/shutdown, logical→physical fan-out | IK, geometry, trajectory |
| `oscillation_planner_node` | mechanism geometry, inverse kinematics (LUT), the elliptical trajectory, per-lap timing, torque feasibility | DynamixelSDK, motor IDs, ticks |

> The planner quantizes LUT angles to the encoder lattice (4096 ticks/rev, a
> published XC430 spec) but still imports **no** motor IDs, calibration, or
> `dynamixel_sdk` — the lattice spacing is direction/offset independent, so the
> boundary holds.

### Logical joints
The actuator exposes **3 logical joints** — `theta`, `alpha`, `beta` — even
though `theta` is **two physical motors** (IDs 1 & 2, mirror-mounted, each with
its own sign/offset). The fan-out is invisible to publishers.

| logical | physical IDs | note |
|---------|--------------|------|
| theta | 1, 2 | both `direction −1`; identical goal ticks after homing |
| alpha | 3 | `direction −1` |
| beta  | 4 | `direction −1` |

### Motion & inverse kinematics

The planner drives a single continuous **ellipse** in (height, tilt) space:

```
h(phi)     = h_mid + (stroke/2) cos(phi)      h_mid = (h_high + h_low)/2
gamma(phi) = gamma_amp * sin(phi)             phi in [0, 2*pi)
```

`h_high`/`h_low` are reached only at `gamma=0`; the tilt extremes occur at
`h_mid`. The achievable (h, gamma) envelope **narrows toward h_mid as |gamma|
grows** (theta has no gamma term, but the side-link coupling
`h_alpha = h + (d/2)sin g`, `h_beta = h - (d/2)sin g` does), so the ellipse stays
inside it by construction. The loop repeats for the laps in `lap_durations_s`,
each independently timed.

Inverse kinematics uses a **precomputed lookup table** (built offline by
`generate_ik_lut.py`, cached to `~/.cache/ldlidar_platform/ik_lut.npz`, rebuilt
on first startup if missing):

- `theta`: 1-D table on `h` (5 mm grid). `theta` has **no** gamma dependence.
- `alpha`, `beta`: 2-D tables on `(h, gamma)` (5 mm × 1 deg grid), NaN outside
  the reachable envelope.
- Each grid angle is rounded to the encoder lattice, then the height it *actually*
  produces is forward-recomputed and stored, so the table is self-consistent with
  what the hardware can hit (sub-mm quantization drift).
- **Runtime** does bilinear (alpha/beta) / linear (theta) interpolation only:
  O(1), fully deterministic, **no root finder in the control path** — the right
  shape for the tight reactive loop this becomes when a lidar planner replaces it.

> **Sign convention (correctness-critical):** the central-link map uses
> `u = b cos(theta) + (a-d)/2` (**PLUS**, `motor_sim_utils.h_from_theta`), NOT the
> `mech_opt_redesign.py` geometry-search convention (MINUS). Because `(a-d)/2` is
> only ~5 mm, the wrong sign yields a plausible-but-off theta range (5.6..72.8 vs
> the correct 6.5..73.3). The side-link map uses `u = b cos(angle) - k` (MINUS).
> See `platform_kinematics.py`.

### Control approach (bench-measured 2026-08)
- **Baud fixed at 1,000,000** — the actuator connects directly, no probing on
  launch (`find_servos.py --scan` is the opt-in diagnostic).
- **Extended Position Control Mode (op mode 4)** on all 4 motors — ID2's homed
  target goes negative across its real range, outside plain Position Control.
- **Calibration = EEPROM Homing Offset** (not a static YAML offset), so the
  runtime formula collapses to `goal_ticks = round(direction × angle_deg ×
  4096/360)` with `direction = −1` for all four. A static offset could silently
  drift a whole revolution (4096 ticks); Homing Offset + a startup
  revolution-safety check avoid commanding a pose 360° away.

## First-time bring-up

```bash
# 0. (once) add DynamixelSDK to the image and rebuild — see ../../docker/README.md

# 1. (optional) confirm the 4 servos answer on the fixed 1,000,000 baud bus.
#    find_servos.py pings at 1M by default; add --scan only to troubleshoot.
ros2 run ldlidar_node find_servos.py --port /dev/ttyUSB0
ros2 run ldlidar_node find_servos.py --port /dev/ttyUSB0 --scan   # full sweep

# 2. Homing / calibration — writes each motor's EEPROM Homing Offset so Present
#    Position reads 0 at the physical-zero pose (theta=alpha=beta=0 deg).
ros2 run ldlidar_node dynamixel_calibrate.py \
    --params $(ros2 pkg prefix ldlidar_node)/share/ldlidar_node/params/dynamixel_actuator.yaml
#    -> saves a record to ~/dynamixel_offsets.yaml; the ZERO itself lives in
#       motor EEPROM (persists across power cycles), not in a YAML offset.

# 2b. (optional) pre-build the IK lookup table at deploy time so the first
#     planner launch does not pay the ~10 s one-time build. Otherwise the node
#     builds + caches it automatically on first startup.
ros2 run ldlidar_node generate_ik_lut.py \
    --params $(ros2 pkg prefix ldlidar_node)/share/ldlidar_node/params/oscillation_planner.yaml \
    --benchmark

# 3a. Actuator + planner separately (two terminals)
ros2 launch ldlidar_node dynamixel_actuator.launch.py
ros2 launch ldlidar_node oscillation_planner.launch.py

# 3b. …or both at once
ros2 launch ldlidar_node dynamixel_bringup.launch.py
```

Edit `lap_durations_s` (and geometry/trajectory) in `oscillation_planner.yaml`
to change the motion — there is no longer a `mode`/`repeat` launch argument.

> **Bench safety:** verify the θ pair (IDs 1 & 2) sign convention at a **low
> `current_limit_ma`** before full power. If they fight each other, flip one
> sign in `motors.theta.signs`.

## Laps & feasibility (planner)

Each entry of `lap_durations_s` is one full elliptical loop; the default runs 3
laps (slow → medium → fast):

| lap | duration | peak torque | rms | verdict |
|-----|----------|-------------|-----|---------|
| 1 | 12.0 s | 0.74 N·m (49% stall) | 0.46 N·m | PASS |
| 2 | 9.0 s | 0.90 N·m (60% stall) | 0.54 N·m | PASS |
| 3 | 7.0 s | 1.14 N·m (76% stall) | 0.67 N·m | PASS |

The planner runs a **shape-aware torque-feasibility gate** at startup for every
lap. It anchors on the validated one-way leg at `T_REF=3.0 s` (peak ~1.07 N·m /
rms ~0.70 N·m) and scales the inertial term by the lap's actual peak/rms joint
**acceleration** relative to that anchor (the gravity term is constant). A lap
whose estimated peak exceeds 90% of stall (1.35 N·m) or whose rms exceeds rated
(0.83 N·m) is **auto-slowed** (loud warning) to the fastest duration that passes,
using the `accel ∝ 1/T²` relation. The ellipse adds a full tilt oscillation the
old monotonic leg lacked, so the side links carry ~1.5× the inertial load — which
is why a naive 6.0 s lap trips the gate and 7.0 s is the fast default.

> The feasibility sweep uses the **precise** root finder, not the LUT: double-
> differentiating the tick-quantized LUT would manufacture phantom acceleration.
> It is a startup-only computation, so accuracy is free.

## Safety (actuator, independent of the planner)

- Per-logical-joint **soft angle limits** clamp every goal (backstop).
- Independent **current-based cutoff**: GroupSyncRead Present Current →
  torque-disable + latch fault if it exceeds `current_limit_ma`.
- **Torque-disable** on SIGINT/SIGTERM and on any current fault.
- **Creep** slowly to the first commanded pose (`creep_profile_velocity_rad_s`).
- **Revolution-safety check** at startup: reads Present Position and, if it sits
  a whole multiple of 4096 outside the soft-limit band, software-corrects it and
  warns loudly instead of driving a full turn to the wrong pose. Startup also
  logs the expected raw/homed tick ranges to eyeball before power.

## Files

```
dynamixel/
  dynamixel_actuator_node.py    hardware layer (rclpy + dynamixel_sdk)
  oscillation_planner_node.py   ellipse trajectory / LUT IK / laps / feasibility
  platform_kinematics.py        shared forward/inverse kinematics (pure math)
  generate_ik_lut.py            offline IK lookup-table builder + runtime IKTable
  find_servos.py                baud + ID discovery CLI
  dynamixel_calibrate.py        homing / offset capture CLI
params/
  dynamixel_actuator.yaml       port, baud, mapping, limits, current, calibration
  oscillation_planner.yaml      geometry, ellipse trajectory, LUT grid, laps, torque
launch/
  dynamixel_actuator.launch.py
  oscillation_planner.launch.py   (params_file:=…)
  dynamixel_bringup.launch.py     (both nodes)
```
