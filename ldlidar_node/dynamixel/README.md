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
| `dynamixel_actuator_node` | motor IDs, DynamixelSDK, control table, calibration offsets, soft limits, current cutoff, torque-disable on fault/shutdown, logical→physical fan-out | IK, geometry, modes |
| `oscillation_planner_node` | mechanism geometry, inverse kinematics, the 3 speed modes, torque feasibility | DynamixelSDK, motor IDs, ticks |

### Logical joints
The actuator exposes **3 logical joints** — `theta`, `alpha`, `beta` — even
though `theta` is **two physical motors** (IDs 1 & 2, mirror-mounted, each with
its own sign/offset). The fan-out is invisible to publishers.

| logical | physical IDs | note |
|---------|--------------|------|
| theta | 1, 2 | both `direction −1`; identical goal ticks after homing |
| alpha | 3 | `direction −1` |
| beta  | 4 | `direction −1` |

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

# 3a. Actuator + planner separately (two terminals)
ros2 launch ldlidar_node dynamixel_actuator.launch.py
ros2 launch ldlidar_node oscillation_planner.launch.py mode:=2

# 3b. …or both at once
ros2 launch ldlidar_node dynamixel_bringup.launch.py mode:=2 repeat:=1
```

> **Bench safety:** verify the θ pair (IDs 1 & 2) sign convention at a **low
> `current_limit_ma`** before full power. If they fight each other, flip one
> sign in `motors.theta.signs`.

## Speed modes (planner)

`T` is the **one-way (half-cycle) leg** time; a full round trip = `2·T`.

| mode | name | leg T | round trip |
|------|------|-------|-----------|
| 1 | slow | 6.0 s | 12.0 s |
| 2 | normal | 4.5 s | 9.0 s |
| 3 | fast | 3.0 s | 6.0 s (== validated baseline) |

The planner runs a **torque-feasibility gate** at startup for every mode and
**refuses to publish** (falling back to the fastest feasible mode with a loud
warning) if a mode's estimated peak torque exceeds 90% of stall (1.35 N·m) or
its rms exceeds rated (0.83 N·m). Because the fast mode equals the validated
`T=3.0 s` point and torque is non-increasing with `T`, all three modes pass.

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
  oscillation_planner_node.py   geometry / IK / modes / feasibility
  find_servos.py                baud + ID discovery CLI
  dynamixel_calibrate.py        homing / offset capture CLI
params/
  dynamixel_actuator.yaml       port, baud, mapping, limits, current, calibration
  oscillation_planner.yaml      geometry, trajectory, per-mode T, torque model
launch/
  dynamixel_actuator.launch.py
  oscillation_planner.launch.py   (mode:=1|2|3)
  dynamixel_bringup.launch.py     (both nodes)
```
