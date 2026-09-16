# DYNAMIXEL parallel-linkage platform nodes

Two ROS2 (Humble) nodes that actuate the 4x **DYNAMIXEL XC430-W150-T** servos of
the parallel-linkage platform, each through a **7:1 single-stage cycloidal
reducer** (fitted 2026-09). They live in this `dynamixel/` subfolder of
`ldlidar_node` and are **fully decoupled from the lidar code**.

## Architecture — two nodes, split on purpose

```
  oscillation_planner_node                 dynamixel_actuator_node
  (geometry / IK / modes)   --/joint_goal-->   (motors / SDK / ticks / GEARBOX)
                            <--/joint_states--
```

The **only** coupling is the `/joint_goal` topic
(`sensor_msgs/JointState`, `name: [theta, alpha, beta]`, `position` in JOINT
rad, `velocity` in JOINT rad/s, **signed**). A future `lidar_planner_node` can
replace the planner with **zero changes** to the actuator. The gearbox is a
hardware-layer detail: it never appears in the message.

| Node | Owns | Never touches |
|------|------|---------------|
| `dynamixel_actuator_node` | motor IDs, DynamixelSDK, control table, **gear model**, control mode (position / velocity cascade), calibration offsets, soft limits, load cutoff, watchdog, torque-disable on fault/shutdown, logical→physical fan-out | IK, geometry, trajectory |
| `oscillation_planner_node` | mechanism geometry, inverse kinematics (LUT), the elliptical trajectory, per-lap timing, the torque-feasibility gate (verified statics + inertia) | DynamixelSDK, motor IDs, ticks |

> The planner knows exactly two hardware facts — `gear.ratio` and
> `gear.efficiency` — because the torque gate and the joint-side encoder lattice
> of the LUT need them. It still imports **no** motor IDs, calibration, or
> `dynamixel_sdk`, and no `scipy` (the verified dynamics are ported into
> `platform_kinematics.PlatformDynamics`, cross-checked by
> `diagnostics/gear_check.py`).

### Logical joints
The actuator exposes **3 logical joints** — `theta`, `alpha`, `beta` — even
though `theta` is **two physical motors** (IDs 1 & 2). The fan-out is invisible
to publishers. In velocity mode each physical motor closes its own outer loop
on its own encoder against the shared joint goal.

| logical | physical IDs | note |
|---------|--------------|------|
| theta | 1, 2 | both `direction −1`; identical goal ticks after homing |
| alpha | 3 | `direction −1` |
| beta  | 4 | `direction −1` |

### Gear model (actuator `gear.*`, defaults = gearless build)

```
motor_deg   = gear_sign * gear.ratio * joint_deg       gear_sign = -1 if gear.reversing
ticks       = round(direction * motor_deg * 4096/360) + turn_offset
motor rad/s = gear.ratio * joint rad/s                 (same for rad/s^2)
joint N.m   = motor N.m * gear.ratio * gear.efficiency
```

`direction` keeps its calibration meaning (the measured raw-tick-vs-angle sense
of the servo itself, −1 on all four); the gear sign is a separate multiplier so
neither fact is hidden inside the other. The fitted cycloid is `ratio: 7.0`,
`reversing: true`, `efficiency: 0.88` (assumed — measure it). With
`ratio: 1.0` the node reproduces the pre-gearbox behaviour exactly.

Consequences at 7:1 (all from `diagnostics/gear_check.py`):
- joint-side encoder lattice 4096 × 7 = 28672 ticks/rev = **0.01256°** (was 0.088°);
  the LUT quantizes to it and its cache signature includes the ratio;
- soft limits **30..85° joint** (all three, since 2026-09-16) = 210..595° motor =
  1.07 rev, 2389..6770 ticks — well inside Extended Position (±1,048,575) and
  Homing Offset (±1,044,479);
- one motor revolution = **51.4° of joint** — see the revolution-safety check below;
- reflected crank inertia 0.1135 / 49 = 0.0023 kg·m²; cycloid ring-pin ripple
  8 × 0.56 rev/s = **4.5 Hz** at the lap's 33 rpm peak.

### Motion & inverse kinematics

The planner drives a single continuous **ellipse** in (height, tilt) space:

```
h(phi)     = h_mid + (stroke/2) cos(phi)      h_mid = (h_high + h_low)/2
gamma(phi) = gamma_amp * sin(phi)             phi in [0, 2*pi)
```

`h_high`/`h_low` are reached only at `gamma=0`; the tilt extremes occur at
`h_mid`. The achievable (h, gamma) envelope **narrows toward h_mid as |gamma|
grows**, so the ellipse stays inside it by construction. The loop repeats for
the laps in `lap_durations_s`, each independently timed.

The stroke ends: the geometric floor is **0.635 m** (alpha/beta binding, ~2°)
and the ceiling **1.210 m**. Since 2026-09-16 the joints are limited to
**30..85°**, so `h_low = 0.92` (the ellipse then bottoms at theta 33.8°,
alpha/beta 32.9° — `diagnostics/gear_check.py` §3b prints the margins; at 0.90
they touch 30.6°). Stroke is 0.26 m. `h_high = 1.18` stays 30 mm below the
ceiling where `dh/dtheta` collapses (a real singularity). Set `h_low: 0.65` to
get the old 0.53 m stroke back if the limits are widened again.

Inverse kinematics uses a **precomputed lookup table** (built offline by
`generate_ik_lut.py`, cached to `~/.cache/ldlidar_platform/ik_lut.npz`, rebuilt
on first startup if missing or if geometry / grid / **gear ratio** changed):

- `theta`: 1-D table on `h` (5 mm grid). `theta` has **no** gamma dependence.
- `alpha`, `beta`: 2-D tables on `(h, gamma)` (5 mm × 1 deg grid), NaN outside
  the reachable envelope.
- Each grid angle is rounded to the **joint-side** encoder lattice, then the
  height it *actually* produces is forward-recomputed and stored.
- **Runtime** does bilinear / linear interpolation only: O(1), deterministic,
  **no root finder in the control path**.

> **Sign convention (correctness-critical):** the central-link map uses
> `u = b cos(theta) + (a-d)/2` (**PLUS**, `motor_sim_utils.h_from_theta`), NOT the
> `mech_opt_redesign.py` geometry-search convention (MINUS). The side-link map
> uses `u = b cos(angle) - k` (MINUS). See `platform_kinematics.py`.

### Velocity on `/joint_goal` is signed

The planner publishes the one-step look-ahead velocity **with its sign** (and
`0` while it holds the start pose). The position-mode actuator uses `|v|` as the
firmware profile speed; the velocity-mode cascade uses `v` as feed-forward. A
future planner should do the same — publishing a nonzero velocity for a held
pose biases the cascade by `−v/Kp`.

## Control modes (actuator `control.mode`)

### `position` — Extended Position Control (op mode 4)
Every goal writes one 12-byte block `ProfileAccel + ProfileVel + GoalPosition`;
the servo closes the position loop. **All three are gear-scaled**: the profile
velocity/acceleration written to the servo are the planner's joint values × 7.
Before 2026-09 they were written unscaled — the servo was handed 1/7 of the
speed it needed per 20 ms waypoint (4.8 rpm at the lap peak instead of 33.4),
lagged, then lurched at the next goal. That is the kickoff prompt's **Step 0**
hypothesis for the wobble; run this mode first after re-gearing.

### `velocity` — Velocity Control (op mode 1) with the position loop closed in ROS
Per physical motor, every `control.rate_hz` period:

```
q_ref  = q_des + v_ff * age(goal)                       (dead-reckoned between goals)
corr   = clamp(Kp*(q_ref - q_meas) + Ki*I, ±catchup_rate_rad_s)
v      = clamp(v_ff + corr, ±velocity_limit_rad_s)      (JOINT side)
v      = 0 if at/past a soft limit and v points outward
Goal Velocity(104) = direction * gear_sign * ratio * v   (0.229 rpm units)
```

- **Watchdog**: no `/joint_goal` for `watchdog_timeout_s` (0.1 s) → Goal
  Velocity 0 on all motors (a velocity-loop hold). Released when goals resume.
- **Creep**: the first goal is approached with `v_ff = 0` and the correction
  capped at `creep_profile_velocity_rad_s`, until within `creep_done_deg`;
  `creep_timeout_s` → fault (wrong sign / stalled joint).
- **Goal discontinuities never re-creep**: the correction cap closes them while
  the feed-forward keeps following the stream (no way to get trapped behind a
  moving goal); a jump > `goal_jump_warn_deg` is logged.
- **Tracking trip**: error > `max_tracking_error_deg` **and not closing** for
  `tracking_fault_persist` periods → torque off + latch.
- **Soft limits** clamp the goal (as in position mode) *and* the commanded
  velocity direction against the measured angle.
- Goal Velocity is zeroed **before** torque-on and before torque-off.
- Present Position tracks multi-turn in op mode 1 (e-manual); a jump of more
  than half a motor revolution between periods is unwrapped and logged as an
  error (it should never happen without a power cycle).
- `velocity_mode_profile_accel_rad_s2: 0` writes Profile Acceleration 0 ("no
  profile"); Profile Velocity is ignored by the firmware in this mode.

Verified on a simulated bus (`diagnostics/actuator_sim_test.py`, real nodes,
fake `dynamixel_sdk`): the cascade tracks the full ellipse at **0.4–0.7° worst /
0.2° rms** with a 40 ms first-order servo lag; watchdog, limits, trips and the
startup wrap logic are exercised there too. It has **not** yet run on hardware.

> **Drop hazard** (`redesign_report.md` §0): the mechanism's gravity rest state
> is full extension. In position mode a servo holds by position feedback; in
> velocity mode with Goal Velocity 0 it holds by the velocity loop's integral
> action — a **softer** hold. Whether the cycloid back-drives at 4 N·m output is
> unmeasured. Before trusting velocity mode: hold a mid-stroke pose under the
> real payload, measure the sag, and test whether the platform drops on torque
> disable. A fault or Ctrl-C at mid-stroke means up to 0.29 m of fall unless
> the gearbox friction holds it.

> **Console logging is synchronous.** On a slow terminal the INFO snapshot line
> inside the 50 Hz loop can stall it (2–3° of phantom error was measured on WSL).
> Lengthen or disable `log.load_snapshot_period_s`, or run with
> `--log-level warn`, if the cascade looks jittery on the Pi.

## Control approach (common)
- **Baud fixed at 1,000,000** — the actuator connects directly, no probing on
  launch (`find_servos.py --scan` is the opt-in diagnostic).
- **Calibration = EEPROM Homing Offset** (not a static YAML offset). It applies
  in every operating mode. **Every Homing Offset stored before the gearboxes
  were fitted is stale — re-run `dynamixel_calibrate.py`**, which is now a
  motor-driven jog tool (see "Calibrating with a gearbox" below).
- **Revolution-safety check at startup, torque OFF.** The multi-turn count is
  RAM: on power-up *and* on an Operating Mode change, Present Position is
  re-initialised to the single-turn value (e-manual). At 1:1 the soft-limit
  band is narrower than one revolution, so a 4096-tick wrap has at most one
  in-band candidate and is corrected silently (unchanged behaviour). At 7:1 one
  motor revolution is **51.4° of joint** and the band spans 1.65 motor revs, so
  a raw reading has 2–3 candidates. Policy: 0 candidates → refuse to start;
  1 → accept (warn if wrapped); >1 → accept only if
  a pose hint selects exactly one within `pose_hint_tolerance_deg` (8°), else
  **refuse to start** and list the candidates. Hint sources, in order:
  `startup.expected_pose_deg.<joint>` (explicit, for when you *know* the pose),
  then the **last-pose file** `startup.pose_file`
  (`~/.cache/ldlidar_platform/last_pose.yaml`) that the actuator writes every
  second and on shutdown/fault, and that the calibrator writes at the pose it
  calibrated. The cycloid does not back-drive, so the platform is still where
  it was switched off. The file is trusted only if the motors' EEPROM Homing
  Offsets match the ones it was written under (a re-calibration invalidates
  it). **Any hint must be true**: if something moved the platform without
  updating the file (Dynamixel Wizard, a hard crash mid-motion), pass the
  measured pose in `expected_pose_deg`. The chosen interpretation is logged —
  check it against the crank before the first goal. A stale file 51 ± 8° off
  would pick the *wrong* candidate, hence the tight tolerance. (A homing switch
  would remove the ambiguity entirely — still recommended.)
- Startup also logs the firmware **gains** (read-only; defaults Pos P/I/D
  460/0/0, Vel P/I 100/1920) so the reflected-inertia hypothesis can be
  checked, and the soft-limit envelope in joint deg / motor deg / ticks.

## First-time bring-up (after fitting the gearboxes)

```bash
# 0. (once) add DynamixelSDK to the image and rebuild — see ../../docker/README.md

# 1. (optional) confirm the 4 servos answer on the fixed 1,000,000 baud bus.
ros2 run ldlidar_node find_servos.py --port /dev/ttyUSB0

# 2. Homing / calibration — MANDATORY after re-gearing. Motor-driven: see
#    "Calibrating with a gearbox" below for the procedure.
ros2 run ldlidar_node dynamixel_calibrate.py \
    --params $(ros2 pkg prefix ldlidar_node)/share/ldlidar_node/params/dynamixel_actuator.yaml \
    --planner-params $(ros2 pkg prefix ldlidar_node)/share/ldlidar_node/params/oscillation_planner.yaml
#    -> writes the EEPROM Homing Offsets, ~/dynamixel_offsets.yaml (record) and
#       the last-pose file the actuator uses as its startup hint; copy the
#       printed raw_offsets_ticks into the YAML (reference-only log values).

# 2b. (optional) pre-build the IK LUT (now on the 7:1 joint lattice).
ros2 run ldlidar_node generate_ik_lut.py \
    --params $(ros2 pkg prefix ldlidar_node)/share/ldlidar_node/params/oscillation_planner.yaml \
    --benchmark

# 3. Launch. The startup hint comes from the last-pose file (written by the
#    calibrator / the previous run); pass startup.expected_pose_deg.* only if
#    the platform was moved by something else.
ros2 launch ldlidar_node dynamixel_bringup.launch.py
#    Read the "revolution-safety check" lines: the resolved joint angles must
#    match the crank. If the node refuses to start, it says why.
```

### Calibrating with a gearbox (cannot be moved by hand, large backlash)

The reference pose can no longer be set by hand, and "physical zero" is not
reachable inside the 30..85° range, so `dynamixel_calibrate.py` jogs the joints
under torque and captures the reference from a measurement you can make.

1. **Start the tool** (command above). It sets Extended Position mode, clears
   the Homing Offsets, enables torque *at the current position* (nothing moves)
   and sets a slow jog profile (0.1 rad/s joint). Any jog that raises a
   Present Load above `--load-abort-pct` (45 %) stops every motor where it is.
2. **Sign check** (automatic): each joint is jogged +2° and back; you answer
   whether the crank angle *increased*. If not, `gear.reversing` is wrong (or,
   if only one theta motor misbehaves, that ID's `direction`); fix the YAML and
   rerun. If the theta pair fights, the load abort fires — same diagnosis.
3. **Jog to a mid-stroke pose** with a solid gravity preload, roughly 55–65°
   (`t+5`, `all+2`, `a-1` … `s` shows ticks, load and the last move direction).
   Do not calibrate near full extension: the gravity torque vanishes at the
   toggle and the backlash floats.
4. **`norm`** — backlash normalisation: every joint moves −2° then +2°, so the
   final motion is *lifting*. Gravity always loads the reducer on that flank
   during operation, so a capture after an upward move is consistent with how
   the encoder relates to the crank while working. `cap` refuses if the last
   move on any motor was downward.
5. **Measure.** Put a spirit level on the platform plate (γ = 0) and measure
   the central height **h, pivot line to pivot line** (the kinematic `h`), or
   read the three crank angles with a protractor. Height is the better
   reference: 1 mm of height error is ~0.1° of joint here, and one tape reading
   gives all three angles through the forward kinematics (unique on 0..90°).
6. **`cap h 1.05`** (or `cap h 1.05 g 3` with a known tilt, or `cap deg θ α β`).
   The tool prints the angles it inferred, asks for confirmation, writes and
   verifies each Homing Offset, saves the record, writes the **last-pose file**,
   prints the `raw_offsets_ticks` for the YAML and the 30..85° envelope in
   ticks, and reminds you about the wrap ambiguity.
7. **`q`** (torque off) or `qon` (leave torque on). Then launch the actuator: it
   reads the pose file and resolves the multi-turn wrap from it.

Backlash after calibration: the capture is exact for the loaded flank. When the
ellipse reverses (twice per lap) the reducer crosses its lash; in velocity mode
the per-motor loop shows it as a short error step in `/joint_states`, and in
position mode as the servo overshooting through the lash. The ID1/ID3 vs
ID2/ID4 asymmetry (`diagnostics/asymmetry.py`) now includes four independent
lash amounts — measure it (jog +1/−1 and read the tick difference at zero
load) before tuning gains around it.

### Bench procedure — answer Step 0, then evaluate velocity mode

1. `control.mode: 'position'` (the shipped default). Run the 3 laps. This is
   the old mode with the profile registers now gear-scaled. Record: does the
   wobble survive? Note `/joint_states` vs `/joint_goal` error and the reg[126]
   peaks (now printed as joint N·m).
2. If the wobble survives, read the gains printed at startup; try halving
   Position P Gain(84) (RAM, not EEPROM) and repeat — the reflected inertia
   fell by 49×.
3. `control.mode: 'velocity'`. Same laps. Compare tracking error and
   smoothness. Start with `kp: 6`, `ki: 0`; raise `kp` until it hunts, back off.
4. Hold test: stop the planner mid-lap (watchdog → zero velocity). Measure sag
   over 60 s. Then Ctrl-C (torque off) with hands clear: does it drop?
5. Report which mode wins and whether the cycloid back-drives.

> **Bench safety:** first runs at a low `load_cutoff.joint_torque_nm` (e.g. 3.0)
> and `control.velocity_limit_rad_s: 0.3`. If the θ pair (IDs 1 & 2) fight, one
> `direction` is wrong — fix it in `motors.theta.directions`, not in `gear.*`.

## Laps & feasibility (planner)

Each entry of `lap_durations_s` is one full elliptical loop. At startup every
lap is checked with the **verified virtual-work statics + position-dependent
inertia** (`PlatformDynamics`, a dependency-free port of
`motor_selection/motor_sim_utils.py`, corrected 2026-08-27), joint-side, then
compared through `gear.ratio × gear.efficiency × motor.efficiency` against
`0.9 × stall` (peak), `rated` (rms), no-load speed and the torque–speed
envelope. A lap that fails is **auto-slowed** to the fastest passing duration;
a lap whose *static* holding torque alone fails is refused outright. The old
anchored-ratio heuristic (`TORQUE_PEAK_REF_NM = 1.07`, a fingerprint of the
broken gravity term, with gravity as a fixed 50 %) is gone.

At 7:1 on the 0.92..1.18 m ellipse (`diagnostics/gear_check.py`, motor side,
η 0.88 × 0.9):

| payload | lap | joint peak | motor peak | motor rms | rpm |
|---|---|---|---|---|---|
| 0.5 kg (config.json, field) | 7 s | 1.53 N·m | 0.276 N·m (18 % stall) | 0.213 (26 % rated) | 20.8 (30 %) |
| 2.5 kg (design case) | 7 s | 3.97 N·m | 0.716 N·m (48 % stall) | 0.565 (68 % rated) | 20.8 (30 %) |

(The old 0.65..1.18 m ellipse: 1.79 / 4.31 N·m joint, 33.4 rpm.)

Static hold over the full 0–90° sweep: 1.57 N·m joint (0.5 kg) → 0.283 N·m motor;
4.08 N·m (2.5 kg) → 0.735 N·m = 89 % of rated → **holds any pose** either way.
Set `dynamics.M` in `oscillation_planner.yaml` to the payload actually bolted on.

> The creep phase now waits for `/joint_states` to arrive within
> `start_arrival_tol_deg` (0.75°, tighter than the actuator's `creep_done_deg`)
> before the first lap, with `start_timeout_s` as the fallback — a lap never
> starts while the platform is still on its way.

## Safety (actuator, independent of the planner)

- Per-logical-joint **soft angle limits** (joint side, **30..85°** on all
  three) clamp every goal; in velocity mode also the commanded direction
  against the measured angle. The planner's `h_low` must keep the ellipse
  inside them (`gear_check.py` §3b checks).
- Independent **Present-Load cutoff** (reg 126, 0.1 %/unit — the XC430 has no
  current sensor and no Current Limit register; addr 38 is a reserved gap and
  is no longer written). The trip is set in **joint N·m**
  (`load_cutoff.joint_torque_nm`, 6.0 → 0.974 N·m motor = 65 % load = 649
  units) and converted through the gear model. The legacy motor-side
  `current_limit_ma: 1200` (44.6 % load) is only used if the joint value is 0 —
  at 7:1 it would correspond to ~4.1 N·m joint, *below* the 2.5 kg lap peak.
  `/joint_states.effort` now carries the joint torque in N·m (it used to be a
  fictitious "mA").
- **Torque-disable** on SIGINT/SIGTERM and on any fault (Goal Velocity zeroed
  first in velocity mode). Physically this now hands the platform to gravity
  plus whatever back-drive friction the cycloid has.
- **Watchdog** and **tracking trip** in velocity mode (above).
- **Creep** to the first commanded pose in both modes.
- **Revolution-safety check** (above) — refuses to start on an ambiguous or
  off-band Present Position.
- **Operating-range logging, learned not hardcoded** — the soft-limit envelope
  at startup (joint deg / motor deg / ticks), then the running min/max of the
  angles the planner really commands.

## Still open (not changed here)
- The **ID1/ID3 vs ID2/ID4 load asymmetry** (`diagnostics/asymmetry.py`). Four
  reducers with independent lost motion make the load-sharing question worse,
  not better; the per-motor outer loop in velocity mode will show it directly
  in the reg[126] snapshot. Not addressed by this change.
- Gearbox efficiency (0.88) and back-drive behaviour are assumed, not measured.

## Files

```
dynamixel/
  dynamixel_actuator_node.py    hardware layer: gear model, position / velocity cascade
  oscillation_planner_node.py   ellipse trajectory / LUT IK / laps / verified torque gate
  platform_kinematics.py        forward/inverse kinematics + PlatformDynamics (pure math)
  generate_ik_lut.py            offline IK lookup-table builder (joint-side lattice)
  find_servos.py                baud + ID discovery CLI
  dynamixel_calibrate.py        motor-driven jog + homing capture CLI (gear-aware)
params/
  dynamixel_actuator.yaml       port, baud, mapping, gear, control mode, limits, cutoff
  oscillation_planner.yaml      geometry, ellipse, LUT grid, laps, gear, motor, dynamics
launch/
  dynamixel_actuator.launch.py
  oscillation_planner.launch.py   (params_file:=…)
  dynamixel_bringup.launch.py     (both nodes)
```
