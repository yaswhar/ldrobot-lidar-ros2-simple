#!/usr/bin/env python3
"""
dynamixel_actuator_node.py  --  HARDWARE LAYER ONLY

Drives the 4x DYNAMIXEL XC430-W150-T servos of the parallel-linkage platform,
each through a single-stage 7:1 cycloidal reducer (fitted 2026-09; see
gearbox/cycloid_review.py). The gearbox is modelled here and ONLY here.

This node knows NOTHING about the mechanism, inverse kinematics, oscillation
modes or geometry.  Its only job is:

    * take joint-space position (+ velocity) commands on /joint_goal and drive
      the motors,
    * report present state on /joint_states.

It exposes exactly THREE logical joints -- "theta", "alpha", "beta" -- even
though "theta" is physically TWO motors (IDs 1 and 2).  The fan-out from 1
logical joint -> 2 physical motor IDs happens entirely inside this node and is
invisible to any publisher.

Topic contract (the ONLY coupling to a planner -- a future lidar_planner_node
can reuse this unchanged):
    subscribe  /joint_goal    sensor_msgs/JointState  name=[theta,alpha,beta]
                                position [rad] (joint/output side),
                                velocity [rad/s] (joint side, SIGNED; optional).
                                Position mode uses |velocity| as the firmware
                                profile speed; velocity mode uses it signed as
                                feed-forward.
    publish    /joint_states  sensor_msgs/JointState  name=[theta,alpha,beta]
                                position [rad], velocity [rad/s] (joint side),
                                effort [N.m] = JOINT torque estimated from
                                Present Load x stall x ratio x efficiency.
                                (Before 2026-09 this field carried a fictitious
                                "mA" -- reg 126 is load, not current.)

GEAR MODEL (gear.* parameters; defaults reproduce a gearless build exactly):
    motor_deg  = gear_sign * gear.ratio * joint_deg      gear_sign = -1 if reversing
    ticks      = round(direction * motor_deg * 4096/360) + turn_offset
    motor_vel  = gear.ratio * joint_vel     (same for acceleration)
    joint N.m  = motor N.m * gear.ratio * gear.efficiency
  `direction` stays what dynamixel_calibrate.py documents: the MEASURED raw-tick
  vs angle sense of the motor itself. The gear sign is a separate multiplier so
  the two physical facts are never conflated.

CONTROL MODES (control.mode):
    position  -- Extended Position Control (op mode 4). Every goal writes one
                 12-byte block ProfileAccel+ProfileVel+GoalPosition (all
                 gear-scaled). The servo closes the position loop.
    velocity  -- Velocity Control (op mode 1) with the POSITION loop closed HERE
                 as a cascade: per motor,
                     v_cmd = v_ff + clamp(Kp*(q_des - q_meas) [+ Ki*integral],
                                          +/- catchup_rate)
                 clamped joint-side, then gear-scaled and written to Goal
                 Velocity every control period. The correction term is
                 rate-limited so a goal discontinuity is closed gradually while
                 the feed-forward keeps following the stream (it can never trap
                 the loop behind a moving goal). Requires a goal WATCHDOG (stale
                 goal -> zero velocity), a tracking-error trip that fires only
                 when the error is NOT closing, and a direction-aware soft-limit
                 clamp -- all implemented below. See the "drop hazard" note in
                 _enable_torque.

CONTROL APPROACH (bench-measured, 2026-08):
    * Baud is FIXED at 1,000,000 -- connect directly, no probing on launch.
    * Extended Position (multi-turn) is needed in position mode because ID2's
      homed target goes negative across its real range.
    * Calibration lives in each motor's EEPROM HOMING OFFSET (set by
      dynamixel_calibrate.py at the physical-zero pose), NOT a static YAML
      offset. Homing Offset applies in every operating mode (e-manual).
    * Startup revolution-safety check (see _check_present_positions): the
      multi-turn count is RAM and is re-initialised from the single-turn
      absolute encoder on power-up AND on an Operating Mode change. At 7:1 one
      motor revolution is 51.4 deg of JOINT, so an out-of-band reading is
      resolved from the soft-limit band plus a startup pose hint (explicit
      startup.expected_pose_deg.*, else the last-pose file this node and the
      calibrator maintain -- the cycloid does not back-drive, so the platform
      stays put when switched off), and the node REFUSES TO START when the
      answer is ambiguous.

Safety (this node cannot trust the planner):
    * per-logical-joint soft angle limits enforced on EVERY goal (backstop); in
      velocity mode also as a direction clamp on the measured position,
    * independent Present-Load cutoff (joint-torque-derived) -> torque disable,
    * velocity-mode goal watchdog and tracking-error trip,
    * torque disable on SIGINT/SIGTERM and on any fault,
    * slow "creep to first commanded pose" on the first goal (and on any goal
      discontinuity in velocity mode).
"""

import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

try:
    from dynamixel_sdk import (
        COMM_SUCCESS,
        DXL_HIBYTE,
        DXL_HIWORD,
        DXL_LOBYTE,
        DXL_LOWORD,
        GroupSyncRead,
        GroupSyncWrite,
        PacketHandler,
        PortHandler,
    )
    DXL_SDK_AVAILABLE = True
except ImportError:  # pragma: no cover - handled at runtime with a clear message
    DXL_SDK_AVAILABLE = False


# =============================================================================
# XC430-W150-T CONTROL TABLE  (Protocol 2.0, ROBOTIS X-series)
# -----------------------------------------------------------------------------
# VERIFIED against the official e-manual 2026-08-25 and again 2026-09-13:
#     https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/
# CRITICAL: this is an ENTRY-level X-series motor with NO current sensor. It does
# NOT share the XM/XH current-sensing table. In particular:
#   * addr 126 is PRESENT LOAD (0.1 %/unit, signed, +-1000 == +-100 %), NOT
#     Present Current. There is NO Present Current register on this motor.
#   * there is NO Current Limit register. Addr 38 is a RESERVED GAP here (it is
#     Current Limit only on the XM/XH series). The one firmware output cap is
#     PWM Limit(36). This node therefore never writes addr 38; the effective
#     cutoff is the software Present-Load check in _read_state.
# =============================================================================
ADDR_OPERATING_MODE = 11      # EEPROM, 1 byte
ADDR_HOMING_OFFSET = 20       # EEPROM, 4 bytes (signed) -- calibration zero
ADDR_PWM_LIMIT = 36           # EEPROM, 2 bytes (unit 0.113 %) -- the REAL output cap
ADDR_VELOCITY_LIMIT = 44      # EEPROM, 4 bytes (unit 0.229 rev/min, default 460)
ADDR_TORQUE_ENABLE = 64       # RAM,    1 byte
ADDR_VELOCITY_I_GAIN = 76     # RAM,    2 bytes (default 1920)
ADDR_VELOCITY_P_GAIN = 78     # RAM,    2 bytes (default 100)
ADDR_POSITION_D_GAIN = 80     # RAM,    2 bytes (default 0)
ADDR_POSITION_I_GAIN = 82     # RAM,    2 bytes (default 0)
ADDR_POSITION_P_GAIN = 84     # RAM,    2 bytes (default 460)
ADDR_GOAL_VELOCITY = 104      # RAM,    4 bytes (unit 0.229 rev/min, SIGNED; op mode 1)
ADDR_PROFILE_ACCEL = 108      # RAM,    4 bytes (unit 214.577 rev/min^2; 0 = no profile)
ADDR_PROFILE_VELOCITY = 112   # RAM,    4 bytes (unit 0.229 rev/min; IGNORED in op mode 1)
ADDR_GOAL_POSITION = 116      # RAM,    4 bytes (pulse, SIGNED in extended mode)
ADDR_PRESENT_LOAD = 126       # RAM,    2 bytes (PRESENT LOAD, 0.1 %/unit, signed).
ADDR_PRESENT_VELOCITY = 128   # RAM,    4 bytes (unit 0.229 rev/min, signed)
ADDR_PRESENT_POSITION = 132   # RAM,    4 bytes (pulse, signed multi-turn)

# Position mode write block: ProfileAccel(108..111) ProfileVel(112..115)
# GoalPosition(116..119) = 12 bytes -> one GroupSyncWrite = atomic move.
ADDR_GOAL_BLOCK = ADDR_PROFILE_ACCEL
LEN_GOAL_BLOCK = 12
# Velocity mode write block: GoalVelocity(104..107) = 4 bytes.
LEN_GOAL_VELOCITY = 4

# One contiguous read block: PresentLoad(126..127) PresentVelocity(128..131)
# PresentPosition(132..135) = 10 bytes -> a single GroupSyncRead per cycle.
ADDR_STATE_BLOCK = ADDR_PRESENT_LOAD
LEN_STATE_BLOCK = 10

# X-series Operating Modes (e-manual, verified 2026-09-13):
#   1 = Velocity Control, 3 = Position (single-turn), 4 = Extended Position.
OP_MODE_VELOCITY = 1
OP_MODE_EXTENDED_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# X-series conversion constants
TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV       # 0.088 deg/pulse (MOTOR side)
LOAD_UNIT_PCT = 0.1                         # % per Present Load unit (reg 126)
CURRENT_UNIT_MA = 2.69                      # LEGACY mA/unit (XM/XH only) -- kept
                                            # solely to convert the legacy
                                            # current_limit_ma param into units.
VELOCITY_UNIT_RPM = 0.229                   # rev/min per velocity unit
ACCEL_UNIT_RPM2 = 214.577                   # rev/min^2 per accel unit
RAD_S_TO_RPM = 60.0 / (2.0 * math.pi)

# Register ranges (e-manual): Extended Position +-256 rev, Homing Offset.
EXT_POSITION_RANGE_TICKS = 1_048_575
HOMING_OFFSET_RANGE_TICKS = 1_044_479

# Standard Dynamixel baud set -- used ONLY by the opt-in auto_probe_baud path.
STANDARD_BAUDS = [1000000, 57600, 115200, 9600, 2000000, 3000000, 4000000]

CONTROL_MODES = ('position', 'velocity')

# Last-known joint pose, shared with dynamixel_calibrate.py. The 7:1 cycloid does
# not back-drive, so the platform stays where it was switched off; this file is
# the startup wrap-ambiguity hint. It carries the motors' Homing Offsets as a
# fingerprint: if those changed (re-calibration) the pose is not trusted.
POSE_FILE_DEFAULT = '~/.cache/ldlidar_platform/last_pose.yaml'


def write_pose_file(path, pose_deg, homing_offsets, source):
    """Atomically write {joint: deg} + Homing Offset fingerprint. Best effort."""
    if not path or yaml is None:
        return False
    path = os.path.expanduser(path)
    doc = {'written': time.strftime('%Y-%m-%dT%H:%M:%S'),
           'source': source,
           'pose_deg': {k: round(float(v), 3) for k, v in pose_deg.items()},
           'homing_offset_ticks': {int(k): int(v) for k, v in homing_offsets.items()}}
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w') as fh:
            yaml.safe_dump(doc, fh, default_flow_style=False)
        os.replace(tmp, path)
        return True
    except OSError:
        return False


def read_pose_file(path):
    """Return the parsed pose record or None."""
    if not path or yaml is None:
        return None
    path = os.path.expanduser(path)
    try:
        with open(path, 'r') as fh:
            doc = yaml.safe_load(fh)
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(doc, dict) or 'pose_deg' not in doc:
        return None
    return doc


def _to_signed(value, bits):
    """Interpret an unsigned SDK read as a two's-complement signed integer."""
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


def _le4(value):
    """Split a (possibly negative) 4-byte value into little-endian bytes."""
    value = int(value) & 0xFFFFFFFF
    return [
        DXL_LOBYTE(DXL_LOWORD(value)),
        DXL_HIBYTE(DXL_LOWORD(value)),
        DXL_LOBYTE(DXL_HIWORD(value)),
        DXL_HIBYTE(DXL_HIWORD(value)),
    ]


class GearModel:
    """Reducer between motor shaft and joint. ratio=1, reversing=False,
    efficiency=1 is a gearless build and reproduces the pre-gearbox maths."""

    def __init__(self, ratio=1.0, reversing=False, efficiency=1.0):
        self.ratio = float(ratio)
        self.reversing = bool(reversing)
        self.efficiency = float(efficiency)
        if self.ratio <= 0.0:
            raise ValueError(f'gear.ratio must be > 0 (got {self.ratio})')
        if not (0.0 < self.efficiency <= 1.0):
            raise ValueError(f'gear.efficiency must be in (0, 1] (got {self.efficiency})')
        self.sign = -1 if self.reversing else 1

    # --- angle / rate -------------------------------------------------------
    def joint_to_motor_deg(self, joint_deg):
        return self.sign * self.ratio * joint_deg

    def motor_to_joint_deg(self, motor_deg):
        return motor_deg / (self.sign * self.ratio)

    def joint_to_motor_rate(self, joint_rate):
        """Signed rate (rad/s or rad/s^2) joint -> motor shaft."""
        return self.sign * self.ratio * joint_rate

    def motor_to_joint_rate(self, motor_rate):
        return motor_rate / (self.sign * self.ratio)

    # --- torque -------------------------------------------------------------
    def motor_to_joint_torque(self, motor_nm):
        return motor_nm * self.ratio * self.efficiency

    def joint_to_motor_torque(self, joint_nm):
        return joint_nm / (self.ratio * self.efficiency)

    @property
    def joint_deg_per_tick(self):
        return DEG_PER_TICK / self.ratio

    def describe(self):
        return (f'ratio {self.ratio:g}:1, {"REVERSING" if self.reversing else "same-sense"}, '
                f'eta {self.efficiency:g}; joint lattice {self.joint_deg_per_tick:.5f} deg/tick')


class PhysicalMotor:
    """One physical servo of a logical joint.

    `direction` is the MEASURED raw-tick-vs-angle sense of the motor itself
    (all four are -1 on the bench: raw ticks decrease as the motor's own output
    angle increases). It is a calibration fact about the servo and must not
    absorb the gearbox sign -- that lives in GearModel.sign.

    Calibration zero lives in the motor's EEPROM Homing Offset, so the tick
    formula collapses to direction * motor_deg (+ revolution correction).
    `turn_offset` is a startup-only revolution-safety correction (normally 0),
    also used to unwrap a (never expected) 4096-tick jump at runtime.
    `raw_offset` is a REFERENCE-ONLY bench value (raw ticks at physical 0 deg)
    used solely for the startup log; it must be re-measured after re-gearing.
    """

    def __init__(self, motor_id, direction, gear, raw_offset=0):
        self.id = int(motor_id)
        self.direction = 1 if direction >= 0 else -1
        self.gear = gear
        self.raw_offset = int(raw_offset)   # reference only (startup log)
        self.turn_offset = 0                # +/- k*4096, set by startup check
        # velocity-mode outer-loop state
        self.integral = 0.0                 # rad
        self.last_ticks = None              # for runtime wrap detection
        self.q_meas_deg = None              # last measured JOINT angle

    # joint deg <-> homed ticks (the runtime formula)
    def angle_deg_to_ticks(self, joint_deg):
        motor_deg = self.gear.joint_to_motor_deg(joint_deg)
        return int(round(self.direction * motor_deg / DEG_PER_TICK)) + self.turn_offset

    def ticks_to_angle_deg(self, ticks):
        motor_deg = self.direction * (ticks - self.turn_offset) * DEG_PER_TICK
        return self.gear.motor_to_joint_deg(motor_deg)

    # joint rad/s <-> motor velocity register units (signed)
    def joint_rate_to_units(self, joint_rad_s):
        motor_rad_s = self.gear.joint_to_motor_rate(joint_rad_s)
        rpm = self.direction * motor_rad_s * RAD_S_TO_RPM
        return int(round(rpm / VELOCITY_UNIT_RPM))

    def units_to_joint_rate(self, units):
        motor_rad_s = self.direction * units * VELOCITY_UNIT_RPM / RAD_S_TO_RPM
        return self.gear.motor_to_joint_rate(motor_rad_s)

    # raw ticks (Homing Offset = 0) for the startup log
    def raw_ticks_at(self, joint_deg):
        motor_deg = self.gear.joint_to_motor_deg(joint_deg)
        return int(round(self.raw_offset + self.direction * motor_deg / DEG_PER_TICK))


class LogicalJoint:
    """A named joint ("theta"/"alpha"/"beta") backed by 1 or 2 physical motors.

    `obs_lo`/`obs_hi` are the OBSERVED operating envelope: the running min/max of
    the angles actually commanded on /joint_goal. They start empty (None) and are
    filled in as goals arrive, so the "expected operating range" is learned from
    whatever planner is connected rather than hand-maintained in YAML -- a future
    lidar_planner_node needs no config change here. The genuine hardware bound is
    min_deg/max_deg (the soft-limit safety backstop), which never goes stale.
    """

    def __init__(self, name, motors, min_deg, max_deg):
        self.name = name
        self.motors = motors          # list[PhysicalMotor]
        self.min_deg = float(min_deg)
        self.max_deg = float(max_deg)
        self.obs_lo = None            # observed min commanded angle (deg)
        self.obs_hi = None            # observed max commanded angle (deg)
        # velocity-mode goal (joint side)
        self.q_des_deg = None
        self.v_ff_rad_s = 0.0

    def clamp_deg(self, angle_deg):
        return max(self.min_deg, min(self.max_deg, angle_deg))

    def observe(self, angle_deg):
        """Fold a commanded angle into the observed envelope. Returns True if the
        envelope expanded (so the caller can re-log the operating range)."""
        expanded = False
        if self.obs_lo is None or angle_deg < self.obs_lo:
            self.obs_lo = angle_deg
            expanded = True
        if self.obs_hi is None or angle_deg > self.obs_hi:
            self.obs_hi = angle_deg
            expanded = True
        return expanded


class DynamixelActuatorNode(Node):
    def __init__(self):
        super().__init__('dynamixel_actuator')

        # ---- parameters -----------------------------------------------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)      # FIXED 1M (bench-measured)
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('auto_probe_baud', False)  # do NOT probe on launch

        # gear model (defaults = gearless build)
        self.declare_parameter('gear.ratio', 1.0)
        self.declare_parameter('gear.reversing', False)
        self.declare_parameter('gear.efficiency', 1.0)
        # motor datum used only for load <-> torque conversion
        self.declare_parameter('motor.stall_torque_nm', 1.5)

        # logical -> physical motor mapping (config; logged at startup).
        # All four motors resolve to the SAME direction (-1): raw ticks decrease
        # as the motor's own output angle increases (gearbox sign is separate).
        self.declare_parameter('motors.theta.ids', [1, 2])
        self.declare_parameter('motors.theta.directions', [-1, -1])
        self.declare_parameter('motors.theta.raw_offsets_ticks', [2048, 618])
        self.declare_parameter('motors.alpha.ids', [3])
        self.declare_parameter('motors.alpha.directions', [-1])
        self.declare_parameter('motors.alpha.raw_offsets_ticks', [1024])
        self.declare_parameter('motors.beta.ids', [4])
        self.declare_parameter('motors.beta.directions', [-1])
        self.declare_parameter('motors.beta.raw_offsets_ticks', [3072])

        # per-logical-joint soft limits (JOINT deg) -- SAFETY BACKSTOP
        # (30..85 on every joint since 2026-09-16; the YAML is authoritative)
        self.declare_parameter('limits.theta.min_deg', 30.0)
        self.declare_parameter('limits.theta.max_deg', 85.0)
        self.declare_parameter('limits.alpha.min_deg', 30.0)
        self.declare_parameter('limits.alpha.max_deg', 85.0)
        self.declare_parameter('limits.beta.min_deg', 30.0)
        self.declare_parameter('limits.beta.max_deg', 85.0)

        # NOTE: there is deliberately NO sanity_range_deg parameter. The expected
        # operating range is LEARNED from the live /joint_goal stream (see
        # LogicalJoint.observe + _log_observed_range), not hand-maintained here.

        # startup revolution-safety check (joint-side terms)
        self.declare_parameter('startup.plausible_margin_deg', 22.5)  # = 256 ticks at 1:1
        self.declare_parameter('startup.expected_pose_deg.theta', float('nan'))
        self.declare_parameter('startup.expected_pose_deg.alpha', float('nan'))
        self.declare_parameter('startup.expected_pose_deg.beta', float('nan'))
        self.declare_parameter('startup.pose_file', POSE_FILE_DEFAULT)
        self.declare_parameter('startup.pose_hint_tolerance_deg', 8.0)

        # motion defaults / profiles (JOINT side; gear-scaled before writing)
        self.declare_parameter('default_profile_velocity_rad_s', 0.6)
        self.declare_parameter('profile_acceleration_rad_s2', 3.0)
        self.declare_parameter('creep_profile_velocity_rad_s', 0.15)

        # control mode + velocity-cascade tuning (all joint side)
        self.declare_parameter('control.mode', 'position')
        self.declare_parameter('control.rate_hz', 50.0)
        self.declare_parameter('control.kp', 6.0)                  # 1/s
        self.declare_parameter('control.ki', 0.0)                  # 1/s^2
        self.declare_parameter('control.integral_limit_rad_s', 0.2)
        self.declare_parameter('control.deadband_deg', 0.15)
        self.declare_parameter('control.velocity_limit_rad_s', 1.0)
        self.declare_parameter('control.watchdog_timeout_s', 0.1)
        self.declare_parameter('control.catchup_rate_rad_s', 0.3)  # cap on the P/I correction
        self.declare_parameter('control.max_tracking_error_deg', 12.0)
        self.declare_parameter('control.tracking_fault_persist', 5)
        self.declare_parameter('control.goal_jump_warn_deg', 5.0)  # log only
        self.declare_parameter('control.creep_done_deg', 1.0)
        self.declare_parameter('control.creep_timeout_s', 40.0)
        self.declare_parameter('control.velocity_mode_profile_accel_rad_s2', 0.0)

        # safety: Present-Load cutoff.  load_cutoff.joint_torque_nm > 0 wins;
        # otherwise the legacy motor-side current_limit_ma is used unchanged.
        self.declare_parameter('load_cutoff.joint_torque_nm', 0.0)
        self.declare_parameter('current_limit_ma', 1200.0)
        self.declare_parameter('current_fault_persist', 5)

        # timing / topics / logging
        self.declare_parameter('state_publish_rate_hz', 20.0)
        self.declare_parameter('log.load_snapshot_period_s', 0.5)   # 0 = off
        self.declare_parameter('goal_topic', '/joint_goal')
        self.declare_parameter('state_topic', '/joint_states')

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.port_name = gp('port')
        self.baudrate = int(gp('baudrate'))
        self.protocol_version = float(gp('protocol_version'))
        self.auto_probe_baud = bool(gp('auto_probe_baud'))
        self.gear = GearModel(gp('gear.ratio'), gp('gear.reversing'), gp('gear.efficiency'))
        self.stall_torque_nm = float(gp('motor.stall_torque_nm'))
        self.default_profile_velocity = float(gp('default_profile_velocity_rad_s'))
        self.profile_acceleration = float(gp('profile_acceleration_rad_s2'))
        self.creep_profile_velocity = float(gp('creep_profile_velocity_rad_s'))
        self.plausible_margin_deg = float(gp('startup.plausible_margin_deg'))
        self.pose_hint_tol_deg = float(gp('startup.pose_hint_tolerance_deg'))
        self.pose_hint_deg = {n: float(gp(f'startup.expected_pose_deg.{n}'))
                              for n in ('theta', 'alpha', 'beta')}
        self.pose_file = str(gp('startup.pose_file') or '')
        self._homing_offsets = {}          # id -> EEPROM Homing Offset (read at startup)
        self._pose_file_last = None        # last pose written (deg) / time

        self.mode = str(gp('control.mode')).strip().lower()
        if self.mode not in CONTROL_MODES:
            raise ValueError(f"control.mode must be one of {CONTROL_MODES}, got '{self.mode}'")
        self.control_rate = float(gp('control.rate_hz'))
        self.kp = float(gp('control.kp'))
        self.ki = float(gp('control.ki'))
        self.integral_limit = float(gp('control.integral_limit_rad_s'))
        self.deadband_rad = math.radians(float(gp('control.deadband_deg')))
        self.velocity_limit = float(gp('control.velocity_limit_rad_s'))
        self.watchdog_timeout = float(gp('control.watchdog_timeout_s'))
        self.catchup_rate = float(gp('control.catchup_rate_rad_s'))
        self.max_tracking_error_rad = math.radians(float(gp('control.max_tracking_error_deg')))
        self.tracking_fault_persist = int(gp('control.tracking_fault_persist'))
        self.goal_jump_warn_deg = float(gp('control.goal_jump_warn_deg'))
        self.creep_done_deg = float(gp('control.creep_done_deg'))
        self.creep_timeout = float(gp('control.creep_timeout_s'))
        self.vel_mode_profile_accel = float(gp('control.velocity_mode_profile_accel_rad_s2'))

        self.load_cutoff_joint_nm = float(gp('load_cutoff.joint_torque_nm'))
        self.current_limit_ma = float(gp('current_limit_ma'))
        self.current_fault_persist = int(gp('current_fault_persist'))
        self.state_publish_rate = float(gp('state_publish_rate_hz'))
        self.load_log_period = float(gp('log.load_snapshot_period_s'))
        self.goal_topic = gp('goal_topic')
        self.state_topic = gp('state_topic')

        # Present-Load trip threshold in register units (0.1 %); 100 % ~ stall.
        if self.load_cutoff_joint_nm > 0.0:
            motor_nm = self.gear.joint_to_motor_torque(self.load_cutoff_joint_nm)
            self.load_trip_units = int(round(motor_nm / self.stall_torque_nm * 1000.0))
            self.load_trip_source = (f'load_cutoff.joint_torque_nm={self.load_cutoff_joint_nm:.2f} '
                                     f'-> motor {motor_nm:.3f} N.m')
        else:
            self.load_trip_units = int(round(self.current_limit_ma / CURRENT_UNIT_MA))
            self.load_trip_source = (f'LEGACY current_limit_ma={self.current_limit_ma:.0f} '
                                     f'(motor-side; set load_cutoff.joint_torque_nm instead)')
        self.load_trip_units = max(1, min(1000, self.load_trip_units))

        # ---- build logical joint map ---------------------------------------
        self.joints = self._build_joint_map()
        self.joint_names = ['theta', 'alpha', 'beta']
        self.all_motors = [m for jn in self.joint_names for m in self.joints[jn].motors]

        # ---- runtime state --------------------------------------------------
        self.first_goal_received = False   # first goal -> creep behaviour
        self._first_goal_deg = None        # position mode: creep while goal == first pose
        self.fault_latched = False
        self._current_over_count = {m.id: 0 for m in self.all_motors}
        self._tracking_over_count = {m.id: 0 for m in self.all_motors}
        self._last_abs_err = {m.id: 0.0 for m in self.all_motors}
        self._reg_peak = {m.id: 0 for m in self.all_motors}  # peak |reg[126]| seen
        self._observed_dirty = False       # observed envelope grew -> re-log
        # velocity mode
        self._last_goal_time = None        # monotonic seconds
        self._creeping = False
        self._creep_started = None
        self._watchdog_tripped = False
        self._vel_written = None           # last goal-velocity units per id

        # These are pure calculation -- log them BEFORE touching hardware so
        # the operator can eyeball the mapping/limits even if the bus is down.
        self._log_mapping()
        self._log_softlimit_envelope()

        if not DXL_SDK_AVAILABLE:
            self.get_logger().fatal(
                'dynamixel_sdk is not installed in this environment. '
                'Add it to the ros2_lidar_image (see docker/Dockerfile.dynamixel).')
            raise RuntimeError('dynamixel_sdk missing')

        # ---- hardware bring-up ---------------------------------------------
        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(self.protocol_version)
        self._connect()
        self._configure_motors()          # operating mode only; torque stays OFF

        # ---- GroupSync handles ---------------------------------------------
        if self.mode == 'position':
            self.group_write = GroupSyncWrite(
                self.port, self.packet, ADDR_GOAL_BLOCK, LEN_GOAL_BLOCK)
        else:
            self.group_write = GroupSyncWrite(
                self.port, self.packet, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)
        self.group_read = GroupSyncRead(
            self.port, self.packet, ADDR_STATE_BLOCK, LEN_STATE_BLOCK)
        for m in self.all_motors:
            if not self.group_read.addParam(m.id):
                self.get_logger().error(f'GroupSyncRead addParam failed for ID {m.id}')

        # ---- revolution-safety check (before ANY motion, torque still OFF) --
        self._read_homing_offsets()
        self._apply_pose_file_hint()
        self._check_present_positions()
        self._log_gains()

        # ---- torque on (velocity mode: Goal Velocity zeroed first) ----------
        self._enable_torque()

        # ---- ROS interfaces -------------------------------------------------
        self.state_pub = self.create_publisher(JointState, self.state_topic, 10)
        self.goal_sub = self.create_subscription(
            JointState, self.goal_topic, self._on_goal, 10)
        if self.mode == 'position':
            self.state_timer = self.create_timer(
                1.0 / max(1.0, self.state_publish_rate), self._position_mode_tick)
        else:
            self._dt = 1.0 / max(1.0, self.control_rate)
            self.state_timer = self.create_timer(self._dt, self._velocity_mode_tick)
        # slow timer: re-log the observed operating range whenever it expands
        self.range_log_timer = self.create_timer(2.0, self._log_observed_range)
        # remember where the platform is (the cycloid holds it there when off)
        if self.pose_file:
            self.pose_file_timer = self.create_timer(1.0, self._save_pose_file)

        self.get_logger().info(
            f'dynamixel_actuator ready in {self.mode.upper()} mode. Listening on '
            f'{self.goal_topic}, publishing {self.state_topic}. First goal will CREEP '
            f'({self.creep_profile_velocity:.2f} rad/s joint) to the commanded pose.')

    # ------------------------------------------------------------------ config
    def _build_joint_map(self):
        joints = {}
        for name in ('theta', 'alpha', 'beta'):
            ids = list(self.get_parameter(f'motors.{name}.ids').value)
            dirs = list(self.get_parameter(f'motors.{name}.directions').value)
            raws = list(self.get_parameter(f'motors.{name}.raw_offsets_ticks').value)
            if not (len(ids) == len(dirs) == len(raws)):
                raise ValueError(
                    f'motors.{name}: ids/directions/raw_offsets_ticks length mismatch')
            motors = [PhysicalMotor(mid, d, self.gear, raw)
                      for mid, d, raw in zip(ids, dirs, raws)]
            min_deg = float(self.get_parameter(f'limits.{name}.min_deg').value)
            max_deg = float(self.get_parameter(f'limits.{name}.max_deg').value)
            if min_deg >= max_deg:
                raise ValueError(f'limits.{name}: min_deg must be < max_deg')
            joints[name] = LogicalJoint(name, motors, min_deg, max_deg)
        return joints

    def _log_mapping(self):
        lg = self.get_logger()
        lg.info('=== logical -> physical motor mapping ===')
        lg.info(f'  gear: {self.gear.describe()}')
        lg.info(f'  control mode: {self.mode}')
        for name in self.joint_names:
            j = self.joints[name]
            desc = ', '.join(f'ID{m.id}(dir{m.direction:+d})' for m in j.motors)
            lg.info(f'  {name:6s} -> [{desc}]  soft limits [{j.min_deg:.1f}, '
                    f'{j.max_deg:.1f}] JOINT deg')
        lg.info('  calibration zero = EEPROM Homing Offset (run dynamixel_calibrate.py); '
                f'goal_ticks = round(direction * {self.gear.sign:+d} * {self.gear.ratio:g} '
                '* joint_deg * 4096/360)')
        lg.info(f'  Present-Load cutoff: {self.load_trip_units} units = '
                f'{self.load_trip_units * LOAD_UNIT_PCT:.1f}% load = '
                f'{self.load_trip_units / 1000 * self.stall_torque_nm:.3f} N.m motor = '
                f'{self.gear.motor_to_joint_torque(self.load_trip_units / 1000 * self.stall_torque_nm):.2f} '
                f'N.m JOINT  [{self.load_trip_source}]')
        if self.mode == 'velocity':
            lg.info(f'  cascade: kp={self.kp:g}/s ki={self.ki:g}/s^2 deadband '
                    f'{math.degrees(self.deadband_rad):.2f} deg, correction cap '
                    f'{self.catchup_rate:g} rad/s, v_limit {self.velocity_limit:g} rad/s joint '
                    f'(= {self.velocity_limit * self.gear.ratio * RAD_S_TO_RPM:.1f} rpm motor), '
                    f'watchdog {self.watchdog_timeout * 1000:.0f} ms, tracking trip '
                    f'{math.degrees(self.max_tracking_error_rad):.1f} deg (not closing), loop '
                    f'{self.control_rate:.0f} Hz')
        lg.info('=========================================')

    def _log_joint_ticks(self, joint, lo, hi):
        """Log one joint's [lo, hi] JOINT deg as motor deg + raw/homed tick ranges."""
        for m in joint.motors:
            raw_lo, raw_hi = m.raw_ticks_at(lo), m.raw_ticks_at(hi)
            saved = m.turn_offset
            m.turn_offset = 0
            homed_lo, homed_hi = m.angle_deg_to_ticks(lo), m.angle_deg_to_ticks(hi)
            m.turn_offset = saved
            self.get_logger().info(
                f'  ID{m.id} ({joint.name}): joint {lo:.1f}..{hi:.1f} deg = motor '
                f'{self.gear.joint_to_motor_deg(lo):.0f}..{self.gear.joint_to_motor_deg(hi):.0f} '
                f'deg -> raw {raw_lo}..{raw_hi} ticks | homed present {homed_lo}..{homed_hi}'
                + (f' (+turn_offset {saved})' if saved else ''))

    def _log_softlimit_envelope(self):
        """Log the SOFT-LIMIT envelope (joint deg, motor deg, raw + homed ticks)
        so the operator can eyeball a worst-case bound BEFORE power is applied."""
        lg = self.get_logger()
        lg.info('=== soft-limit envelope (eyeball before power) ===')
        worst = 0
        for name in self.joint_names:
            j = self.joints[name]
            self._log_joint_ticks(j, j.min_deg, j.max_deg)
            for m in j.motors:
                saved = m.turn_offset
                m.turn_offset = 0
                worst = max(worst, abs(m.angle_deg_to_ticks(j.min_deg)),
                            abs(m.angle_deg_to_ticks(j.max_deg)))
                m.turn_offset = saved
        lg.info(f'  worst |homed ticks| {worst} vs Extended Position range '
                f'+/-{EXT_POSITION_RANGE_TICKS:,} and Homing Offset range '
                f'+/-{HOMING_OFFSET_RANGE_TICKS:,}: '
                f'{"OK" if worst < HOMING_OFFSET_RANGE_TICKS else "OUT OF RANGE"}')
        lg.info('  worst-case bound; actual operating range is logged from the live '
                '/joint_goal stream (learned from the planner, not hardcoded).')
        lg.info('==================================================')

    def _log_observed_range(self):
        """Log the operating range LEARNED from /joint_goal so far, but only when
        it has grown since the last log. Runs on a slow timer -- not in the goal
        hot path."""
        if not self._observed_dirty:
            return
        if any(self.joints[n].obs_lo is None for n in self.joint_names):
            return  # wait until every logical joint has been commanded at least once
        self._observed_dirty = False
        self.get_logger().info('=== observed operating range (from /joint_goal) ===')
        for name in self.joint_names:
            j = self.joints[name]
            self._log_joint_ticks(j, j.obs_lo, j.obs_hi)
        self.get_logger().info('==================================================')

    # ------------------------------------------------------------- bring-up
    def _connect(self):
        try:
            opened = self.port.openPort()
        except OSError as exc:
            raise RuntimeError(
                f'Cannot open serial port {self.port_name}: {exc}. Check the '
                f'device path (prefer /dev/serial/by-id/...) and permissions.')
        if not opened:
            raise RuntimeError(
                f'Failed to open port {self.port_name} (in use or wrong path?).')
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f'Failed to set baudrate {self.baudrate}')

        expected_ids = [m.id for m in self.all_motors]
        if self._ping_all(expected_ids):
            self.get_logger().info(
                f'All motors {expected_ids} responded at {self.baudrate} baud on '
                f'{self.port_name}.')
            return

        if not self.auto_probe_baud:
            raise RuntimeError(
                f'Motors {expected_ids} did not all respond at {self.baudrate} baud. '
                f'Bus is fixed at 1,000,000 -- check wiring/power, or run '
                f'find_servos.py --scan to diagnose.')

        self.get_logger().warn(
            f'Not all motors responded at {self.baudrate} baud -- probing standard '
            f'set (auto_probe_baud=true). Normally this is disabled.')
        for baud in STANDARD_BAUDS:
            if baud == self.baudrate:
                continue
            if not self.port.setBaudRate(baud):
                continue
            if self._ping_all(expected_ids):
                self.baudrate = baud
                self.get_logger().warn(
                    f'Motors found at {baud} baud. PIN baudrate: {baud} into the '
                    f'actuator params YAML.')
                return
        raise RuntimeError(
            f'Could not find all motors {expected_ids} at any standard baud on '
            f'{self.port_name}. Run find_servos.py --scan.')

    def _ping_all(self, ids):
        ok = True
        for mid in ids:
            _model, comm, err = self.packet.ping(self.port, mid)
            if comm != COMM_SUCCESS or err != 0:
                ok = False
        return ok

    def _configure_motors(self):
        """Set the Operating Mode for the selected control mode. Read-before-write
        to spare EEPROM. NEVER touches Homing Offset (that IS the calibration).
        Leaves torque OFF: the revolution-safety check must run first, because a
        mode change resets Present Position to its single-turn value (e-manual).

        Addr 38 is a RESERVED GAP on the XC430 (no Current Limit register) -- it
        is deliberately NOT written. The only firmware output cap is PWM Limit(36);
        the effective cutoff is the software Present-Load check in _read_state."""
        want = OP_MODE_VELOCITY if self.mode == 'velocity' else OP_MODE_EXTENDED_POSITION
        changed = []
        for m in self.all_motors:
            mode = self._read1(m.id, ADDR_OPERATING_MODE)
            if mode != want:
                self._write1(m.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)  # EEPROM needs torque off
                self._write1(m.id, ADDR_OPERATING_MODE, want)
                changed.append(m.id)
        if changed:
            self.get_logger().warn(
                f'Operating Mode written to {want} on IDs {changed} (EEPROM). Present '
                f'Position of those motors is now re-initialised to single-turn.')
        name = 'VELOCITY Control' if want == OP_MODE_VELOCITY else 'EXTENDED Position Control'
        self.get_logger().info(
            f'Motors in {name} Mode (op mode {want}). NOTE: addr 38 is a reserved gap '
            f'on the XC430 (no Current Limit register) -- no firmware current limit '
            f'exists; the active cutoff is the software Present-Load check.')
        if self.mode == 'velocity':
            # Velocity Limit(44) bounds Goal Velocity(104); clamp our joint-side
            # limit to it and log both. EEPROM -> read only, never written here.
            lim_units = min(self._read4(m.id, ADDR_VELOCITY_LIMIT) for m in self.all_motors)
            lim_joint = abs(self.all_motors[0].units_to_joint_rate(lim_units))
            self.get_logger().info(
                f'  firmware Velocity Limit(44) = {lim_units} units = '
                f'{lim_units * VELOCITY_UNIT_RPM:.1f} rpm motor = {lim_joint:.2f} rad/s joint')
            if self.velocity_limit > lim_joint:
                self.get_logger().warn(
                    f'  control.velocity_limit_rad_s {self.velocity_limit:g} exceeds the '
                    f'firmware limit -> clamped to {lim_joint:.2f} rad/s joint.')
                self.velocity_limit = lim_joint

    def _log_gains(self):
        """Report the firmware loop gains (read-only). With the 7:1 reducer the
        motor sees the crank inertia / 49, so the position gains that were used
        direct-drive are a candidate for hunting (kickoff section 2). Defaults:
        Pos P/I/D 460/0/0, Vel P/I 100/1920."""
        lg = self.get_logger()
        lg.info('=== firmware gains (read-only; defaults P/I/D 460/0/0, vel P/I 100/1920) ===')
        for m in self.all_motors:
            pp = self._read2(m.id, ADDR_POSITION_P_GAIN)
            pi = self._read2(m.id, ADDR_POSITION_I_GAIN)
            pd = self._read2(m.id, ADDR_POSITION_D_GAIN)
            vp = self._read2(m.id, ADDR_VELOCITY_P_GAIN)
            vi = self._read2(m.id, ADDR_VELOCITY_I_GAIN)
            lg.info(f'  ID{m.id}: position P/I/D = {pp}/{pi}/{pd}   velocity P/I = {vp}/{vi}')
        lg.info('=============================================================================')

    def _read_homing_offsets(self):
        for m in self.all_motors:
            self._homing_offsets[m.id] = _to_signed(self._read4(m.id, ADDR_HOMING_OFFSET), 32)
        self.get_logger().info(
            '  EEPROM Homing Offsets: '
            + ', '.join(f'ID{i}={v}' for i, v in self._homing_offsets.items()))

    def _apply_pose_file_hint(self):
        """Fill NaN entries of startup.expected_pose_deg from the last-pose file,
        but only if it was written under the Homing Offsets the motors have NOW
        (a re-calibration in between makes the recorded pose meaningless)."""
        missing = [n for n in self.joint_names if not math.isfinite(self.pose_hint_deg[n])]
        if not missing or not self.pose_file:
            return
        rec = read_pose_file(self.pose_file)
        lg = self.get_logger()
        if rec is None:
            lg.warn(f'  no usable pose file at {self.pose_file}; no startup hint for '
                    f'{missing} (set startup.expected_pose_deg.* if the check is ambiguous).')
            return
        fp = {int(k): int(v) for k, v in (rec.get('homing_offset_ticks') or {}).items()}
        if any(fp.get(m.id) != self._homing_offsets.get(m.id) for m in self.all_motors):
            lg.warn(f'  pose file {self.pose_file} was written under different Homing '
                    f'Offsets ({fp} vs now {self._homing_offsets}) -> IGNORED. Re-run the '
                    f'calibrator (it rewrites the file) or set startup.expected_pose_deg.*.')
            return
        pose = rec.get('pose_deg') or {}
        used = {}
        for n in missing:
            if n in pose and math.isfinite(float(pose[n])):
                self.pose_hint_deg[n] = float(pose[n])
                used[n] = self.pose_hint_deg[n]
        if used:
            lg.warn(f'  startup pose hint from {self.pose_file} (written '
                    f'{rec.get("written")}, by {rec.get("source")}): {used}. This assumes '
                    f'the platform has NOT moved since -- if it has, stop and set '
                    f'startup.expected_pose_deg.* to the measured pose.')

    def _current_pose_deg(self):
        pose = {}
        for n in self.joint_names:
            qs = [m.q_meas_deg for m in self.joints[n].motors if m.q_meas_deg is not None]
            if qs:
                pose[n] = sum(qs) / len(qs)
        return pose

    def _save_pose_file(self, force=False):
        if not self.pose_file:
            return
        pose = self._current_pose_deg()
        if len(pose) != len(self.joint_names):
            return
        now = time.monotonic()
        if not force and self._pose_file_last is not None:
            last_pose, last_t = self._pose_file_last
            moved = max(abs(pose[n] - last_pose[n]) for n in pose)
            if moved < 0.05 and now - last_t < 10.0:
                return
        if write_pose_file(self.pose_file, pose, self._homing_offsets, 'dynamixel_actuator'):
            self._pose_file_last = (pose, now)

    def _check_present_positions(self):
        """Startup revolution-safety check -- runs with torque OFF, before motion.

        WHY: Extended Position / Velocity mode keep a multi-turn count in RAM
        that is re-initialised from the single-turn absolute encoder on power-up
        and on an Operating Mode change (e-manual). So on boot a motor's Present
        Position is only known modulo 4096 ticks = one MOTOR revolution. With a
        gearless build that is one JOINT revolution, and the joint's soft-limit
        band is narrower than that, so at most one wrap candidate ever lands in
        band and it can be corrected silently (the pre-gearbox behaviour).

        With a reducer, one motor revolution is 360/ratio deg of JOINT (51.4 deg
        at 7:1) while the band spans ratio * (max-min)/360 motor revs (1.65 at
        7:1), so a raw reading can have SEVERAL in-band candidates. Auto-picking
        one would command a pose up to 51 deg from where the operator believes
        the platform is. Policy:
            0 candidates -> refuse to start (reading is off the band: lost
                            calibration, wrong gear params, or moved past range),
            1 candidate  -> accept; warn if it needed a wrap (turn_offset != 0),
            >1 candidates -> accept ONLY if startup.expected_pose_deg.<joint>
                            selects exactly one within pose_hint_tolerance_deg;
                            otherwise refuse to start and list the candidates.
        A true joint revolution (4096*ratio ticks) can never happen on this
        linkage, so it is not treated specially -- it simply falls under the
        candidate enumeration above. The margin is expressed in JOINT degrees
        (22.5 deg = the old 256 ticks at 1:1) so it does not shrink with ratio.
        """
        comm = self.group_read.txRxPacket()
        if comm != COMM_SUCCESS:
            raise RuntimeError(
                'Could not read Present Position for the revolution-safety check '
                f'({self.packet.getTxRxResult(comm)}). Refusing to start.')
        lg = self.get_logger()
        lg.info('=== revolution-safety check (Present Position, torque OFF) ===')
        errors = []
        for name in self.joint_names:
            j = self.joints[name]
            hint = self.pose_hint_deg.get(name, float('nan'))
            for m in j.motors:
                m.turn_offset = 0
                band_a = m.angle_deg_to_ticks(j.min_deg - self.plausible_margin_deg)
                band_b = m.angle_deg_to_ticks(j.max_deg + self.plausible_margin_deg)
                lo, hi = min(band_a, band_b), max(band_a, band_b)
                raw = _to_signed(self.group_read.getData(m.id, ADDR_PRESENT_POSITION, 4), 32)
                kmax = int(math.ceil((hi - lo) / TICKS_PER_REV)) + 2
                k0 = int(round((raw - 0.5 * (lo + hi)) / TICKS_PER_REV))
                cands = [k for k in range(k0 - kmax, k0 + kmax + 1)
                         if lo <= raw - k * TICKS_PER_REV <= hi]
                as_deg = {k: m.ticks_to_angle_deg(raw - k * TICKS_PER_REV) for k in cands}
                cand_txt = ', '.join(f'k={k}: {as_deg[k]:.1f} deg' for k in cands)

                if not cands:
                    errors.append(
                        f'ID{m.id} ({name}): Present {raw} ticks is OUTSIDE the plausible '
                        f'band [{lo},{hi}] (joint {j.min_deg - self.plausible_margin_deg:.1f}..'
                        f'{j.max_deg + self.plausible_margin_deg:.1f} deg) for every wrap. '
                        f'Is the motor calibrated (dynamixel_calibrate.py) and are gear.* '
                        f'right?')
                    continue
                if len(cands) > 1:
                    if math.isfinite(hint):
                        near = [k for k in cands if abs(as_deg[k] - hint) <= self.pose_hint_tol_deg]
                        if len(near) == 1:
                            cands = near
                            lg.warn(f'  ID{m.id} ({name}): AMBIGUOUS wrap [{cand_txt}] resolved by '
                                    f'startup.expected_pose_deg.{name}={hint:.1f} '
                                    f'(+/-{self.pose_hint_tol_deg:.0f} deg).')
                        else:
                            errors.append(
                                f'ID{m.id} ({name}): AMBIGUOUS Present {raw} ticks -> candidates '
                                f'[{cand_txt}]; the hint {hint:.1f} deg selects {len(near)} of '
                                f'them (+/-{self.pose_hint_tol_deg:.0f} deg). Move the platform '
                                f'or fix the hint.')
                            continue
                    else:
                        errors.append(
                            f'ID{m.id} ({name}): AMBIGUOUS Present {raw} ticks -> candidates '
                            f'[{cand_txt}] (one motor rev = {360.0 / self.gear.ratio:.1f} joint '
                            f'deg at {self.gear.ratio:g}:1). Set startup.expected_pose_deg.{name} '
                            f'to the pose the platform is REALLY at (measure it), or move it to '
                            f'a pose with a single candidate, then restart.')
                        continue
                k = cands[0]
                m.turn_offset = k * TICKS_PER_REV
                if k == 0:
                    lg.info(f'  ID{m.id} ({name}): Present {raw} ticks = '
                            f'{m.ticks_to_angle_deg(raw):.1f} joint deg (in band).')
                else:
                    lg.warn(f'  ID{m.id} ({name}): Present {raw} ticks is {k:+d} motor '
                            f'revolution(s) off the band [{lo},{hi}] -> corrected in software '
                            f'(turn_offset={m.turn_offset}); present = '
                            f'{m.ticks_to_angle_deg(raw):.1f} joint deg.')
        lg.info('==================================================')
        if errors:
            for e in errors:
                lg.fatal('  ' + e)
            raise RuntimeError(
                'revolution-safety check FAILED (see above). Refusing to start rather '
                'than command a pose up to one motor revolution away. Torque was not '
                'enabled by this node.')

    def _enable_torque(self):
        """Torque ON. In velocity mode, Goal Velocity is zeroed FIRST so a stale
        RAM value from a previous run cannot spin a motor the instant torque
        comes on; Profile Acceleration is written (0 = no profile) since op mode
        1 uses it and ignores Profile Velocity.

        DROP HAZARD (redesign_report.md section 0): the mechanism's gravity rest
        state is full extension. In position mode a torque-enabled servo holds
        by position feedback. In velocity mode with Goal Velocity 0 it holds by
        its velocity loop's integral action -- a SOFTER hold. Whether the 7:1
        cycloid back-drives at 4 N.m output is unmeasured; until it is, assume a
        torque-disable at mid-stroke drops the platform up to 0.29 m."""
        if self.mode == 'velocity':
            accel_units = self._accel_units(self.vel_mode_profile_accel)
            for m in self.all_motors:
                self.packet.write4ByteTxRx(self.port, m.id, ADDR_GOAL_VELOCITY, 0)
                self.packet.write4ByteTxRx(self.port, m.id, ADDR_PROFILE_ACCEL, accel_units)
            self._vel_written = {m.id: 0 for m in self.all_motors}
        for m in self.all_motors:
            self._write1(m.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.get_logger().info('Torque ON on all motors'
                               + (' (Goal Velocity = 0: velocity-loop hold).'
                                  if self.mode == 'velocity' else '.'))

    # ---------------------------------------------------------------- read/write
    def _write1(self, mid, addr, value):
        self.packet.write1ByteTxRx(self.port, mid, addr, int(value) & 0xFF)

    def _write2(self, mid, addr, value):
        self.packet.write2ByteTxRx(self.port, mid, addr, int(value) & 0xFFFF)

    def _read1(self, mid, addr):
        val, _c, _e = self.packet.read1ByteTxRx(self.port, mid, addr)
        return val

    def _read2(self, mid, addr):
        val, _c, _e = self.packet.read2ByteTxRx(self.port, mid, addr)
        return val

    def _read4(self, mid, addr):
        val, _c, _e = self.packet.read4ByteTxRx(self.port, mid, addr)
        return val

    # ------------------------------------------------------------- goal input
    def _on_goal(self, msg: JointState):
        if self.fault_latched:
            self.get_logger().warn(
                'Ignoring /joint_goal: fault latched. Restart the node after '
                'clearing the mechanical fault.', throttle_duration_sec=5.0)
            return

        goal_deg = {}
        goal_vel = {}
        for i, name in enumerate(msg.name):
            if name not in self.joints:
                continue
            angle_deg = math.degrees(msg.position[i] if i < len(msg.position) else 0.0)
            clamped = self.joints[name].clamp_deg(angle_deg)
            if abs(clamped - angle_deg) > 1e-6:
                self.get_logger().warn(
                    f'{name}: goal {angle_deg:.2f} deg outside soft limits '
                    f'[{self.joints[name].min_deg:.1f}, {self.joints[name].max_deg:.1f}] '
                    f'-> clamped to {clamped:.2f} deg (SAFETY BACKSTOP).',
                    throttle_duration_sec=1.0)
            goal_deg[name] = clamped
            goal_vel[name] = float(msg.velocity[i]) if i < len(msg.velocity) else 0.0
            # learn the operating envelope from what the planner actually commands
            if self.joints[name].observe(clamped):
                self._observed_dirty = True

        if not goal_deg:
            self.get_logger().warn('Received /joint_goal with no known joint names.')
            return

        if self.mode == 'position':
            self._write_position_goals(goal_deg, goal_vel)
        else:
            self._store_velocity_goals(goal_deg, goal_vel)

    # ================================================================ POSITION
    def _write_position_goals(self, goal_deg, goal_vel):
        # Creep (slow profile) for the first commanded pose and for every
        # re-publication of that SAME pose (a planner holds its start pose for a
        # settle period with velocity 0); switch to the planner's |v| as soon as
        # the pose changes, i.e. the trajectory starts moving.
        if not self.first_goal_received:
            creep = True
            self._first_goal_deg = dict(goal_deg)
            self.get_logger().info('First goal: creeping slowly to commanded pose.')
        else:
            creep = (self._first_goal_deg is not None and all(
                abs(goal_deg[n] - self._first_goal_deg.get(n, 1e9)) < 1e-3 for n in goal_deg))
            if not creep and self._first_goal_deg is not None:
                self._first_goal_deg = None
                self.get_logger().info('Goal pose is moving: profile speed now from /joint_goal.')

        # Profile registers are MOTOR-side: joint rad/s(^2) x gear.ratio. Without
        # this the servo was handed 1/ratio of the speed it needs per waypoint
        # and lagged/lurched at every 20 ms goal (the "wobble" of kickoff Step 0).
        accel_units = self._accel_units(self.profile_acceleration * self.gear.ratio)

        self.group_write.clearParam()
        for name, angle_deg in goal_deg.items():
            if creep:
                vel_rad_s = self.creep_profile_velocity
            elif abs(goal_vel.get(name, 0.0)) > 1e-6:
                vel_rad_s = abs(goal_vel[name])
            else:
                vel_rad_s = self.default_profile_velocity
            vel_units = self._velocity_units(vel_rad_s * self.gear.ratio)

            for m in self.joints[name].motors:
                ticks = m.angle_deg_to_ticks(angle_deg)  # signed (extended mode)
                param = _le4(accel_units) + _le4(vel_units) + _le4(ticks)
                if not self.group_write.addParam(m.id, param):
                    self.get_logger().error(
                        f'GroupSyncWrite addParam failed for ID {m.id}')

        comm = self.group_write.txPacket()
        self.group_write.clearParam()
        if comm != COMM_SUCCESS:
            self.get_logger().error(
                f'GroupSyncWrite goal failed: {self.packet.getTxRxResult(comm)}')
            return
        self.first_goal_received = True

    @staticmethod
    def _velocity_units(motor_rad_s):
        """Unsigned profile-velocity units from a MOTOR-side rad/s; never 0
        (0 = unlimited in firmware)."""
        rpm = abs(motor_rad_s) * RAD_S_TO_RPM
        return max(1, int(round(rpm / VELOCITY_UNIT_RPM)))

    @staticmethod
    def _accel_units(motor_rad_s2):
        """Profile-acceleration units from a MOTOR-side rad/s^2. 0 -> 0 (firmware
        'no profile'); otherwise never 0."""
        if abs(motor_rad_s2) < 1e-12:
            return 0
        rpm2 = abs(motor_rad_s2) * 3600.0 / (2.0 * math.pi)  # rad/s^2 -> rev/min^2
        return max(1, int(round(rpm2 / ACCEL_UNIT_RPM2)))

    def _position_mode_tick(self):
        state = self._read_state()
        if state is not None:
            self._publish_state(state)

    # ================================================================ VELOCITY
    def _store_velocity_goals(self, goal_deg, goal_vel):
        """Velocity mode: goals are consumed by the control loop, not written here."""
        now = time.monotonic()
        jump = 0.0
        for name, angle_deg in goal_deg.items():
            j = self.joints[name]
            if j.q_des_deg is not None:
                jump = max(jump, abs(angle_deg - j.q_des_deg))
            j.q_des_deg = angle_deg
            j.v_ff_rad_s = goal_vel.get(name, 0.0)
        if not self.first_goal_received:
            self.first_goal_received = True
            self._start_creep('first goal')
        elif jump > self.goal_jump_warn_deg:
            # A discontinuity in a streaming trajectory (dropped messages, a
            # planner restart). Not re-creeping: the rate-limited correction
            # closes it while the feed-forward keeps following the stream.
            self.get_logger().warn(
                f'/joint_goal jumped {jump:.1f} deg between consecutive messages; '
                f'closing at <= {self.catchup_rate:.2f} rad/s.', throttle_duration_sec=1.0)
        if self._watchdog_tripped:
            self.get_logger().info('/joint_goal stream resumed; watchdog released.')
            self._watchdog_tripped = False
        self._last_goal_time = now

    def _start_creep(self, why):
        self._creeping = True
        self._creep_started = time.monotonic()
        for m in self.all_motors:
            m.integral = 0.0
        self.get_logger().info(
            f'Creeping to commanded pose at <= {self.creep_profile_velocity:.2f} rad/s '
            f'joint ({why}); feed-forward ignored until within '
            f'{self.creep_done_deg:.1f} deg.')

    def _velocity_mode_tick(self):
        """One cascade period: read -> outer position loop -> Goal Velocity."""
        state = self._read_state()
        if state is None:
            if not self.fault_latched:
                # Bus hiccup: hold. A missed read must not leave a stale velocity
                # running; the previous command stays only for this one period.
                self._write_velocities({m.id: 0 for m in self.all_motors}, force=False)
            return
        now = time.monotonic()

        # --- goal watchdog ---------------------------------------------------
        stale = (self._last_goal_time is None
                 or now - self._last_goal_time > self.watchdog_timeout)
        if stale:
            if self._last_goal_time is not None and not self._watchdog_tripped:
                self._watchdog_tripped = True
                self.get_logger().warn(
                    f'WATCHDOG: no /joint_goal for > {self.watchdog_timeout * 1000:.0f} ms '
                    f'-> commanding ZERO velocity (velocity-loop hold).')
            self._write_velocities({m.id: 0 for m in self.all_motors})
            self._publish_state(state)
            return

        # --- creep bookkeeping ------------------------------------------------
        v_cap = self.velocity_limit
        if self._creeping:
            worst = 0.0
            for name in self.joint_names:
                j = self.joints[name]
                if j.q_des_deg is None:
                    continue
                for m in j.motors:
                    worst = max(worst, abs(j.q_des_deg - m.q_meas_deg))
            if worst <= self.creep_done_deg:
                self._creeping = False
                self.get_logger().info(
                    f'Creep complete (max error {worst:.2f} deg); tracking with feed-forward.')
            elif now - self._creep_started > self.creep_timeout:
                self._trip_fault(
                    f'CREEP TIMEOUT: still {worst:.1f} deg from the commanded pose after '
                    f'{self.creep_timeout:.0f} s. Wrong gear.reversing / direction sign, a '
                    f'stalled joint, or an unreachable goal.')
                return

        # --- outer loop, per physical motor ----------------------------------
        # Between goal messages the reference is dead-reckoned with the planner's
        # velocity (bounded by the watchdog timeout), which removes the 50 Hz
        # sample-and-hold lag (0.5 deg at the lap's 25 deg/s peaks).
        age = min(max(0.0, now - self._last_goal_time), self.watchdog_timeout)
        cmd_units = {}
        trip_msgs = []
        for name in self.joint_names:
            j = self.joints[name]
            if j.q_des_deg is None:
                for m in j.motors:
                    cmd_units[m.id] = 0
                continue
            v_ff = 0.0 if self._creeping else j.v_ff_rad_s
            q_des = math.radians(j.q_des_deg) + v_ff * age
            for m in j.motors:
                q_meas = math.radians(m.q_meas_deg)
                e = q_des - q_meas

                # tracking-error trip: |e| over the limit AND not closing at least
                # half as fast as the correction cap allows, for `persist` cycles.
                # (Not during creep, where |e| is large by design.) A goal jump
                # that IS being closed never trips; a stalled or slipping joint,
                # or a wrong sign, does.
                abs_e = abs(e)
                closing = abs_e < self._last_abs_err[m.id] - 0.5 * self.catchup_rate * self._dt
                self._last_abs_err[m.id] = abs_e
                if not self._creeping and abs_e > self.max_tracking_error_rad and not closing:
                    self._tracking_over_count[m.id] += 1
                    if self._tracking_over_count[m.id] >= self.tracking_fault_persist:
                        trip_msgs.append(f'ID{m.id} ({name}) error {math.degrees(e):+.1f} deg')
                else:
                    self._tracking_over_count[m.id] = 0

                # PI correction (rate-limited) + feed-forward, with deadband at rest
                if abs_e < self.deadband_rad and abs(v_ff) < 1e-6:
                    m.integral = 0.0
                    v = 0.0
                else:
                    if self.ki > 0.0:
                        m.integral += e * self._dt
                        lim = self.integral_limit / self.ki
                        m.integral = max(-lim, min(lim, m.integral))
                    corr = self.kp * e + self.ki * m.integral
                    cap = self.creep_profile_velocity if self._creeping else self.catchup_rate
                    corr = max(-cap, min(cap, corr))
                    v = v_ff + corr

                # joint-side velocity clamp (BEFORE the gear ratio)
                v = max(-v_cap, min(v_cap, v))

                # SOFT LIMITS as a direction clamp on the MEASURED position: at or
                # past a bound, only motion back inside is allowed. This is the one
                # place velocity mode is inherently less safe than position mode.
                if m.q_meas_deg >= j.max_deg and v > 0.0:
                    v = 0.0
                elif m.q_meas_deg <= j.min_deg and v < 0.0:
                    v = 0.0

                cmd_units[m.id] = m.joint_rate_to_units(v)

        if trip_msgs:
            self._trip_fault('TRACKING ERROR > '
                             f'{math.degrees(self.max_tracking_error_rad):.1f} deg for '
                             f'{self.tracking_fault_persist} cycles [{", ".join(trip_msgs)}]. '
                             'Stall, gearbox slip, or a wrong sign.')
            return

        self._write_velocities(cmd_units)
        self._publish_state(state)

    def _write_velocities(self, units_by_id, force=True):
        """GroupSyncWrite Goal Velocity(104) for all motors."""
        if self.fault_latched:
            return
        if not force and self._vel_written == units_by_id:
            return
        self.group_write.clearParam()
        for m in self.all_motors:
            u = int(units_by_id.get(m.id, 0))
            if not self.group_write.addParam(m.id, _le4(u)):
                self.get_logger().error(f'GroupSyncWrite addParam failed for ID {m.id}')
        comm = self.group_write.txPacket()
        self.group_write.clearParam()
        if comm != COMM_SUCCESS:
            self.get_logger().error(
                f'GroupSyncWrite Goal Velocity failed: {self.packet.getTxRxResult(comm)}',
                throttle_duration_sec=1.0)
            return
        self._vel_written = dict(units_by_id)

    # ----------------------------------------------------------- state I/O
    def _read_state(self):
        """One GroupSyncRead of load/velocity/position for every motor.
        Updates m.q_meas_deg, runs the Present-Load cutoff. Returns
        {id: (joint_deg, joint_rad_s, joint_nm, load_raw)} or None on failure /
        fault."""
        if self.fault_latched:
            return None
        comm = self.group_read.txRxPacket()
        if comm != COMM_SUCCESS:
            self.get_logger().warn(
                f'GroupSyncRead failed: {self.packet.getTxRxResult(comm)}',
                throttle_duration_sec=2.0)
            return None

        raw = {}
        over_limit_ids = []
        for m in self.all_motors:
            if not self.group_read.isAvailable(m.id, ADDR_STATE_BLOCK, LEN_STATE_BLOCK):
                self.get_logger().warn(
                    f'No state data for ID {m.id}', throttle_duration_sec=2.0)
                return None
            load_raw = _to_signed(self.group_read.getData(m.id, ADDR_PRESENT_LOAD, 2), 16)
            vel = _to_signed(self.group_read.getData(m.id, ADDR_PRESENT_VELOCITY, 4), 32)
            pos = _to_signed(self.group_read.getData(m.id, ADDR_PRESENT_POSITION, 4), 32)

            # Runtime wrap guard: a jump of more than half a motor revolution in
            # one period is physically impossible (that would be > 1500 rpm), so
            # it can only be the multi-turn count re-initialising. Should never
            # happen in op modes 1/4 without a power cycle -- log loudly.
            if m.last_ticks is not None:
                jump = pos - m.last_ticks
                if abs(jump) > TICKS_PER_REV // 2:
                    k = int(round(jump / float(TICKS_PER_REV)))
                    m.turn_offset += k * TICKS_PER_REV
                    self.get_logger().error(
                        f'ID{m.id}: Present Position jumped {jump:+d} ticks in one period '
                        f'-> treated as {k:+d} motor-rev wrap (turn_offset={m.turn_offset}). '
                        f'Was the servo power-cycled or its mode changed while running?')
            m.last_ticks = pos

            joint_deg = m.ticks_to_angle_deg(pos)
            joint_rad_s = m.units_to_joint_rate(vel)
            motor_nm = load_raw * LOAD_UNIT_PCT / 100.0 * self.stall_torque_nm
            joint_nm = self.gear.motor_to_joint_torque(motor_nm)
            m.q_meas_deg = joint_deg
            raw[m.id] = (joint_deg, joint_rad_s, joint_nm, load_raw)
            self._reg_peak[m.id] = max(self._reg_peak[m.id], abs(load_raw))

            if abs(load_raw) > self.load_trip_units:
                self._current_over_count[m.id] += 1
                if self._current_over_count[m.id] >= self.current_fault_persist:
                    over_limit_ids.append((m.id, load_raw, joint_nm))
            else:
                self._current_over_count[m.id] = 0

        # GROUND-TRUTH raw log: reg[126] as the raw signed integer, its % load and
        # the JOINT torque it implies, for all 4 motors. '[pk N]' is the peak
        # |raw| since node start, so throttling can never hide the true peak.
        # This sits in the control path: console writes are synchronous, so on a
        # slow terminal lengthen log.load_snapshot_period_s (or set 0) rather
        # than let it stall the 50 Hz cascade.
        if self.load_log_period > 0.0:
            snap = '  '.join(
                f'ID{mid}={raw[mid][3]:+d}({raw[mid][3] * LOAD_UNIT_PCT:+.1f}%|'
                f'{raw[mid][2]:+.2f}Nm)[pk{self._reg_peak[mid]}]'
                for mid in raw)
            self.get_logger().info(
                f'reg[126] [Present Load 0.1%/unit | joint N.m]: {snap}',
                throttle_duration_sec=self.load_log_period)

        if over_limit_ids:
            self._trip_load_fault(over_limit_ids)
            return None
        return raw

    def _publish_state(self, raw):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        for name in self.joint_names:
            motors = self.joints[name].motors
            n = len(motors)
            js.position.append(math.radians(sum(raw[m.id][0] for m in motors) / n))
            js.velocity.append(sum(raw[m.id][1] for m in motors) / n)
            js.effort.append(sum(raw[m.id][2] for m in motors) / n)   # JOINT N.m
        self.state_pub.publish(js)

    # ----------------------------------------------------------------- faults
    def _trip_load_fault(self, over_limit_ids):
        detail = ', '.join(
            f'ID{i}: reg={r:+d} -> {r * LOAD_UNIT_PCT:+.1f}% load = {t:+.2f} N.m joint'
            for i, r, t in over_limit_ids)
        self._trip_fault(
            f'LOAD CUTOFF TRIPPED [{detail}] (threshold {self.load_trip_units} units = '
            f'{self.load_trip_units * LOAD_UNIT_PCT:.1f}% = '
            f'{self.gear.motor_to_joint_torque(self.load_trip_units / 1000 * self.stall_torque_nm):.2f} '
            f'N.m joint; reg[126] is PRESENT LOAD on the XC430, not current).')

    def _trip_fault(self, why):
        self.fault_latched = True
        self._save_pose_file(force=True)
        self.get_logger().error(
            f'{why} Torque DISABLED on all motors. Fault latched -- restart to recover. '
            f'The platform will fall to its gravity rest pose unless the gearbox '
            f'back-drive friction holds it.')
        self.disable_torque_all()

    # --------------------------------------------------------------- shutdown
    def disable_torque_all(self):
        """Zero Goal Velocity (velocity mode) then torque OFF. Physically, with
        the reducer this hands the platform to gravity + cycloid back-drive
        friction -- measure the latter before relying on it (kickoff section 4)."""
        try:
            if self.mode == 'velocity':
                for m in self.all_motors:
                    self.packet.write4ByteTxRx(self.port, m.id, ADDR_GOAL_VELOCITY, 0)
            for m in self.all_motors:
                self._write1(m.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        except Exception:  # pragma: no cover - best effort on shutdown
            pass

    def safe_shutdown(self):
        self.get_logger().info('Disabling torque on all motors and closing port.')
        try:
            self._save_pose_file(force=True)
        except Exception:  # pragma: no cover - never block the torque-off path
            pass
        self.disable_torque_all()
        try:
            self.port.closePort()
        except Exception:  # pragma: no cover
            pass


def main(args=None):
    rclpy.init(args=args)

    # Ensure SIGTERM (docker stop) unwinds through `finally` like Ctrl-C does,
    # so torque is disabled on the way out instead of the process being killed.
    def _sigterm(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)

    node = None
    try:
        node = DynamixelActuatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # bring-up/hardware failure -> clean fatal + cleanup
        print(f'[dynamixel_actuator] fatal: {exc}')
    finally:
        if node is not None:
            node.safe_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
