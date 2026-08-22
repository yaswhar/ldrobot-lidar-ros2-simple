#!/usr/bin/env python3
"""
dynamixel_actuator_node.py  --  HARDWARE LAYER ONLY

Drives the 4x DYNAMIXEL XC430-W150-T servos of the parallel-linkage platform.

This node knows NOTHING about the mechanism, inverse kinematics, oscillation
modes or geometry.  Its only job is:

    * take joint-space position commands on /joint_goal and drive the motors,
    * report present state on /joint_states.

It exposes exactly THREE logical joints -- "theta", "alpha", "beta" -- even
though "theta" is physically TWO motors (IDs 1 and 2).  The fan-out from 1
logical joint -> 2 physical motor IDs happens entirely inside this node and is
invisible to any publisher.

Topic contract (the ONLY coupling to a planner -- a future lidar_planner_node
can reuse this unchanged):
    subscribe  /joint_goal    sensor_msgs/JointState  name=[theta,alpha,beta]
                                position [rad], velocity [rad/s] (optional).
    publish    /joint_states  sensor_msgs/JointState  name=[theta,alpha,beta]
                                position [rad], velocity [rad/s], effort [A].

CONTROL APPROACH (bench-measured, 2026-08):
    * Baud is FIXED at 1,000,000 -- connect directly, no probing on launch.
    * Operating Mode = EXTENDED POSITION CONTROL (multi-turn) on ALL 4 motors,
      because ID2's homed target goes negative across its real range (outside
      plain Position Control's 0..4095).
    * Calibration lives in each motor's EEPROM HOMING OFFSET (set by
      dynamixel_calibrate.py at the physical-zero pose), NOT a static YAML
      offset -- so the runtime formula collapses to, per motor:
          goal_ticks = round(direction * angle_deg * 4096/360)   (direction=-1)
      A static YAML offset could silently drift by a whole revolution (4096
      ticks) if the multi-turn baseline shifts; Homing Offset avoids that.
    * Startup revolution-safety check: read Present Position and, if it sits a
      whole multiple of 4096 outside the plausible band, correct it in software
      and warn LOUDLY rather than commanding a pose a full turn away.

Safety (this node cannot trust the planner):
    * per-logical-joint soft angle limits enforced on EVERY goal (backstop),
    * independent current-based cutoff -> torque disable if it exceeds a limit,
    * torque disable on SIGINT/SIGTERM and on any current fault,
    * slow "creep to first commanded pose" on the first goal after activation.
"""

import math
import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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
# Addresses/units below are the X-series common control table, cross-checked
# against the XC430-W150-T e-manual.  VERIFY against the official e-manual if you
# swap motor models:  https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/
# =============================================================================
ADDR_OPERATING_MODE = 11      # EEPROM, 1 byte
ADDR_HOMING_OFFSET = 20       # EEPROM, 4 bytes (signed) -- calibration zero
ADDR_CURRENT_LIMIT = 38       # EEPROM, 2 bytes (unit 2.69 mA)
ADDR_TORQUE_ENABLE = 64       # RAM,    1 byte
ADDR_PROFILE_ACCEL = 108      # RAM,    4 bytes (unit 214.577 rev/min^2)
ADDR_PROFILE_VELOCITY = 112   # RAM,    4 bytes (unit 0.229 rev/min)
ADDR_GOAL_POSITION = 116      # RAM,    4 bytes (pulse, SIGNED in extended mode)
ADDR_PRESENT_CURRENT = 126    # RAM,    2 bytes (unit 2.69 mA, signed)
ADDR_PRESENT_VELOCITY = 128   # RAM,    4 bytes (unit 0.229 rev/min, signed)
ADDR_PRESENT_POSITION = 132   # RAM,    4 bytes (pulse, signed multi-turn)

# One contiguous write block: ProfileAccel(108..111) ProfileVel(112..115)
# GoalPosition(116..119) = 12 bytes -> a single GroupSyncWrite makes all 4
# motors accept accel+vel+goal on the SAME command (atomic move).
ADDR_GOAL_BLOCK = ADDR_PROFILE_ACCEL
LEN_GOAL_BLOCK = 12

# One contiguous read block: PresentCurrent(126..127) PresentVelocity(128..131)
# PresentPosition(132..135) = 10 bytes -> a single GroupSyncRead per cycle.
ADDR_STATE_BLOCK = ADDR_PRESENT_CURRENT
LEN_STATE_BLOCK = 10

# X-series Operating Modes: 3 = Position (single-turn 0..4095),
#                           4 = Extended Position (multi-turn, signed).  <-- used
# VERIFY value 4 against the XC430-W150-T e-manual before running.
OP_MODE_EXTENDED_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# X-series conversion constants
TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV       # 0.088 deg/pulse
CURRENT_UNIT_MA = 2.69                      # mA per Present/Goal Current unit
VELOCITY_UNIT_RPM = 0.229                   # rev/min per velocity unit
ACCEL_UNIT_RPM2 = 214.577                   # rev/min^2 per accel unit

# Revolution-safety: how far outside the soft-limit band (in ticks) a Present
# Position may sit before we treat a full-revolution (4096) wrap as the cause.
PLAUSIBLE_MARGIN_TICKS = 256

# Standard Dynamixel baud set -- used ONLY by the opt-in auto_probe_baud path.
STANDARD_BAUDS = [1000000, 57600, 115200, 9600, 2000000, 3000000, 4000000]


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


class PhysicalMotor:
    """One physical servo of a logical joint.

    Calibration zero lives in the motor's EEPROM Homing Offset, so the tick
    formula collapses to direction * angle.  `turn_offset` is a startup-only
    revolution-safety correction (normally 0).  `raw_offset` is a REFERENCE-ONLY
    bench value (raw ticks at physical 0 deg) used solely for the startup log.
    """

    def __init__(self, motor_id, direction, raw_offset=0):
        self.id = int(motor_id)
        self.direction = 1 if direction >= 0 else -1
        self.raw_offset = int(raw_offset)   # reference only (startup log)
        self.turn_offset = 0                # +/- k*4096, set by startup check

    def angle_deg_to_ticks(self, angle_deg):
        # homed frame: goal = direction * angle / 0.088 (+ revolution correction)
        return int(round(self.direction * angle_deg / DEG_PER_TICK)) + self.turn_offset

    def ticks_to_angle_deg(self, ticks):
        return self.direction * (ticks - self.turn_offset) * DEG_PER_TICK


class LogicalJoint:
    """A named joint ("theta"/"alpha"/"beta") backed by 1 or 2 physical motors."""

    def __init__(self, name, motors, min_deg, max_deg, sanity_range):
        self.name = name
        self.motors = motors          # list[PhysicalMotor]
        self.min_deg = float(min_deg)
        self.max_deg = float(max_deg)
        self.sanity_range = sanity_range  # (lo_deg, hi_deg) for startup log

    def clamp_deg(self, angle_deg):
        return max(self.min_deg, min(self.max_deg, angle_deg))


class DynamixelActuatorNode(Node):
    def __init__(self):
        super().__init__('dynamixel_actuator')

        # ---- parameters -----------------------------------------------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)      # FIXED 1M (bench-measured)
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('auto_probe_baud', False)  # do NOT probe on launch

        # logical -> physical motor mapping (config; logged at startup).
        # All four motors resolve to the SAME direction (-1): raw ticks decrease
        # as the physical/kinematic angle increases.
        self.declare_parameter('motors.theta.ids', [1, 2])
        self.declare_parameter('motors.theta.directions', [-1, -1])
        self.declare_parameter('motors.theta.raw_offsets_ticks', [2048, 618])
        self.declare_parameter('motors.alpha.ids', [3])
        self.declare_parameter('motors.alpha.directions', [-1])
        self.declare_parameter('motors.alpha.raw_offsets_ticks', [1024])
        self.declare_parameter('motors.beta.ids', [4])
        self.declare_parameter('motors.beta.directions', [-1])
        self.declare_parameter('motors.beta.raw_offsets_ticks', [3072])

        # per-logical-joint soft limits (deg) -- SAFETY BACKSTOP
        self.declare_parameter('limits.theta.min_deg', 0.0)
        self.declare_parameter('limits.theta.max_deg', 80.0)
        self.declare_parameter('limits.alpha.min_deg', 0.0)
        self.declare_parameter('limits.alpha.max_deg', 85.0)
        self.declare_parameter('limits.beta.min_deg', 0.0)
        self.declare_parameter('limits.beta.max_deg', 80.0)

        # kinematic operating range per joint (startup sanity log ONLY)
        self.declare_parameter('sanity_range_deg.theta', [6.5, 73.3])
        self.declare_parameter('sanity_range_deg.alpha', [1.9, 82.6])
        self.declare_parameter('sanity_range_deg.beta', [8.1, 66.3])

        # motion defaults / profiles
        self.declare_parameter('default_profile_velocity_rad_s', 0.6)
        self.declare_parameter('profile_acceleration_rad_s2', 3.0)
        self.declare_parameter('creep_profile_velocity_rad_s', 0.15)

        # safety: current-based cutoff
        self.declare_parameter('current_limit_ma', 1200.0)
        self.declare_parameter('current_fault_persist', 5)

        # timing / topics
        self.declare_parameter('state_publish_rate_hz', 20.0)
        self.declare_parameter('goal_topic', '/joint_goal')
        self.declare_parameter('state_topic', '/joint_states')

        self.port_name = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.protocol_version = float(self.get_parameter('protocol_version').value)
        self.auto_probe_baud = bool(self.get_parameter('auto_probe_baud').value)
        self.default_profile_velocity = float(
            self.get_parameter('default_profile_velocity_rad_s').value)
        self.profile_acceleration = float(
            self.get_parameter('profile_acceleration_rad_s2').value)
        self.creep_profile_velocity = float(
            self.get_parameter('creep_profile_velocity_rad_s').value)
        self.current_limit_ma = float(self.get_parameter('current_limit_ma').value)
        self.current_fault_persist = int(
            self.get_parameter('current_fault_persist').value)
        self.state_publish_rate = float(
            self.get_parameter('state_publish_rate_hz').value)
        self.goal_topic = self.get_parameter('goal_topic').value
        self.state_topic = self.get_parameter('state_topic').value

        # ---- build logical joint map ---------------------------------------
        self.joints = self._build_joint_map()
        self.joint_names = ['theta', 'alpha', 'beta']
        self.all_motors = [m for jn in self.joint_names for m in self.joints[jn].motors]

        # ---- runtime state --------------------------------------------------
        self.first_goal_received = False   # first goal -> creep behaviour
        self.fault_latched = False
        self._current_over_count = {m.id: 0 for m in self.all_motors}

        # These two are pure calculation -- log them BEFORE touching hardware so
        # the operator can eyeball the mapping/ranges even if the bus is down.
        self._log_mapping()
        self._log_sanity_ranges()

        if not DXL_SDK_AVAILABLE:
            self.get_logger().fatal(
                'dynamixel_sdk is not installed in this environment. '
                'Add it to the ros2_lidar_image (see docker/Dockerfile.dynamixel).')
            raise RuntimeError('dynamixel_sdk missing')

        # ---- hardware bring-up ---------------------------------------------
        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(self.protocol_version)
        self._connect()
        self._configure_motors()

        # ---- GroupSync handles ---------------------------------------------
        self.group_write = GroupSyncWrite(
            self.port, self.packet, ADDR_GOAL_BLOCK, LEN_GOAL_BLOCK)
        self.group_read = GroupSyncRead(
            self.port, self.packet, ADDR_STATE_BLOCK, LEN_STATE_BLOCK)
        for m in self.all_motors:
            if not self.group_read.addParam(m.id):
                self.get_logger().error(f'GroupSyncRead addParam failed for ID {m.id}')

        # ---- revolution-safety check (before ANY motion) -------------------
        self._check_present_positions()

        # ---- ROS interfaces -------------------------------------------------
        self.state_pub = self.create_publisher(JointState, self.state_topic, 10)
        self.goal_sub = self.create_subscription(
            JointState, self.goal_topic, self._on_goal, 10)
        self.state_timer = self.create_timer(
            1.0 / max(1.0, self.state_publish_rate), self._publish_state)

        self.get_logger().info(
            f'dynamixel_actuator ready. Listening on {self.goal_topic}, '
            f'publishing {self.state_topic}. First goal will CREEP '
            f'({self.creep_profile_velocity:.2f} rad/s) to the commanded pose.')

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
            motors = [PhysicalMotor(mid, d, raw)
                      for mid, d, raw in zip(ids, dirs, raws)]
            min_deg = float(self.get_parameter(f'limits.{name}.min_deg').value)
            max_deg = float(self.get_parameter(f'limits.{name}.max_deg').value)
            sr = list(self.get_parameter(f'sanity_range_deg.{name}').value)
            sanity = (float(sr[0]), float(sr[1])) if len(sr) >= 2 else (min_deg, max_deg)
            joints[name] = LogicalJoint(name, motors, min_deg, max_deg, sanity)
        return joints

    def _log_mapping(self):
        self.get_logger().info('=== logical -> physical motor mapping ===')
        for name in self.joint_names:
            j = self.joints[name]
            desc = ', '.join(f'ID{m.id}(dir{m.direction:+d})' for m in j.motors)
            self.get_logger().info(
                f'  {name:6s} -> [{desc}]  soft limits [{j.min_deg:.1f}, '
                f'{j.max_deg:.1f}] deg')
        self.get_logger().info(
            '  calibration zero = EEPROM Homing Offset (run dynamixel_calibrate.py); '
            'goal_ticks = round(direction * angle_deg * 4096/360)')
        self.get_logger().info('=========================================')

    def _log_sanity_ranges(self):
        """Log expected raw + homed tick ranges so the operator can eyeball them
        BEFORE power is applied (reproduces the bench-measured values)."""
        c = 1.0 / DEG_PER_TICK
        self.get_logger().info('=== startup sanity ranges (eyeball before power) ===')
        for name in self.joint_names:
            j = self.joints[name]
            lo, hi = j.sanity_range
            for m in j.motors:
                raw_lo = round(m.raw_offset - lo * c)   # raw @ homing offset 0
                raw_hi = round(m.raw_offset - hi * c)
                homed_lo = round(m.direction * lo * c)  # homed present (runtime)
                homed_hi = round(m.direction * hi * c)
                self.get_logger().info(
                    f'  ID{m.id} ({name}): {lo:.1f}..{hi:.1f} deg -> raw '
                    f'{raw_lo}..{raw_hi} ticks | homed present {homed_lo}..{homed_hi}')
        self.get_logger().info('====================================================')

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
        """Set Extended Position Control Mode + current limit. Read-before-write
        to spare EEPROM. NEVER touches Homing Offset (that IS the calibration)."""
        current_limit_units = int(round(self.current_limit_ma / CURRENT_UNIT_MA))
        for m in self.all_motors:
            mode = self._read1(m.id, ADDR_OPERATING_MODE)
            cur_lim = self._read2(m.id, ADDR_CURRENT_LIMIT)
            need_mode = mode != OP_MODE_EXTENDED_POSITION
            need_cur = cur_lim != current_limit_units
            if need_mode or need_cur:
                # EEPROM writes require torque OFF.
                self._write1(m.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                if need_mode:
                    self._write1(m.id, ADDR_OPERATING_MODE, OP_MODE_EXTENDED_POSITION)
                if need_cur:
                    self._write2(m.id, ADDR_CURRENT_LIMIT, current_limit_units)
            self._write1(m.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.get_logger().info(
            f'Motors set to EXTENDED Position Control Mode (op mode '
            f'{OP_MODE_EXTENDED_POSITION}), current limit {self.current_limit_ma:.0f} '
            f'mA ({current_limit_units} units), torque ON.')

    def _check_present_positions(self):
        """Read Present Position on all motors before any move. If a reading sits
        a whole multiple of 4096 outside the plausible (soft-limit) band, correct
        it in software (turn_offset) and warn LOUDLY -- never command a pose a
        full revolution away."""
        comm = self.group_read.txRxPacket()
        if comm != COMM_SUCCESS:
            self.get_logger().warn(
                'Could not read Present Position for the revolution-safety check '
                f'({self.packet.getTxRxResult(comm)}). Proceeding with turn_offset=0.')
            return
        c = 1.0 / DEG_PER_TICK
        self.get_logger().info('=== revolution-safety check (Present Position) ===')
        # Per-motor evaluation (band from the joint's soft limits).
        for name in self.joint_names:
            j = self.joints[name]
            for m in j.motors:
                band_a = m.direction * j.min_deg * c
                band_b = m.direction * j.max_deg * c
                lo = min(band_a, band_b) - PLAUSIBLE_MARGIN_TICKS
                hi = max(band_a, band_b) + PLAUSIBLE_MARGIN_TICKS
                center = 0.5 * (lo + hi)
                raw = _to_signed(
                    self.group_read.getData(m.id, ADDR_PRESENT_POSITION, 4), 32)
                k = round((raw - center) / float(TICKS_PER_REV))
                corrected = raw - k * TICKS_PER_REV
                if lo <= corrected <= hi and k != 0:
                    m.turn_offset = k * TICKS_PER_REV
                    self.get_logger().warn(
                        f'  ID{m.id} ({name}): Present {raw} ticks is {k} full '
                        f'revolution(s) outside band [{lo:.0f},{hi:.0f}] -> '
                        f'auto-corrected in software (turn_offset={m.turn_offset}). '
                        f'Corrected present {corrected} = '
                        f'{m.ticks_to_angle_deg(raw):.1f} deg.')
                elif not (lo <= corrected <= hi):
                    m.turn_offset = 0
                    self.get_logger().warn(
                        f'  ID{m.id} ({name}): Present {raw} ticks is OUTSIDE the '
                        f'plausible band [{lo:.0f},{hi:.0f}] and NOT a clean 4096 '
                        f'wrap. Is the motor calibrated / near its range? '
                        f'(turn_offset=0, no correction applied)')
                else:
                    self.get_logger().info(
                        f'  ID{m.id} ({name}): Present {raw} ticks = '
                        f'{m.ticks_to_angle_deg(raw):.1f} deg (in band).')
        self.get_logger().info('==================================================')

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

    # ------------------------------------------------------------- goal input
    def _on_goal(self, msg: JointState):
        if self.fault_latched:
            self.get_logger().warn(
                'Ignoring /joint_goal: current-fault latched. Restart the node '
                'after clearing the mechanical fault.', throttle_duration_sec=5.0)
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
                    f'-> clamped to {clamped:.2f} deg (SAFETY BACKSTOP).')
            goal_deg[name] = clamped
            goal_vel[name] = abs(msg.velocity[i]) if i < len(msg.velocity) else 0.0

        if not goal_deg:
            self.get_logger().warn('Received /joint_goal with no known joint names.')
            return

        creep = not self.first_goal_received
        if creep:
            self.get_logger().info('First goal: creeping slowly to commanded pose.')

        accel_units = self._accel_units(self.profile_acceleration)

        self.group_write.clearParam()
        for name, angle_deg in goal_deg.items():
            if creep:
                vel_rad_s = self.creep_profile_velocity
            elif goal_vel.get(name, 0.0) > 1e-6:
                vel_rad_s = goal_vel[name]
            else:
                vel_rad_s = self.default_profile_velocity
            vel_units = self._velocity_units(vel_rad_s)

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
    def _velocity_units(rad_s):
        rpm = abs(rad_s) * 60.0 / (2.0 * math.pi)
        return max(1, int(round(rpm / VELOCITY_UNIT_RPM)))  # never 0 (=unlimited)

    @staticmethod
    def _accel_units(rad_s2):
        rpm2 = abs(rad_s2) * 3600.0 / (2.0 * math.pi)  # rad/s^2 -> rev/min^2
        return max(1, int(round(rpm2 / ACCEL_UNIT_RPM2)))

    # ----------------------------------------------------------- state output
    def _publish_state(self):
        if self.fault_latched:
            return
        comm = self.group_read.txRxPacket()
        if comm != COMM_SUCCESS:
            self.get_logger().warn(
                f'GroupSyncRead failed: {self.packet.getTxRxResult(comm)}',
                throttle_duration_sec=2.0)
            return

        raw = {}  # id -> (pos_deg_logical, vel_rad_s_logical, current_ma_signed)
        over_limit_ids = []
        for m in self.all_motors:
            if not self.group_read.isAvailable(m.id, ADDR_STATE_BLOCK, LEN_STATE_BLOCK):
                self.get_logger().warn(
                    f'No state data for ID {m.id}', throttle_duration_sec=2.0)
                return
            cur = _to_signed(
                self.group_read.getData(m.id, ADDR_PRESENT_CURRENT, 2), 16)
            vel = _to_signed(
                self.group_read.getData(m.id, ADDR_PRESENT_VELOCITY, 4), 32)
            pos = _to_signed(
                self.group_read.getData(m.id, ADDR_PRESENT_POSITION, 4), 32)

            current_ma = cur * CURRENT_UNIT_MA
            vel_rad_s = m.direction * (vel * VELOCITY_UNIT_RPM) * 2.0 * math.pi / 60.0
            pos_deg = m.ticks_to_angle_deg(pos)
            raw[m.id] = (pos_deg, vel_rad_s, current_ma)

            if abs(current_ma) > self.current_limit_ma:
                self._current_over_count[m.id] += 1
                if self._current_over_count[m.id] >= self.current_fault_persist:
                    over_limit_ids.append((m.id, current_ma))
            else:
                self._current_over_count[m.id] = 0

        if over_limit_ids:
            self._trip_current_fault(over_limit_ids)
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        for name in self.joint_names:
            motors = self.joints[name].motors
            js.position.append(math.radians(sum(raw[m.id][0] for m in motors) / len(motors)))
            js.velocity.append(sum(raw[m.id][1] for m in motors) / len(motors))
            js.effort.append(sum(raw[m.id][2] for m in motors) / len(motors) / 1000.0)  # A
        self.state_pub.publish(js)

    def _trip_current_fault(self, over_limit_ids):
        self.fault_latched = True
        detail = ', '.join(f'ID{i}={c:.0f}mA' for i, c in over_limit_ids)
        self.get_logger().error(
            f'CURRENT LIMIT EXCEEDED ({detail}) > {self.current_limit_ma:.0f} mA. '
            f'Torque DISABLED on all motors. Fault latched -- restart to recover.')
        self.disable_torque_all()

    # --------------------------------------------------------------- shutdown
    def disable_torque_all(self):
        try:
            for m in self.all_motors:
                self._write1(m.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        except Exception:  # pragma: no cover - best effort on shutdown
            pass

    def safe_shutdown(self):
        self.get_logger().info('Disabling torque on all motors and closing port.')
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
