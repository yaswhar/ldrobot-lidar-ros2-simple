#!/usr/bin/env python3
"""
dynamixel_calibrate.py  --  MOTOR-DRIVEN homing / calibration CLI (NOT a ROS node)

Writes each motor's EEPROM HOMING OFFSET so that Present Position corresponds to
the true JOINT (gearbox-output / crank) angle. After that the actuator's runtime
tick formula is:

    goal_ticks = round(direction * gear_sign * gear_ratio * joint_deg * 4096/360)

    direction  = the MEASURED raw-tick-vs-angle sense of the motor itself (-1)
    gear_sign  = -1 if the reducer reverses (gear.reversing: true), else +1
    gear_ratio = gear.ratio (7.0 for the fitted 7:1 cycloid; 1.0 = direct drive)

WHY MOTOR-DRIVEN (2026-09-16): with the 7:1 cycloid the platform cannot be
moved by hand (the drive does not back-drive), the reducer has large backlash,
and the joint's 30 deg floor means "physical zero" is not even reachable. So
this tool JOGS the joints under torque, lets you check the signs, takes up the
backlash on the flank the motor actually works against, and captures the
reference pose from a measurement you CAN make: the platform height with the
platform level (one tape-measure reading -> all three joint angles through the
forward kinematics), or a protractor on the crank.

BACKLASH: gravity always pushes the crank toward full extension, so in normal
operation (holding or moving under load) the reducer works on ONE flank. The
capture is only consistent with operation if the last motion before it was
LIFTING (joint angle increasing). The `norm` command does that for you
(-2 deg then +2 deg on every joint), and the capture refuses to proceed unless
the last move on every joint was upward.

MULTI-TURN: Present Position is re-initialised to single-turn on power-up and on
an Operating Mode change, so the actuator resolves the wrap at startup from the
soft-limit band plus a pose hint. This tool writes the pose it calibrated at to
the last-pose file (~/.cache/ldlidar_platform/last_pose.yaml) so the next
actuator start knows where the platform is.

FROM-PARAMS MODE (default, 2026-09-22): the calibration is computed from motor-
side angles measured at the two ends of the range and stored in the params YAML:

    calibration:
      joint_at_max_deg: 85.0            # the pose called "h_max"
      joint_at_min_deg: 30.0            # the pose called "h_min"
      motor_deg_at_max: {'1': 230.0, '2': 500.0, '3': 180.0, '4': 415.0}
      motor_deg_at_min: {'1': 615.0, '2': 885.0, '3': 565.0, '4': 800.0}

(Present Position in degrees with Homing Offset = 0, e.g. from the Dynamixel
Wizard.) The tool checks that every span equals gear_ratio x (max - min), that
all motors share one sense, and that this sense agrees with directions x
gear.reversing -- and refuses with the exact YAML fix if not. It then writes the
Homing Offsets, verifies them, asks where the platform is NOW (the multi-turn
count may have been reset since the readings), and writes the last-pose file.
Nothing moves. Run with --jog for the interactive jog / sign-check / capture
tool instead.

Commands (interactive, --jog):
    t+2  t-0.5  a+1  b-1  all+1     jog a joint / all joints by JOINT deg
    s                               status: present ticks, load %, joint deg
    speed 0.1                       jog speed [rad/s, joint side]
    norm                            backlash normalisation (-2 then +2 deg, all)
    cap h 1.05                      capture: platform LEVEL, height 1.05 m (pivot
                                    line to pivot line, see --h-offset)
    cap h 1.05 g 5                  ... with a known tilt gamma = +5 deg
    cap deg 57.3 56.8 56.8          capture: joint angles theta alpha beta
    off / on                        torque off / on
    q                               quit (torque off), qon = quit, torque on

Usage:
    ros2 run ldlidar_node dynamixel_calibrate.py \\
        --params  .../params/dynamixel_actuator.yaml \\
        --planner-params .../params/oscillation_planner.yaml   # geometry a,b,c,d
"""

import argparse
import math
import os
import sys
import time

import yaml

try:
    from dynamixel_sdk import PacketHandler, PortHandler
except ImportError:
    print('ERROR: dynamixel_sdk is not installed. Add it to the ros2_lidar_image '
          '(see docker/Dockerfile.dynamixel).', file=sys.stderr)
    sys.exit(2)

try:
    from platform_kinematics import PlatformKinematics
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from platform_kinematics import PlatformKinematics

ADDR_OPERATING_MODE = 11
ADDR_HOMING_OFFSET = 20        # EEPROM, 4 bytes, signed, +/-1,044,479
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126        # 0.1 %/unit, signed (XC430: load, NOT current)
ADDR_PRESENT_POSITION = 132    # 4 bytes, signed (multi-turn)
OP_MODE_EXTENDED_POSITION = 4
TORQUE_DISABLE, TORQUE_ENABLE = 0, 1
TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV
VELOCITY_UNIT_RPM = 0.229
ACCEL_UNIT_RPM2 = 214.577
HOMING_OFFSET_RANGE = 1_044_479
EXT_POSITION_RANGE = 1_048_575
POSE_FILE_DEFAULT = '~/.cache/ldlidar_platform/last_pose.yaml'
JOINTS = ('theta', 'alpha', 'beta')
STALL_JOINT_DEG = 0.75           # residual above this after a jog = the joint did not get there


def _s32(v):
    return v - (1 << 32) if v >= (1 << 31) else v


def _s16(v):
    return v - (1 << 16) if v >= (1 << 15) else v


def _extract_ros_params(doc):
    if not isinstance(doc, dict):
        return {}
    for key in ('/**', 'dynamixel_actuator', '/dynamixel_actuator', 'oscillation_planner'):
        node = doc.get(key)
        if isinstance(node, dict) and 'ros__parameters' in node:
            return node['ros__parameters']
    for node in doc.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            return node['ros__parameters']
    return doc


class Config:
    def __init__(self, params_path, planner_path=None, geometry=None):
        with open(params_path, 'r') as fh:
            p = _extract_ros_params(yaml.safe_load(fh))
        self.port = p.get('port', '/dev/ttyUSB0')
        self.baud = int(p.get('baudrate', 1000000))
        self.protocol = float(p.get('protocol_version', 2.0))
        gear = p.get('gear', {}) or {}
        self.ratio = float(gear.get('ratio', 1.0))
        self.gsign = -1 if bool(gear.get('reversing', False)) else 1
        motors = p.get('motors', {}) or {}
        limits = p.get('limits', {}) or {}
        cal = p.get('calibration', {}) or {}
        startup = p.get('startup', {}) or {}
        self.reference = {n: float((cal.get('reference_deg') or {}).get(n, 0.0)) for n in JOINTS}
        self.offsets_file = os.path.expanduser(cal.get('offsets_file', '~/dynamixel_offsets.yaml'))
        # measured end-of-range motor angles (from-params mode); keys may be int or str
        self.joint_at_max = cal.get('joint_at_max_deg')
        self.joint_at_min = cal.get('joint_at_min_deg')
        self.motor_at_max = {int(k): float(v) for k, v in (cal.get('motor_deg_at_max') or {}).items()}
        self.motor_at_min = {int(k): float(v) for k, v in (cal.get('motor_deg_at_min') or {}).items()}
        self.has_measurements = bool(self.motor_at_max) and self.joint_at_max is not None
        self.pose_file = os.path.expanduser(startup.get('pose_file', POSE_FILE_DEFAULT) or '')
        self.joints = {}   # name -> {'ids': [...], 'dirs': [...], 'lim': (lo, hi)}
        for n in JOINTS:
            jm = motors.get(n, {}) or {}
            ids = [int(i) for i in jm.get('ids', [])]
            dirs = [1 if d >= 0 else -1 for d in jm.get('directions', [-1] * len(ids))]
            lim = limits.get(n, {}) or {}
            self.joints[n] = {'ids': ids, 'dirs': dirs,
                              'lim': (float(lim.get('min_deg', 0.0)), float(lim.get('max_deg', 90.0)))}
        self.motor_of = {}  # id -> (joint, direction)
        for n, j in self.joints.items():
            for mid, d in zip(j['ids'], j['dirs']):
                self.motor_of[mid] = (n, d)
        # geometry for the height method
        a, b, c, d = 0.11, 0.44, 0.77, 0.10
        if planner_path:
            with open(planner_path, 'r') as fh:
                q = _extract_ros_params(yaml.safe_load(fh))
            g = q.get('geometry', {}) or {}
            a, b, c, d = (float(g.get('a', a)), float(g.get('b', b)),
                          float(g.get('c', c)), float(g.get('d', d)))
        if geometry:
            a, b, c, d = geometry
        self.kin = PlatformKinematics(a, b, c, d)

    def ticks_per_joint_deg(self, direction):
        return direction * self.gsign * self.ratio / DEG_PER_TICK


class Bus:
    def __init__(self, cfg, load_abort_pct):
        self.cfg = cfg
        self.port = PortHandler(cfg.port)
        self.packet = PacketHandler(cfg.protocol)
        if not self.port.openPort() or not self.port.setBaudRate(cfg.baud):
            print(f'ERROR: could not open {cfg.port} at {cfg.baud} baud.', file=sys.stderr)
            sys.exit(2)
        self.ids = list(cfg.motor_of)
        for mid in self.ids:
            _m, comm, err = self.packet.ping(self.port, mid)
            if comm != 0 or err != 0:
                print(f'ERROR: ID {mid} did not answer (comm={comm}, err={err}).', file=sys.stderr)
                sys.exit(2)
        print(f'Connected to {cfg.port} @ {cfg.baud} baud; IDs {self.ids} answer.')
        self.load_abort = load_abort_pct
        self.last_dir = {mid: 0 for mid in self.ids}   # +1 lifting, -1 lowering, 0 none
        self.speed = 0.1                               # rad/s joint

    # --- register helpers ---
    def w1(self, mid, addr, v):
        self.packet.write1ByteTxRx(self.port, mid, addr, int(v) & 0xFF)

    def w4(self, mid, addr, v):
        self.packet.write4ByteTxRx(self.port, mid, addr, int(v) & 0xFFFFFFFF)

    def r1(self, mid, addr):
        v, comm, err = self.packet.read1ByteTxRx(self.port, mid, addr)
        return v if comm == 0 and err == 0 else -1

    def r4s(self, mid, addr):
        v, comm, err = self.packet.read4ByteTxRx(self.port, mid, addr)
        if comm != 0 or err != 0:
            raise RuntimeError(f'ID {mid} read @{addr} failed (comm={comm}, err={err})')
        return _s32(v)

    def load_pct(self, mid):
        v, comm, err = self.packet.read2ByteTxRx(self.port, mid, ADDR_PRESENT_LOAD)
        return _s16(v) * 0.1 if comm == 0 and err == 0 else float('nan')

    def present(self, mid):
        return self.r4s(mid, ADDR_PRESENT_POSITION)

    # --- setup ---
    def prepare(self, clear_homing):
        print('Extended Position mode, Homing Offset ' + ('CLEARED' if clear_homing else 'kept')
              + ', torque ON, slow profile...')
        for mid in self.ids:
            self.w1(mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if self.r1(mid, ADDR_OPERATING_MODE) != OP_MODE_EXTENDED_POSITION:
                self.w1(mid, ADDR_OPERATING_MODE, OP_MODE_EXTENDED_POSITION)   # resets multi-turn count
            if clear_homing:
                self.w4(mid, ADDR_HOMING_OFFSET, 0)
        self.set_speed(self.speed)
        for mid in self.ids:
            # hold exactly where it is before torque comes on
            self.w4(mid, ADDR_GOAL_POSITION, self.present(mid))
            self.w1(mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_speed(self, joint_rad_s):
        self.speed = max(0.01, float(joint_rad_s))
        rpm = self.speed * self.cfg.ratio * 60.0 / (2.0 * math.pi)
        units = max(1, int(round(rpm / VELOCITY_UNIT_RPM)))
        acc = max(1, int(round(2.0 * self.cfg.ratio * 3600.0 / (2.0 * math.pi) / ACCEL_UNIT_RPM2)))
        for mid in self.ids:
            self.w4(mid, ADDR_PROFILE_VELOCITY, units)
            self.w4(mid, ADDR_PROFILE_ACCEL, acc)
        print(f'  jog speed {self.speed:.3f} rad/s joint = {rpm:.1f} rpm motor ({units} units)')

    def torque(self, on):
        for mid in self.ids:
            if on:
                self.w4(mid, ADDR_GOAL_POSITION, self.present(mid))
            self.w1(mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE if on else TORQUE_DISABLE)
        print('  torque ' + ('ON' if on else 'OFF -- if the reducer back-drives under load the '
                                            'platform will move; keep clear'))

    # --- motion ---
    def jog(self, joints_deg):
        """joints_deg: {joint: delta_deg}. Both theta motors move together.

        "Settled" = every motor is within 3 ticks of its goal OR has stopped
        moving. The XC430 position loop is P-only by default (I gain 0), so under
        load it parks a few ticks short of the goal; that residual is reported,
        not treated as a fault. Only a residual above STALL_JOINT_DEG (a joint
        that really did not get there) or a Present Load above the abort
        threshold fails the jog."""
        goals = {}
        for n, ddeg in joints_deg.items():
            if abs(ddeg) < 1e-9:
                continue
            for mid, d in zip(self.cfg.joints[n]['ids'], self.cfg.joints[n]['dirs']):
                dt = int(round(ddeg * self.cfg.ticks_per_joint_deg(d)))
                goals[mid] = self.present(mid) + dt
                self.last_dir[mid] = 1 if ddeg > 0 else -1
        if not goals:
            return True
        max_ticks = max(abs(g - self.present(mid)) for mid, g in goals.items())
        for mid, g in goals.items():
            self.w4(mid, ADDR_GOAL_POSITION, g)
        ticks_per_s = self.speed * self.cfg.ratio / (2.0 * math.pi) * TICKS_PER_REV
        deadline = time.monotonic() + max_ticks / max(1.0, ticks_per_s) + 3.0
        stall_ticks = max(20, int(round(STALL_JOINT_DEG * self.cfg.ratio / DEG_PER_TICK)))
        last = {mid: None for mid in goals}
        still = 0
        while time.monotonic() < deadline:
            time.sleep(0.05)
            for mid in goals:
                lp = self.load_pct(mid)
                if abs(lp) > self.load_abort:
                    for m2 in goals:                       # stop everything where it is
                        self.w4(m2, ADDR_GOAL_POSITION, self.present(m2))
                    print(f'  !! ID{mid} load {lp:+.1f} % > {self.load_abort:.0f} % -- jog ABORTED. '
                          f'If this is a theta motor, IDs 1/2 may be fighting (one direction '
                          f'or gear.reversing wrong) or a joint is at a mechanical stop.')
                    return False
            now = {mid: self.present(mid) for mid in goals}
            if all(abs(now[mid] - g) <= 3 for mid, g in goals.items()):
                break
            if all(last[mid] is not None and abs(now[mid] - last[mid]) <= 1 for mid in goals):
                still += 1
                if still >= 6:                              # stopped for 0.3 s
                    break
            else:
                still = 0
            last = now
        ok = True
        for mid, g in goals.items():
            p = self.present(mid)
            res = p - g
            note = ''
            if abs(res) > stall_ticks:
                ok = False
                note = f'  <-- STALLED ({abs(res) * DEG_PER_TICK / self.cfg.ratio:.2f} joint deg short)'
            elif abs(res) > 3:
                note = f'  (steady-state error {abs(res)} ticks = {abs(res) * DEG_PER_TICK / self.cfg.ratio:.2f} joint deg; P-only servo under load, normal)'
            print(f'  ID{mid}: present {p} (goal {g}), load {self.load_pct(mid):+.1f} %{note}')
        if not ok:
            print('  !! a joint did not reach its goal -- mechanical stop, fighting theta pair, or wrong sign.')
        return ok

    def status(self, homed=None):
        print('  ID   present ticks   load %   last move' + ('   joint deg (homed)' if homed else ''))
        for mid in self.ids:
            n, d = self.cfg.motor_of[mid]
            p = self.present(mid)
            mv = {1: 'UP', -1: 'DOWN', 0: '-'}[self.last_dir[mid]]
            line = f'  {mid:<3}  {p:>13}   {self.load_pct(mid):+6.1f}   {mv:<8}'
            if homed:
                line += f'  {n}: {p / self.cfg.ticks_per_joint_deg(d):8.3f}'
            print(line)

    def close(self):
        self.port.closePort()


# ----------------------------------------------------------------- capture
def angles_from_height(cfg, h, gamma_deg=0.0):
    """Joint angles (deg) for a platform at central height h and tilt gamma.
    On [0, 90] deg the height maps are monotonic, so each root is unique."""
    kin = cfg.kin
    th = kin.solve_theta(h, seed_deg=57.0, lo_deg=0.0, hi_deg=90.0)
    h_al, h_be = kin.side_targets(h, gamma_deg)
    al = kin.solve_side(h_al, gamma_deg, seed_deg=57.0, lo_deg=0.0, hi_deg=90.0)
    be = kin.solve_side(h_be, gamma_deg, seed_deg=57.0, lo_deg=0.0, hi_deg=90.0)
    if None in (th, al, be):
        return None
    return {'theta': th, 'alpha': al, 'beta': be}


def write_pose_file(path, pose_deg, homing_offsets, source):
    if not path:
        return False
    doc = {'written': time.strftime('%Y-%m-%dT%H:%M:%S'), 'source': source,
           'pose_deg': {k: round(float(v), 3) for k, v in pose_deg.items()},
           'homing_offset_ticks': {int(k): int(v) for k, v in homing_offsets.items()}}
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path + '.tmp', 'w') as fh:
            yaml.safe_dump(doc, fh, default_flow_style=False)
        os.replace(path + '.tmp', path)
        return True
    except OSError as exc:
        print(f'  !! could not write pose file {path}: {exc}')
        return False


def capture(cfg, bus, pose_deg, offsets_file):
    """Write Homing Offsets so Present == ticks(pose_deg) right now."""
    not_up = [mid for mid in bus.ids if bus.last_dir[mid] != 1]
    if not_up:
        print(f'  !! last move on ID{not_up} was not UPWARD -- run `norm` first so the backlash '
              f'is taken up on the working flank, then measure, then capture.')
        return None
    print('\nWriting Homing Offsets for pose ' + ', '.join(f'{n} {v:.2f}' for n, v in pose_deg.items()) + ':')
    record = {'gear_ratio': cfg.ratio, 'gear_sign': cfg.gsign, 'pose_deg': dict(pose_deg),
              'homing_offset_ticks': {}, 'raw_at_capture': {}, 'raw_at_zero_equiv': {}}
    ok = True
    for mid in bus.ids:
        n, d = cfg.motor_of[mid]
        raw = bus.present(mid)                      # Homing Offset is 0 during calibration
        target = int(round(pose_deg[n] * cfg.ticks_per_joint_deg(d)))
        ho = target - raw
        if abs(ho) > HOMING_OFFSET_RANGE:
            print(f'  ID{mid}: Homing Offset {ho} out of range -- aborting.')
            return None
        bus.w1(mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)     # EEPROM write needs torque off
        bus.w4(mid, ADDR_HOMING_OFFSET, ho)
        time.sleep(0.05)
        check = bus.present(mid)
        bus.w4(mid, ADDR_GOAL_POSITION, check)               # hold here
        bus.w1(mid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        status = 'OK' if abs(check - target) <= 2 else 'WARN(off by >2 ticks)'
        ok = ok and status == 'OK'
        record['homing_offset_ticks'][mid] = ho
        record['raw_at_capture'][mid] = raw
        record['raw_at_zero_equiv'][mid] = int(round(raw - pose_deg[n] * cfg.ticks_per_joint_deg(d)))
        print(f'  {n:6s} ID{mid}: raw {raw} -> HomingOffset {ho} -> Present {check} ticks = '
              f'{check / cfg.ticks_per_joint_deg(d):+.3f} joint deg (target {target}) [{status}]')
    os.makedirs(os.path.dirname(os.path.abspath(offsets_file)), exist_ok=True)
    with open(offsets_file, 'w') as fh:
        yaml.safe_dump(record, fh, default_flow_style=False)
    print(f'  calibration record -> {offsets_file}')
    if write_pose_file(cfg.pose_file, pose_deg, record['homing_offset_ticks'], 'dynamixel_calibrate'):
        print(f'  last-pose file    -> {cfg.pose_file} (the actuator\'s startup hint)')
    print('  raw_offsets_ticks for the params YAML (reference-only startup log): '
          + ', '.join(f'ID{m}={v}' for m, v in record['raw_at_zero_equiv'].items()))
    if not ok:
        print('  WARNING: a motor did not verify to its target -- re-check and re-capture.')
    return record


def envelope_report(cfg):
    print('\nSoft-limit envelope through this calibration:')
    worst = 0
    for n, j in cfg.joints.items():
        lo, hi = j['lim']
        for mid, d in zip(j['ids'], j['dirs']):
            k = cfg.ticks_per_joint_deg(d)
            t_lo, t_hi = int(round(lo * k)), int(round(hi * k))
            worst = max(worst, abs(t_lo), abs(t_hi))
            print(f'  {n:6s} ID{mid}: joint {lo:.0f}..{hi:.0f} deg = motor '
                  f'{cfg.gsign * cfg.ratio * lo:.0f}..{cfg.gsign * cfg.ratio * hi:.0f} deg '
                  f'({cfg.ratio * (hi - lo) / 360.0:.2f} rev) -> homed present {t_lo}..{t_hi} ticks')
    print(f'  worst |ticks| {worst} vs Extended Position +/-{EXT_POSITION_RANGE:,} and Homing '
          f'Offset +/-{HOMING_OFFSET_RANGE:,}: {"OK" if worst < HOMING_OFFSET_RANGE else "OUT OF RANGE"}')
    if cfg.ratio > 1.0:
        print(f'  NOTE: after a power cycle each motor knows its angle only modulo one motor '
              f'rev = {360.0 / cfg.ratio:.1f} joint deg. The actuator resolves that from the '
              f'last-pose file written above (or startup.expected_pose_deg.*).')


# ----------------------------------------------------------------- from params
SPAN_TOL_MOTOR_DEG = 2.0     # |measured span - ratio*(max-min)| allowed (0.3 joint deg at 7:1)


def verify_measurements(cfg):
    """Check the YAML end-of-range readings and derive the Homing Offsets.
    Returns (sense, offsets) or raises ValueError with the exact fix."""
    jmax, jmin = float(cfg.joint_at_max), float(cfg.joint_at_min)
    if jmax <= jmin:
        raise ValueError('calibration.joint_at_max_deg must be > joint_at_min_deg')
    expected = cfg.ratio * (jmax - jmin)
    senses, offsets = {}, {}
    print('\nMeasured end-of-range readings (Present Position, Homing Offset 0):')
    print(f'  {"ID":>3} {"joint":>6}{"motor@max":>11}{"motor@min":>11}{"span":>8}{"expected":>10}'
          f'{"ticks/jdeg":>12}')
    for mid, (n, d) in sorted(cfg.motor_of.items()):
        if mid not in cfg.motor_at_max or mid not in cfg.motor_at_min:
            raise ValueError(f'calibration.motor_deg_at_max/at_min: no entry for ID {mid}')
        mx, mn = cfg.motor_at_max[mid], cfg.motor_at_min[mid]
        span = mn - mx
        if abs(abs(span) - expected) > SPAN_TOL_MOTOR_DEG:
            raise ValueError(
                f'ID {mid}: span {span:+.1f} motor deg between the two readings, but '
                f'{cfg.ratio:g} x ({jmax:g} - {jmin:g}) = {expected:.1f} expected. Re-measure, or '
                f'fix gear.ratio / joint_at_*_deg.')
        sense = -1 if span > 0 else 1          # sign of d(ticks)/d(joint)
        senses[mid] = sense
        print(f'  {mid:>3} {n:>6}{mx:11.1f}{mn:11.1f}{span:8.1f}{expected:10.1f}'
              f'{sense * cfg.ratio / DEG_PER_TICK:12.2f}')
    if len(set(senses.values())) != 1:
        ref = senses[min(senses)]
        odd = [m for m, v in senses.items() if v != ref]
        raise ValueError(f'IDs {odd} have the opposite joint-vs-tick sense from the others: '
                         f'check their readings, or flip their motors.<joint>.directions.')
    sense = senses[min(senses)]
    bad = [mid for mid, (n, d) in cfg.motor_of.items() if d * cfg.gsign != sense]
    if bad:
        if len(bad) == len(cfg.motor_of):
            raise ValueError(
                f'measured sense is {sense:+d} ticks per +joint deg on EVERY motor, but directions x '
                f'gear.reversing give {-sense:+d}. One of the two physical facts in the YAML is '
                f'wrong: if the servos were re-mounted the other way round, flip ALL '
                f'motors.<joint>.directions; if the reducer does not actually reverse, flip '
                f'gear.reversing. (2026-09-22 build: reversing cycloid + re-mounted servos '
                f'-> directions +1, reversing true.)')
        raise ValueError(
            f'measured sense is {sense:+d} on IDs {bad} but their directions give the opposite: '
            f'flip motors.<joint>.directions for those IDs and re-run.')
    target_max = int(round(sense * cfg.ratio * jmax / DEG_PER_TICK))
    for mid in cfg.motor_of:
        raw_max = int(round(cfg.motor_at_max[mid] / DEG_PER_TICK))
        offsets[mid] = target_max - raw_max
        if abs(offsets[mid]) > HOMING_OFFSET_RANGE:
            raise ValueError(f'ID {mid}: Homing Offset {offsets[mid]} out of +/-{HOMING_OFFSET_RANGE}')
    print(f'  -> sense {sense:+d}: raw ticks {"DECREASE" if sense < 0 else "increase"} as the joint '
          f'rises; directions x gear.reversing agree.')
    return sense, offsets


def calibrate_from_params(cfg, bus, offsets_file):
    sense, offsets = verify_measurements(cfg)
    jmax, jmin = float(cfg.joint_at_max), float(cfg.joint_at_min)
    k_tick = sense * cfg.ratio / DEG_PER_TICK       # ticks per joint deg
    print('\nWriting Homing Offsets (torque OFF, nothing moves):')
    for mid in bus.ids:
        bus.w1(mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if bus.r1(mid, ADDR_OPERATING_MODE) != OP_MODE_EXTENDED_POSITION:
            bus.w1(mid, ADDR_OPERATING_MODE, OP_MODE_EXTENDED_POSITION)
        bus.w4(mid, ADDR_HOMING_OFFSET, offsets[mid])
    time.sleep(0.1)
    readback = {}
    for mid in bus.ids:
        ho = bus.r4s(mid, ADDR_HOMING_OFFSET)
        if ho != offsets[mid]:
            print(f'  !! ID{mid}: Homing Offset read back {ho}, wrote {offsets[mid]} -- aborting.')
            return None
        readback[mid] = bus.present(mid)
    # where is the platform now?  The readings' multi-turn baseline may be gone
    # (power cycle since), so the k=0 interpretation is only a suggestion.
    print(f'  {"ID":>3}{"offset":>9}{"present":>9}   joint deg if k=0   (alternatives, one motor rev apart)')
    k0 = {}
    for mid in bus.ids:
        n, _d = cfg.motor_of[mid]
        pres = readback[mid]
        q0 = pres / k_tick
        alts = ', '.join(f'{(pres - k * TICKS_PER_REV) / k_tick:.1f}' for k in (-1, 1))
        k0[n] = q0
        print(f'  {mid:>3}{offsets[mid]:9d}{pres:9d}   {q0:8.1f}          ({alts})')
    lo = min(j['lim'][0] for j in cfg.joints.values())
    hi = max(j['lim'][1] for j in cfg.joints.values())
    plausible = all(lo - 5.0 <= q <= hi + 5.0 for q in k0.values())
    print("\nWhere is the platform NOW?  This becomes the actuator's startup hint.")
    print(f'   max  = the h_max pose (joint {jmax:g})      min = the h_min pose (joint {jmin:g})')
    print('   k0   = accept the k=0 column above' + (' (plausible)' if plausible else ' (NOT plausible)'))
    print('   <deg> = a joint angle, same on all joints    skip = write no pose file')
    while True:
        try:
            ans = input('now> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            ans = 'skip'
        if ans == 'max':
            pose = {n: jmax for n in JOINTS}
        elif ans == 'min':
            pose = {n: jmin for n in JOINTS}
        elif ans == 'k0':
            pose = dict(k0)
        elif ans == 'skip':
            pose = None
        else:
            try:
                pose = {n: float(ans) for n in JOINTS}
            except ValueError:
                print('   ? max | min | k0 | <deg> | skip')
                continue
        break
    record = {'mode': 'from_params', 'gear_ratio': cfg.ratio, 'gear_sign': cfg.gsign,
              'sense_ticks_per_joint_deg': k_tick,
              'joint_at_max_deg': jmax, 'joint_at_min_deg': jmin,
              'motor_deg_at_max': cfg.motor_at_max, 'motor_deg_at_min': cfg.motor_at_min,
              'homing_offset_ticks': offsets, 'pose_deg_now': pose}
    os.makedirs(os.path.dirname(os.path.abspath(offsets_file)), exist_ok=True)
    with open(offsets_file, 'w') as fh:
        yaml.safe_dump(record, fh, default_flow_style=False)
    print(f'\n  calibration record -> {offsets_file}')
    if pose:
        if write_pose_file(cfg.pose_file, pose, offsets, 'dynamixel_calibrate'):
            print(f'  last-pose file    -> {cfg.pose_file}  '
                  f'({", ".join(f"{n} {v:.1f}" for n, v in pose.items())})')
        h = cfg.kin.h_center(pose['theta'])
        if h is not None:
            print(f'  tape-measure check: at theta {pose["theta"]:.1f} deg the platform should sit at '
                  f'h = {h:.3f} m (pivot line to pivot line, level)')
    else:
        print('  no pose file written: set startup.expected_pose_deg.* before launching the actuator.')
    print('  raw_offsets_ticks for the params YAML (reference-only): '
          + ', '.join(f'ID{m}={(-offsets[m]) % TICKS_PER_REV}' for m in sorted(offsets)))
    envelope_report(cfg)
    return record


# ----------------------------------------------------------------- sign check
def sign_check(cfg, bus, step=2.0):
    print('\nSIGN CHECK -- each joint is jogged +%.1f deg (joint) and back. Watch the crank.' % step)
    for n in JOINTS:
        if not cfg.joints[n]['ids']:
            continue
        input(f'\n>>> Ready to jog {n} (IDs {cfg.joints[n]["ids"]}) UP by {step:.1f} deg? Enter... ')
        if not bus.jog({n: +step}):
            print('  jog aborted -- fix the fault before continuing.')
            return False
        ans = input(f'>>> Did the {n} crank angle INCREASE (platform toward full extension)? [y/n] ')
        bus.jog({n: -step})
        bus.last_dir.update({mid: 0 for mid in cfg.joints[n]['ids']})
        if ans.strip().lower() != 'y':
            print(f'\n  {n} moved the WRONG way. With directions measured at -1 on the bare motor, '
                  f'the reducer sign is wrong: flip gear.reversing in dynamixel_actuator.yaml '
                  f'(if ONLY one theta motor is wrong, fix that ID in motors.theta.directions '
                  f'instead). Then re-run this tool.')
            return False
    print('\n  signs OK.')
    return True


# ----------------------------------------------------------------- REPL
def repl(cfg, bus, offsets_file):
    print('\nJOG / CALIBRATE. Type `help` for commands.')
    calibrated = None
    while True:
        try:
            cmd = input('cal> ').strip()
        except (EOFError, KeyboardInterrupt):
            cmd = 'q'
        if not cmd:
            continue
        parts = cmd.split()
        c = parts[0].lower()
        try:
            if c == 'help':
                print(__doc__.split('Commands (interactive, --jog):')[1].split('Usage:')[0])
            elif c in ('s', 'status'):
                bus.status(homed=calibrated is not None)
            elif c == 'speed' and len(parts) == 2:
                bus.set_speed(float(parts[1]))
            elif c == 'norm':
                print('  backlash normalisation: -2 deg then +2 deg on every joint')
                if bus.jog({n: -2.0 for n in JOINTS if cfg.joints[n]['ids']}):
                    bus.jog({n: +2.0 for n in JOINTS if cfg.joints[n]['ids']})
                print('  now MEASURE (height + level, or crank angles), then `cap ...`')
            elif c in ('off', 'on'):
                bus.torque(c == 'on')
            elif c in ('q', 'qon'):
                if c == 'q':
                    bus.torque(False)
                return calibrated
            elif c == 'cap':
                if len(parts) >= 3 and parts[1] == 'h':
                    h = float(parts[2])
                    g = float(parts[4]) if len(parts) >= 5 and parts[3] == 'g' else 0.0
                    pose = angles_from_height(cfg, h, g)
                    if pose is None:
                        print(f'  no joint solution for h={h} m, gamma={g} deg (reachable '
                              f'central h is about {cfg.kin.achievable_center_h_range()[0]:.3f}..'
                              f'{cfg.kin.achievable_center_h_range()[1]:.3f} m)')
                        continue
                    print(f'  h={h:.3f} m, gamma={g:.1f} deg -> theta {pose["theta"]:.2f}, '
                          f'alpha {pose["alpha"]:.2f}, beta {pose["beta"]:.2f} deg '
                          f'(dtheta/dh = {1.0 / max(1e-6, cfg.kin.h_center(pose["theta"] + 0.5) - cfg.kin.h_center(pose["theta"] - 0.5)):.0f} deg/m: '
                          f'1 mm of height error ~ {0.001 / max(1e-6, cfg.kin.h_center(pose["theta"] + 0.5) - cfg.kin.h_center(pose["theta"] - 0.5)):.2f} deg)')
                elif len(parts) == 5 and parts[1] == 'deg':
                    pose = dict(zip(JOINTS, (float(parts[2]), float(parts[3]), float(parts[4]))))
                elif len(parts) == 2 and parts[1] == 'ref':
                    pose = dict(cfg.reference)
                else:
                    print('  usage: cap h <m> [g <deg>] | cap deg <theta> <alpha> <beta> | cap ref')
                    continue
                for n in JOINTS:
                    lo, hi = cfg.joints[n]['lim']
                    if not (lo - 10.0 <= pose[n] <= hi + 10.0):
                        print(f'  !! {n} = {pose[n]:.1f} deg is far outside the soft limits '
                              f'{lo:.0f}..{hi:.0f}; check the measurement.')
                if input('  write Homing Offsets for this pose? [y/N] ').strip().lower() == 'y':
                    calibrated = capture(cfg, bus, pose, offsets_file)
                    if calibrated:
                        envelope_report(cfg)
            else:
                # jog: t+2  a-1.5  b+0.5  all+1
                key = c[0] if c[0] in 'tab' else ('all' if c.startswith('all') else None)
                rest = c[1:] if key in 'tab' and len(key) == 1 else c[3:]
                if key is None or not rest or rest[0] not in '+-':
                    print('  ? unknown command (help)')
                    continue
                ddeg = float(rest)
                if abs(ddeg) > 15.0:
                    print('  max jog step is 15 deg')
                    continue
                names = {'t': ['theta'], 'a': ['alpha'], 'b': ['beta'],
                         'all': [n for n in JOINTS if cfg.joints[n]['ids']]}[key]
                bus.jog({n: ddeg for n in names})
        except (ValueError, IndexError) as exc:
            print(f'  ? {exc}')


def main():
    ap = argparse.ArgumentParser(description='DYNAMIXEL motor-driven homing / calibration tool.')
    ap.add_argument('--params', required=True, help='dynamixel_actuator params YAML')
    ap.add_argument('--planner-params', default=None,
                    help='oscillation_planner params YAML (geometry a,b,c,d for `cap h`)')
    ap.add_argument('--geometry', nargs=4, type=float, metavar=('A', 'B', 'C', 'D'),
                    help='override geometry a b c d [m]')
    ap.add_argument('--out', default=None, help='override the calibration record path')
    ap.add_argument('--jog', action='store_true',
                    help='interactive jog / sign-check / capture tool (default when the YAML has '
                         'no calibration.motor_deg_at_* block)')
    ap.add_argument('--no-sign-check', action='store_true', help='skip the +2/-2 deg sign check')
    ap.add_argument('--keep-homing', action='store_true',
                    help='do NOT clear the Homing Offsets (jog-only session)')
    ap.add_argument('--load-abort-pct', type=float, default=45.0,
                    help='abort a jog if any Present Load exceeds this [%%]')
    args = ap.parse_args()

    cfg = Config(args.params, args.planner_params, args.geometry)
    offsets_file = os.path.expanduser(args.out) if args.out else cfg.offsets_file
    if not cfg.motor_of:
        print('ERROR: no motors found in params (motors.theta/alpha/beta).', file=sys.stderr)
        sys.exit(1)
    print(f'Gear model: ratio {cfg.ratio:g}:1, '
          f'{"REVERSING (gear_sign -1)" if cfg.gsign < 0 else "same-sense (gear_sign +1)"}; '
          f'one motor rev = {360.0 / cfg.ratio:.2f} joint deg; '
          f'geometry a={cfg.kin.a} b={cfg.kin.b} c={cfg.kin.c} d={cfg.kin.d}')
    print('  homed ticks = round(direction * %+d * %g * joint_deg * 4096/360)' % (cfg.gsign, cfg.ratio))
    print('  SAFETY: keep hands clear; a jog that raises any Present Load above '
          f'{args.load_abort_pct:.0f} % stops all motors where they are.')

    if cfg.has_measurements and not args.jog:
        try:
            verify_measurements(cfg)            # fail fast, before touching the bus
        except ValueError as exc:
            print(f'\nERROR: {exc}', file=sys.stderr)
            sys.exit(1)
        bus = Bus(cfg, args.load_abort_pct)
        rec = calibrate_from_params(cfg, bus, offsets_file)
        bus.close()
        sys.exit(0 if rec else 1)

    bus = Bus(cfg, args.load_abort_pct)
    bus.prepare(clear_homing=not args.keep_homing)
    bus.status()
    if not args.no_sign_check and not args.keep_homing:
        if not sign_check(cfg, bus):
            bus.torque(False)
            bus.close()
            sys.exit(1)
    print('\nProcedure: jog to a comfortable mid-stroke pose (gravity preload solid, e.g. '
          '55-65 deg), `norm`, measure the platform height with it LEVEL, then `cap h <m>`.')
    rec = repl(cfg, bus, offsets_file)
    bus.close()
    if rec is None:
        print('No calibration written.')


if __name__ == '__main__':
    main()
