#!/usr/bin/env python3
"""
dynamixel_calibrate.py  --  one-time homing / calibration CLI (NOT a ROS node)

Writes each motor's EEPROM HOMING OFFSET so that Present Position reads exactly
0 at the physical-zero reference pose (theta = alpha = beta = 0 deg).  After
that, the actuator's runtime tick formula collapses to:

    goal_ticks = round(direction * angle_deg * 4096/360)   (direction = -1)

Using the EEPROM Homing Offset (control-table address 20 -- VERIFY against the
XC430-W150-T e-manual) instead of a static YAML offset avoids the multi-turn
baseline drift problem: in Extended Position Control Mode a static numeric offset
could silently shift by a whole revolution (4096 ticks) across power cycles.

Procedure:
    1. Motors are switched to Extended Position Control Mode, torque DISABLED and
       Homing Offset cleared to 0 so we read the true raw position.
    2. Move the platform to the reference pose BY HAND (theta=alpha=beta=0 deg).
    3. Press Enter -- Homing Offset is written per motor so Present Position = 0
       there, then verified. A record is saved for reference.

Usage:
    ros2 run ldlidar_node dynamixel_calibrate.py \
        --params /path/to/dynamixel_actuator.yaml
"""

import argparse
import os
import sys

import yaml

try:
    from dynamixel_sdk import PacketHandler, PortHandler
except ImportError:
    print('ERROR: dynamixel_sdk is not installed. Add it to the ros2_lidar_image '
          '(see docker/Dockerfile.dynamixel).', file=sys.stderr)
    sys.exit(2)

ADDR_OPERATING_MODE = 11
ADDR_HOMING_OFFSET = 20        # EEPROM, 4 bytes, signed
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132    # 4 bytes, signed (multi-turn)
OP_MODE_EXTENDED_POSITION = 4  # VERIFY against the e-manual
TORQUE_DISABLE = 0
DEG_PER_TICK = 360.0 / 4096.0


def _to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def _extract_ros_params(doc):
    """Return the ros__parameters dict from a ROS2 params YAML (/** or node key)."""
    if not isinstance(doc, dict):
        return {}
    for key in ('/**', 'dynamixel_actuator', '/dynamixel_actuator'):
        node = doc.get(key)
        if isinstance(node, dict) and 'ros__parameters' in node:
            return node['ros__parameters']
    for node in doc.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            return node['ros__parameters']
    return doc


def load_config(params_path):
    with open(params_path, 'r') as fh:
        doc = yaml.safe_load(fh)
    p = _extract_ros_params(doc)

    port = p.get('port', '/dev/ttyUSB0')
    baud = int(p.get('baudrate', 1000000))
    protocol = float(p.get('protocol_version', 2.0))
    motors = p.get('motors', {})
    cal = p.get('calibration', {}) or {}
    ref = cal.get('reference_deg', {}) or {}
    offsets_file = cal.get('offsets_file', '~/dynamixel_offsets.yaml')

    entries = []  # (logical_name, id, direction, ref_deg)
    for name in ('theta', 'alpha', 'beta'):
        jm = motors.get(name, {}) or {}
        ids = jm.get('ids', [])
        dirs = jm.get('directions', [-1] * len(ids))
        ref_deg = float(ref.get(name, 0.0))
        for mid, d in zip(ids, dirs):
            entries.append((name, int(mid), 1 if d >= 0 else -1, ref_deg))
    return port, baud, protocol, entries, os.path.expanduser(offsets_file)


def main():
    ap = argparse.ArgumentParser(description='DYNAMIXEL homing/calibration tool.')
    ap.add_argument('--params', required=True,
                    help='path to the dynamixel_actuator params YAML')
    ap.add_argument('--out', default=None, help='override record output path')
    args = ap.parse_args()

    port_name, baud, protocol, entries, offsets_file = load_config(args.params)
    if args.out:
        offsets_file = os.path.expanduser(args.out)
    if not entries:
        print('ERROR: no motors found in params (motors.theta/alpha/beta).',
              file=sys.stderr)
        sys.exit(1)

    port = PortHandler(port_name)
    packet = PacketHandler(protocol)
    if not port.openPort() or not port.setBaudRate(baud):
        print(f'ERROR: could not open {port_name} at {baud} baud.', file=sys.stderr)
        sys.exit(2)
    print(f'Connected to {port_name} @ {baud} baud.')

    # Extended Position Control Mode + torque off + clear Homing Offset (EEPROM).
    print('Setting Extended Position Control Mode, disabling torque, clearing '
          'Homing Offset...')
    for _name, mid, _d, _ref in entries:
        packet.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packet.write1ByteTxRx(port, mid, ADDR_OPERATING_MODE, OP_MODE_EXTENDED_POSITION)
        packet.write4ByteTxRx(port, mid, ADDR_HOMING_OFFSET, 0)

    print('\nReference pose (move the platform here BY HAND):')
    for name, ref in {n: r for n, _i, _d, r in entries}.items():
        print(f'   {name}: {ref:.2f} deg')
    try:
        input('\n>>> Move to the REFERENCE POSE, then press Enter to capture... ')
    except (EOFError, KeyboardInterrupt):
        print('\nAborted.')
        port.closePort()
        sys.exit(1)

    record = {'homing_offset_ticks': {}, 'raw_at_zero': {}}
    print('\nWriting Homing Offsets:')
    ok = True
    for name, mid, direction, ref in entries:
        raw, comm, err = packet.read4ByteTxRx(port, mid, ADDR_PRESENT_POSITION)
        if comm != 0 or err != 0:
            print(f'  ID {mid}: READ FAILED (comm={comm}, err={err}) -- aborting.')
            port.closePort()
            sys.exit(2)
        raw = _to_signed32(raw)
        # Present = Actual + HomingOffset. We want Present == target at this pose.
        target = int(round(direction * ref / DEG_PER_TICK))  # 0 for ref 0 deg
        homing_offset = target - raw
        packet.write4ByteTxRx(port, mid, ADDR_HOMING_OFFSET,
                              homing_offset & 0xFFFFFFFF)
        # verify
        check, _c, _e = packet.read4ByteTxRx(port, mid, ADDR_PRESENT_POSITION)
        check = _to_signed32(check)
        status = 'OK' if abs(check - target) <= 2 else 'WARN(off by >2 ticks)'
        if status != 'OK':
            ok = False
        record['homing_offset_ticks'][mid] = homing_offset
        record['raw_at_zero'][mid] = raw
        print(f'  {name:6s} ID {mid}: raw {raw} -> HomingOffset {homing_offset} '
              f'-> Present now {check} (target {target}) [{status}]')

    os.makedirs(os.path.dirname(os.path.abspath(offsets_file)), exist_ok=True)
    with open(offsets_file, 'w') as fh:
        yaml.safe_dump(record, fh, default_flow_style=False)
    print(f'\nWrote calibration record to {offsets_file}')
    print('Homing Offset is stored in each motor\'s EEPROM (persists across power '
          'cycles); the actuator reads Present Position directly, no YAML offset.')
    if not ok:
        print('WARNING: at least one motor did not verify to ~0 -- re-check the '
              'reference pose and re-run.')
    port.closePort()


if __name__ == '__main__':
    main()
