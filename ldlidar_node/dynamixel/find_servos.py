#!/usr/bin/env python3
"""
find_servos.py  --  DYNAMIXEL baud / ID discovery utility (NOT a ROS node)

The bus is fixed at 1,000,000 baud (bench-measured), so this tool is an OPT-IN
diagnostic: by default it just PINGs (Protocol 2.0 broadcast) once at 1M and
lists the IDs that answer.  Pass --scan to sweep the whole standard baud set
when troubleshooting (e.g. after a factory reset or a mis-set baud).

Usage:
    ros2 run ldlidar_node find_servos.py --port /dev/ttyUSB0          # ping @ 1M
    ros2 run ldlidar_node find_servos.py --port /dev/ttyUSB0 --scan   # full sweep
    ./find_servos.py --port /dev/serial/by-id/usb-ROBOTIS_... --baud 57600

Prefer a stable /dev/serial/by-id/... path so it does not collide with the
LIDAR's /dev/ttyUSB0 on the same Pi.
"""

import argparse
import sys

try:
    from dynamixel_sdk import PacketHandler, PortHandler
except ImportError:
    print('ERROR: dynamixel_sdk is not installed. Add it to the ros2_lidar_image '
          '(see docker/Dockerfile.dynamixel).', file=sys.stderr)
    sys.exit(2)

# Fixed operating baud (bench-measured). Protocol 2.0 factory default is 57600.
DEFAULT_BAUD = 1000000
# Full sweep set for --scan troubleshooting.
SCAN_BAUDS = [1000000, 57600, 115200, 9600, 2000000, 3000000, 4000000, 4500000]


def probe(port_name, protocol, bauds):
    port = PortHandler(port_name)
    packet = PacketHandler(protocol)

    if not port.openPort():
        print(f'ERROR: could not open port {port_name}', file=sys.stderr)
        return None

    found = {}  # baud -> {id: model_number}
    try:
        for baud in bauds:
            if not port.setBaudRate(baud):
                print(f'  {baud:>8} baud: could not set baud, skipping')
                continue
            data_list, comm = packet.broadcastPing(port)
            ids = {}
            if data_list:
                for dxl_id in data_list:
                    model = data_list[dxl_id][0]
                    ids[dxl_id] = model
            if ids:
                found[baud] = ids
                pretty = ', '.join(
                    f'ID {i} (model {m})' for i, m in sorted(ids.items()))
                print(f'  {baud:>8} baud: FOUND {len(ids)} -> {pretty}')
            else:
                print(f'  {baud:>8} baud: (none)')
    finally:
        port.closePort()
    return found


def main():
    ap = argparse.ArgumentParser(description='Discover DYNAMIXEL servos and baud.')
    ap.add_argument('--port', default='/dev/ttyUSB0',
                    help='serial device (prefer /dev/serial/by-id/...)')
    ap.add_argument('--protocol', type=float, default=2.0,
                    help='DYNAMIXEL protocol version (default 2.0)')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                    help=f'single baud to ping (default {DEFAULT_BAUD})')
    ap.add_argument('--scan', action='store_true',
                    help='sweep the whole standard baud set (troubleshooting)')
    args = ap.parse_args()

    bauds = SCAN_BAUDS if args.scan else [args.baud]
    mode = 'full sweep' if args.scan else f'{args.baud} baud only'
    print(f'Probing {args.port} (protocol {args.protocol}) -- {mode}...')
    found = probe(args.port, args.protocol, bauds)
    if found is None:
        sys.exit(2)

    print()
    if not found:
        print('No servos responded at any probed baud. Check power, wiring and '
              'the USB2DXL adapter.')
        sys.exit(1)

    # Recommend the baud that found the most motors.
    best = max(found.items(), key=lambda kv: len(kv[1]))
    print('=== SUMMARY ===')
    for baud, ids in sorted(found.items()):
        print(f'  {baud} baud -> IDs {sorted(ids.keys())}')
    print(f'\nRecommendation: pin baudrate: {best[0]} into the actuator params '
          f'YAML (found IDs {sorted(best[1].keys())}).')


if __name__ == '__main__':
    main()
