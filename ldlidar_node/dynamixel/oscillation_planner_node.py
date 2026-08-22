#!/usr/bin/env python3
"""
oscillation_planner_node.py  --  MOTION-PLANNING LAYER

Owns the parallel-linkage mechanism geometry, inverse kinematics and the three
speed modes.  It computes theta/alpha/beta goal angles for the coordinated
move+tilt oscillation and publishes them to /joint_goal.

This node is DELIBERATELY throw-away: a future lidar_planner_node can replace it
with ZERO changes to the actuator, because the ONLY thing they share is the
message contract:
    publish  /joint_goal   sensor_msgs/JointState  name=[theta,alpha,beta]
                            position [rad], velocity [rad/s].

NOTHING about DynamixelSDK, motor IDs or raw ticks belongs in this node.  If you
find yourself wanting to `import dynamixel_sdk` here, the boundary is wrong.

Motion shape (identical every time, only the cycle time T differs):
    one full round trip  start-limit -> end-limit -> back-to-start-limit
    per invocation (not continuous unless repeat > 1).
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# =============================================================================
# MECHANISM CONSTANTS  (single labelled block -- do NOT scatter these)
# =============================================================================
# --- Baseline design point (metres) ---
DESIGN_A = 0.11
DESIGN_B = 0.44
DESIGN_C = 0.77
DESIGN_D = 0.10

# --- Baseline trajectory (the validated reference case) ---
H_START = 1.18          # m  (start limit, high)
H_END = 0.68            # m  (end limit, low)
GAMMA_START_DEG = +30.0  # tilt at start
GAMMA_END_DEG = -30.0    # tilt at end

# --- Reference case timing (defines the peak-speed constants below) ---
#  T_REF is the ONE-WAY (half-cycle / leg) sweep duration for which the joint
#  ranges & peak speeds were computed.  A full round trip = 2 * leg time.
T_REF = 3.0

# --- Known joint ranges & peak speeds @ T_REF=3.0 s leg (reference values) ---
REF_RANGE_DEG = {           # (min, max)
    'theta': (6.5, 73.3),
    'alpha': (1.9, 82.6),
    'beta': (8.1, 66.3),
}
REF_PEAK_SPEED_DEG_S = {    # peak angular speed at the reference leg time
    'theta': 34.5,
    'alpha': 42.2,
    'beta': 28.5,
}

# --- XC430-W150-T catalog (direct drive, gear_ratio = 1.0 unless overridden) ---
MOTOR_STALL_TORQUE_NM = 1.5
MOTOR_RATED_TORQUE_NM = 0.83
MOTOR_NO_LOAD_RPM = 70.0
MOTOR_RATED_CURRENT_A = 1.3

# --- Torque anchor: the ONE computed load point we have (@ T_REF=3.0 s leg) ---
#  peak ~= 1.07 N.m, rms ~= 0.70 N.m at the joint (direct drive).  For other
#  leg times we extrapolate with a gravity(constant)+inertia(1/T^2) model
#  anchored here.  Because the FAST mode's leg == T_REF, and torque is
#  non-increasing with T, every mode is at or below this validated point.
TORQUE_PEAK_REF_NM = 1.07
TORQUE_RMS_REF_NM = 0.70

# Feasibility gate thresholds (per spec):
#   refuse if estimated peak > 90% of stall, or rms > rated torque.
PEAK_TORQUE_LIMIT_NM = 0.90 * MOTOR_STALL_TORQUE_NM   # 1.35 N.m
RMS_TORQUE_LIMIT_NM = MOTOR_RATED_TORQUE_NM           # 0.83 N.m

# Mode -> leg (one-way) cycle time T [s].  Fastest == validated baseline.
DEFAULT_MODE_T = {1: 6.0, 2: 4.5, 3: 3.0}
MODE_NAME = {1: 'slow', 2: 'normal', 3: 'fast'}


class PlatformKinematics:
    """Inverse kinematics for the parallel-linkage platform.

    All angles in degrees internally.  h(angle) is non-monotonic (multiple IK
    roots) so each solve picks the branch closest to the previous solution.
    """

    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    # --- forward height maps -------------------------------------------------
    def h_center(self, theta_deg):
        """Height of the central link:  h = b sin(th) + sqrt(c^2 - (b cos(th) - (a-d)/2)^2)."""
        th = math.radians(theta_deg)
        inner = self.c ** 2 - (self.b * math.cos(th) - (self.a - self.d) / 2.0) ** 2
        if inner < 0.0:
            return None
        return self.b * math.sin(th) + math.sqrt(inner)

    def h_side(self, angle_deg, gamma_deg):
        """Height of a side link at tilt gamma:
        h = b sin(a) + sqrt(c^2 - (b cos(a) - (a - d cos(g))/2)^2)."""
        ang = math.radians(angle_deg)
        gam = math.radians(gamma_deg)
        inner = self.c ** 2 - (
            self.b * math.cos(ang) - (self.a - self.d * math.cos(gam)) / 2.0) ** 2
        if inner < 0.0:
            return None
        return self.b * math.sin(ang) + math.sqrt(inner)

    # --- root finding --------------------------------------------------------
    @staticmethod
    def _roots(func, target, lo_deg=0.0, hi_deg=90.0, step_deg=0.5):
        """Return all angle-deg roots of func(angle) == target in [lo, hi]."""
        roots = []
        prev_a = lo_deg
        prev_f = func(prev_a)
        a = lo_deg + step_deg
        while a <= hi_deg + 1e-9:
            f = func(a)
            if prev_f is not None and f is not None:
                if (prev_f - target) == 0.0:
                    roots.append(prev_a)
                elif (prev_f - target) * (f - target) < 0.0:
                    # bisect the bracket [prev_a, a]
                    xa, xb = prev_a, a
                    fa = prev_f - target
                    for _ in range(60):
                        xm = 0.5 * (xa + xb)
                        fm = func(xm)
                        if fm is None:
                            break
                        fm -= target
                        if abs(fm) < 1e-9 or (xb - xa) < 1e-5:
                            break
                        if fa * fm < 0.0:
                            xb = xm
                        else:
                            xa, fa = xm, fm
                    roots.append(0.5 * (xa + xb))
            prev_a, prev_f = a, f
            a += step_deg
        return roots

    def _solve(self, func, target, seed_deg, lo_deg, hi_deg):
        roots = self._roots(func, target, lo_deg, hi_deg)
        if not roots:
            return None
        # pick the branch closest to the previous solution
        return min(roots, key=lambda r: abs(r - seed_deg))

    def solve(self, h_center_target, gamma_deg, seed=None,
              lo_deg=0.0, hi_deg=90.0):
        """Solve theta/alpha/beta (deg) for a target central height and tilt.

        Coupling:  h_alpha = h + (d/2) sin(g),  h_beta = h - (d/2) sin(g).
        `seed` is the previous (theta, alpha, beta) solution used to stay on the
        same IK branch and avoid solution jumps.
        """
        gam = math.radians(gamma_deg)
        h_alpha = h_center_target + (self.d / 2.0) * math.sin(gam)
        h_beta = h_center_target - (self.d / 2.0) * math.sin(gam)

        seed_th = seed[0] if seed else 40.0
        seed_al = seed[1] if seed else 40.0
        seed_be = seed[2] if seed else 37.0

        theta = self._solve(self.h_center, h_center_target, seed_th, lo_deg, hi_deg)
        alpha = self._solve(lambda x: self.h_side(x, gamma_deg), h_alpha,
                            seed_al, lo_deg, hi_deg)
        beta = self._solve(lambda x: self.h_side(x, gamma_deg), h_beta,
                           seed_be, lo_deg, hi_deg)
        if None in (theta, alpha, beta):
            return None
        return (theta, alpha, beta)


class OscillationPlannerNode(Node):
    def __init__(self):
        super().__init__('oscillation_planner')

        # ---- parameters -----------------------------------------------------
        self.declare_parameter('mode', 2)                    # 1|2|3
        self.declare_parameter('repeat', 1)                  # full round trips
        self.declare_parameter('goal_topic', '/joint_goal')
        self.declare_parameter('state_topic', '/joint_states')

        # geometry (baseline design point)
        self.declare_parameter('geometry.a', DESIGN_A)
        self.declare_parameter('geometry.b', DESIGN_B)
        self.declare_parameter('geometry.c', DESIGN_C)
        self.declare_parameter('geometry.d', DESIGN_D)

        # trajectory
        self.declare_parameter('trajectory.h_start', H_START)
        self.declare_parameter('trajectory.h_end', H_END)
        self.declare_parameter('trajectory.gamma_start_deg', GAMMA_START_DEG)
        self.declare_parameter('trajectory.gamma_end_deg', GAMMA_END_DEG)

        # per-mode leg (one-way) cycle times
        self.declare_parameter('mode_times.mode1_s', DEFAULT_MODE_T[1])
        self.declare_parameter('mode_times.mode2_s', DEFAULT_MODE_T[2])
        self.declare_parameter('mode_times.mode3_s', DEFAULT_MODE_T[3])

        # torque/feasibility model
        self.declare_parameter('gear_ratio', 1.0)            # motor:joint (direct=1.0)
        self.declare_parameter('gravity_torque_fraction', 0.5)

        # arrival detection
        self.declare_parameter('arrival_tolerance_deg', 1.5)
        self.declare_parameter('start_settle_s', 3.0)        # extra time for creep-to-start

        self.mode = int(self.get_parameter('mode').value)
        self.repeat = int(self.get_parameter('repeat').value)
        self.goal_topic = self.get_parameter('goal_topic').value
        self.state_topic = self.get_parameter('state_topic').value

        self.kin = PlatformKinematics(
            self.get_parameter('geometry.a').value,
            self.get_parameter('geometry.b').value,
            self.get_parameter('geometry.c').value,
            self.get_parameter('geometry.d').value)

        self.h_start = float(self.get_parameter('trajectory.h_start').value)
        self.h_end = float(self.get_parameter('trajectory.h_end').value)
        self.gamma_start = float(self.get_parameter('trajectory.gamma_start_deg').value)
        self.gamma_end = float(self.get_parameter('trajectory.gamma_end_deg').value)

        self.mode_t = {
            1: float(self.get_parameter('mode_times.mode1_s').value),
            2: float(self.get_parameter('mode_times.mode2_s').value),
            3: float(self.get_parameter('mode_times.mode3_s').value),
        }
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.gravity_fraction = float(self.get_parameter('gravity_torque_fraction').value)
        self.arrival_tol_deg = float(self.get_parameter('arrival_tolerance_deg').value)
        self.start_settle_s = float(self.get_parameter('start_settle_s').value)

        if self.mode not in (1, 2, 3):
            self.get_logger().fatal(f'Invalid mode {self.mode} (must be 1, 2 or 3).')
            raise RuntimeError('invalid mode')

        if abs(self.gear_ratio - 1.0) > 1e-6:
            self.get_logger().warn(
                f'gear_ratio={self.gear_ratio} (NOT direct drive). Torque estimates '
                f'scaled by 1/gear_ratio and speeds by gear_ratio -- confirm this is '
                f'the real build.')

        # ---- solve start/end poses (branch-consistent) ----------------------
        self.pose_start = self.kin.solve(self.h_start, self.gamma_start)
        if self.pose_start is None:
            self.get_logger().fatal('IK failed for the START pose. Check geometry.')
            raise RuntimeError('IK start failed')
        self.pose_end = self.kin.solve(self.h_end, self.gamma_end, seed=self.pose_start)
        if self.pose_end is None:
            self.get_logger().fatal('IK failed for the END pose. Check geometry.')
            raise RuntimeError('IK end failed')

        self.get_logger().info(
            'IK poses (theta, alpha, beta) deg:  '
            f'start=({self.pose_start[0]:.1f}, {self.pose_start[1]:.1f}, '
            f'{self.pose_start[2]:.1f})  '
            f'end=({self.pose_end[0]:.1f}, {self.pose_end[1]:.1f}, '
            f'{self.pose_end[2]:.1f})')

        # ---- feasibility check for ALL modes; gate the selected one ---------
        self.feasibility = {m: self._assess_mode(m) for m in (1, 2, 3)}
        self._log_feasibility()

        chosen = self.feasibility[self.mode]
        if not chosen['feasible']:
            fallback = self._fastest_feasible()
            if fallback is None:
                self.get_logger().fatal(
                    'No mode passes the torque feasibility check. Refusing to '
                    'publish any goals. Re-check geometry / trajectory / gearing.')
                raise RuntimeError('no feasible mode')
            self.get_logger().error(
                f'Mode {self.mode} ({MODE_NAME[self.mode]}) FAILS the torque check '
                f'(peak {chosen["peak_nm"]:.2f} N.m, rms {chosen["rms_nm"]:.2f} N.m). '
                f'Falling back to the fastest feasible mode {fallback} '
                f'({MODE_NAME[fallback]}, T={self.mode_t[fallback]:.1f} s).')
            self.mode = fallback
            chosen = self.feasibility[self.mode]

        self.leg_time = self.mode_t[self.mode]
        self.leg_peak_speed = chosen['peak_speed_rad_s']  # per-joint {name: rad/s}
        self.get_logger().info(
            f'Selected mode {self.mode} ({MODE_NAME[self.mode]}): leg T='
            f'{self.leg_time:.1f} s, full round trip {2*self.leg_time:.1f} s, '
            f'repeat={self.repeat}.')

        # ---- ROS interfaces -------------------------------------------------
        self.goal_pub = self.create_publisher(JointState, self.goal_topic, 10)
        self.state_sub = self.create_subscription(
            JointState, self.state_topic, self._on_state, 10)
        self.last_state = None  # {name: pos_deg}

        # ---- sequencer state machine ---------------------------------------
        # legs: first move to START (settle), then repeat x (END, START)
        self._legs = [('start', self.start_settle_s)]
        for _ in range(max(1, self.repeat)):
            self._legs.append(('end', self.leg_time))
            self._legs.append(('start', self.leg_time))
        self._leg_index = -1
        self._leg_deadline = None
        self._current_goal_deg = None
        self._finished = False

        self.control_timer = self.create_timer(0.05, self._tick)  # 20 Hz sequencer
        self.get_logger().info(
            f'Publishing oscillation to {self.goal_topic}. '
            f'{len(self._legs)} legs queued.')

    # ------------------------------------------------------- feasibility model
    def _sample_leg_kinematics(self, leg_time, n=200):
        """Sample one leg (start<->end) with a raised-cosine profile and return
        per-joint peak angular speed (rad/s) and peak |accel| (rad/s^2)."""
        prev = self.pose_start
        angles = {'theta': [], 'alpha': [], 'beta': []}
        for i in range(n + 1):
            s = 0.5 * (1.0 - math.cos(math.pi * i / n))  # 0->1 smooth
            h = self.h_start + s * (self.h_end - self.h_start)
            g = self.gamma_start + s * (self.gamma_end - self.gamma_start)
            sol = self.kin.solve(h, g, seed=prev)
            if sol is None:
                sol = prev
            prev = sol
            angles['theta'].append(math.radians(sol[0]))
            angles['alpha'].append(math.radians(sol[1]))
            angles['beta'].append(math.radians(sol[2]))

        dt = leg_time / n
        peak_speed = {}
        peak_accel = {}
        for name, seq in angles.items():
            vel = [(seq[i + 1] - seq[i]) / dt for i in range(n)]
            acc = [(vel[i + 1] - vel[i]) / dt for i in range(len(vel) - 1)]
            peak_speed[name] = max(abs(v) for v in vel) if vel else 0.0
            peak_accel[name] = max(abs(a) for a in acc) if acc else 0.0
        return peak_speed, peak_accel

    def _assess_mode(self, mode):
        leg_time = self.mode_t[mode]
        peak_speed, _peak_accel = self._sample_leg_kinematics(leg_time)

        # Max joint speed -> RPM at the joint, then at the motor (x gear_ratio).
        max_joint_rad_s = max(peak_speed.values())
        joint_rpm = max_joint_rad_s * 60.0 / (2.0 * math.pi)
        motor_rpm = joint_rpm * self.gear_ratio

        # Torque model anchored at (T_REF -> peak/rms ref).  Gravity term is
        # T-independent; inertial term scales with 1/T^2 (fixed motion shape).
        ratio2 = (T_REF / leg_time) ** 2
        f = max(0.0, min(1.0, self.gravity_fraction))
        peak_load = TORQUE_PEAK_REF_NM * (f + (1.0 - f) * ratio2)
        rms_load = TORQUE_RMS_REF_NM * (f + (1.0 - f) * ratio2)
        # motor torque = load / gear_ratio (reduction multiplies output torque)
        peak_motor = peak_load / self.gear_ratio
        rms_motor = rms_load / self.gear_ratio

        feasible = (
            peak_motor <= PEAK_TORQUE_LIMIT_NM + 1e-9
            and rms_motor <= RMS_TORQUE_LIMIT_NM + 1e-9
            and motor_rpm <= MOTOR_NO_LOAD_RPM
        )
        return {
            'leg_time': leg_time,
            'peak_speed_rad_s': peak_speed,
            'max_rpm': motor_rpm,
            'peak_nm': peak_motor,
            'rms_nm': rms_motor,
            'feasible': feasible,
        }

    def _fastest_feasible(self):
        # fastest = smallest leg time among feasible modes
        feas = [m for m in (1, 2, 3) if self.feasibility[m]['feasible']]
        if not feas:
            return None
        return min(feas, key=lambda m: self.mode_t[m])

    def _log_feasibility(self):
        self.get_logger().info('=== torque / speed feasibility (per mode) ===')
        self.get_logger().info(
            f'  motor: stall {MOTOR_STALL_TORQUE_NM} N.m, rated '
            f'{MOTOR_RATED_TORQUE_NM} N.m, no-load {MOTOR_NO_LOAD_RPM} RPM | '
            f'gates: peak<={PEAK_TORQUE_LIMIT_NM:.2f} N.m, rms<={RMS_TORQUE_LIMIT_NM:.2f} N.m')
        for m in (1, 2, 3):
            f = self.feasibility[m]
            verdict = 'PASS' if f['feasible'] else 'FAIL'
            self.get_logger().info(
                f'  mode {m} ({MODE_NAME[m]:6s}) leg T={f["leg_time"]:.1f}s: '
                f'peak {f["peak_nm"]:.2f} N.m ({100*f["peak_nm"]/MOTOR_STALL_TORQUE_NM:.0f}% '
                f'stall), rms {f["rms_nm"]:.2f} N.m, max {f["max_rpm"]:.1f} RPM -> {verdict}')
        self.get_logger().info('=============================================')

    # ------------------------------------------------------------- sequencer
    def _on_state(self, msg: JointState):
        state = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                state[name] = math.degrees(msg.position[i])
        self.last_state = state

    def _arrived(self):
        if self._current_goal_deg is None or self.last_state is None:
            return False
        for name, goal in self._current_goal_deg.items():
            if name not in self.last_state:
                return False
            if abs(self.last_state[name] - goal) > self.arrival_tol_deg:
                return False
        return True

    def _publish_pose(self, pose_deg, per_joint_speed):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['theta', 'alpha', 'beta']
        msg.position = [math.radians(pose_deg[0]),
                        math.radians(pose_deg[1]),
                        math.radians(pose_deg[2])]
        msg.velocity = [per_joint_speed['theta'],
                        per_joint_speed['alpha'],
                        per_joint_speed['beta']]
        self.goal_pub.publish(msg)
        self._current_goal_deg = {
            'theta': pose_deg[0], 'alpha': pose_deg[1], 'beta': pose_deg[2]}

    def _tick(self):
        if self._finished:
            return
        now = self.get_clock().now()

        # advance to next leg when the previous one is done (arrived or timeout)
        if self._leg_deadline is None or self._arrived() or now >= self._leg_deadline:
            self._leg_index += 1
            if self._leg_index >= len(self._legs):
                self.get_logger().info('Oscillation sequence complete.')
                self._finished = True
                return
            which, duration = self._legs[self._leg_index]
            pose = self.pose_start if which == 'start' else self.pose_end
            # first "start" leg is the creep-to-start; use conservative speeds
            speeds = self.leg_peak_speed
            self._publish_pose(pose, speeds)
            self._leg_deadline = now + rclpy.duration.Duration(seconds=duration)
            self.get_logger().info(
                f'Leg {self._leg_index + 1}/{len(self._legs)}: -> {which} pose '
                f'(<= {duration:.1f} s).')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OscillationPlannerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if isinstance(exc, RuntimeError):
            print(f'[oscillation_planner] fatal: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
