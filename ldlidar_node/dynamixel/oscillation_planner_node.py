#!/usr/bin/env python3
"""
oscillation_planner_node.py  --  MOTION-PLANNING LAYER

Owns the parallel-linkage mechanism geometry and inverse kinematics.  It drives
a smooth elliptical move+tilt oscillation and publishes theta/alpha/beta goals
to /joint_goal.

This node is DELIBERATELY throw-away: a future lidar_planner_node can replace it
with ZERO changes to the actuator, because the ONLY thing they share is the
message contract:
    publish  /joint_goal   sensor_msgs/JointState  name=[theta,alpha,beta]
                            position [rad], velocity [rad/s].

NOTHING about DynamixelSDK, motor IDs or raw ticks belongs in this node.  If you
find yourself wanting to `import dynamixel_sdk` here, the boundary is wrong.

Motion shape (step 2): a single continuous closed ELLIPSE in (h, gamma) space
    h(phi)     = h_mid + (stroke/2) * cos(phi)
    gamma(phi) = gamma_amp * sin(phi)          phi in [0, 2*pi)
with h_mid=(h_high+h_low)/2, stroke=h_high-h_low.  h_high/h_low are only reached
at gamma=0; the tilt extremes occur at h_mid -- the ellipse stays inside the
achievable envelope (which narrows toward h_mid as |gamma| grows) by construction.

Runtime IK (step 3): NO live root finding.  A precomputed lookup table (built
offline by generate_ik_lut.py, cached to disk) is queried by bilinear/linear
interpolation -- O(1) and fully deterministic, the right tool for the tight
reactive loop this is meant to become.
"""

import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from platform_kinematics import PlatformKinematics
    from generate_ik_lut import (IKTable, build_lut, load_lut, save_lut,
                                  signature_matches)
except ImportError:
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from platform_kinematics import PlatformKinematics
    from generate_ik_lut import (IKTable, build_lut, load_lut, save_lut,
                                  signature_matches)


# =============================================================================
# MECHANISM / MOTOR CONSTANTS  (single labelled block -- do NOT scatter these)
# =============================================================================
DESIGN_A = 0.11
DESIGN_B = 0.44
DESIGN_C = 0.77
DESIGN_D = 0.10

H_HIGH = 1.18            # m  (top of the stroke, reached at gamma=0)
H_LOW = 0.65            # m  (bottom of the stroke, reached at gamma=0; 15 mm above
                        #     the true 0.635 m floor -- alpha/beta binding, no
                        #     singularity there, dh/dangle ~ 0.44 m/rad at the floor)
GAMMA_AMP_DEG = 30.0    # tilt amplitude (reached at h_mid)

# --- Reference LEG (one-way) motion that anchors the torque model -------------
#  The validated baseline is the 1.18<->0.68 m, gamma +amp<->-amp one-way sweep
#  at T_REF; it produces peak ~1.07 N.m / rms ~0.70 N.m at the joint (direct
#  drive).  We anchor the (shape-aware) torque model on THIS motion's peak/rms
#  joint acceleration, then scale a candidate motion's torque by its acceleration
#  ratio (inertial term) plus a constant gravity term.
#
#  CRITICAL: the reference leg is PINNED to these fixed constants, NOT to the
#  trajectory's h_low/h_high. The 1.07/0.70 anchor was measured for the 0.68
#  stroke; if the reference tracked trajectory.h_low, then widening the stroke
#  (e.g. h_low 0.68->0.65) would grow the denominator and UNDER-report torque for
#  a MORE demanding motion -- the ratio would fall when the load actually rises.
T_REF = 3.0
TORQUE_PEAK_REF_NM = 1.07
TORQUE_RMS_REF_NM = 0.70
H_REF_HIGH = 1.18       # pinned reference-leg stroke top (do NOT tie to trajectory)
H_REF_LOW = 0.68        # pinned reference-leg stroke bottom (the validated baseline)

# --- XC430-W150-T catalog (direct drive, gear_ratio = 1.0 unless overridden) --
MOTOR_STALL_TORQUE_NM = 1.5
MOTOR_RATED_TORQUE_NM = 0.83
MOTOR_NO_LOAD_RPM = 70.0

# Feasibility gate: refuse if estimated peak > 90% of stall, or rms > rated.
PEAK_TORQUE_LIMIT_NM = 0.90 * MOTOR_STALL_TORQUE_NM   # 1.35 N.m
RMS_TORQUE_LIMIT_NM = MOTOR_RATED_TORQUE_NM           # 0.83 N.m

# 3 laps, each independently timed. The ellipse adds a full tilt oscillation
# (gamma 0->+amp->0->-amp->0 per lap) that the old monotonic leg lacked, so the
# side links see ~1.5x the inertial load of the leg baseline; the fast lap (7.0s)
# is chosen for comfortable margin under the 90%-stall gate. A lap that still
# fails the gate is auto-slowed (loud warning) to the fastest duration that passes.
DEFAULT_LAP_DURATIONS_S = [12.0, 9.0, 7.0]


class OscillationPlannerNode(Node):
    def __init__(self):
        super().__init__('oscillation_planner')

        # ---- parameters -----------------------------------------------------
        self.declare_parameter('goal_topic', '/joint_goal')
        self.declare_parameter('state_topic', '/joint_states')

        self.declare_parameter('geometry.a', DESIGN_A)
        self.declare_parameter('geometry.b', DESIGN_B)
        self.declare_parameter('geometry.c', DESIGN_C)
        self.declare_parameter('geometry.d', DESIGN_D)

        self.declare_parameter('trajectory.h_high', H_HIGH)
        self.declare_parameter('trajectory.h_low', H_LOW)
        self.declare_parameter('trajectory.gamma_amplitude_deg', GAMMA_AMP_DEG)

        # 3 laps per invocation, each independently timed (step 2).
        self.declare_parameter('lap_durations_s', DEFAULT_LAP_DURATIONS_S)

        # LUT (step 3)
        self.declare_parameter('lut.h_step_m', 0.005)
        self.declare_parameter('lut.gamma_step_deg', 1.0)
        self.declare_parameter('lut.gamma_amp_deg', GAMMA_AMP_DEG)
        self.declare_parameter('lut.cache_path', '~/.cache/ldlidar_platform/ik_lut.npz')
        self.declare_parameter('lut.force_rebuild', False)

        # streaming / profile
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('start_settle_s', 3.0)
        self.declare_parameter('min_profile_velocity_rad_s', 0.05)
        self.declare_parameter('max_profile_velocity_rad_s', 6.0)
        self.declare_parameter('creep_velocity_rad_s', 0.15)

        # torque/feasibility model
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('gravity_torque_fraction', 0.5)

        self.goal_topic = self.get_parameter('goal_topic').value
        self.state_topic = self.get_parameter('state_topic').value

        a = self.get_parameter('geometry.a').value
        b = self.get_parameter('geometry.b').value
        c = self.get_parameter('geometry.c').value
        d = self.get_parameter('geometry.d').value
        self.kin = PlatformKinematics(a, b, c, d)

        self.h_high = float(self.get_parameter('trajectory.h_high').value)
        self.h_low = float(self.get_parameter('trajectory.h_low').value)
        self.gamma_amp = float(self.get_parameter('trajectory.gamma_amplitude_deg').value)
        self.h_mid = 0.5 * (self.h_high + self.h_low)
        self.stroke = self.h_high - self.h_low

        self.lap_durations = [float(x) for x in
                              self.get_parameter('lap_durations_s').value]
        if not self.lap_durations:
            self.lap_durations = list(DEFAULT_LAP_DURATIONS_S)

        self.lut_h_step = float(self.get_parameter('lut.h_step_m').value)
        self.lut_gamma_step = float(self.get_parameter('lut.gamma_step_deg').value)
        self.lut_gamma_amp = float(self.get_parameter('lut.gamma_amp_deg').value)
        self.lut_cache = os.path.expanduser(self.get_parameter('lut.cache_path').value)
        self.lut_force = bool(self.get_parameter('lut.force_rebuild').value)

        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.start_settle_s = float(self.get_parameter('start_settle_s').value)
        self.min_vel = float(self.get_parameter('min_profile_velocity_rad_s').value)
        self.max_vel = float(self.get_parameter('max_profile_velocity_rad_s').value)
        self.creep_vel = float(self.get_parameter('creep_velocity_rad_s').value)

        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.gravity_fraction = float(self.get_parameter('gravity_torque_fraction').value)

        if self.gamma_amp > self.lut_gamma_amp + 1e-6:
            self.get_logger().warn(
                f'trajectory gamma amplitude {self.gamma_amp} deg exceeds the LUT '
                f'gamma range +/-{self.lut_gamma_amp} deg; tilt will be clamped.')

        # ---- inverse-kinematics LUT (load cache or build offline) -----------
        self.table = self._load_or_build_lut(a, b, c, d)

        # ---- start pose (phi=0: h_high, gamma=0) ----------------------------
        start = self.table.solve_deg(self.h_high, 0.0)
        if start is None:
            self.get_logger().fatal(
                'IK LUT has no solution for the start pose (h_high, gamma=0). '
                'Check geometry / trajectory.')
            raise RuntimeError('start pose unreachable')
        self.start_pose = start

        self._log_trajectory()

        # ---- feasibility check for every lap; gate/adjust -------------------
        self.ref_peak_accel, self.ref_rms_accel = self._reference_leg_accel()
        self.lap_plans = self._plan_laps()

        # ---- ROS interfaces -------------------------------------------------
        self.goal_pub = self.create_publisher(JointState, self.goal_topic, 10)
        self.state_sub = self.create_subscription(
            JointState, self.state_topic, self._on_state, 10)
        self.last_state = None

        # ---- sequencer: creep-to-start, then the timed laps -----------------
        self._phases = [('creep', self.start_settle_s)]
        for dur in self.lap_plans:
            self._phases.append(('lap', dur))
        self._phase_index = -1
        self._phase_start = None
        self._finished = False

        self._dt_ctrl = 1.0 / max(1.0, self.control_rate)
        self.control_timer = self.create_timer(self._dt_ctrl, self._tick)
        self.get_logger().info(
            f'Publishing elliptical oscillation to {self.goal_topic}: '
            f'{len(self.lap_plans)} laps at {self.control_rate:.0f} Hz.')

    # ---------------------------------------------------------------- LUT
    def _load_or_build_lut(self, a, b, c, d):
        want = (a, b, c, d, self.lut_h_step, self.lut_gamma_step,
                self.lut_gamma_amp, 0.0, 90.0)
        if not self.lut_force and os.path.isfile(self.lut_cache):
            try:
                lut = load_lut(self.lut_cache)
                if signature_matches(lut, *want):
                    self.get_logger().info(
                        f'Loaded IK LUT from cache: {self.lut_cache} '
                        f'(theta {lut["theta_angle"].shape[0]}, alpha/beta '
                        f'{lut["alpha_angle"].shape[0]}x{lut["alpha_angle"].shape[1]}).')
                    return IKTable(lut)
                self.get_logger().warn(
                    'Cached IK LUT signature does not match current geometry/grid '
                    '-- rebuilding.')
            except Exception as exc:  # noqa: BLE001 - cache is best-effort
                self.get_logger().warn(f'Could not read IK LUT cache ({exc}); rebuilding.')

        self.get_logger().info('Building IK LUT (one-time, precise root finder)...')
        lut = build_lut(a, b, c, d, self.lut_h_step, self.lut_gamma_step,
                        self.lut_gamma_amp, log=lambda m: self.get_logger().info(m))
        try:
            path = save_lut(lut, self.lut_cache)
            self.get_logger().info(f'Cached IK LUT to {path}.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not cache IK LUT ({exc}); continuing in-memory.')
        return IKTable(lut)

    # --------------------------------------------------------- trajectory
    def _ellipse_point(self, phi):
        return (self.h_mid + 0.5 * self.stroke * math.cos(phi),
                self.gamma_amp * math.sin(phi))

    def _log_trajectory(self):
        # sample the ellipse via the LUT for the achieved joint ranges
        rng = {'theta': [1e9, -1e9], 'alpha': [1e9, -1e9], 'beta': [1e9, -1e9]}
        n = 720
        for k in range(n):
            phi = 2.0 * math.pi * k / n
            h, g = self._ellipse_point(phi)
            sol = self.table.solve_deg(h, g)
            if sol is None:
                continue
            for name, v in zip(('theta', 'alpha', 'beta'), sol):
                rng[name][0] = min(rng[name][0], v)
                rng[name][1] = max(rng[name][1], v)
        self.get_logger().info(
            f'Ellipse: h_mid={self.h_mid:.3f} m, stroke={self.stroke:.3f} m, '
            f'gamma=+/-{self.gamma_amp:.0f} deg  (h {self.h_low:.2f}..{self.h_high:.2f}).')
        self.get_logger().info(
            'Achieved joint ranges over the ellipse (deg, via LUT): '
            f'theta {rng["theta"][0]:.1f}..{rng["theta"][1]:.1f}, '
            f'alpha {rng["alpha"][0]:.1f}..{rng["alpha"][1]:.1f}, '
            f'beta {rng["beta"][0]:.1f}..{rng["beta"][1]:.1f}.')

    # -------------------------------------------------------- feasibility
    def _reference_leg_accel(self):
        """Per-joint peak & rms |accel| of the validated one-way leg at T_REF.

        Uses the precise root finder (startup-only) so the anchor is exact.

        PINNED to the fixed H_REF_* / GAMMA_AMP_DEG baseline, NOT the trajectory
        params: this leg is the physical motion that was measured at 1.07/0.70
        N.m. Tying it to self.h_low would let a wider stroke shrink the anchor's
        acceleration and silently UNDER-report the torque of the bigger motion.
        """
        n = 600
        seqs = {'theta': [], 'alpha': [], 'beta': []}
        seed = None
        for i in range(n + 1):
            s = 0.5 * (1.0 - math.cos(math.pi * i / n))   # raised cosine, one way
            h = H_REF_HIGH + s * (H_REF_LOW - H_REF_HIGH)
            g = GAMMA_AMP_DEG + s * (-GAMMA_AMP_DEG - GAMMA_AMP_DEG)
            sol = self.kin.solve(h, g, seed=seed)
            if sol is None:
                sol = seed if seed else (0.0, 0.0, 0.0)
            seed = sol
            for name, v in zip(('theta', 'alpha', 'beta'), sol):
                seqs[name].append(math.radians(v))
        return self._accel_stats(seqs, T_REF)

    @staticmethod
    def _accel_stats(seqs, duration):
        dt = duration / (len(next(iter(seqs.values()))) - 1)
        peak, rms = {}, {}
        for name, s in seqs.items():
            vel = [(s[i + 1] - s[i]) / dt for i in range(len(s) - 1)]
            acc = [(vel[i + 1] - vel[i]) / dt for i in range(len(vel) - 1)]
            peak[name] = max((abs(x) for x in acc), default=0.0)
            rms[name] = math.sqrt(sum(x * x for x in acc) / len(acc)) if acc else 0.0
        return peak, rms

    def _sample_lap(self, duration, n=360):
        """Per-joint peak speed, peak & rms |accel| of the ellipse over one lap.

        Uses the PRECISE root finder, NOT the runtime LUT: the LUT stores
        tick-quantized angles, and double-differencing that staircase at a fine
        dt manufactures phantom acceleration spikes. The physically meaningful
        torque comes from the smooth trajectory (the firmware executes a
        trapezoidal profile between waypoints, it does not jerk per tick). This
        is a startup-only computation, so the slow precise solve is fine.
        """
        seqs = {'theta': [], 'alpha': [], 'beta': []}
        seed = self.start_pose
        for i in range(n + 1):
            phi = 2.0 * math.pi * i / n
            h, g = self._ellipse_point(phi)
            sol = self.kin.solve(h, g, seed=seed) or seed
            seed = sol
            for name, v in zip(('theta', 'alpha', 'beta'), sol):
                seqs[name].append(math.radians(v))
        dt = duration / n
        peak_speed = {}
        for name, s in seqs.items():
            vel = [(s[i + 1] - s[i]) / dt for i in range(len(s) - 1)]
            peak_speed[name] = max((abs(x) for x in vel), default=0.0)
        peak_acc, rms_acc = self._accel_stats(seqs, duration)
        return peak_speed, peak_acc, rms_acc

    def _assess_lap(self, duration):
        peak_speed, peak_acc, rms_acc = self._sample_lap(duration)
        inertia_peak = max(peak_acc[n] / self.ref_peak_accel[n]
                           for n in peak_acc if self.ref_peak_accel[n] > 1e-9)
        inertia_rms = max(rms_acc[n] / self.ref_rms_accel[n]
                          for n in rms_acc if self.ref_rms_accel[n] > 1e-9)
        f = max(0.0, min(1.0, self.gravity_fraction))
        peak_nm = TORQUE_PEAK_REF_NM * (f + (1.0 - f) * inertia_peak) / self.gear_ratio
        rms_nm = TORQUE_RMS_REF_NM * (f + (1.0 - f) * inertia_rms) / self.gear_ratio
        max_joint_rad_s = max(peak_speed.values())
        motor_rpm = max_joint_rad_s * 60.0 / (2.0 * math.pi) * self.gear_ratio
        feasible = (peak_nm <= PEAK_TORQUE_LIMIT_NM + 1e-9
                    and rms_nm <= RMS_TORQUE_LIMIT_NM + 1e-9
                    and motor_rpm <= MOTOR_NO_LOAD_RPM)
        return {'duration': duration, 'peak_nm': peak_nm, 'rms_nm': rms_nm,
                'rpm': motor_rpm, 'inertia_peak': inertia_peak,
                'inertia_rms': inertia_rms, 'feasible': feasible}

    def _min_feasible_duration(self, assess):
        """Slowest-limited duration >= requested that passes, using accel ~ 1/T^2.

        inertia scales as (duration/T)^2, speed as (duration/T); solve each gate
        for T and take the max. Gravity-only always passes, so this converges.
        """
        d0 = assess['duration']
        f = max(0.0, min(1.0, self.gravity_fraction))
        needed = [d0]
        # peak torque gate:  Tpeak_ref*(f + (1-f)*Ip*(d0/T)^2)/gear <= LIMIT
        for ref, inertia, limit in (
                (TORQUE_PEAK_REF_NM, assess['inertia_peak'], PEAK_TORQUE_LIMIT_NM),
                (TORQUE_RMS_REF_NM, assess['inertia_rms'], RMS_TORQUE_LIMIT_NM)):
            grav = ref * f / self.gear_ratio
            if limit <= grav:      # gravity alone already exceeds -> infeasible
                return None
            coeff = ref * (1.0 - f) * inertia / self.gear_ratio  # * (d0/T)^2
            if coeff > 1e-12:
                needed.append(d0 * math.sqrt(coeff / (limit - grav)))
        # speed gate: rpm0*(d0/T) <= NO_LOAD -> T >= d0*rpm0/NO_LOAD
        if assess['rpm'] > MOTOR_NO_LOAD_RPM:
            needed.append(d0 * assess['rpm'] / MOTOR_NO_LOAD_RPM)
        return max(needed)

    def _plan_laps(self):
        self.get_logger().info('=== torque / speed feasibility (per lap) ===')
        self.get_logger().info(
            f'  motor: stall {MOTOR_STALL_TORQUE_NM} N.m, rated '
            f'{MOTOR_RATED_TORQUE_NM} N.m, no-load {MOTOR_NO_LOAD_RPM} RPM | gates: '
            f'peak<={PEAK_TORQUE_LIMIT_NM:.2f} N.m, rms<={RMS_TORQUE_LIMIT_NM:.2f} N.m')
        planned = []
        for idx, dur in enumerate(self.lap_durations):
            a = self._assess_lap(dur)
            verdict = 'PASS' if a['feasible'] else 'FAIL'
            self.get_logger().info(
                f'  lap {idx + 1} T={dur:.1f}s: inertia x{a["inertia_peak"]:.2f}(pk)'
                f'/{a["inertia_rms"]:.2f}(rms) -> peak {a["peak_nm"]:.2f} N.m '
                f'({100 * a["peak_nm"] / MOTOR_STALL_TORQUE_NM:.0f}% stall), rms '
                f'{a["rms_nm"]:.2f} N.m, {a["rpm"]:.1f} RPM -> {verdict}')
            if a['feasible']:
                planned.append(dur)
                continue
            slow = self._min_feasible_duration(a)
            if slow is None:
                self.get_logger().fatal(
                    f'Lap {idx + 1} is infeasible at ANY speed (gravity torque alone '
                    f'exceeds the limit). Refusing. Re-check geometry / gearing.')
                raise RuntimeError('no feasible duration')
            slow = math.ceil(slow * 10.0) / 10.0
            self.get_logger().error(
                f'  lap {idx + 1} T={dur:.1f}s FAILS the torque gate; slowing to '
                f'{slow:.1f}s (fastest that passes).')
            planned.append(slow)
        self.get_logger().info('============================================')
        return planned

    # ----------------------------------------------------------- sequencer
    def _on_state(self, msg: JointState):
        self.last_state = {n: math.degrees(msg.position[i])
                           for i, n in enumerate(msg.name) if i < len(msg.position)}

    def _publish(self, pose_deg, vel_rad_s):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['theta', 'alpha', 'beta']
        msg.position = [math.radians(pose_deg[0]), math.radians(pose_deg[1]),
                        math.radians(pose_deg[2])]
        msg.velocity = [vel_rad_s[0], vel_rad_s[1], vel_rad_s[2]]
        self.goal_pub.publish(msg)

    def _clamp_vel(self, v):
        return max(self.min_vel, min(self.max_vel, v))

    def _tick(self):
        if self._finished:
            return
        now = self.get_clock().now()

        if self._phase_start is None:
            elapsed = 0.0
        else:
            elapsed = (now - self._phase_start).nanoseconds * 1e-9

        # advance phase when the current one is done
        if self._phase_index < 0 or (
                self._phase_index < len(self._phases)
                and elapsed >= self._phases[self._phase_index][1]):
            self._phase_index += 1
            if self._phase_index >= len(self._phases):
                self.get_logger().info('Oscillation sequence complete.')
                self._finished = True
                return
            self._phase_start = now
            elapsed = 0.0
            kind, dur = self._phases[self._phase_index]
            if kind == 'creep':
                self.get_logger().info(
                    f'Creeping to start pose (theta,alpha,beta)='
                    f'({self.start_pose[0]:.1f}, {self.start_pose[1]:.1f}, '
                    f'{self.start_pose[2]:.1f}) deg over <= {dur:.1f}s.')
            else:
                lap_no = self._phase_index  # phases[0] is creep
                self.get_logger().info(
                    f'Lap {lap_no}/{len(self.lap_plans)}: elliptical loop over '
                    f'{dur:.1f}s.')

        kind, dur = self._phases[self._phase_index]
        if kind == 'creep':
            self._publish(self.start_pose,
                          (self.creep_vel, self.creep_vel, self.creep_vel))
            return

        # --- lap: stream the ellipse, velocity from a one-step look-ahead ----
        t = min(elapsed, dur)
        phi = 2.0 * math.pi * (t / dur)
        h, g = self._ellipse_point(phi)
        sol = self.table.solve_deg(h, g)
        if sol is None:
            return
        phi2 = 2.0 * math.pi * ((t + self._dt_ctrl) / dur)
        h2, g2 = self._ellipse_point(phi2)
        nxt = self.table.solve_deg(h2, g2) or sol
        vel = tuple(self._clamp_vel(abs(math.radians(nxt[j] - sol[j])) / self._dt_ctrl)
                    for j in range(3))
        self._publish(sol, vel)


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
