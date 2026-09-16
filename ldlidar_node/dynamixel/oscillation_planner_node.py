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

Feasibility gate (2026-09): the old anchored-ratio heuristic (a 1.07 N.m
"reference peak" that was a fingerprint of the broken gravity term, scaled by
an acceleration ratio with gravity as a fixed 50 %) is gone. Each lap is now
checked with the VERIFIED virtual-work statics + position-dependent inertia
(platform_kinematics.PlatformDynamics, a port of motor_sim_utils), joint-side,
against the motor's stall/rated through gear.ratio x gear.efficiency. The gear
ratio and efficiency are the only hardware facts this node knows -- it needs
them for the torque gate and for the joint-side encoder lattice of the LUT.
Still nothing about DynamixelSDK, motor IDs or ticks belongs here.

/joint_goal velocity is SIGNED (rad/s, joint side). The position-mode actuator
takes |v| as the profile speed; the velocity-mode cascade uses it as
feed-forward. A future lidar_planner_node should publish it signed too.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from platform_kinematics import PlatformDynamics, PlatformKinematics
    from generate_ik_lut import (IKTable, build_lut, load_lut, save_lut,
                                  signature_matches)
except ImportError:
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from platform_kinematics import PlatformDynamics, PlatformKinematics
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
H_LOW = 0.92            # m  (bottom of the stroke, reached at gamma=0). Raised from
                        #     0.65 on 2026-09-16 so every joint stays >= 30 deg, the
                        #     actuator's soft-limit floor (min angles 33.8/32.9/32.9).
                        #     The geometric floor is 0.635 m (alpha/beta binding).
GAMMA_AMP_DEG = 30.0    # tilt amplitude (reached at h_mid)

# --- Link masses / inertias (motor_selection/config.json, 2026-08-28) --------
#  m1 = coupler (c), m2 = crank (b), M + m3 = hanging load under plate d.
#  config.json carries the 0.5 kg field payload (M 0.2 + plate 0.3); the design
#  case in redesign_report.md is 2.5 kg (M 2.2 + 0.3). Override dynamics.* in
#  the params YAML to gate against the payload actually bolted on.
DEFAULT_DYNAMICS = {
    'm1': 0.1427, 'm2': 0.1442, 'm3': 0.3, 'M': 0.2,
    'J1': 0.00705, 'J2': 0.00233, 'g': 9.81, 'b_damp': 0.5, 'k_stiff': 0.01,
}

# --- XC430-W150-T catalog (motor-side; the gate converts through gear.*) ------
MOTOR_STALL_TORQUE_NM = 1.5
MOTOR_RATED_TORQUE_NM = 0.83
MOTOR_NO_LOAD_RPM = 70.0
MOTOR_EFFICIENCY = 0.90       # joint -> shaft coupling loss used across the report
PEAK_STALL_FRACTION = 0.90    # gate: peak <= 90 % of stall; rms <= rated

# --- Gearbox (7:1 cycloid fitted 2026-09; defaults here are DIRECT DRIVE so a
#     gearless build still runs -- the YAML sets 7.0 / 0.88) -------------------
DEFAULT_GEAR_RATIO = 1.0
DEFAULT_GEAR_EFFICIENCY = 1.0

# 3 laps, each independently timed. A lap that fails the gate is auto-slowed
# (loud warning) to the fastest duration that passes; a lap whose STATIC
# holding torque alone exceeds the gate is refused at any speed.
DEFAULT_LAP_DURATIONS_S = [12.0, 9.0, 7.0]
LAP_SAMPLES = 360             # phi samples per lap for the torque gate
MAX_AUTO_SLOW_S = 120.0       # give up searching for a passing duration beyond this


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
        self.declare_parameter('start_settle_s', 3.0)        # MIN creep-phase time
        self.declare_parameter('start_arrival_tol_deg', 0.75)  # ...then wait for arrival (< actuator creep_done_deg)
        self.declare_parameter('start_timeout_s', 30.0)      # ...but not forever
        self.declare_parameter('min_profile_velocity_rad_s', 0.05)
        self.declare_parameter('max_profile_velocity_rad_s', 6.0)
        self.declare_parameter('creep_velocity_rad_s', 0.15)

        # gearbox (the only hardware facts this node knows; see module docstring)
        self.declare_parameter('gear.ratio', DEFAULT_GEAR_RATIO)
        self.declare_parameter('gear.efficiency', DEFAULT_GEAR_EFFICIENCY)

        # motor catalog values (motor side)
        self.declare_parameter('motor.stall_torque_nm', MOTOR_STALL_TORQUE_NM)
        self.declare_parameter('motor.rated_torque_nm', MOTOR_RATED_TORQUE_NM)
        self.declare_parameter('motor.no_load_rpm', MOTOR_NO_LOAD_RPM)
        self.declare_parameter('motor.efficiency', MOTOR_EFFICIENCY)

        # link masses / inertias for the torque gate (joint side)
        for key, val in DEFAULT_DYNAMICS.items():
            self.declare_parameter(f'dynamics.{key}', float(val))

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
        self.start_arrival_tol = float(self.get_parameter('start_arrival_tol_deg').value)
        self.start_timeout_s = float(self.get_parameter('start_timeout_s').value)
        self.min_vel = float(self.get_parameter('min_profile_velocity_rad_s').value)
        self.max_vel = float(self.get_parameter('max_profile_velocity_rad_s').value)
        self.creep_vel = float(self.get_parameter('creep_velocity_rad_s').value)  # legacy; the
        # actuator owns the creep speed (creep_profile_velocity_rad_s). Kept so old YAMLs load.

        self.gear_ratio = float(self.get_parameter('gear.ratio').value)
        self.gear_eta = float(self.get_parameter('gear.efficiency').value)
        if self.gear_ratio <= 0.0 or not (0.0 < self.gear_eta <= 1.0):
            raise RuntimeError('gear.ratio must be > 0 and gear.efficiency in (0, 1]')
        self.stall_nm = float(self.get_parameter('motor.stall_torque_nm').value)
        self.rated_nm = float(self.get_parameter('motor.rated_torque_nm').value)
        self.no_load_rpm = float(self.get_parameter('motor.no_load_rpm').value)
        self.motor_eta = float(self.get_parameter('motor.efficiency').value)
        dyn = {k: float(self.get_parameter(f'dynamics.{k}').value) for k in DEFAULT_DYNAMICS}
        self.dyn = PlatformDynamics(self.kin, dyn['m1'], dyn['m2'], dyn['m3'], dyn['M'],
                                    dyn['J1'], dyn['J2'], dyn['g'], dyn['b_damp'],
                                    dyn['k_stiff'])
        # joint <-> motor torque factor: tau_motor = tau_joint / (N * eta_gear * eta_motor)
        self.torque_factor = self.gear_ratio * self.gear_eta * self.motor_eta
        self.peak_limit_joint = PEAK_STALL_FRACTION * self.stall_nm * self.torque_factor
        self.rms_limit_joint = self.rated_nm * self.torque_factor

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
        self._lap_geom = self._sample_lap_geometry()
        self.lap_plans = self._plan_laps()

        # ---- ROS interfaces -------------------------------------------------
        self.goal_pub = self.create_publisher(JointState, self.goal_topic, 10)
        self.state_sub = self.create_subscription(
            JointState, self.state_topic, self._on_state, 10)
        self.last_state = None

        # ---- sequencer: creep-to-start, then the timed laps -----------------
        # The creep phase lasts at least start_settle_s and then until
        # /joint_states reports arrival (or start_timeout_s, with a warning).
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
                self.lut_gamma_amp, 0.0, 90.0, self.gear_ratio)
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
                    'Cached IK LUT signature does not match current geometry/grid/'
                    'gear ratio -- rebuilding.')
            except Exception as exc:  # noqa: BLE001 - cache is best-effort
                self.get_logger().warn(f'Could not read IK LUT cache ({exc}); rebuilding.')

        self.get_logger().info('Building IK LUT (one-time, precise root finder)...')
        lut = build_lut(a, b, c, d, self.lut_h_step, self.lut_gamma_step,
                        self.lut_gamma_amp, log=lambda m: self.get_logger().info(m),
                        gear_ratio=self.gear_ratio)
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
    def _sample_lap_geometry(self, n=LAP_SAMPLES):
        """Sample the ellipse ONCE in the phase variable phi (duration-free).

        Uses the PRECISE root finder, NOT the runtime LUT: the LUT stores
        tick-quantized angles, and double-differencing that staircase manufactures
        phantom acceleration spikes. For any lap duration T the joint rates follow
        from phi = 2*pi*t/T:  qd = q' * (2*pi/T),  qdd = q'' * (2*pi/T)^2, so the
        (T-independent) inertia J(q) and gravity tau_g(q) are evaluated here once
        and every lap / auto-slow candidate is then a vectorised expression.
        """
        phi = 2.0 * math.pi * np.arange(n + 1) / n
        seed = self.start_pose
        q = {k: [] for k in ('theta', 'alpha', 'beta')}
        gam = []
        for ph in phi:
            h, g = self._ellipse_point(ph)
            sol = self.kin.solve(h, g, seed=seed) or seed
            seed = sol
            for name, v in zip(('theta', 'alpha', 'beta'), sol):
                q[name].append(math.radians(v))
            gam.append(math.radians(g))
        gam = np.asarray(gam)
        dphi = phi[1] - phi[0]
        geom = {}
        for name in q:
            qq = np.asarray(q[name])
            q1 = np.gradient(qq, dphi)      # dq/dphi
            q2 = np.gradient(q1, dphi)      # d2q/dphi2
            kind = 'theta' if name == 'theta' else 'side'
            if kind == 'theta':
                J = np.array([self.dyn.inertia_theta_rad(x) for x in qq])
                tg = np.array([self.dyn.gravity_theta_rad(x) for x in qq])
            else:
                J = np.array([self.dyn.inertia_side_rad(x, g) for x, g in zip(qq, gam)])
                tg = np.array([self.dyn.gravity_side_rad(x, g) for x, g in zip(qq, gam)])
            geom[name] = {'q': qq, 'q1': q1, 'q2': q2, 'J': J, 'tau_g': tg}
        return geom

    def _assess_lap(self, duration):
        """Joint-side torque of one lap of `duration` through the verified model
        (PlatformDynamics == motor_sim_utils), gated against the motor through
        gear.ratio x gear.efficiency x motor.efficiency."""
        w = 2.0 * math.pi / duration
        peak_joint = rms_sq = 0.0
        n_tot = 0
        max_rad_s = 0.0
        env_margin = math.inf
        static_peak = 0.0
        for g in self._lap_geom.values():
            qd = g['q1'] * w
            qdd = g['q2'] * w * w
            tau = (g['J'] * qdd + self.dyn.b_damp * qd + self.dyn.k_stiff * g['q']
                   - g['tau_g'])
            peak_joint = max(peak_joint, float(np.max(np.abs(tau))))
            static_peak = max(static_peak, float(np.max(np.abs(g['tau_g']))))
            rms_sq += float(np.sum(tau ** 2))
            n_tot += tau.size
            max_rad_s = max(max_rad_s, float(np.max(np.abs(qd))))
            # torque-speed envelope (motor side): |tau_m| <= stall*(1 - rpm/no_load)
            rpm = np.abs(qd) * self.gear_ratio * 60.0 / (2.0 * math.pi)
            avail = self.stall_nm * (1.0 - rpm / self.no_load_rpm)
            env_margin = min(env_margin,
                             float(np.min(avail - np.abs(tau) / self.torque_factor)))
        rms_joint = math.sqrt(rms_sq / n_tot)
        motor_rpm = max_rad_s * 60.0 / (2.0 * math.pi) * self.gear_ratio
        feasible = (peak_joint <= self.peak_limit_joint + 1e-9
                    and rms_joint <= self.rms_limit_joint + 1e-9
                    and motor_rpm <= self.no_load_rpm
                    and env_margin > 0.0)
        return {'duration': duration,
                'peak_joint': peak_joint, 'rms_joint': rms_joint,
                'peak_motor': peak_joint / self.torque_factor,
                'rms_motor': rms_joint / self.torque_factor,
                'static_peak_joint': static_peak,
                'rpm': motor_rpm, 'env_margin': env_margin, 'feasible': feasible}

    def _min_feasible_duration(self, assess):
        """Slowest-limited duration >= requested that passes the gate.

        Gravity is duration-independent, so if the STATIC holding torque alone
        already exceeds the peak or rms limit no duration can pass -> None.
        Otherwise scan T upward geometrically (cheap: the lap geometry is
        pre-sampled) until the full gate passes, up to MAX_AUTO_SLOW_S.
        """
        if assess['static_peak_joint'] > min(self.peak_limit_joint, self.rms_limit_joint):
            return None
        d = assess['duration']
        while d <= MAX_AUTO_SLOW_S:
            d *= 1.05
            if self._assess_lap(d)['feasible']:
                return d
        return None

    def _plan_laps(self):
        lg = self.get_logger()
        lg.info('=== torque / speed feasibility (per lap, verified statics + inertia) ===')
        lg.info(f'  motor: stall {self.stall_nm} N.m, rated {self.rated_nm} N.m, no-load '
                f'{self.no_load_rpm:.0f} RPM | gear {self.gear_ratio:g}:1 eta {self.gear_eta:g} '
                f'| coupling eta {self.motor_eta:g} -> joint limits: peak <= '
                f'{self.peak_limit_joint:.2f} N.m ({100 * PEAK_STALL_FRACTION:.0f}% stall), '
                f'rms <= {self.rms_limit_joint:.2f} N.m (rated)')
        hang = self.dyn.M + self.dyn.m3
        lg.info(f'  dynamics: hanging load M+m3 = {hang:.2f} kg, crank m2 {self.dyn.m2:.3f} kg, '
                f'coupler m1 {self.dyn.m1:.3f} kg')
        planned = []
        for idx, dur in enumerate(self.lap_durations):
            a = self._assess_lap(dur)
            verdict = 'PASS' if a['feasible'] else 'FAIL'
            lg.info(
                f'  lap {idx + 1} T={dur:.1f}s: joint peak {a["peak_joint"]:.2f} N.m '
                f'(static {a["static_peak_joint"]:.2f}), rms {a["rms_joint"]:.2f} -> motor '
                f'peak {a["peak_motor"]:.3f} N.m ({100 * a["peak_motor"] / self.stall_nm:.0f}% '
                f'stall), rms {a["rms_motor"]:.3f} ({100 * a["rms_motor"] / self.rated_nm:.0f}% '
                f'rated), {a["rpm"]:.1f} RPM ({100 * a["rpm"] / self.no_load_rpm:.0f}% no-load), '
                f'envelope margin {a["env_margin"]:+.2f} N.m -> {verdict}')
            if a['feasible']:
                planned.append(dur)
                continue
            slow = self._min_feasible_duration(a)
            if slow is None:
                lg.fatal(
                    f'Lap {idx + 1} is infeasible at ANY speed: static holding torque '
                    f'{a["static_peak_joint"]:.2f} N.m joint = '
                    f'{a["static_peak_joint"] / self.torque_factor:.2f} N.m motor exceeds the '
                    f'gate. Refusing. Re-check payload (dynamics.*), gearing, geometry.')
                raise RuntimeError('no feasible duration')
            slow = math.ceil(slow * 10.0) / 10.0
            lg.error(f'  lap {idx + 1} T={dur:.1f}s FAILS the gate; slowing to {slow:.1f}s '
                     f'(fastest that passes).')
            planned.append(slow)
        lg.info('======================================================================')
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
        """Clamp a SIGNED joint velocity by magnitude: |v| in [min_vel, max_vel]."""
        mag = max(self.min_vel, min(self.max_vel, abs(v)))
        return math.copysign(mag, v) if v != 0.0 else mag

    def _phase_may_end(self, elapsed):
        """Creep phase: also require /joint_states within start_arrival_tol_deg of
        the start pose, so a lap never begins while the platform is still on its
        way (a velocity-mode actuator creeps at a capped speed). Falls through
        with a warning after start_timeout_s, or if no state is being published."""
        kind, _dur = self._phases[self._phase_index]
        if kind != 'creep':
            return True
        if self.last_state is None:
            if elapsed >= self.start_timeout_s:
                self.get_logger().warn(
                    f'No /joint_states after {elapsed:.0f}s; starting laps blind.')
                return True
            return False
        err = max(abs(self.last_state.get(n, 0.0) - self.start_pose[i])
                  for i, n in enumerate(('theta', 'alpha', 'beta')))
        if err <= self.start_arrival_tol:
            return True
        if elapsed >= self.start_timeout_s:
            self.get_logger().warn(
                f'Start pose not reached after {elapsed:.0f}s (max error {err:.1f} deg); '
                f'starting laps anyway.')
            return True
        return False

    def _tick(self):
        if self._finished:
            return
        now = self.get_clock().now()

        if self._phase_start is None:
            elapsed = 0.0
        else:
            elapsed = (now - self._phase_start).nanoseconds * 1e-9

        # advance phase when the current one is done (creep: min time AND arrival)
        if self._phase_index < 0 or (
                self._phase_index < len(self._phases)
                and elapsed >= self._phases[self._phase_index][1]
                and self._phase_may_end(elapsed)):
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
            # A held pose has ZERO velocity: the actuator approaches its first
            # pose at its own creep speed (position mode: creep profile until
            # the goal pose changes; velocity mode: rate-limited P, no
            # feed-forward). Publishing creep_vel here would be a feed-forward
            # bias that parks the velocity cascade at -v/Kp from the pose.
            self._publish(self.start_pose, (0.0, 0.0, 0.0))
            return

        # --- lap: stream the ellipse, SIGNED velocity from a one-step look-ahead
        t = min(elapsed, dur)
        phi = 2.0 * math.pi * (t / dur)
        h, g = self._ellipse_point(phi)
        sol = self.table.solve_deg(h, g)
        if sol is None:
            return
        phi2 = 2.0 * math.pi * ((t + self._dt_ctrl) / dur)
        h2, g2 = self._ellipse_point(phi2)
        nxt = self.table.solve_deg(h2, g2) or sol
        vel = tuple(self._clamp_vel(math.radians(nxt[j] - sol[j]) / self._dt_ctrl)
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
