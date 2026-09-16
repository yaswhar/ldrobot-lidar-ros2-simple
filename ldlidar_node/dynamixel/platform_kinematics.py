#!/usr/bin/env python3
"""
platform_kinematics.py  --  parallel-linkage forward/inverse kinematics.

Single source of truth for the mechanism geometry, shared by:
  * generate_ik_lut.py       (offline LUT builder -- uses the precise root finder)
  * oscillation_planner_node (startup: feasibility sampling + first-run LUT build)

Also hosts PlatformDynamics: a dependency-free port of the VERIFIED joint-torque
model from motor_selection/motor_sim_utils.py (gravity corrected 2026-08-27),
so the planner's feasibility gate can use the real statics on the Pi without
scipy. diagnostics/gear_check.py asserts the port agrees with motor_sim_utils
to ~1e-9 N.m -- edit both or neither.

SIGN CONVENTIONS (this is the dynamics/IK convention from motor_sim_utils.py,
NOT mech_opt_redesign.py's geometry-search convention -- see CLAUDE.md; the two
modules intentionally differ and only this one is correct for joint-angle/torque):

    central link (theta):   k = (a - d)/2
                            u = b*cos(theta) + k          <-- PLUS  k
                            h = b*sin(theta) + sqrt(c^2 - u^2)

    side link (alpha/beta): k = (a - d*cos(gamma))/2
                            u = b*cos(angle) - k          <-- MINUS k
                            h = b*sin(angle) + sqrt(c^2 - u^2)

Because k = (a-d)/2 is only ~5 mm for the baseline design, the WRONG central-link
sign still yields a plausible-but-off theta range (5.6..72.8 deg vs the correct
6.5..73.3 deg). The PLUS form is the one that reproduces the reference ranges.

This module is pure math: no ROS, no dynamixel_sdk, no numpy, no scipy.
"""

import math


class PlatformKinematics:
    """Forward height maps + precise (root-finding) inverse kinematics.

    All angles are in DEGREES.  h(angle) is non-monotonic (multiple IK roots),
    so each solve picks the branch nearest a seed to avoid solution jumps.
    """

    def __init__(self, a, b, c, d):
        self.a = float(a)
        self.b = float(b)
        self.c = float(c)
        self.d = float(d)

    # --- forward height maps -------------------------------------------------
    def h_center(self, theta_deg):
        """Central-link height (PLUS convention, motor_sim_utils.h_from_theta)."""
        th = math.radians(theta_deg)
        k = (self.a - self.d) / 2.0
        u = self.b * math.cos(th) + k          # PLUS k
        inner = self.c ** 2 - u ** 2
        if inner < 0.0:
            return None
        return self.b * math.sin(th) + math.sqrt(inner)

    def h_side(self, angle_deg, gamma_deg):
        """Side-link height at tilt gamma (MINUS convention, h_from_alpha_beta)."""
        ang = math.radians(angle_deg)
        gam = math.radians(gamma_deg)
        k = (self.a - self.d * math.cos(gam)) / 2.0
        u = self.b * math.cos(ang) - k         # MINUS k
        inner = self.c ** 2 - u ** 2
        if inner < 0.0:
            return None
        return self.b * math.sin(ang) + math.sqrt(inner)

    # --- side-target coupling ------------------------------------------------
    def side_targets(self, h_center_target, gamma_deg):
        """Given a central height and tilt, the two side-link target heights:
            h_alpha = h + (d/2) sin(gamma),   h_beta = h - (d/2) sin(gamma)."""
        s = math.sin(math.radians(gamma_deg))
        return (h_center_target + (self.d / 2.0) * s,
                h_center_target - (self.d / 2.0) * s)

    # --- precise root finding (OFFLINE / startup only -- never per control tick)
    @staticmethod
    def _roots(func, target, lo_deg=0.0, hi_deg=90.0, step_deg=0.5):
        """All angle-deg roots of func(angle) == target in [lo, hi] by bisection."""
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
                    xa, xb = prev_a, a
                    fa = prev_f - target
                    for _ in range(60):
                        xm = 0.5 * (xa + xb)
                        fm = func(xm)
                        if fm is None:
                            break
                        fm -= target
                        if abs(fm) < 1e-12 or (xb - xa) < 1e-6:
                            break
                        if fa * fm < 0.0:
                            xb = xm
                        else:
                            xa, fa = xm, fm
                    roots.append(0.5 * (xa + xb))
            prev_a, prev_f = a, f
            a += step_deg
        return roots

    def _solve_branch(self, func, target, seed_deg, lo_deg, hi_deg):
        roots = self._roots(func, target, lo_deg, hi_deg)
        if not roots:
            return None
        return min(roots, key=lambda r: abs(r - seed_deg))

    def solve_theta(self, h_center_target, seed_deg=40.0, lo_deg=0.0, hi_deg=90.0):
        """Precise inverse: theta (deg) for a central height. None if unreachable."""
        return self._solve_branch(self.h_center, h_center_target,
                                   seed_deg, lo_deg, hi_deg)

    def solve_side(self, h_side_target, gamma_deg, seed_deg=40.0,
                   lo_deg=0.0, hi_deg=90.0):
        """Precise inverse: side angle (deg) for a side-link height at gamma."""
        return self._solve_branch(lambda x: self.h_side(x, gamma_deg),
                                   h_side_target, seed_deg, lo_deg, hi_deg)

    def solve(self, h_center_target, gamma_deg, seed=None,
              lo_deg=0.0, hi_deg=90.0):
        """Precise inverse for (theta, alpha, beta) at a central height + tilt.

        `seed` = previous (theta, alpha, beta) to stay on the same IK branch.
        Returns None if any joint is unreachable.
        """
        h_alpha, h_beta = self.side_targets(h_center_target, gamma_deg)
        seed_th = seed[0] if seed else 40.0
        seed_al = seed[1] if seed else 40.0
        seed_be = seed[2] if seed else 37.0
        theta = self.solve_theta(h_center_target, seed_th, lo_deg, hi_deg)
        alpha = self.solve_side(h_alpha, gamma_deg, seed_al, lo_deg, hi_deg)
        beta = self.solve_side(h_beta, gamma_deg, seed_be, lo_deg, hi_deg)
        if None in (theta, alpha, beta):
            return None
        return (theta, alpha, beta)

    # --- achievable ranges (for LUT domain sizing) ---------------------------
    def achievable_center_h_range(self, lo_deg=0.0, hi_deg=90.0, step_deg=0.25):
        """(h_min, h_max) reachable by the central link over [lo, hi] deg."""
        hs = []
        a = lo_deg
        while a <= hi_deg + 1e-9:
            h = self.h_center(a)
            if h is not None:
                hs.append(h)
            a += step_deg
        return (min(hs), max(hs)) if hs else (None, None)


class PlatformDynamics:
    """Joint-side (crank) torque of ONE motor's linkage: statics + inertia.

    Port of motor_selection/motor_sim_utils.py (compute_inertia_*, the
    CORRECTED compute_gravity_* of 2026-08-27, and compute_motor_torque's
    tau_load) into dependency-free math so the ROS planner can run the same
    verified model on the Pi. Numerically identical to the source -- see
    diagnostics/gear_check.py, which asserts agreement to ~1e-9 N.m.

    ANGLES IN RADIANS here (matching motor_sim_utils), unlike PlatformKinematics
    which speaks degrees. Every public method name carries `_rad` to make that
    impossible to miss.

    Sign convention is inherited unchanged from motor_sim_utils:
        gravity_*_rad() returns the (negative) virtual-work term tau_grav, and
        load_torque_rad() = J*qdd + b_damp*qd + k_stiff*q - tau_grav,
    so the static (zero-speed) holding torque is -gravity_*_rad() > 0.
    Consumers take |tau|; do not "fix" the sign here without fixing the source.

    Gravity term (virtual work, cross-checked 3 ways in diagnostics/):
        tau_grav = -(g/4) * [ 2*m2*b*cos q + 2*m1*(h' + b*cos q) + (M+m3)*h' ]
    with h' = dh/dq the transmission ratio [m/rad]. The /4 is the four-motor
    share; m1 = coupler (c), m2 = crank (b), M+m3 = hanging load under plate d.
    """

    def __init__(self, kin, m1, m2, m3, M, J1, J2, g=9.81, b_damp=0.5,
                 k_stiff=0.01):
        self.kin = kin
        self.m1 = float(m1)
        self.m2 = float(m2)
        self.m3 = float(m3)
        self.M = float(M)
        self.J1 = float(J1)
        self.J2 = float(J2)
        self.g = float(g)
        self.b_damp = float(b_damp)
        self.k_stiff = float(k_stiff)

    # --- forward height with the motor_sim_utils clamp (never None) ----------
    def _h_theta(self, q):
        k = (self.kin.a - self.kin.d) / 2.0
        u = self.kin.b * math.cos(q) + k
        return self.kin.b * math.sin(q) + math.sqrt(max(self.kin.c ** 2 - u ** 2, 1e-10))

    def _h_side(self, q, gamma):
        k = (self.kin.a - self.kin.d * math.cos(gamma)) / 2.0
        u = self.kin.b * math.cos(q) - k
        return self.kin.b * math.sin(q) + math.sqrt(max(self.kin.c ** 2 - u ** 2, 1e-10))

    # --- transmission ratio dh/dq [m/rad] (analytic) --------------------------
    def dh_dtheta_rad(self, q):
        k = (self.kin.a - self.kin.d) / 2.0
        u = self.kin.b * math.cos(q) + k
        root = math.sqrt(max(self.kin.c ** 2 - u ** 2, 1e-12))
        return self.kin.b * math.cos(q) + u * self.kin.b * math.sin(q) / root

    def dh_dside_rad(self, q, gamma):
        k = (self.kin.a - self.kin.d * math.cos(gamma)) / 2.0
        u = self.kin.b * math.cos(q) - k
        root = math.sqrt(max(self.kin.c ** 2 - u ** 2, 1e-12))
        return self.kin.b * math.cos(q) + u * self.kin.b * math.sin(q) / root

    # --- position-dependent inertia about the crank pivot [kg m^2] ----------
    def _inertia(self, r):
        return (self.J2 + self.m2 * self.kin.b ** 2 / 4.0
                + self.J1 + self.m1 * r ** 2)

    def inertia_theta_rad(self, q):
        k = (self.kin.a - self.kin.d) / 2.0
        h = self._h_theta(q)
        r = 0.5 * math.sqrt((self.kin.b * math.cos(q) + k) ** 2
                            + (h + self.kin.b * math.sin(q)) ** 2)
        return self._inertia(r)

    def inertia_side_rad(self, q, gamma):
        k = (self.kin.a - self.kin.d * math.cos(gamma)) / 2.0
        h = self._h_side(q, gamma)
        r = 0.5 * math.sqrt((self.kin.b * math.cos(q) - k) ** 2
                            + (h + self.kin.b * math.sin(q)) ** 2)
        return self._inertia(r)

    # --- gravity (virtual work) ------------------------------------------------
    def _gravity_common(self, q, hp):
        bc = self.kin.b * math.cos(q)
        return -(self.g / 4.0) * (2.0 * self.m2 * bc
                                  + 2.0 * self.m1 * (hp + bc)
                                  + (self.M + self.m3) * hp)

    def gravity_theta_rad(self, q):
        return self._gravity_common(q, self.dh_dtheta_rad(q))

    def gravity_side_rad(self, q, gamma):
        return self._gravity_common(q, self.dh_dside_rad(q, gamma))

    # --- joint-side load torque ------------------------------------------------
    def load_torque_rad(self, q, qd, qdd, gamma, kind='theta'):
        """tau_load at the crank [N.m] for joint state (q, qd, qdd) and tilt
        gamma. kind = 'theta' (central) or 'side' (alpha/beta). Independent of
        any gearbox: joint torque is what the crank carries."""
        if kind == 'theta':
            J = self.inertia_theta_rad(q)
            tg = self.gravity_theta_rad(q)
        else:
            J = self.inertia_side_rad(q, gamma)
            tg = self.gravity_side_rad(q, gamma)
        return J * qdd + self.b_damp * qd + self.k_stiff * q - tg

    def static_torque_rad(self, q, gamma, kind='theta'):
        """Zero-speed holding torque at the crank [N.m] (= -tau_grav, > 0)."""
        if kind == 'theta':
            return -self.gravity_theta_rad(q)
        return -self.gravity_side_rad(q, gamma)
