#!/usr/bin/env python3
"""
platform_kinematics.py  --  parallel-linkage forward/inverse kinematics.

Single source of truth for the mechanism geometry, shared by:
  * generate_ik_lut.py       (offline LUT builder -- uses the precise root finder)
  * oscillation_planner_node (startup: feasibility sampling + first-run LUT build)

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

This module is pure math: no ROS, no dynamixel_sdk, no numpy required.
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
