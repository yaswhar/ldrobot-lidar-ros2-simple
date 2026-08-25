#!/usr/bin/env python3
"""
generate_ik_lut.py  --  OFFLINE inverse-kinematics lookup-table builder.

The runtime control loop must be deterministic with bounded per-tick cost (it is
meant to become a tight reactive loop later). scipy.brentq / any root finder has
a variable iteration count -- wrong for that. So we solve the (slow, precise)
inverse kinematics ONCE here, on a regular grid, and cache the result to disk.
At runtime the planner does O(1) interpolation only (see IKTable below).

Tables
------
  theta : 1D, keyed on central height h        (theta has NO gamma dependence).
  alpha : 2D, keyed on (h, gamma).
  beta  : 2D, keyed on (h, gamma).

Grid (defaults): 5 mm steps in h, 1 deg steps in gamma, across the ACHIEVABLE
envelope (unreachable cells are stored as NaN, not clamped to a fake value).

Tick self-consistency (per the original design idea)
----------------------------------------------------
For every grid point we (1) solve the precise angle, (2) round it to the nearest
angle the encoder can actually hold, then (3) forward-recompute the h/gamma that
the ROUNDED angle really produces and store the angle together with that true
height. So the table never claims an angle the hardware can't hit, and we can
report exactly how far the quantized table drifts from the ideal (sub-mm here).

Encoder quantization is direction/offset INDEPENDENT: the set of reachable
physical angles is the lattice {n * 360/4096 deg}; a motor's Homing Offset and
direction only relabel which integer n a given angle maps to, never the spacing.
So quantizing needs ONLY the published XC430 encoder resolution (4096 ticks/rev)
-- no motor IDs, no calibration, no dynamixel_sdk. The planner/actuator boundary
is preserved: nothing hardware-specific leaks into the planner.

Usage:
    ros2 run ldlidar_node generate_ik_lut.py \
        --params $(ros2 pkg prefix ldlidar_node)/share/ldlidar_node/params/oscillation_planner.yaml \
        --out ~/.cache/ldlidar_platform/ik_lut.npz
    ./generate_ik_lut.py --a 0.11 --b 0.44 --c 0.77 --d 0.10 --benchmark
"""

import argparse
import math
import os
import sys

import numpy as np

try:
    from platform_kinematics import PlatformKinematics
except ImportError:  # allow running from an arbitrary cwd
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from platform_kinematics import PlatformKinematics

# --- encoder resolution (published XC430-W150-T spec; NOT motor calibration) ---
TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV        # ~0.08789 deg
LUT_FORMAT_VERSION = 2


def quantize_angle_deg(angle_deg):
    """Round to the nearest angle the encoder lattice can hold (0.088 deg grid).
    Direction/offset independent -- see module docstring."""
    return round(angle_deg / DEG_PER_TICK) * DEG_PER_TICK


def _signature(a, b, c, d, h_step_m, gamma_step_deg, gamma_amp_deg,
               lo_deg, hi_deg):
    """Stable fingerprint of the generation params; used to detect a stale cache."""
    return np.array([LUT_FORMAT_VERSION, a, b, c, d, h_step_m, gamma_step_deg,
                     gamma_amp_deg, lo_deg, hi_deg, TICKS_PER_REV], dtype=np.float64)


def build_lut(a, b, c, d, h_step_m=0.005, gamma_step_deg=1.0,
              gamma_amp_deg=30.0, lo_deg=0.0, hi_deg=90.0, log=print):
    """Build the IK LUT. Returns a dict of numpy arrays + metadata.

    h grid spans the achievable central-h range; gamma grid spans
    [-gamma_amp, +gamma_amp]. Unreachable cells are NaN.
    """
    kin = PlatformKinematics(a, b, c, d)

    h_lo, h_hi = kin.achievable_center_h_range(lo_deg, hi_deg)
    if h_lo is None:
        raise RuntimeError('no achievable central height -- check geometry.')
    # Snap grid onto whole h_step multiples, slightly inside the singular ends.
    margin = h_step_m
    g_lo = math.ceil((h_lo + margin) / h_step_m) * h_step_m
    g_hi = math.floor((h_hi - margin) / h_step_m) * h_step_m
    n_h = int(round((g_hi - g_lo) / h_step_m)) + 1
    h_grid = g_lo + h_step_m * np.arange(n_h)

    n_g = int(round(2 * gamma_amp_deg / gamma_step_deg)) + 1
    gamma_grid = -gamma_amp_deg + gamma_step_deg * np.arange(n_g)

    log(f'  building LUT: {n_h} h-steps [{g_lo:.3f}..{g_hi:.3f} m @ '
        f'{h_step_m*1000:.0f} mm], {n_g} gamma-steps '
        f'[{-gamma_amp_deg:.0f}..{gamma_amp_deg:.0f} deg @ {gamma_step_deg:.0f} deg]')

    theta_angle = np.full(n_h, np.nan)
    theta_h_actual = np.full(n_h, np.nan)
    alpha_angle = np.full((n_h, n_g), np.nan)
    beta_angle = np.full((n_h, n_g), np.nan)
    alpha_h_actual = np.full((n_h, n_g), np.nan)
    beta_h_actual = np.full((n_h, n_g), np.nan)

    # --- theta 1D (walk low->high h on the operating branch, seed with prev) --
    seed = lo_deg
    for i, h in enumerate(h_grid):
        th = kin.solve_theta(float(h), seed_deg=seed, lo_deg=lo_deg, hi_deg=hi_deg)
        if th is None:
            continue
        seed = th
        th_q = quantize_angle_deg(th)
        theta_angle[i] = th_q
        theta_h_actual[i] = kin.h_center(th_q)

    # --- alpha/beta 2D (walk low->high h per gamma column, seed with prev) ----
    reach = 0
    for j, g in enumerate(gamma_grid):
        seed_a = lo_deg
        seed_b = lo_deg
        for i, h in enumerate(h_grid):
            h_al, h_be = kin.side_targets(float(h), float(g))
            al = kin.solve_side(h_al, float(g), seed_deg=seed_a,
                                lo_deg=lo_deg, hi_deg=hi_deg)
            be = kin.solve_side(h_be, float(g), seed_deg=seed_b,
                                lo_deg=lo_deg, hi_deg=hi_deg)
            if al is not None:
                seed_a = al
                al_q = quantize_angle_deg(al)
                alpha_angle[i, j] = al_q
                alpha_h_actual[i, j] = kin.h_side(al_q, float(g))
            if be is not None:
                seed_b = be
                be_q = quantize_angle_deg(be)
                beta_angle[i, j] = be_q
                beta_h_actual[i, j] = kin.h_side(be_q, float(g))
            if al is not None and be is not None:
                reach += 1

    # --- self-consistency report (grid h vs the h the quantized angle yields) -
    def _max_drift(desired, actual):
        m = np.isfinite(actual)
        return float(np.max(np.abs(desired[m] - actual[m]))) if m.any() else 0.0

    theta_drift = _max_drift(h_grid, theta_h_actual)
    h2d = np.repeat(h_grid[:, None], n_g, axis=1)
    # side targets differ from central h by <= (d/2)|sin g|; compare to the
    # per-cell side target, not central h, for an honest quantization drift.
    sin_g = np.sin(np.radians(gamma_grid))[None, :]
    alpha_target = h2d + (d / 2.0) * sin_g
    beta_target = h2d - (d / 2.0) * sin_g
    alpha_drift = _max_drift(alpha_target, alpha_h_actual)
    beta_drift = _max_drift(beta_target, beta_h_actual)

    log(f'  reachable (h,gamma) cells: {reach}/{n_h*n_g} '
        f'({100.0*reach/(n_h*n_g):.0f}%)')
    log(f'  tick-quantization drift (max): theta {theta_drift*1000:.3f} mm, '
        f'alpha {alpha_drift*1000:.3f} mm, beta {beta_drift*1000:.3f} mm')

    return {
        'signature': _signature(a, b, c, d, h_step_m, gamma_step_deg,
                                gamma_amp_deg, lo_deg, hi_deg),
        'geometry': np.array([a, b, c, d], dtype=np.float64),
        'h_grid': h_grid,
        'gamma_grid': gamma_grid,
        'theta_angle': theta_angle,
        'theta_h_actual': theta_h_actual,
        'alpha_angle': alpha_angle,
        'beta_angle': beta_angle,
        'alpha_h_actual': alpha_h_actual,
        'beta_h_actual': beta_h_actual,
    }


def save_lut(lut, path):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez_compressed(path, **lut)
    return path


def load_lut(path):
    path = os.path.expanduser(path)
    with np.load(path) as data:
        return {k: data[k] for k in data.files}


def signature_matches(lut, a, b, c, d, h_step_m, gamma_step_deg,
                      gamma_amp_deg, lo_deg, hi_deg):
    want = _signature(a, b, c, d, h_step_m, gamma_step_deg, gamma_amp_deg,
                      lo_deg, hi_deg)
    have = lut.get('signature')
    return have is not None and have.shape == want.shape and np.allclose(have, want)


class IKTable:
    """O(1) runtime inverse kinematics by interpolation over a prebuilt LUT.

    theta(h)         -> linear interpolation on the 1D table.
    alpha_beta(h, g) -> bilinear interpolation on the 2D tables.

    No root finding, no iteration: a fixed handful of array reads + multiply-adds.
    NaN (unreachable) neighbours are dropped and the remaining valid ones
    renormalised, so a query just inside the envelope still returns a value.
    """

    def __init__(self, lut):
        self.h = np.asarray(lut['h_grid'], dtype=np.float64)
        self.g = np.asarray(lut['gamma_grid'], dtype=np.float64)
        self.theta = np.asarray(lut['theta_angle'], dtype=np.float64)
        self.alpha = np.asarray(lut['alpha_angle'], dtype=np.float64)
        self.beta = np.asarray(lut['beta_angle'], dtype=np.float64)
        self.h0 = float(self.h[0])
        self.g0 = float(self.g[0])
        self.dh = float(self.h[1] - self.h[0]) if self.h.size > 1 else 1.0
        self.dg = float(self.g[1] - self.g[0]) if self.g.size > 1 else 1.0

    def _hbin(self, h):
        f = (h - self.h0) / self.dh
        i = int(math.floor(f))
        i = max(0, min(self.h.size - 2, i))
        return i, min(1.0, max(0.0, f - i))

    def _gbin(self, g):
        f = (g - self.g0) / self.dg
        j = int(math.floor(f))
        j = max(0, min(self.g.size - 2, j))
        return j, min(1.0, max(0.0, f - j))

    def theta_deg(self, h):
        i, t = self._hbin(h)
        a, b = self.theta[i], self.theta[i + 1]
        if math.isnan(a) and math.isnan(b):
            return None
        if math.isnan(a):
            return float(b)
        if math.isnan(b):
            return float(a)
        return float(a * (1.0 - t) + b * t)

    @staticmethod
    def _bilerp(v00, v10, v01, v11, t, u):
        """Bilinear with NaN-drop + renormalise. t along h, u along gamma."""
        vals = ((v00, (1 - t) * (1 - u)), (v10, t * (1 - u)),
                (v01, (1 - t) * u), (v11, t * u))
        num = 0.0
        wsum = 0.0
        for v, w in vals:
            if not math.isnan(v) and w > 0.0:
                num += v * w
                wsum += w
        if wsum <= 0.0:
            # all weighted corners NaN -> take any finite corner as a fallback
            for v, _w in vals:
                if not math.isnan(v):
                    return float(v)
            return None
        return num / wsum

    def alpha_beta_deg(self, h, g):
        i, t = self._hbin(h)
        j, u = self._gbin(g)
        al = self._bilerp(self.alpha[i, j], self.alpha[i + 1, j],
                          self.alpha[i, j + 1], self.alpha[i + 1, j + 1], t, u)
        be = self._bilerp(self.beta[i, j], self.beta[i + 1, j],
                          self.beta[i, j + 1], self.beta[i + 1, j + 1], t, u)
        return al, be

    def solve_deg(self, h, g):
        """Full (theta, alpha, beta) in degrees, or None if unreachable."""
        th = self.theta_deg(h)
        al, be = self.alpha_beta_deg(h, g)
        if None in (th, al, be):
            return None
        return (th, al, be)


def benchmark_lookup(table, n=200000):
    """Estimate per-lookup latency over the demo envelope."""
    import time
    h_lo, h_hi = float(table.h[0]), float(table.h[-1])
    g_lo, g_hi = float(table.g[0]), float(table.g[-1])
    hs = h_lo + (h_hi - h_lo) * (np.arange(n) % 997) / 997.0
    gs = g_lo + (g_hi - g_lo) * (np.arange(n) % 991) / 991.0
    t0 = time.perf_counter()
    for k in range(n):
        table.solve_deg(float(hs[k]), float(gs[k]))
    dt = time.perf_counter() - t0
    return dt / n


def _extract_geometry(params_path):
    import yaml
    with open(params_path, 'r') as fh:
        doc = yaml.safe_load(fh)
    node = doc.get('/**', doc)
    p = node.get('ros__parameters', node) if isinstance(node, dict) else {}
    geo = p.get('geometry', {}) or {}
    lut = p.get('lut', {}) or {}
    return (float(geo.get('a', 0.11)), float(geo.get('b', 0.44)),
            float(geo.get('c', 0.77)), float(geo.get('d', 0.10)),
            float(lut.get('h_step_m', 0.005)),
            float(lut.get('gamma_step_deg', 1.0)),
            float(lut.get('gamma_amp_deg', 30.0)))


def main():
    ap = argparse.ArgumentParser(description='Build the platform IK lookup table.')
    ap.add_argument('--params', help='oscillation_planner params YAML (geometry+lut)')
    ap.add_argument('--a', type=float, default=0.11)
    ap.add_argument('--b', type=float, default=0.44)
    ap.add_argument('--c', type=float, default=0.77)
    ap.add_argument('--d', type=float, default=0.10)
    ap.add_argument('--h-step', type=float, default=0.005, help='h grid step [m]')
    ap.add_argument('--gamma-step', type=float, default=1.0, help='gamma step [deg]')
    ap.add_argument('--gamma-amp', type=float, default=30.0, help='gamma amp [deg]')
    ap.add_argument('--out', default='~/.cache/ldlidar_platform/ik_lut.npz')
    ap.add_argument('--benchmark', action='store_true',
                    help='measure per-lookup latency after building')
    args = ap.parse_args()

    if args.params:
        a, b, c, d, h_step, g_step, g_amp = _extract_geometry(args.params)
    else:
        a, b, c, d = args.a, args.b, args.c, args.d
        h_step, g_step, g_amp = args.h_step, args.gamma_step, args.gamma_amp

    print(f'Geometry a={a} b={b} c={c} d={d}')
    lut = build_lut(a, b, c, d, h_step, g_step, g_amp)
    path = save_lut(lut, args.out)
    size_kb = os.path.getsize(path) / 1024.0
    print(f'\nTable sizes:')
    print(f'  theta : {lut["theta_angle"].shape[0]} rows (1D on h)')
    print(f'  alpha : {lut["alpha_angle"].shape[0]} x {lut["alpha_angle"].shape[1]}'
          f' (h x gamma)')
    print(f'  beta  : {lut["beta_angle"].shape[0]} x {lut["beta_angle"].shape[1]}'
          f' (h x gamma)')
    print(f'  file  : {path} ({size_kb:.1f} KiB)')

    if args.benchmark:
        table = IKTable(lut)
        per = benchmark_lookup(table)
        print(f'\nPer-lookup latency (theta+alpha+beta, interpolated): '
              f'{per*1e6:.3f} us  ({per*1e9:.0f} ns)')


if __name__ == '__main__':
    main()
