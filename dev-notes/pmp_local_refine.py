#!/usr/bin/env python
"""Local refinement of the indirect-PMP swarm solution.

The indirect_pmp swarm scores its optimality conditions as soft penalties --
terminal altitude, speed and flight-path angle, and the transversality residual
H_burn_end + H_coast_end - H_burn_start -- so it may trade a small violation for a
shorter burn, and it stops after a fixed number of generations. This script starts
from its best point and enforces those conditions as hard equality constraints
with SLSQP, minimising burn time:

  A  altitude, speed, FPA and transversality = 0      the PMP's own conditions
  B  altitude, speed, FPA = 0; transversality free    best burn the costate-steered
                                                      control family reaches here
  B1-B3  B restarted from three perturbed starts -- does it return to the same point?
  C  as B, starting from B's result, with the kick angle gamma_p freed too

  A2 as A, with the transversality residual taken against H at the start of the LAST
     thrust arc (after the coast) instead of at Stage-2 ignition. Pontani (2014) writes
     H_f^last + H_f^coast - H_0^last = 0; this checks which instant H_0^last must be.

--frame selects INDIRECT_PMP_STAGE2_FRAME. Under "rotating" (the formulation flown until
2026-09-13) the speed target sqrt(mu/r) - v_rot is, in the rotation-free Stage-2
equations, the APOAPSIS of an ellipse with periapsis -890 km: freed, variant C deletes the
circularisation burn to reach it ballistically, so no "rotating" variant-C figure is a
propellant result. Under "inertial" (the default) the target is a circular orbit of the
equations the arc is flown with. The logged swarm point was optimised under "rotating";
under "inertial" it is only a starting guess.

A and B hold gamma_p at the swarm's value: it is a Stage-1 parameter that no costate
condition governs, and Stage 1 is where the event discontinuities live.

  C1000  as C with the coast capped at 1000 s, the pso_coast solver's own bound, so the
     PMP family is compared with the laws inside the same search box

Every result is reported with its largest scaled residual, a feasibility verdict on the
physical residuals (10 m / 1 cm/s / 0.001 deg / 0.01 in H), and the KKT stationarity of
the equality-constrained problem, |g - Jc^T mu| / |g| -- a propellant figure from an
infeasible point is not a result.

Propellant left follows exactly from the burn fraction,
    prop_left = M_PROP_2 * (1 - delta_tr_pct / 100),
because the PMP objective's J_nd is that fraction (checked against the archived row
to the gram). No dense replay is needed to price a refined point.

Starting point: the solution printed in Tese/src/Output/pmp_prod_v2.log -- the
archive does not store the decision vector -- so it is rounded to the printed
digits. The replay's J' is checked against the archived row before refining.

Run from the repository root:
    PYTHONIOENCODING=utf-8 C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/pmp_local_refine.py
"""

import argparse
import contextlib
import io
import json
import subprocess
import sys
import time
import warnings
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "Tese" / "src"
sys.path.insert(0, str(SRC))

from Input_File import simulation_parameters as sp  # noqa: E402
import run_results_matrix as rrm  # noqa: E402

# The configuration the harness flew pmp_baseline under, applied BEFORE the
# solver modules are imported.
rrm._apply(sp, rrm.BASELINE)
rrm._apply(sp, {"GUIDANCE_MODE": "indirect_pmp",
                "PSO_N_PARTICLES": 250, "PSO_MAX_GENERATIONS": 1000})

from Auxiliary import constants as c  # noqa: E402
from Auxiliary import rocket_specs as rs  # noqa: E402
from Auxiliary import earth_rotation as earth_rot  # noqa: E402
import Simulation.indirect_pso_solver as ips  # noqa: E402

ARCHIVE = SRC / "Output" / "pmp_prod_v2" / "pmp_baseline"
OUT_DIR = SRC / "Output" / "pmp_refine"

# Swarm best point as printed in Output/pmp_prod_v2.log (flown 2026-09-11):
# [lambda0_r, lambda0_v, lambda0_g, delta_tc, delta_tr_pct, coast_start_pct, gamma_p]
X_SWARM = [-0.002087, -0.938028, 0.995508, 280.11, 78.17, 93.21, 1.547791]

# Post-MECO-fix comparison rows, prop left [kg].
COMPARE = {
    "gravity_turn, pseudo-forces off (pf_off_v2)": 20688.8,
    "gravity_turn, pseudo-forces on (pf_on_v2)": 20647.0,
    "linear_tangent, 100x250 (audit_fix/lts_prod)": 20405.9,
}

M_PROP_2 = float(rs.M_PROP_2)
R_T = c.R_EARTH + sp.TARGET_ORBITAL_ALTITUDE
V_C = None   # the solver's terminal speed target, set in main() once the frame is chosen

# SLSQP sees each residual divided by these units. "H resid" is the solver's
# transversality residual (H at ignition); "H2 resid" takes H at the last-burn start.
C_UNITS = np.array([1000.0, 10.0, 0.1, 10.0, 10.0])   # m, m/s, deg, H, H
C_NAMES = ["alt [m]", "vel [m/s]", "fpa [deg]", "H resid", "H2 resid"]
NC = len(C_UNITS)
FEAS_TOL = np.array([10.0, 0.01, 1e-3, 1e-2, 1e-2])   # physical: m, m/s, deg, H, H

# Decision vector u = [a, b, delta_tc, delta_tr_pct, coast_start_pct, gamma_p], the
# costate direction as two angles so the solver's unit-norm gauge holds exactly.
# SLSQP moves z, with u = u0 + z * U_SCALE on the freed components.
U_SCALE = np.array([1e-3, 1e-3, 1.0, 0.01, 0.01, 1e-4])
U_NAMES = ["costate angle a [rad]", "costate angle b [rad]", "coast [s]",
           "burn [% of T_max]", "coast start [% of burn]", "gamma_p [rad]"]


def costate_angles(lr, lv, lg):
    n = np.sqrt(lr * lr + lv * lv + lg * lg)
    lr, lv, lg = lr / n, lv / n, lg / n
    return float(np.arctan2(lg, lv)), float(np.arcsin(lr))


def to_x(u):
    a, b, tc, tr, cs, gp = u
    return [np.sin(b), np.cos(b) * np.cos(a), np.cos(b) * np.sin(a), tc, tr, cs, gp]


def prop_left(u):
    return M_PROP_2 * (1.0 - u[3] / 100.0)


# Half-widths of the local search box around each variant's start (see --coast-box).
BOX = {"angle": 0.3, "coast": 300.0, "pct": 5.0}


def dense_check(u):
    """Replay u with the solver's dense runner. Stage 2 is flown drag-free, so report
    where it flies: staging, the lowest point after Stage-2 ignition, the time spent
    below 120 km after ignition, and the osculating orbit at the start of the first
    coast in the frame the arc was flown in (its apoapsis is where the vehicle
    circularises)."""
    with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
        warnings.simplefilter("ignore")
        t, d, thr, _alpha, t_ign, res = ips.run_indirect_full(to_x(u), verbose=False)
    t = np.asarray(t, dtype=float)
    d = np.asarray(d, dtype=float)
    thr = np.asarray(thr, dtype=float)
    alt_km = (d[1] - c.R_EARTH) / 1e3
    after = t >= t_ign
    i_min = int(np.argmin(np.where(after, alt_km, np.inf)))
    i_sep = int(np.searchsorted(t, res["t_stage2_start"], side="right") - 1)
    below = (after & (alt_km < 120.0))[:-1]
    out = {"staging_t_s": float(res["t_stage2_start"]),
           "staging_alt_km": float(alt_km[max(i_sep, 0)]),
           "min_alt_after_ignition_km": float(alt_km[i_min]),
           "min_alt_t_s": float(t[i_min]),
           "final_t_s": float(t[-1]),
           "time_below_120km_after_ignition_s": float(np.sum(np.diff(t)[below]))}
    burned = after & (thr > 0)
    if np.any(burned):
        coast = (np.arange(len(t)) > int(np.argmax(burned))) & (thr == 0)
        if np.any(coast):
            i_c = int(np.argmax(coast))
            r_c, v_c, g_c = d[1, i_c], d[2, i_c], d[3, i_c]
            if res.get("stage2_frame") == "inertial":
                v_c, g_c = earth_rot.rotating_to_inertial_planar(
                    v_c, g_c, np.deg2rad(sp.LAUNCH_LATITUDE), r_c)
            eps = v_c ** 2 / 2.0 - c.MU_EARTH / r_c
            out.update({"coast_start_t_s": float(t[i_c]),
                        "coast_start_alt_km": float(alt_km[i_c])})
            if eps < 0:
                a = -c.MU_EARTH / (2.0 * eps)
                e = np.sqrt(max(0.0, 1.0 - (r_c * v_c * np.cos(g_c)) ** 2 / (c.MU_EARTH * a)))
                out.update({"coast_periapsis_km": float((a * (1 - e) - c.R_EARTH) / 1e3),
                            "coast_apoapsis_km": float((a * (1 + e) - c.R_EARTH) / 1e3)})
    return out


class Evaluator:
    """One Stage-1 + Stage-2 PMP trajectory per distinct u, cached (scalars only --
    the solver's result dict carries the Stage-1 arrays)."""

    def __init__(self):
        self.cache = {}
        self.n_traj = 0

    def raw(self, u):
        key = tuple(float(v) for v in u)
        if key in self.cache:
            return self.cache[key]
        with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
            warnings.simplefilter("ignore")
            res = ips.run_indirect_trajectory(*to_x(u))
        self.n_traj += 1
        if res["crashed"] or res["state_final"] is None:
            out = None
        else:
            s = res.get("state_final_propagated", res["state_final"])   # as flown
            terms = ips.breakdown_objective(res)
            out = {
                "alt": float(s[1] - R_T), "vel": float(s[2] - V_C),
                "fpa": float(np.rad2deg(s[3])),
                "H": float(res["H_burn_end"] + res["H_coast_end"] - res["H_burn_start"]),
                "H2": float(res["H_burn_end"] + res["H_coast_end"]
                            - res["H_last_burn_start"]),
                "H_last_burn_start": float(res["H_last_burn_start"]),
                "H_burn_start": float(res["H_burn_start"]),
                "H_coast_end": float(res["H_coast_end"]),
                "H_burn_end": float(res["H_burn_end"]),
                "J_prime": float(ips.compute_augmented_objective(res)),
                "obj_J": float(terms["J"]), "obj_transv": float(terms["transv"]),
            }
        self.cache[key] = out
        return out

    def cvec(self, u):
        o = self.raw(u)
        if o is None:
            return np.full(NC, 1e3)
        return np.array([o["alt"], o["vel"], o["fpa"], o["H"], o["H2"]]) / C_UNITS


def describe(ev, u, label):
    o = ev.raw(u)
    print(f"\n--- {label}")
    for name, val in zip(U_NAMES, u):
        print(f"  {name:<26} {val:.12g}")
    x = to_x(u)
    print(f"  costates (unit norm)       λr={x[0]:+.9f} λv={x[1]:+.9f} λγ={x[2]:+.9f}")
    if o is None:
        print("  CRASHED")
        return None
    print(f"  prop left                  {prop_left(u):.3f} kg")
    print(f"  residuals                  alt {o['alt']:+.4e} m | vel {o['vel']:+.4e} m/s | "
          f"fpa {o['fpa']:+.4e} deg | H {o['H']:+.4e} | H2 {o['H2']:+.4e}")
    print(f"  H components               burn_start {o['H_burn_start']:+.5f} | "
          f"coast_end {o['H_coast_end']:+.5f} | last_burn_start {o['H_last_burn_start']:+.5f}"
          f" | burn_end {o['H_burn_end']:+.5f}")
    print(f"  solver objective           J' {o['J_prime']:.8f} = obj_J {o['obj_J']:.8f}"
          f" + transv {o['obj_transv']:.8f} + terminal")
    return o


def fd_noise_check(ev, u0, idx, h):
    """Central-difference Jacobian at h and 3h; a large disagreement means the
    integration noise is comparable to the step."""
    def jac(step):
        J = np.zeros((NC, len(idx)))
        for j, k in enumerate(idx):
            e = np.zeros(6)
            e[k] = step * U_SCALE[k]
            J[:, j] = (ev.cvec(u0 + e) - ev.cvec(u0 - e)) / (2 * step)
        return J
    J1, J3 = jac(h), jac(3 * h)
    rel = np.abs(J1 - J3) / np.maximum(np.abs(J3), 1e-12)
    print("\n  finite-difference check (|J(h) - J(3h)| / |J(3h)|, rows = residuals):")
    print("  " + " " * 12 + "".join(f"{U_NAMES[k][:14]:>16}" for k in idx))
    for i, name in enumerate(C_NAMES):
        print(f"  {name:<12}" + "".join(f"{rel[i, j]:16.2e}" for j in range(len(idx))))
    return float(np.max(rel[np.abs(J3) > 1e-9])) if np.any(np.abs(J3) > 1e-9) else 0.0


def refine(ev, u0, idx, cidx, h, maxiter, label, coast_max=None):
    idx = list(idx)
    u0 = np.array(u0, dtype=float)
    S = U_SCALE[idx]

    def u_of(z):
        u = u0.copy()
        u[idx] = u0[idx] + np.asarray(z) * S
        return u

    # Objective: propellant burned relative to the start, in tonnes (linear in burn %).
    j_tr = idx.index(3)
    grad = np.zeros(len(idx))
    grad[j_tr] = S[j_tr] / 100.0 * M_PROP_2 / 1000.0

    def f(z):
        return float((u_of(z)[3] - u0[3]) / 100.0 * M_PROP_2 / 1000.0)

    jac_cache = {}

    def cfun(z):
        return ev.cvec(u_of(z))[cidx]

    def cjac(z):
        key = tuple(float(v) for v in z)
        if key not in jac_cache:
            J = np.zeros((NC, len(idx)))
            for j in range(len(idx)):
                e = np.zeros(len(idx))
                e[j] = h
                J[:, j] = (ev.cvec(u_of(z + e)) - ev.cvec(u_of(z - e))) / (2 * h)
            jac_cache[key] = J
        return jac_cache[key][cidx]

    # Box: stay inside the swarm's bounds and within a local neighbourhood.
    lo = np.array([u0[0] - BOX["angle"], u0[1] - BOX["angle"],
                   max(sp.PSO_LB[3], u0[2] - BOX["coast"]),
                   max(sp.PSO_LB[4], u0[3] - BOX["pct"]), max(sp.PSO_LB[5], u0[4] - BOX["pct"]),
                   sp.PSO_LB[6]])
    hi = np.array([u0[0] + BOX["angle"], u0[1] + BOX["angle"],
                   min(sp.PSO_UB[3] if coast_max is None else coast_max,
                       u0[2] + BOX["coast"]),
                   min(sp.PSO_UB[4], u0[3] + BOX["pct"]), min(sp.PSO_UB[5], u0[4] + BOX["pct"]),
                   sp.PSO_UB[6]])
    bounds = [((lo[k] - u0[k]) / U_SCALE[k], (hi[k] - u0[k]) / U_SCALE[k]) for k in idx]

    n0, t0 = ev.n_traj, time.time()
    history = []

    def cb(z):
        u = u_of(z)
        cv = ev.cvec(u)
        history.append((prop_left(u), float(np.max(np.abs(cv[cidx])))))
        if len(history) % 5 == 0:
            print(f"    iter {len(history):3d}  prop left {prop_left(u):10.3f} kg  "
                  f"max|c| {history[-1][1]:.2e}  ({ev.n_traj - n0} trajectories)", flush=True)

    print(f"\n=== Variant {label}: free {[U_NAMES[k] for k in idx]}, "
          f"constraints {[C_NAMES[i] for i in cidx]}", flush=True)
    res = minimize(f, np.zeros(len(idx)), jac=lambda z: grad, method="SLSQP",
                   bounds=bounds, constraints=[{"type": "eq", "fun": cfun, "jac": cjac}],
                   callback=cb, options={"maxiter": maxiter, "ftol": 1e-6})
    u = u_of(res.x)
    print(f"  SLSQP: success={res.success} status={res.status} nit={res.nit} "
          f"message={res.message!r}")
    print(f"  {ev.n_traj - n0} trajectories, {time.time() - t0:.1f} s")
    o = describe(ev, u, f"Variant {label} result")
    cmax = float(np.max(np.abs(cfun(res.x))))
    feasible = bool(np.all(np.abs(ev.cvec(u) * C_UNITS)[cidx] <= FEAS_TOL[cidx]))
    Jc = cjac(res.x)
    mu, *_ = np.linalg.lstsq(Jc.T, grad, rcond=None)
    kkt = float(np.linalg.norm(grad - Jc.T @ mu) / np.linalg.norm(grad))
    print(f"  max |scaled residual| {cmax:.2e} ({'feasible' if feasible else 'INFEASIBLE'})   "
          f"KKT |g - Jc^T mu| / |g| = {kkt:.2e}")
    at_bound = [U_NAMES[k] for j, k in enumerate(idx)
                if abs(res.x[j] - bounds[j][0]) < 1e-6 or abs(res.x[j] - bounds[j][1]) < 1e-6]
    if at_bound:
        print(f"  WARNING at a box bound: {at_bound}")
    dense = dense_check(u) if feasible else None
    if dense:
        print(f"  staging {dense['staging_alt_km']:.1f} km at t={dense['staging_t_s']:.1f} s | "
              f"min altitude after Stage-2 ignition {dense['min_alt_after_ignition_km']:.1f} km "
              f"at t={dense['min_alt_t_s']:.1f} s | cutoff t={dense['final_t_s']:.1f} s")
        print(f"  {dense['time_below_120km_after_ignition_s']:.0f} s below 120 km after ignition"
              + (f" | first coast from {dense['coast_start_alt_km']:.1f} km at "
                 f"t={dense['coast_start_t_s']:.1f} s, osculating periapsis "
                 f"{dense['coast_periapsis_km']:.1f} km / apoapsis {dense['coast_apoapsis_km']:.1f} km"
                 if "coast_periapsis_km" in dense else ""))
    return {
        "dense": dense,
        "max_scaled_residual": cmax, "feasible": feasible, "kkt_rel": kkt,
        "label": label, "free": [U_NAMES[k] for k in idx],
        "constraints": [C_NAMES[i] for i in cidx],
        "success": bool(res.success), "status": int(res.status), "nit": int(res.nit),
        "message": str(res.message), "trajectories": ev.n_traj - n0,
        "wall_s": time.time() - t0, "u": [float(v) for v in u],
        "x": [float(v) for v in to_x(u)], "prop_left_kg": prop_left(u),
        "residuals": o, "at_bound": at_bound,
        "history": history,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--variants", default="A,A2,B,C")
    ap.add_argument("--maxiter", type=int, default=150)
    ap.add_argument("--h", type=float, default=1e-2,
                    help="finite-difference step in scaled units (see U_SCALE)")
    ap.add_argument("--no-restarts", dest="restarts", action="store_false",
                    help="skip the three perturbed restarts of variant B")
    ap.add_argument("--coast-box", type=float, default=300.0,
                    help="half-width of the coast-duration search box around each start [s]")
    ap.add_argument("--pct-box", type=float, default=5.0,
                    help="half-width of the burn and coast-start search boxes [percentage points]")
    ap.add_argument("--frame", choices=["inertial", "rotating"], default="inertial",
                    help="INDIRECT_PMP_STAGE2_FRAME for this run")
    args = ap.parse_args()
    BOX["coast"] = args.coast_box
    BOX["pct"] = args.pct_box
    sp.INDIRECT_PMP_STAGE2_FRAME = args.frame
    global V_C
    V_C = ips.terminal_speed_target(R_T)
    print(f"Stage-2 frame: {args.frame}   terminal speed target {V_C:.2f} m/s")
    variants = [v.strip().upper() for v in args.variants.split(",") if v.strip()]

    # --- configuration check against the archived run ------------------------
    man = json.loads((ARCHIVE / "pmp_baseline.manifest.json").read_text(encoding="utf-8"))
    row = json.loads((ARCHIVE / "pmp_baseline.json").read_text(encoding="utf-8"))
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--", "Tese/src"],
                           capture_output=True, text=True).stdout.strip()
    print(f"archived run commit {man['git']['commit'][:7]}   current HEAD {head[:7]}   "
          f"Tese/src {'DIRTY' if dirty else 'clean'}")
    diffs = []
    for k, v in man["config"].items():
        if not hasattr(sp, k):
            continue
        cur = getattr(sp, k)
        try:
            same = json.loads(json.dumps(cur, default=str)) == v
        except (TypeError, ValueError):
            same = str(cur) == str(v)
        if not same:
            diffs.append(k)
    print(f"config keys differing from the archived manifest: {diffs if diffs else 'none'}")

    ev = Evaluator()
    a0, b0 = costate_angles(*X_SWARM[:3])
    u_swarm = np.array([a0, b0] + X_SWARM[3:], dtype=float)

    t_start = time.time()
    o = describe(ev, u_swarm, "Swarm point (rounded to the logged digits), replayed")
    print(f"  archived row: J' {row['J_prime']:.8f}  obj_J {row['obj_J']:.8f}  "
          f"obj_transv {row['obj_transv']:.8f}  prop left {row['prop_remaining_kg']:.3f} kg")
    if o is None:
        sys.exit("swarm point crashed on replay -- configuration does not match")
    print(f"  replay - archive: J' {o['J_prime'] - row['J_prime']:+.2e}")

    fd_noise_check(ev, u_swarm, [0, 1, 2, 3, 4], args.h)

    results = {}
    if "A" in variants:
        results["A"] = refine(ev, u_swarm, [0, 1, 2, 3, 4], [0, 1, 2, 3], args.h,
                              args.maxiter, "A")
    if "A2" in variants:
        results["A2"] = refine(ev, u_swarm, [0, 1, 2, 3, 4], [0, 1, 2, 4], args.h,
                               args.maxiter, "A2")
    if "B" in variants:
        results["B"] = refine(ev, u_swarm, [0, 1, 2, 3, 4], [0, 1, 2], args.h,
                              args.maxiter, "B")
    if "B" in variants and args.restarts:
        for i, (k, d) in enumerate([(3, +0.3), (2, -15.0), (0, +0.02)]):
            start = u_swarm.copy()
            start[k] += d
            results[f"B{i + 1}"] = refine(ev, start, [0, 1, 2, 3, 4], [0, 1, 2], args.h,
                                          args.maxiter,
                                          f"B{i + 1} (restart: {U_NAMES[k]} {d:+g})")
    if "C" in variants:
        start = (np.array(results["B"]["u"]) if results.get("B", {}).get("feasible")
                 else u_swarm)
        fd_noise_check(ev, start, [0, 1, 2, 3, 4, 5], args.h)
        results["C"] = refine(ev, start, [0, 1, 2, 3, 4, 5], [0, 1, 2], args.h,
                              args.maxiter, "C")
    if "C1000" in variants:
        start = (np.array(results["B"]["u"]) if results.get("B", {}).get("feasible")
                 else u_swarm)
        results["C1000"] = refine(ev, start, [0, 1, 2, 3, 4, 5], [0, 1, 2], args.h,
                                  args.maxiter, "C1000 (coast <= 1000 s, the pso_coast bound)",
                                  coast_max=1000.0)

    # --- summary ---------------------------------------------------------------
    base = prop_left(u_swarm)
    swarm_dense = dense_check(u_swarm)
    print("\n" + "=" * 95)
    print(f"{'':44}{'prop left [kg]':>16}{'vs swarm':>10}{'min alt S2 [km]':>17}")
    print(f"{'swarm point (replayed)':44}{base:16.1f}{'':>10}"
          f"{swarm_dense['min_alt_after_ignition_km']:17.1f}")
    for k, r in results.items():
        flag = (("" if r["feasible"] else "  INFEASIBLE")
                + ("" if r["success"] else "  (SLSQP did not converge)")
                + ("  at box bound" if r["at_bound"] else ""))
        ma = (f"{r['dense']['min_alt_after_ignition_km']:17.1f}" if r["dense"]
              else f"{'-':>17}")
        print(f"{'variant ' + k + ' — ' + ', '.join(n.split(' ')[0] for n in r['constraints']):44}"
              f"{r['prop_left_kg']:16.1f}{r['prop_left_kg'] - base:+10.1f}{ma}{flag}")
    print("-" * 78)
    for name, val in COMPARE.items():
        print(f"{name:44}{val:16.1f}{val - base:+10.1f}")
    print("=" * 78)
    print(f"total {ev.n_traj} trajectories, {time.time() - t_start:.0f} s")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = OUT_DIR / f"pmp_refine_{stamp}.json"
    out.write_text(json.dumps({
        "source_archive": str(ARCHIVE.relative_to(SRC)), "x_swarm_logged": X_SWARM,
        "commit": head, "config_diffs_vs_manifest": diffs,
        "swarm_replay": {"u": [float(v) for v in u_swarm], "prop_left_kg": base,
                         "residuals": ev.raw(u_swarm)},
        "variants": results, "compare_prop_left_kg": COMPARE,
        "fd_step_scaled": args.h, "u_scale": U_SCALE.tolist(), "c_units": C_UNITS.tolist(),
    }, indent=2), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
