#!/usr/bin/env python
"""Polish indirect_pmp solutions into extremals: Levenberg-Marquardt plus gamma_p continuation.

1. Optional swarm (--swarm-budget P,G): run_pso_optimization under the results-matrix
   configuration of --case, so its best point can be polished in the same process.
2. Polish at fixed gamma_p: Levenberg-Marquardt (least_squares 'lm') on the square system
       alt, vel, fpa, H_coast_end, H_burn1_end - H_last_burn_start = 0
   in u = [costate angles a, b; D1, Dc, D3]. If a step takes the coast outside
   [PSO_LB, PSO_UB] it is pinned to that bound and H_coast_end is dropped; the one-sided
   condition is checked instead.
3. Outer: continuation in gamma_p (no costate condition governs it) -- step it both ways from
   the fixed-gamma_p extremal, re-solving 2 from the previous converged point, and keep the
   best converged extremal. A pinned-coast point counts only if its one-sided condition holds.
   (A golden-section search failed: its jumps leave the extremal family's basin.)

Starting points (--start, repeatable):
  <case>.npz      a results-matrix archive; its full-precision ``decision_vector`` (archived
                  since 2026-09-17) and, from the manifest beside it, the seed it came from
  x JSON          dev-notes/pmp_prod_20260916_x.json -- the ``x_raw`` of --case, as the
                  solver printed it (rounded: re-flown it misses the insertion by ~100 m)
  refined JSON    any JSON holding ``u = [a, b, coast s, burn %, coast start %, gamma_p]``,
                  the SLSQP refinement format
The 2026-09-13 refinement points B and C (pseudo-force-free Stage 1, inertial Stage 2) are
stale and are polished only with --old-refine-starts.

Configuration: rrm.BASELINE plus the overrides of --case, the Stage-2 form of --frame (default
the config's own, "rotating_pseudo_forces" since 2026-09-16) and duration-stationarity
transversality. The best converged extremal of each start is re-flown densely and written as a
standard three-file archive (Archive/store.save_run) under --archive-out, so it loads,
compares and seeds the PMP reference like any harness case; its manifest records the seed of
the swarm the start came from.

Run from the repository root:
    PYTHONIOENCODING=utf-8 C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/pmp_swarm_polish.py \
        --case pmp_baseline --start Tese/src/Output/pmp_seeds/pmp_baseline/seed_1/pmp_baseline/pmp_baseline.npz
"""

import argparse
import contextlib
import io
import json
import sys
import time
import warnings
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "Tese" / "src"
sys.path.insert(0, str(SRC))

from Input_File import simulation_parameters as sp  # noqa: E402
import run_results_matrix as rrm  # noqa: E402

rrm._apply(sp, rrm.BASELINE)
rrm._apply(sp, {"GUIDANCE_MODE": "indirect_pmp",
                "INDIRECT_PMP_TRANSVERSALITY": "duration_stationarity",
                "EVENTS_PRINT": False, "INTERRUPTS_PRINT": False})

from Auxiliary import constants as c  # noqa: E402
from Auxiliary import rocket_specs as rs  # noqa: E402
from Auxiliary import earth_rotation as earth_rot  # noqa: E402
import Simulation.indirect_pso_solver as ips  # noqa: E402

OUT = SRC / "Output" / "pmp_refine"
REFINE_JSON = OUT / "pmp_refine_20260913_170035.json"
T_MAX = ips._T_MAX_2
M_PROP_2 = float(rs.M_PROP_2)
R_T = c.R_EARTH + sp.TARGET_ORBITAL_ALTITUDE
V_T = None   # the solver's terminal speed target, set in main() once the frame is chosen
DC_LB, DC_UB = float(sp.PSO_LB[3]), float(sp.PSO_UB[3])
GP_LB, GP_UB = float(sp.PSO_LB[6]), float(sp.PSO_UB[6])

# A residual of 1.0 is the convergence tolerance: 1 m, 1 cm/s, 1e-4 deg, 0.01 in H.
UNITS = np.array([1.0, 0.01, 1e-4, 0.01, 0.01])
S = np.array([1e-3, 1e-3, 1.0, 1.0, 1.0])   # scale of u = [a, b, D1, Dc, D3]

POLISH_OUT = SRC / "Output" / "pmp_polish"


def x_from(u, gp):
    a, b, d1, dc, d3 = u
    T = d1 + d3
    return [np.sin(b), np.cos(b) * np.cos(a), np.cos(b) * np.sin(a),
            dc, 100.0 * T / T_MAX, 100.0 * d1 / T, gp]


def u_from_x(x):
    lam = np.asarray(x[:3], dtype=float)
    lam = lam / np.linalg.norm(lam)
    T = x[4] / 100.0 * T_MAX
    d1 = x[5] / 100.0 * T
    return np.array([np.arctan2(lam[2], lam[1]), np.arcsin(lam[0]), d1, x[3], T - d1]), float(x[6])


def prop_left(u):
    return M_PROP_2 * (1.0 - (u[2] + u[4]) / T_MAX)


def fly(u, gp, strict=True):
    """strict rejects a burn shorter than 0.01 s -- the polish's conditions need both burns.
    Reporting a swarm point uses strict=False, since the swarm may drop the last burn. The coast
    upper bound is NOT enforced here: the polish has to see a solution beyond it to pin it."""
    if strict and (u[2] <= 0.01 or u[4] <= 0.01):
        return None
    if u[3] < 0.0 or not (GP_LB <= gp <= GP_UB):
        return None
    with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
        warnings.simplefilter("ignore")
        res = ips.run_indirect_trajectory(*x_from(u, gp))
    return None if res["crashed"] else res


def residual(u, gp, coast_free):
    res = fly(u, gp)
    n = 5 if coast_free else 4
    if res is None:
        return np.full(n, 1e6), None
    s = res["state_final_propagated"]
    r = np.array([s[1] - R_T, s[2] - V_T, np.rad2deg(s[3]), res["H_coast_end"],
                  res["H_burn1_end"] - res["H_last_burn_start"]]) / UNITS
    return (r if coast_free else r[[0, 1, 2, 4]]), res


def newton_gn(u0, gp, maxit=40, h=1e-2):
    """SUPERSEDED by ``newton`` (LM). Damped Gauss-Newton on the square system; from the
    refinement's point B it stalled -- 40 iterations, the coast moved 0.1 s, max scaled
    residual 477 -- where Levenberg-Marquardt reaches the extremal. Kept for the record."""
    u = np.array(u0, dtype=float)
    coast_free = DC_LB < u[3] < DC_UB
    u[3] = min(max(u[3], DC_LB), DC_UB)
    r, res = residual(u, gp, coast_free)
    for it in range(maxit):
        if res is not None and np.max(np.abs(r)) < 1.0:
            return u, r, res, it, True, coast_free
        idx = [0, 1, 2, 3, 4] if coast_free else [0, 1, 2, 4]
        J = np.zeros((len(r), len(idx)))
        for j, k in enumerate(idx):
            e = np.zeros(5)
            e[k] = h * S[k]
            J[:, j] = (residual(u + e, gp, coast_free)[0]
                       - residual(u - e, gp, coast_free)[0]) / (2 * h)
        dz = np.linalg.lstsq(J, -r, rcond=None)[0]
        step, moved = 1.0, False
        while step > 1e-3:
            un = u.copy()
            un[idx] = u[idx] + step * dz * S[idx]
            if coast_free and not (DC_LB <= un[3] <= DC_UB):
                un[3] = min(max(un[3], DC_LB), DC_UB)       # pin and drop H_coast_end
                u, coast_free = un, False
                r, res = residual(u, gp, coast_free)
                moved = True
                break
            rn, resn = residual(un, gp, coast_free)
            if np.linalg.norm(rn) < np.linalg.norm(r):
                u, r, res, moved = un, rn, resn, True
                break
            step *= 0.5
        if not moved:
            return u, r, res, it, False, coast_free
    ok = res is not None and np.max(np.abs(r)) < 1.0
    return u, r, res, maxit, ok, coast_free


# LM residual scaling -- the one pmp_duration_conditions.py converged with from B -- and the
# physical tolerance a solve must meet to count as converged: m, m/s, deg, H, H.
RES_SCALE = np.array([1000.0, 10.0, 0.1, 1.0, 1.0])
TOL_PHYS = np.array([1.0, 0.01, 1e-4, 0.05, 0.05])


def _raw_residual(u, gp):
    res = fly(u, gp)
    if res is None:
        return None, None
    s = res["state_final_propagated"]
    return np.array([s[1] - R_T, s[2] - V_T, np.rad2deg(s[3]), res["H_coast_end"],
                     res["H_burn1_end"] - res["H_last_burn_start"]]), res


def newton(u0, gp, max_nfev=400, h=1e-2):
    """Levenberg-Marquardt on the square system at fixed gamma_p.

    Returns (u, r, res, n_evaluations, converged, coast_free), r in units of TOL_PHYS. If
    the solution leaves [DC_LB, DC_UB] the coast is pinned to that bound, H_coast_end is
    dropped, and the reduced 4x4 system is solved from there."""
    u = np.array(u0, dtype=float)
    coast_free = DC_LB < u[3] < DC_UB
    u[3] = min(max(u[3], DC_LB), DC_UB)
    nfev = 0
    for _attempt in range(2):
        idx = [0, 1, 2, 3, 4] if coast_free else [0, 1, 2, 4]
        rows = idx
        base = u.copy()

        def f(z, base=base, idx=idx, rows=rows):
            uu = base.copy()
            uu[idx] = base[idx] + z * S[idx]
            raw, _ = _raw_residual(uu, gp)
            return np.full(len(rows), 1e3) if raw is None else raw[rows] / RES_SCALE[rows]

        def jac(z, f=f, n=len(idx)):
            J = np.zeros((n, n))
            for j in range(n):
                e = np.zeros(n)
                e[j] = h
                J[:, j] = (f(z + e) - f(z - e)) / (2 * h)
            return J

        sol = least_squares(f, np.zeros(len(idx)), jac=jac, method="lm",
                            xtol=1e-12, ftol=1e-12, max_nfev=max_nfev)
        nfev += int(sol.nfev)
        u = base.copy()
        u[idx] = base[idx] + sol.x * S[idx]
        if coast_free and not (DC_LB <= u[3] <= DC_UB):
            u[3] = min(max(u[3], DC_LB), DC_UB)
            coast_free = False
            continue
        break
    raw, res = _raw_residual(u, gp)
    rows = [0, 1, 2, 3, 4] if coast_free else [0, 1, 2, 4]
    if raw is None:
        return u, np.full(len(rows), np.inf), None, nfev, False, coast_free
    r = raw[rows] / TOL_PHYS[rows]
    return u, r, res, nfev, bool(np.all(np.abs(r) <= 1.0)), coast_free


def dense_check(u, gp):
    with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
        warnings.simplefilter("ignore")
        t, d, thr, _alpha, t_ign, res = ips.run_indirect_full(x_from(u, gp), verbose=False)
    t, d, thr = (np.asarray(v, dtype=float) for v in (t, d, thr))
    alt_km = (d[1] - c.R_EARTH) / 1e3
    after = t >= t_ign
    i_sep = int(np.searchsorted(t, res["t_stage2_start"], side="right") - 1)
    out = {"staging_alt_km": float(alt_km[max(i_sep, 0)]),
           "min_alt_after_ignition_km": float(np.min(alt_km[after])),
           "time_below_120km_after_ignition_s": float(np.sum(np.diff(t)[(after & (alt_km < 120.0))[:-1]]))}
    burned = after & (thr > 0)
    coast = (np.arange(len(t)) > int(np.argmax(burned))) & (thr == 0)
    if np.any(burned) and np.any(coast):
        i_c = int(np.argmax(coast))
        v_c, g_c = d[2, i_c], d[3, i_c]
        # run_indirect_full reports ground-relative in every Stage-2 form, so the
        # osculating elements need the inertial state whenever the Earth rotates.
        if sp.ENABLE_EARTH_ROTATION:
            v_c, g_c = earth_rot.rotating_to_inertial_planar(
                v_c, g_c, np.deg2rad(sp.LAUNCH_LATITUDE), d[1, i_c])
        eps = v_c ** 2 / 2.0 - c.MU_EARTH / d[1, i_c]
        if eps < 0:
            a = -c.MU_EARTH / (2.0 * eps)
            e = np.sqrt(max(0.0, 1.0 - (d[1, i_c] * v_c * np.cos(g_c)) ** 2 / (c.MU_EARTH * a)))
            out.update({"coast_start_alt_km": float(alt_km[i_c]),
                        "coast_periapsis_km": float((a * (1 - e) - c.R_EARTH) / 1e3),
                        "coast_apoapsis_km": float((a * (1 + e) - c.R_EARTH) / 1e3)})
    return out


def report(label, u, gp, res, extra=""):
    print(f"\n--- {label} {extra}")
    print(f"  gamma_p {gp:.9f} rad ({np.rad2deg(gp):.4f} deg) | D1 {u[2]:.3f} s | "
          f"Dc {u[3]:.3f} s | D3 {u[4]:.3f} s | prop left {prop_left(u):.1f} kg")
    if res is None:
        print("  CRASHED / out of bounds")
        return None
    s = res["state_final_propagated"]
    bd = ips.breakdown_objective(res)
    print(f"  orbit error: alt {s[1] - R_T:+.3e} m | vel {s[2] - V_T:+.3e} m/s | "
          f"fpa {np.rad2deg(s[3]):+.3e} deg")
    print(f"  H: coast_end {res['H_coast_end']:+.5f} | burn1_end {res['H_burn1_end']:+.5f} | "
          f"last_burn_start {res['H_last_burn_start']:+.5f} | burn_end {res['H_burn_end']:+.5f}")
    print(f"  J' {sum(bd.values()):.6f} = J {bd['J']:.6f} + alt {bd['alt']:.2e} + vel {bd['vel']:.2e}"
          f" + fpa {bd['fpa']:.2e} + transv {bd['transv']:.2e}")
    dc = dense_check(u, gp)
    print(f"  staging {dc['staging_alt_km']:.1f} km | min alt after ignition "
          f"{dc['min_alt_after_ignition_km']:.1f} km | {dc['time_below_120km_after_ignition_s']:.0f} s "
          f"below 120 km" + (f" | coast from {dc['coast_start_alt_km']:.1f} km, osculating periapsis "
                             f"{dc['coast_periapsis_km']:.1f} / apoapsis {dc['coast_apoapsis_km']:.1f} km"
                             if "coast_periapsis_km" in dc else ""))
    return {"u": [float(v) for v in u], "gamma_p": float(gp), "x": [float(v) for v in x_from(u, gp)],
            "prop_left_kg": prop_left(u), "breakdown": {k: float(v) for k, v in bd.items()},
            "H": {k: float(res[k]) for k in ("H_coast_end", "H_burn1_end", "H_last_burn_start",
                                            "H_burn_end", "H_burn_start")},
            "orbit_error": [float(s[1] - R_T), float(s[2] - V_T), float(np.rad2deg(s[3]))],
            "dense": dc}


def _coast_condition_ok(u, res, coast_free):
    """Free coast: nothing extra. Pinned at the upper bound: H_coast_end <= 0; lower: >= 0."""
    if coast_free or res is None:
        return True
    return res["H_coast_end"] <= 0.0 if u[3] >= DC_UB else res["H_coast_end"] >= 0.0


def polish(label, u0, gp0, span, step):
    t0 = time.time()
    print(f"\n=== Polish from {label}")
    u, r, res, it, ok, cf = newton(u0, gp0)
    fixed = report(f"{label}: extremal at fixed gamma_p", u, gp0, res,
                   f"(Newton {'converged' if ok else 'NOT converged'} in {it} it, coast "
                   f"{'free' if cf else 'pinned at a bound'})")
    if not cf and res is not None:
        side = "upper" if u[3] >= DC_UB else "lower"
        need = res["H_coast_end"] <= 0 if side == "upper" else res["H_coast_end"] >= 0
        print(f"  coast at its {side} bound: one-sided condition "
              f"{'holds' if need else 'VIOLATED'} (H_coast_end {res['H_coast_end']:+.5f})")

    if not ok:
        print("  fixed-gamma_p solve did not converge -- gamma_p search skipped")
        return {"fixed_gamma_p": fixed, "fixed_converged": False, "fixed_coast_free": bool(cf),
                "outer": None, "outer_converged": False, "outer_coast_free": bool(cf),
                "gamma_p_at_interval_edge": False, "trials": []}
    if not _coast_condition_ok(u, res, cf):
        print("  fixed-gamma_p point violates the one-sided coast condition -- search skipped")
        return {"fixed_gamma_p": fixed, "fixed_converged": False, "fixed_coast_free": bool(cf),
                "outer": None, "outer_converged": False, "outer_coast_free": bool(cf),
                "gamma_p_at_interval_edge": False, "trials": []}
    trials = []
    best = {"prop": prop_left(u), "gp": gp0, "u": u.copy(), "res": res, "cf": cf}
    stopped_by = {}

    for direction in (-1.0, +1.0):
        uu, gp = u.copy(), gp0
        stopped_by[direction] = "span"
        for _ in range(int(round(span / step))):
            gp = gp + direction * step
            if not (GP_LB <= gp <= GP_UB):
                stopped_by[direction] = "gamma_p bound"
                break
            un, rn, resn, nf, okn, cfn = newton(uu, gp, max_nfev=200)
            okn = okn and _coast_condition_ok(un, resn, cfn)
            trials.append((float(gp), bool(okn), prop_left(un), bool(cfn)))
            print(f"    gamma_p {gp:.7f} ({np.rad2deg(gp):.3f} deg): {'ok' if okn else 'FAILED'} "
                  f"({nf} evals) | prop left {prop_left(un):.1f} kg | coast {un[3]:.1f} s "
                  f"({'free' if cfn else 'pinned'})"
                  + (f" | H_coast_end {resn['H_coast_end']:+.4f}" if resn is not None and not cfn else ""),
                  flush=True)
            if not okn:
                stopped_by[direction] = "solve failed"
                break
            uu = un
            if prop_left(un) > best["prop"]:
                best = {"prop": prop_left(un), "gp": gp, "u": un.copy(), "res": resn, "cf": cfn}
            elif prop_left(un) < best["prop"] - 50.0:
                stopped_by[direction] = "past the optimum"
                break

    outer = report(f"{label}: best converged extremal along the gamma_p continuation",
                   best["u"], best["gp"], best["res"],
                   f"(coast {'free' if best['cf'] else 'pinned at its bound'}; sweeps stopped by: "
                   f"down {stopped_by[-1.0]}, up {stopped_by[+1.0]})")
    ok2, cf2 = True, best["cf"]
    converged_gps = [gp0] + [t[0] for t in trials if t[1]]
    at_edge = best["gp"] in (min(converged_gps), max(converged_gps)) and len(converged_gps) > 1
    if at_edge:
        print("  WARNING: best extremal is the last converged point of a sweep -- the optimum may lie beyond")
    print(f"  polish wall time {time.time() - t0:.0f} s")
    return {"fixed_gamma_p": fixed, "fixed_converged": bool(ok), "fixed_coast_free": bool(cf),
            "outer": outer, "outer_converged": bool(ok2), "outer_coast_free": bool(cf2),
            "gamma_p_at_interval_edge": bool(at_edge), "trials": trials}


def load_start(path, case):
    """(u, gamma_p, seed or None, description) from a --start path."""
    path = Path(path)
    if path.suffix == ".npz":
        with np.load(path, allow_pickle=False) as z:
            if "decision_vector" not in z.files:
                raise SystemExit(f"{path} has no decision_vector (archived before 2026-09-17)")
            x = [float(v) for v in z["decision_vector"]]
        seed = None
        man = path.with_name(path.stem + ".manifest.json")
        if man.exists():
            cfg = json.loads(man.read_text(encoding="utf-8")).get("config", {})
            seed = cfg.get("PSO_SEED")
            if cfg.get("INCLUDE_DRAG") is not None and bool(cfg["INCLUDE_DRAG"]) != bool(sp.INCLUDE_DRAG):
                raise SystemExit(f"{path} was flown with INCLUDE_DRAG={cfg['INCLUDE_DRAG']}, "
                                 f"--case {case} has {sp.INCLUDE_DRAG}")
        u, gp = u_from_x(x)
        return u, gp, seed, f"archive {path}"
    data = json.loads(path.read_text(encoding="utf-8"))
    if "u" in data:
        a, b, tc, tr, cs, gp = data["u"]
        T = tr / 100.0 * T_MAX
        d1 = cs / 100.0 * T
        return np.array([a, b, d1, tc, T - d1]), float(gp), None, f"refined point {path}"
    if case in data and "x_raw" in data[case]:
        u, gp = u_from_x(data[case]["x_raw"])
        meta = data.get("_meta", {})
        seed = 42 if "seed 42" in str(meta.get("budget", "")) else None
        return u, gp, seed, f"rounded printout {path}"
    raise SystemExit(f"{path}: no decision_vector, u, or {case}.x_raw")


def write_archive(u, gp, root, name, label, seed):
    """Re-fly an extremal densely and write the standard three-file archive."""
    from Archive import store
    saved_seed = sp.PSO_SEED
    if seed is not None:
        sp.PSO_SEED = int(seed)          # the manifest names the swarm the start came from
    t0 = time.time()
    x = x_from(u, gp)
    with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
        warnings.simplefilter("ignore")
        t, d, thr, alpha, _t_ign, res = ips.run_indirect_full(x, verbose=False)
    saved = store.save_run(sp, t, d, thr, alpha, res, J=float(ips.compute_augmented_objective(res)),
                           history=None, extra={"decision_vector": [float(v) for v in x]},
                           wall_clock=time.time() - t0, name=name, root=root,
                           source="dev-notes/pmp_swarm_polish.py", label=label,
                           tags={"section": "6.4", "factor": "reference"}, verbose=False)
    sp.PSO_SEED = saved_seed
    return saved


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--case", default="pmp_baseline", choices=["pmp_baseline", "pmp_vacuum"],
                    help="results-matrix case whose configuration is applied")
    ap.add_argument("--start", action="append", default=[],
                    help="starting point: archive .npz, x JSON or refined-u JSON (repeatable)")
    ap.add_argument("--old-refine-starts", action="store_true",
                    help="also polish the stale 2026-09-13 refinement points B and C")
    ap.add_argument("--archive-out", default=None,
                    help="root for the archives of the best extremals "
                         "(default Tese/src/Output/pmp_polish/<case>)")
    ap.add_argument("--particles", type=int, default=100)
    ap.add_argument("--generations", type=int, default=200)
    ap.add_argument("--span", type=float, default=0.02,
                    help="how far the gamma_p continuation may go each way [rad]")
    ap.add_argument("--step", type=float, default=0.0005, help="gamma_p continuation step [rad]")
    ap.add_argument("--no-swarm", dest="swarm", action="store_false",
                    help="skip the swarm; polish only the --start points")
    ap.add_argument("--frame", choices=["rotating_pseudo_forces", "inertial", "rotating"],
                    default=None, help="INDIRECT_PMP_STAGE2_FRAME (default: the config's)")
    ap.add_argument("--transversality", choices=["duration_stationarity", "pontani_eq38"],
                    default="duration_stationarity")
    ap.add_argument("--no-polish", dest="polish", action="store_false",
                    help="swarm only (diagnostic A/B runs)")
    ap.add_argument("--tag", default="", help="appended to the output file name")
    args = ap.parse_args()
    cases = {cs["name"]: cs for cs in rrm.build_matrix()}
    rrm._apply(sp, cases[args.case]["overrides"])
    if args.frame is not None:
        sp.INDIRECT_PMP_STAGE2_FRAME = args.frame
    sp.INDIRECT_PMP_TRANSVERSALITY = args.transversality
    if not (args.swarm or args.start or args.old_refine_starts):
        raise SystemExit("nothing to polish: pass --start, --old-refine-starts, or leave the swarm on")
    archive_root = Path(args.archive_out) if args.archive_out else POLISH_OUT / args.case
    global V_T
    V_T = ips.terminal_speed_target(R_T)

    t_start = time.time()
    print(f"case {args.case} (INCLUDE_DRAG {sp.INCLUDE_DRAG}) | "
          f"frame {sp.INDIRECT_PMP_STAGE2_FRAME} | transversality {sp.INDIRECT_PMP_TRANSVERSALITY} | "
          f"target speed {V_T:.2f} m/s | coast bounds [{DC_LB}, {DC_UB}] s | "
          f"swarm {args.particles}x{args.generations}, seed {sp.PSO_SEED}", flush=True)

    results = {}
    if args.swarm:
        best_x, best_f = ips.run_pso_optimization(verbose=True, n_particles=args.particles,
                                                  n_gen=args.generations)
        hist = ips.LAST_PSO_HISTORY
        t_swarm = time.time() - t_start
        u_s, gp_s = u_from_x(best_x)
        print(f"\n[swarm] {t_swarm:.0f} s, best J' {best_f:.6f}, x = {list(map(float, best_x))}")
        results.update({
            "swarm": report("swarm best point", u_s, gp_s, fly(u_s, gp_s, strict=False)),
            "swarm_best_f": float(best_f), "swarm_x": list(map(float, best_x)),
            "swarm_wall_s": t_swarm,
            "swarm_history": None if hist is None else
            {"gen": hist["gen"].tolist(), "gbest": hist["gbest"].tolist()}})
    starts = []
    for i, path in enumerate(args.start):
        u_st, gp_st, seed_st, desc = load_start(path, args.case)
        starts.append((f"start{i}", u_st, gp_st, seed_st, desc))
        results[f"start{i}"] = {"path": str(path), "description": desc, "seed": seed_st,
                                "point": report(f"start {i}: {desc}", u_st, gp_st,
                                                fly(u_st, gp_st, strict=False))}
    if args.polish:
        if args.swarm:
            results["polish_swarm"] = polish("swarm best", u_s, gp_s, args.span, args.step)
            starts.append(("swarm", u_s, gp_s, sp.PSO_SEED, "swarm run in this process"))
        for key, u_st, gp_st, seed_st, desc in starts:
            if key == "swarm":
                continue
            results["polish_" + key] = polish(desc, u_st, gp_st, args.span, args.step)
        if args.old_refine_starts:
            refine = json.loads(REFINE_JSON.read_text(encoding="utf-8"))["variants"]
            for key, name in (("B", "direct refinement B (gamma_p of the old swarm)"),
                              ("C", "direct refinement C (long-coast basin)")):
                cu = refine[key]["u"]
                T = cu[3] / 100.0 * T_MAX
                u_c = np.array([cu[0], cu[1], cu[4] / 100.0 * T, cu[2], T - cu[4] / 100.0 * T])
                results["polish_refine" + key] = polish(name, u_c, cu[5], args.span, args.step)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        for key, _u, _gp, seed_st, desc in starts:
            p = results.get("polish_" + key)
            if not p or not p.get("outer") or not p.get("outer_converged"):
                continue
            ob = p["outer"]
            name = args.case
            root = archive_root / f"{args.tag + '_' if args.tag else ''}{key}_{stamp}"
            saved = write_archive(np.array(ob["u"]), ob["gamma_p"], root, name,
                                  f"polished extremal from {desc}", seed_st)
            p["archive"] = str(root / (name + ".npz"))
            print(f"archived best extremal of {key} ({ob['prop_left_kg']:.1f} kg) to {p['archive']}")

    print("\n" + "=" * 92)
    print(f"{'':60}{'prop left [kg]':>16}{'converged':>12}")
    swarm = results.get("swarm")
    rows = ([("swarm best point (penalised, raw)",
              swarm["prop_left_kg"] if swarm else float("nan"), "-")] if args.swarm else [])
    for i in range(len(args.start)):
        st = results[f"start{i}"]
        if st["point"]:
            rows.append((f"start {i} as given", st["point"]["prop_left_kg"], "-"))
    labels = [("polish_swarm", "swarm")] + [(f"polish_start{i}", f"start {i}")
                                            for i in range(len(args.start))]
    labels += [("polish_refineB", "refinement B"), ("polish_refineC", "refinement C")]
    for key, name in labels:
        p = results.get(key)
        if not p:
            continue
        if p["fixed_gamma_p"]:
            rows.append((f"extremal from {name}, gamma_p fixed", p["fixed_gamma_p"]["prop_left_kg"],
                         str(p["fixed_converged"])))
        if p["outer"]:
            rows.append((f"extremal from {name}, best along gamma_p", p["outer"]["prop_left_kg"],
                         str(p["outer_converged"])))
    for name, val, conv in rows:
        print(f"{name:60}{val:16.1f}{conv:>12}")
    print("=" * 92)
    print(f"total wall time {time.time() - t_start:.0f} s")

    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / (f"pmp_swarm_polish_{args.case}_{args.tag + '_' if args.tag else ''}"
                 f"{time.strftime('%Y%m%d_%H%M%S')}.json")
    out.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
