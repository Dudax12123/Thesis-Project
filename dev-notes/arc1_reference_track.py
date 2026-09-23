#!/usr/bin/env python
"""peg_new tracks the indirect-PMP reference's plan: no optimiser, law-terminated burns.

Three numbers come from the reference archive, and nothing else:
  gamma_p        the kick (decision_vector[6]), so Stage 1 is the reference's own
  arc-1 target   the reference's state at its coast start (t_coast_start)
  coast length   decision_vector[3]
Everything else is an output. Both Stage-2 burns end where peg_new's OWN time-to-go says:
the major loop runs outside the ODE right-hand side on accepted states, every
PEG_MAJOR_LOOP_RATE, and once t_go <= max(freeze threshold, cycle) the coefficients
freeze and the last cycle is flown to exactly t_go (the mechanism of
direct_pso_solver._fly_law_terminated_burn, generalised to an intermediate target and to a
second burn).

Flights (each in its own subprocess, so no rocket_ascent global leaks between archives):
  ref_track          arc 1 -> the reference's coast-start state; coast; arc 3 -> the orbit
  ctrl_no_waypoint   the same, but arc 1 aims at the final orbit: isolates the waypoint
  ctrl_ref_schedule  the reference's own 4-vector through the stock run_pso_coast_full
                     (time-terminated arcs, in-RHS major loop, final-orbit aim)

Configuration: exactly the results matrix's peg_baseline (rrm.BASELINE + peg_new). Each
flight is written as a standard archive under --out/<name>/ and can be overlaid with
run_archive.py compare.

Run from the repository root:
    C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/arc1_reference_track.py
"""

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")   # γ and ° in the report, on a cp1252 console

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "Tese" / "src"
sys.path.insert(0, str(SRC))

from Input_File import simulation_parameters as sp  # noqa: E402
import run_results_matrix as rrm  # noqa: E402

rrm._apply(sp, rrm.BASELINE)
rrm._apply(sp, {"GUIDANCE_MODE": "peg_new",
                "EVENTS_PRINT": False, "INTERRUPTS_PRINT": False})

from Auxiliary import constants as c  # noqa: E402
from Auxiliary import rocket_specs as rs  # noqa: E402
import Simulation.rocket_ascent as ra  # noqa: E402
import Simulation.pso_coast_solver as pcs  # noqa: E402
import Simulation.segment_reference as segref  # noqa: E402
import Guidance.peg_guidance_new as peg_new_mod  # noqa: E402

DEFAULT_REFERENCE = (SRC / "Output" / "pmp_polish_750x1500" / "pmp_baseline"
                     / "b750half_start0_20260921_162253" / "pmp_baseline.npz")
DEFAULT_OUT = SRC / "Output" / "arc1_reference_track"
RUNS = ("ref_track", "ctrl_no_waypoint", "ctrl_ref_schedule")
DT = 0.5
F2 = rs.F_THRUST_2
ISP2 = rs.ISP_2

# Stage-1 gate: shared code at the same gamma_p must reproduce the reference's ignition.
GATE_TOL = {"t": 1e-3, "h": 1.0, "v": 0.01, "gamma": 1e-5, "m": 0.1}


# ---------------------------------------------------------------------------
# Reference
# ---------------------------------------------------------------------------

def _state_at(t_grid, data, t):
    """State at a grid stamp; the last sample when the stamp is duplicated (arc boundary)."""
    i = int(np.searchsorted(t_grid, t, side="right")) - 1
    if i < 0 or abs(t_grid[i] - t) > 1e-6:
        raise SystemExit(f"reference has no sample at t = {t:.6f} s")
    return data[:5, i].copy()


def load_reference(path):
    path = Path(path).resolve()
    man_path = path.with_name(path.stem + ".manifest.json")
    if not man_path.exists():
        raise SystemExit(f"{man_path} missing: cannot check the reference's physics")
    man = json.loads(man_path.read_text(encoding="utf-8"))
    bad = segref._archive_mismatches(man["config"], check_search=False)
    if bad:
        raise SystemExit("reference flown under different physics: " + "; ".join(bad))
    z = np.load(path)
    t, d = z["time"], z["data"]
    x = np.asarray(z["decision_vector"], dtype=float)
    t_meco = float(z["t_meco"])
    t_ign = t_meco + rs.TIME_First_STAGE_SEPARATION + pcs._T_IGNITION_DELAY
    t_cs = float(z["t_coast_start"])
    row_path = path.with_suffix(".json")
    row = json.loads(row_path.read_text(encoding="utf-8")) if row_path.exists() else {}
    return {
        "path": path,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "label": man.get("label"),
        "x": x,
        "gamma_p": float(x[6]),
        "delta_tc": float(x[3]),
        "x_coast": [float(v) for v in x[3:7]],
        "t_meco": t_meco,
        "t_ignition": t_ign,
        "t_coast_start": t_cs,
        "t_seco": float(z["t_seco"]),
        "ignition": _state_at(t, d, t_ign),
        "waypoint": _state_at(t, d, t_cs),
        "seco": _state_at(t, d, float(z["t_seco"])),
        "row": row,
    }


# ---------------------------------------------------------------------------
# Flight primitives
# ---------------------------------------------------------------------------

def _teval(t0, t1):
    pts = np.arange(t0, t1, DT)
    if len(pts) == 0 or pts[-1] < t1:
        pts = np.append(pts, t1)
    return pts


def _ivp(t0, t1, y0, thrust, gs, t_eval):
    return solve_ivp(
        lambda t, y: pcs._stage2_ode_guidance(t, y, thrust, ISP2, gs),
        t_span=(t0, t1), y0=y0, t_eval=t_eval,
        rtol=pcs._RTOL, atol=pcs._ATOL, max_step=pcs._MAX_STEP,
        events=pcs._event_crash)


def fly_law_terminated_arc(t0, y0, gs, target):
    """One peg_new burn, ended where peg_new's own t_go says.

    target=None aims at the objective orbit exactly as _compute_alpha_stage2 resolves it for
    a single-law run; a SegmentTarget aims at (r, v cos g, v sin g). Returns
    (sols, t_end, y_end, crashed)."""
    gs.peg_new_external = True
    gs.target = target
    if target is not None:
        r_tgt, v_theta_T, v_r_T = target.r, target.v_theta_T, target.v_r_T
        freeze = (target.freeze_threshold if target.freeze_threshold is not None
                  else sp.APOLLO_FREEZE_THRESHOLD)
    else:
        r_tgt = c.R_EARTH + sp.TARGET_ORBITAL_ALTITUDE
        v_theta_T, v_r_T = pcs._v_circular_rotating(r_tgt), 0.0
        freeze = sp.APOLLO_FREEZE_THRESHOLD
    Ve = ISP2 * c.G_0
    cycle = float(sp.PEG_MAJOR_LOOP_RATE)
    last = max(float(freeze), cycle)
    t, y = float(t0), np.asarray(y0[:5], dtype=float).copy()
    # Arc 3 starts with whatever arc 1 left, so exhaustion follows the mass, not _T_MAX_2.
    t_exhaust = t + max(y[4] - pcs._DRY_MASS_2, 0.0) / pcs._MDOT_2
    sols = []
    while t < t_exhaust:
        (gs.peg_new_vgo_r, gs.peg_new_vgo_theta,
         gs.peg_new_L0, gs.peg_new_tgo,
         gs.peg_new_t_lambda, gs.peg_new_lambda_r) = peg_new_mod.peg_new_major_loop(
             y, r_tgt, c.MU_EARTH, Ve, F2, v_theta_T=v_theta_T, v_r_T=v_r_T)
        gs.peg_new_t_epoch = t
        tgo = gs.peg_new_tgo
        gs.tgo_time_log.append(t)
        gs.tgo_log.append(tgo)
        if not np.isfinite(tgo) or tgo <= 0.0:
            break
        final = tgo <= last
        gs.peg_new_frozen = final
        t_next = min(t + (tgo if final else cycle), t_exhaust)
        grid = _teval(t, t_next)
        sol = _ivp(t, t_next, y, F2, gs, grid if not sols else grid[1:])
        if len(sol.t_events[0]) > 0:
            return sols, t, y, True
        sols.append(sol)
        t, y = t_next, sol.y[:5, -1].copy()
        if final:
            break
    return sols, t, y, False


def fly_stage1(gamma_p):
    """Stage 1 + pre-ignition coast, exactly as run_pso_coast_full flies them."""
    ra.set_pseudo_forces_for_run(True)
    t2_start, s2, t_meco, t_st1, y_st1, crashed = ra.run_stage1(gamma_p - np.pi / 2.0)
    if crashed:
        raise RuntimeError("Stage 1 crashed")
    t_ign = t2_start + pcs._T_IGNITION_DELAY
    s2 = pcs._strip_to_pmp_state(s2, np.deg2rad(sp.LAUNCH_LATITUDE))
    s2 = ra.shed_fairing_if_due(t2_start, s2)
    sol_pre = _ivp(t2_start, t_ign, s2[:5], 0.0, None, _teval(t2_start, t_ign))
    y_ign = ra.shed_fairing_if_due(t_ign, sol_pre.y[:5, -1].copy())
    return t2_start, t_meco, t_ign, y_ign, t_st1, y_st1, sol_pre


def check_gate(ref, t_meco, t_ign, y_ign, skip):
    got = {"t": t_ign, "h": y_ign[1] - c.R_EARTH, "v": y_ign[2], "gamma": y_ign[3], "m": y_ign[4]}
    want = {"t": ref["t_ignition"], "h": ref["ignition"][1] - c.R_EARTH, "v": ref["ignition"][2],
            "gamma": ref["ignition"][3], "m": ref["ignition"][4]}
    print("\n  Stage-1 gate (ignition state vs the reference)")
    print(f"    t_meco   {t_meco:14.6f} s    ref {ref['t_meco']:14.6f}    "
          f"diff {t_meco - ref['t_meco']:+.3e}")
    fails = []
    for k, unit, scale in (("t", "s", 1.0), ("h", "km", 1e-3), ("v", "m/s", 1.0),
                           ("gamma", "deg", np.rad2deg(1.0)), ("m", "kg", 1.0)):
        diff = got[k] - want[k]
        flag = "" if abs(diff) <= GATE_TOL[k] else "   <-- OUT OF TOLERANCE"
        if flag:
            fails.append(k)
        print(f"    {k:<8} {got[k] * scale:14.6f} {unit:<4} ref {want[k] * scale:14.6f}    "
              f"diff {diff * scale:+.3e}{flag}")
    if fails and not skip:
        raise SystemExit("Stage-1 gate failed on " + ", ".join(fails) + ": this configuration "
                         "does not reproduce the reference's Stage 1 (use --skip-gate to look anyway)")


# ---------------------------------------------------------------------------
# Assembly (copied from run_pso_coast_full's bookkeeping, :1253-1374)
# ---------------------------------------------------------------------------

def assemble(t_st1, y_st1, segments, gs, y_insertion, t_insertion, t_coast_start):
    """segments: [(sol, thrust), ...] for Stage 2 up to insertion. Adds the post-insertion orbit
    coast, builds the full arrays, writes the ra.* channels the archive reads, returns
    (time, data, thrust, alpha)."""
    from Plots.plot_state_utils import interpolate_to_time

    post_init = y_insertion.copy()
    if sp.ENABLE_EARTH_ROTATION:
        v_in, g_in = ra.get_inertial_state_components(
            y_insertion[1], y_insertion[2], y_insertion[3], np.deg2rad(sp.LAUNCH_LATITUDE))
        post_init[2], post_init[3] = v_in, g_in
    ra.PROPAGATING_IN_INERTIAL_FRAME = True
    t_post_end = t_insertion + sp.DURATION_AFTER_SIMULATION
    sol_post = _ivp(t_insertion, t_post_end, post_init, 0.0, None,
                    _teval(t_insertion, t_post_end))
    segments = list(segments) + [(sol_post, 0.0)]

    t_parts, y_parts, th_parts = [], [], []
    for sol, F in segments:
        if sol is None or len(sol.t) == 0:
            continue
        t_parts.append(sol.t)
        y_parts.append(sol.y[:5, :])
        th_parts.append(np.full(len(sol.t), F))
    t2 = np.concatenate(t_parts)
    y2 = np.concatenate(y_parts, axis=1)
    th2 = np.concatenate(th_parts)
    alpha2 = (interpolate_to_time(gs.time_log, gs.alpha_log, t2) if gs.time_log
              else np.zeros(len(t2)))

    time_full = np.concatenate([t_st1, t2])
    data_full = np.concatenate([y_st1[:5, :], y2], axis=1)
    thrust_full = np.concatenate(
        [interpolate_to_time(ra.time_history, ra.thrust_history, t_st1), th2])
    alpha_full = np.concatenate(
        [interpolate_to_time(ra.alpha_time_history, ra.alpha_history, t_st1), alpha2])

    if sp.ENABLE_EARTH_ROTATION:
        lat_row = np.array([ra.get_latitude_from_downrange(s) for s in data_full[0]])
        data_full = np.vstack([data_full, lat_row])

    ra.theta_history = list(alpha_full + data_full[3])
    ra.theta_time_history = list(time_full)
    if gs.tgo_time_log:
        ra.tgo_time_history = list(gs.tgo_time_log)
        ra.tgo_history = list(gs.tgo_log)

    n_post = len(sol_post.t)
    saved = ra.PROPAGATING_IN_INERTIAL_FRAME
    ra.PROPAGATING_IN_INERTIAL_FRAME = False
    try:
        chf, cha, _cor, _cen = ra.pseudo_force_channels_on_grid(time_full, data_full[:5])
    finally:
        ra.PROPAGATING_IN_INERTIAL_FRAME = saved
    if n_post:
        chf[-n_post:] = 0.0
        cha[-n_post:] = 0.0
    if sp.COMPUTE_CROSS_HEADING_COUNTER_FORCE:
        ra.cross_heading_counter_force_history = list(chf)
        ra.cross_heading_accel_history = list(cha)

    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = t_insertion
    ra.PSO_COAST_ARC2_START_TIME = t_coast_start
    return time_full, data_full, thrust_full, alpha_full


# ---------------------------------------------------------------------------
# The flights
# ---------------------------------------------------------------------------

def fly_tracking(ref, waypoint, freeze_arc1, coast_mode, skip_gate):
    t2_start, t_meco, t_ign, y_ign, t_st1, y_st1, sol_pre = fly_stage1(ref["gamma_p"])
    check_gate(ref, t_meco, t_ign, y_ign, skip_gate)

    gs = pcs.GuidanceState()
    target = None
    if waypoint:
        wp = ref["waypoint"]
        target = pcs.SegmentTarget(r=float(wp[1]), alt=float(wp[1]) - c.R_EARTH, v=float(wp[2]),
                                   gamma=float(wp[3]), freeze_threshold=freeze_arc1)
    sols1, t_a1, y_a1, crashed = fly_law_terminated_arc(t_ign, y_ign, gs, target)
    if crashed:
        raise RuntimeError(f"crashed in arc 1 near t = {t_a1:.1f} s")
    n_tgo_arc1 = len(gs.tgo_log)

    if coast_mode == "duration":
        t_coast_end = t_a1 + ref["delta_tc"]
    else:
        t_coast_end = ref["t_coast_start"] + ref["delta_tc"]
        if t_coast_end <= t_a1:
            raise SystemExit("arc 1 ended after the reference's arc-3 start; --coast seco impossible")
    sol_c = _ivp(t_a1, t_coast_end, y_a1, 0.0, None, _teval(t_a1, t_coast_end))
    if len(sol_c.t_events[0]) > 0:
        raise RuntimeError("crashed during the coast")
    y_a3 = sol_c.y[:5, -1].copy()

    gs.restart_for_new_burn()
    sols3, t_a3, y_ins, crashed = fly_law_terminated_arc(t_coast_end, y_a3, gs, None)
    if crashed:
        raise RuntimeError(f"crashed in arc 3 near t = {t_a3:.1f} s")

    segments = ([(sol_pre, 0.0)] + [(s, F2) for s in sols1] + [(sol_c, 0.0)]
                + [(s, F2) for s in sols3])
    time_a, data, thrust, alpha = assemble(t_st1, y_st1, segments, gs, y_ins, t_a3, t_a1)

    arc1, arc3, coast = t_a1 - t_ign, t_a3 - t_coast_end, t_coast_end - t_a1
    burn = arc1 + arc3
    result = {
        "crashed": False, "state_final": y_ins, "t_f": burn + coast, "t_cf": coast,
        "t_stage2_start": t2_start, "t_ignition": t_ign, "t_arc2_start": t_a1,
        "t_arc3_end": t_a3, "t_stage1": t_st1, "y_stage1": y_st1,
    }
    x4 = [coast, 100.0 * burn / pcs._T_MAX_2, 100.0 * arc1 / burn if burn > 0 else 0.0,
          ref["gamma_p"]]
    info = {
        "t_ignition": t_ign, "t_arc1_end": t_a1, "t_arc3_start": t_coast_end, "t_arc3_end": t_a3,
        "y_arc1_end": y_a1, "y_insertion": y_ins, "target": target,
        "tgo_arc1": list(zip(gs.tgo_time_log[:n_tgo_arc1], gs.tgo_log[:n_tgo_arc1])),
        "tgo_arc3": list(zip(gs.tgo_time_log[n_tgo_arc1:], gs.tgo_log[n_tgo_arc1:])),
    }
    return time_a, data, thrust, alpha, result, x4, info


def fly_ref_schedule(ref, skip_gate):
    x4 = ref["x_coast"]
    t2_start, t_meco, t_ign, y_ign, *_ = fly_stage1(ref["gamma_p"])
    check_gate(ref, t_meco, t_ign, y_ign, skip_gate)
    time_a, data, thrust, alpha, t_ign, result, _cor, _cen = pcs.run_pso_coast_full(
        x4, verbose=False)
    t_a1, t_a3 = result["t_arc2_start"], result["t_arc3_end"]
    info = {
        "t_ignition": t_ign, "t_arc1_end": t_a1, "t_arc3_start": t_a1 + x4[0], "t_arc3_end": t_a3,
        "y_arc1_end": data[:5, int(np.searchsorted(time_a, t_a1, side="right")) - 1].copy(),
        "y_insertion": np.asarray(result["state_final"], dtype=float), "target": None,
        "tgo_arc1": [], "tgo_arc3": [],
    }
    return time_a, data, thrust, alpha, result, list(x4), info


# ---------------------------------------------------------------------------
# Report + archive
# ---------------------------------------------------------------------------

def _hvgm(y):
    return (f"h {(y[1] - c.R_EARTH) / 1e3:9.3f} km   v {y[2]:9.3f} m/s   "
            f"γ {np.rad2deg(y[3]):8.4f}°   m {y[4]:10.2f} kg")


def report(name, ref, result, info):
    wp, y1, yi = ref["waypoint"], info["y_arc1_end"], info["y_insertion"]
    print(f"\n  ── {name} " + "─" * (60 - len(name)))
    print(f"  Arc 1   {info['t_ignition']:9.3f} → {info['t_arc1_end']:9.3f} s   "
          f"({info['t_arc1_end'] - info['t_ignition']:.4f} s;  reference "
          f"{ref['t_coast_start'] - ref['t_ignition']:.4f} s, ends {ref['t_coast_start']:.3f})")
    print(f"    achieved  {_hvgm(y1)}")
    print(f"    reference {_hvgm(wp)}")
    dh, dv, dg, dm = y1[1] - wp[1], y1[2] - wp[2], y1[3] - wp[3], y1[4] - wp[4]
    print(f"    miss      Δh {dh / 1e3:+9.3f} km   Δv {dv:+9.3f} m/s   "
          f"Δγ {np.rad2deg(dg):+8.4f}°   Δm {dm:+10.2f} kg")
    for label, trace in (("arc 1", info["tgo_arc1"]), ("arc 3", info["tgo_arc3"])):
        if trace:
            head = ", ".join(f"{tt:.1f}:{tg:.2f}" for tt, tg in trace[:3])
            tail = ", ".join(f"{tt:.1f}:{tg:.3f}" for tt, tg in trace[-3:])
            print(f"    t_go {label} ({len(trace)} cycles)  first [{head}] … last [{tail}]")
    print(f"  Coast   {info['t_arc1_end']:9.3f} → {info['t_arc3_start']:9.3f} s   "
          f"({info['t_arc3_start'] - info['t_arc1_end']:.4f} s)")
    print(f"  Arc 3   {info['t_arc3_start']:9.3f} → {info['t_arc3_end']:9.3f} s   "
          f"({info['t_arc3_end'] - info['t_arc3_start']:.4f} s;  reference SECO "
          f"{ref['t_seco']:.3f})")
    print(f"    insertion {_hvgm(yi)}")
    print(f"    reference {_hvgm(ref['seco'])}")
    prop, prop_ref = yi[4] - pcs._DRY_MASS_2, ref["seco"][4] - pcs._DRY_MASS_2
    J = pcs.compute_coast_objective(result)
    br = pcs.breakdown_coast_objective(result)
    print(f"  Propellant remaining {prop:10.2f} kg   (reference {prop_ref:10.2f}, "
          f"Δ {prop - prop_ref:+9.2f} kg)")
    print(f"  J′ {J:.6f}   [" + ", ".join(f"{k} {v:.6f}" for k, v in br.items()) + "]")
    return J


def write_archive(name, out_root, ref, time_a, data, thrust, alpha, result, x4, info, J,
                  freeze_arc1, coast_mode, wall):
    from Archive import store
    tgt = info["target"]
    y1 = info["y_arc1_end"]
    wp = ref["waypoint"]
    nan3 = [float("nan")] * 3
    extra = {
        "decision_vector": [float(v) for v in x4],
        "arc1_target": nan3 if tgt is None else [tgt.r, tgt.v, tgt.gamma],
        "arc1_achieved": [float(v) for v in y1[1:5]],
        "arc1_miss_vs_reference": [float(y1[1] - wp[1]), float(y1[2] - wp[2]),
                                   float(y1[3] - wp[3]), float(y1[4] - wp[4])],
        "arc1_freeze_threshold": float("nan") if tgt is None else float(freeze_arc1),
        "t_arc1_end": float(info["t_arc1_end"]),
        "t_arc3_start": float(info["t_arc3_start"]),
        "t_arc3_end": float(info["t_arc3_end"]),
        "coast_mode": coast_mode,
        "reference_archive": str(ref["path"]),
        "reference_sha256": ref["sha256"],
        "reference_decision_vector": [float(v) for v in ref["x"]],
        "reference_arc1_end": ref["t_coast_start"],
    }
    saved = store.save_run(sp, time_a, data, thrust, alpha, result, J=float(J), history=None,
                           extra=extra, wall_clock=wall, name=name, root=Path(out_root) / name,
                           source="dev-notes/arc1_reference_track.py",
                           label=f"{name}: peg_new tracking {ref['path'].parent.name}",
                           tags={"section": "dev", "factor": "reference_tracking"},
                           verbose=True)
    return saved


def _name(run, args):
    name = run
    if run == "ref_track" and args.freeze != sp.SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD:
        name += f"_freeze{args.freeze:g}"
    if run != "ctrl_ref_schedule" and args.coast != "duration":
        name += "_coastseco"
    return name


def run_one(run, args):
    ref = load_reference(args.reference)
    name = _name(run, args)
    print(f"\n=== {name} ===   reference: {ref['label']}")
    print(f"    γ_p {ref['gamma_p']:.16g} rad   coast {ref['delta_tc']:.10g} s   "
          f"waypoint at t = {ref['t_coast_start']:.6f} s")
    t0 = time.time()
    if run == "ctrl_ref_schedule":
        out = fly_ref_schedule(ref, args.skip_gate)
    else:
        out = fly_tracking(ref, run == "ref_track", args.freeze, args.coast, args.skip_gate)
    time_a, data, thrust, alpha, result, x4, info = out
    wall = time.time() - t0
    J = report(name, ref, result, info)
    write_archive(name, args.out, ref, time_a, data, thrust, alpha, result, x4, info, J,
                  args.freeze, args.coast, wall)
    print(f"  wall clock {wall:.1f} s")


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def summary(names, out_root, ref):
    rows = [("reference (PMP)", ref["row"], {"t_coast_start": ref["t_coast_start"],
                                              "t_seco": ref["t_seco"]})]
    for name in names:
        stem = Path(out_root) / name / name
        if not stem.with_suffix(".json").exists():
            continue
        row = json.loads(stem.with_suffix(".json").read_text(encoding="utf-8"))
        z = np.load(stem.with_suffix(".npz"))
        rows.append((name, row, {"t_coast_start": float(z["t_coast_start"]),
                                 "t_seco": float(z["t_seco"])}))
    prop_ref = ref["seco"][4] - pcs._DRY_MASS_2
    print("\n" + "=" * 112)
    print(f"  {'flight':<24}{'arc1 end':>10}{'SECO':>11}{'h_ins km':>11}{'v_ins m/s':>11}"
          f"{'γ_ins °':>9}{'prop kg':>11}{'Δ vs ref':>11}{'J′':>11}")
    for name, row, tm in rows:
        prop = row.get("prop_remaining_kg", float("nan"))
        print(f"  {name:<24}{tm['t_coast_start']:10.3f}{tm['t_seco']:11.3f}"
              f"{row.get('insertion_alt_km', float('nan')):11.3f}"
              f"{row.get('insertion_v_ms', float('nan')):11.3f}"
              f"{row.get('insertion_fpa_deg', float('nan')):9.4f}"
              f"{prop:11.2f}{prop - prop_ref:+11.2f}{row.get('J_prime', float('nan')):11.6f}")
    print("=" * 112)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--reference", default=str(DEFAULT_REFERENCE),
                    help="indirect_pmp archive .npz supplying gamma_p, the waypoint and the coast")
    ap.add_argument("--runs", default=",".join(RUNS),
                    help="comma-separated subset of " + ", ".join(RUNS))
    ap.add_argument("--freeze", type=float, default=float(sp.SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD),
                    help="arc-1 freeze threshold for ref_track [s] (arc 3 keeps "
                         "APOLLO_FREEZE_THRESHOLD)")
    ap.add_argument("--coast", choices=["duration", "seco"], default="duration",
                    help="duration: coast the reference's length from wherever arc 1 ends; "
                         "seco: coast until the reference's own arc-3 start")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="archive root")
    ap.add_argument("--skip-gate", action="store_true",
                    help="continue even if Stage 1 does not reproduce the reference")
    args = ap.parse_args()
    runs = [s.strip() for s in args.runs.split(",") if s.strip()]
    unknown = set(runs) - set(RUNS)
    if unknown:
        raise SystemExit("unknown run(s): " + ", ".join(sorted(unknown)))

    if len(runs) == 1:
        run_one(runs[0], args)
        return

    # One subprocess per flight: rocket_ascent keeps its channels in module globals.
    failed = []
    for run in runs:
        cmd = [sys.executable, str(Path(__file__).resolve()), "--runs", run,
               "--reference", args.reference, "--freeze", repr(args.freeze),
               "--coast", args.coast, "--out", args.out]
        if args.skip_gate:
            cmd.append("--skip-gate")
        sys.stdout.flush()
        if subprocess.run(cmd).returncode != 0:
            failed.append(run)
    summary([_name(run, args) for run in runs], args.out, load_reference(args.reference))
    if failed:
        raise SystemExit("failed: " + ", ".join(failed))


if __name__ == "__main__":
    main()
