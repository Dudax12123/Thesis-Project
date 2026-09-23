#!/usr/bin/env python
"""peg_new tracks the indirect-PMP reference's plan: no optimiser, law-terminated burns.

Three numbers come from the reference archive, and nothing else:
  gamma_p        the kick (decision_vector[6]), so Stage 1 is the reference's own
  arc-1 target   the reference's state at its coast start (t_coast_start)
  coast length   decision_vector[3]
Everything else is an output. Both Stage-2 burns end where peg_new's OWN time-to-go says.
The flight itself is Simulation/reference_track_solver.py -- the code behind
COAST_METHOD="reference_track" and the results-matrix case show_ref_track -- so this
script and the matrix fly the same thing. What the script adds is any reference archive
(the matrix reads the tracked cache), the arc-1 freeze and coast mode as flags, and the
two controls.

Flights (each in its own subprocess, so no rocket_ascent global leaks between archives):
  ref_track          arc 1 -> the reference's coast-start state; coast; arc 3 -> the orbit
  ctrl_no_waypoint   the same, but arc 1 aims at the final orbit: isolates the waypoint
  ctrl_ref_schedule  the reference's own 4-vector through the stock run_pso_coast_full
                     (time-terminated arcs, in-RHS major loop, final-orbit aim)

The arc-1 freeze defaults to 10 s (APOLLO_FREEZE_THRESHOLD). Below ~10 s the realigned
peg_new's endgame does not terminate: at 2 s arc 1 burned to propellant exhaustion.

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
import Simulation.pso_coast_solver as pcs  # noqa: E402
import Simulation.reference_track_solver as rts  # noqa: E402
import Simulation.segment_reference as segref  # noqa: E402

DEFAULT_REFERENCE = (SRC / "Output" / "pmp_polish_750x1500" / "pmp_baseline"
                     / "b750half_start0_20260921_162253" / "pmp_baseline.npz")
DEFAULT_OUT = SRC / "Output" / "arc1_reference_track"
DEFAULT_FREEZE = float(sp.APOLLO_FREEZE_THRESHOLD)
RUNS = ("ref_track", "ctrl_no_waypoint", "ctrl_ref_schedule")


# ---------------------------------------------------------------------------
# Reference
# ---------------------------------------------------------------------------

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
    plan = rts.plan_from_reference(z["time"], z["data"], z["decision_vector"], source=str(path))
    t_meco = float(z["t_meco"])
    t_ign = t_meco + rs.TIME_First_STAGE_SEPARATION + pcs._T_IGNITION_DELAY
    t_cs = float(z["t_coast_start"])
    row_path = path.with_suffix(".json")
    row = json.loads(row_path.read_text(encoding="utf-8")) if row_path.exists() else {}
    return {
        "path": path,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "label": man.get("label"),
        "plan": plan,
        "x": plan["x"],
        "gamma_p": plan["gamma_p"],
        "delta_tc": plan["delta_tc"],
        "x_coast": [float(v) for v in plan["x"][3:7]],
        "t_meco": t_meco,
        "t_ignition": t_ign,
        "t_coast_start": t_cs,
        "t_seco": float(z["t_seco"]),
        "ignition": rts.reference_state_at(plan, t_ign),
        "waypoint": rts.reference_state_at(plan, t_cs),
        "seco": rts.reference_state_at(plan, float(z["t_seco"])),
        "row": row,
    }


def print_gate(ref, t_meco, diffs):
    print("\n  Stage-1 gate (ignition state vs the reference)")
    print(f"    t_meco   {t_meco:14.6f} s    ref {ref['t_meco']:14.6f}    "
          f"diff {t_meco - ref['t_meco']:+.3e}")
    for k, unit, scale in (("h", "km", 1e-3), ("v", "m/s", 1.0),
                           ("gamma", "deg", np.rad2deg(1.0)), ("m", "kg", 1.0)):
        d = diffs.get(k, float("nan"))
        flag = "" if abs(d) <= rts.STAGE1_TOL[k] else "   <-- OUT OF TOLERANCE"
        print(f"    {k:<8} diff {d * scale:+.3e} {unit}{flag}")


# ---------------------------------------------------------------------------
# The flights
# ---------------------------------------------------------------------------

def fly_tracking(ref, waypoint, freeze_arc1, coast_mode, skip_gate):
    out = rts.run_reference_track(plan=ref["plan"], verbose=False, waypoint=waypoint,
                                  arc1_freeze=freeze_arc1, coast_mode=coast_mode,
                                  check_stage1_state=not skip_gate)
    time_a, data, thrust, alpha, _t_ign, result, _cor, _cen = out
    info = rts.LAST_REFERENCE_TRACK
    print_gate(ref, info["t_meco"], info["stage1_diffs"])
    if info["crashed_in"]:
        raise RuntimeError(f"crashed in {info['crashed_in']}")
    return time_a, data, thrust, alpha, result, info["realised_schedule"], info


def fly_ref_schedule(ref, skip_gate):
    x4 = ref["x_coast"]
    _t2, t_meco, t_ign, y_ign, *_ = rts.fly_stage1(ref["gamma_p"])
    diffs, failed = rts.check_stage1(ref["plan"], t_ign, y_ign)
    print_gate(ref, t_meco, diffs)
    if failed and not skip_gate:
        raise SystemExit("Stage-1 gate failed on " + ", ".join(failed))
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
                  coast_mode, wall):
    from Archive import store
    if "plan" in info:          # a reference_track flight: the solver's own record
        extra = rts.archive_extra()
    else:                       # ctrl_ref_schedule: the stock pso_coast replay
        y1, wp = info["y_arc1_end"], ref["waypoint"]
        extra = {
            "decision_vector": [float(v) for v in x4],
            "arc1_achieved": [float(v) for v in y1[1:5]],
            "arc1_miss_vs_reference": [float(y1[k] - wp[k]) for k in (1, 2, 3, 4)],
            "t_arc1_end": float(info["t_arc1_end"]),
            "t_arc3_start": float(info["t_arc3_start"]),
            "t_arc3_end": float(info["t_arc3_end"]),
            "coast_mode": coast_mode,
        }
    extra.update({
        "reference_archive": str(ref["path"]),
        "reference_sha256": ref["sha256"],
        "reference_decision_vector": [float(v) for v in ref["x"]],
        "reference_arc1_end": ref["t_coast_start"],
    })
    saved = store.save_run(sp, time_a, data, thrust, alpha, result, J=float(J), history=None,
                           extra=extra, wall_clock=wall, name=name, root=Path(out_root) / name,
                           source="dev-notes/arc1_reference_track.py",
                           label=f"{name}: peg_new tracking {ref['path'].parent.name}",
                           tags={"section": "dev", "factor": "reference_tracking"},
                           verbose=True)
    return saved


def _name(run, args):
    name = run
    if run == "ref_track" and args.freeze != DEFAULT_FREEZE:
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
                  args.coast, wall)
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
    ap.add_argument("--freeze", type=float, default=DEFAULT_FREEZE,
                    help="arc-1 freeze threshold for ref_track [s] (default %(default)g, "
                         "APOLLO_FREEZE_THRESHOLD; arc 3 always uses it)")
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
