"""Repair the thrust record of the results-matrix archives (2026-09-26).

The archived thrust channel was read back from the ODE right-hand-side log,
which holds trial samples past each root-found cutoff: up to ~0.9 s of phantom
Stage-1 thrust after MECO, a ramp over the last samples before it, and on the
legacy apogee_check path the same at its Stage-2 cutoff. Fixed at the source in
rocket_ascent._close_logged_burn / thrust_on_grid (tests/test_thrust_record.py).

The trajectories were never affected -- the log is output-only -- so an archive
is repaired, not re-flown into a new archive:

  1. re-fly the archived decision vector with the fixed code, in a fresh
     process configured from the archive's own manifest;
  2. require time, state and alpha to equal the archive BIT FOR BIT, so the
     flight is provably the one archived;
  3. take the re-flight's thrust record, rebuild the row through
     Archive.run_record.collect_row, and require every field outside the
     delta-v budget to be unchanged;
  4. write the thrust channel into the .npz, the budget fields into the .json,
     and a 'repairs' entry into the manifest.

With --write the originals are first copied to
Output/results_matrix_prerepair_20260926/. Nothing is written for a case that
fails a check.

Usage, from the repository root:
    PY dev-notes/repair_thrust_record.py              # dry run: checks only
    PY dev-notes/repair_thrust_record.py --write      # back up, then repair
"""

import argparse
import datetime
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "Tese" / "src"
sys.path.insert(0, str(SRC))

MATRIX = SRC / "Output" / "results_matrix"
BACKUP = SRC / "Output" / "results_matrix_prerepair_20260926"

BUDGET_KEYS = ("dv_ideal", "dv_gravity", "dv_drag", "dv_steering", "dv_pressure",
               "dv_losses", "dv_gain", "dv_achieved", "residual", "pressure_applicable")
# Not reproducible by a re-flight: the swarm's own record and the clock.
NOT_COMPARED = ("wall_clock_s", "n_evaluations", "pso_generations", "pso_gbest_first",
                "pso_gbest_last", "pso_tail_improvement_frac")


# ---------------------------------------------------------------------------
# Child: one case, in its own process
# ---------------------------------------------------------------------------

def _restore(value, current):
    """A manifest value (JSON) in the type the setting has in the module."""
    if isinstance(current, tuple) and isinstance(value, list):
        return tuple(value)
    if isinstance(current, np.ndarray) and isinstance(value, list):
        return np.asarray(value, dtype=current.dtype)
    if (isinstance(current, list) and current and isinstance(current[0], tuple)
            and isinstance(value, list)):
        return [tuple(v) if isinstance(v, list) else v for v in value]
    return value


def _configure(manifest):
    from Input_File import simulation_parameters as sp
    for key, value in manifest["config"].items():
        if hasattr(sp, key):
            setattr(sp, key, _restore(value, getattr(sp, key)))
    return sp


def _refly(sp, arch, x):
    """(time, data, thrust, alpha, result) of the full flight at x."""
    if arch == "pso_coast":
        import Simulation.pso_coast_solver as pcs
        t, d, th, al, _, res, _, _ = pcs.run_pso_coast_full(x, verbose=False)
        return t, d, th, al, res
    if arch == "direct":
        import Simulation.direct_pso_solver as dps
        t, d, th, al, _, res, _, _ = dps.run_pso_direct_full(x, verbose=False)
        return t, d, th, al, res
    if arch == "indirect_pmp":
        import Simulation.indirect_pso_solver as ips
        t, d, th, al, _, res = ips.run_indirect_full([float(v) for v in x], verbose=False)
        return t, d, th, al, res
    if arch == "reference_track":
        import Simulation.reference_track_solver as rts
        t, d, th, al, _, res, _, _ = rts.run_reference_track(verbose=False)
        return t, d, th, al, res
    if arch == "segmented":
        # As run_segmented builds it, minus the swarm.
        import Simulation.segmented_guidance_solver as sgs
        import Simulation.segment_reference as segref
        sgs.validate_schedule()
        if any(str(m) == "indirect_pmp" for m, _ in sp.GUIDANCE_SEGMENTS):
            t_ref, d_ref, a_ref = segref.get_pmp_reference_full(verbose=False)
            segs = sgs._Segments(t_ref, d_ref, alpha_full=a_ref)
        else:
            t_ref, d_ref = segref.get_pmp_reference(verbose=False)
            segs = sgs._Segments(t_ref, d_ref)
        nb = len(sgs._base_bounds(segs)[0])
        if bool(getattr(sp, "MULTI_GUIDANCE_OPTIMIZE_ALTITUDES", False)) and segs.n > 1:
            lb, ub = sgs._altitude_bounds(segs)
            segs.set_activation_altitudes(
                sgs._alts_from_fractions(x[nb:nb + (segs.n - 1)], lb, ub))
        t, d, th, al, _, res, _, _ = sgs.run_segmented_full(np.asarray(x[:nb]), segs,
                                                           verbose=False)
        return t, d, th, al, res
    if arch == "apogee_check":
        # As run_results_matrix._dispatch flies and assembles it.
        import Simulation.rocket_ascent as ra
        from Plots.plot_state_utils import interpolate_to_time
        ra.SINGLE_BURN_FULL_SIMULATION = True
        (t, d, _alt, delta_v, _m, th_log, t_log, al_log, al_t, _c, _f) = ra.run(float(x[0]))
        res = {'crashed': False, 'state_final': np.asarray(d)[:, -1],
               'circularisation_dv': float(delta_v),
               'state_final_inertial': bool(sp.ENABLE_EARTH_ROTATION)}
        d = ra.append_latitude_row(d)
        return (t, d, ra.thrust_on_grid(t, t_log, th_log),
                interpolate_to_time(al_t, al_log, t), res)
    raise ValueError("unrecognised architecture %r" % arch)


def _same(a, b):
    if isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b):
        return True
    return a == b


def child(case, write, report_path):
    folder = MATRIX / case
    npz_path = folder / (case + ".npz")
    old = dict(np.load(npz_path, allow_pickle=True))
    row = json.loads((folder / (case + ".json")).read_text(encoding="utf-8"))
    manifest = json.loads((folder / (case + ".manifest.json")).read_text(encoding="utf-8"))

    sp = _configure(manifest)
    arch = manifest["architecture"]
    x = np.asarray(old["decision_vector"], dtype=float)

    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        t, d, th, al, res = _refly(sp, arch, x)
    t, d, th, al = (np.asarray(v, dtype=float) for v in (t, d, th, al))

    rep = {"case": case, "architecture": arch, "ok": False}
    same_flight = (t.shape == old["time"].shape and np.array_equal(t, old["time"])
                   and d.shape == old["data"].shape and np.array_equal(d, old["data"])
                   and np.array_equal(al, old["alpha"]))
    rep["same_flight"] = bool(same_flight)
    if not same_flight:
        rep["why"] = "re-flight differs from the archive"
        if t.shape == old["time"].shape:
            rep["max_dt"] = float(np.max(np.abs(t - old["time"])))
            if d.shape == old["data"].shape:
                rep["max_dstate"] = float(np.max(np.abs(d - old["data"])))
        Path(report_path).write_text(json.dumps(rep), encoding="utf-8")
        return

    changed = np.flatnonzero(th != old["thrust"])
    rep["thrust_samples_changed"] = int(changed.size)
    if changed.size:
        rep["changed_span_s"] = [float(t[changed[0]]), float(t[changed[-1]])]

    from Archive import run_record
    new_row = run_record.collect_row(
        case, sp, t, d, th, al, res, row.get("J_prime"), None, row.get("wall_clock_s"),
        {'segment_laws': [str(v) for v in old["segment_laws"]]} if "segment_laws" in old else {},
        tags={"section": row["section"], "factor": row["factor"]})

    # A field collect_row does not produce (a provenance note added by hand, as
    # gt_sea_level_engine's orbit_columns_recomputed) is kept as archived.
    rep["kept_as_archived"] = sorted(k for k in row if k not in new_row)
    drift = {k: (row.get(k), new_row.get(k)) for k in row
             if k in new_row and k not in BUDGET_KEYS and k not in NOT_COMPARED
             and not _same(row.get(k), new_row.get(k))}
    rep["non_budget_drift"] = {k: [str(a), str(b)] for k, (a, b) in drift.items()}
    rep["budget_before"] = {k: row.get(k) for k in BUDGET_KEYS}
    rep["budget_after"] = {k: new_row.get(k) for k in BUDGET_KEYS}
    if drift:
        rep["why"] = "fields outside the budget changed"
        Path(report_path).write_text(json.dumps(rep, default=str), encoding="utf-8")
        return

    rep["ok"] = True
    if write:
        from Archive.store import _write_json
        new = dict(old)
        new["thrust"] = th
        # Written beside the archive and checked there; it replaces the archive
        # only once every check has passed, so a failure leaves nothing half-done.
        tmp_path = folder / (case + ".repair_tmp.npz")
        np.savez_compressed(tmp_path, **new)
        back = dict(np.load(tmp_path, allow_pickle=True))
        assert set(back) == set(old)
        for k in old:
            if k != "thrust":
                # Several arc times are NaN by design (t_guidance_start), and
                # NaN never equals itself.
                assert back[k].dtype == old[k].dtype and back[k].shape == old[k].shape, k
                assert np.array_equal(back[k], old[k],
                                      equal_nan=old[k].dtype.kind in "fc"), k
        assert np.array_equal(back["thrust"], th)
        os.replace(tmp_path, npz_path)

        for k in BUDGET_KEYS:
            row[k] = new_row.get(k)
        _write_json(folder / (case + ".json"), row)

        from Archive.run_record import git_info
        manifest.setdefault("repairs", []).append({
            "date": datetime.date.today().isoformat(),
            "git": git_info(),
            "what": ("thrust channel rebuilt from a bit-identical re-flight with "
                     "rocket_ascent._close_logged_burn / thrust_on_grid; delta-v budget "
                     "fields recomputed through run_record.collect_row; trajectory, "
                     "alpha and every other row field unchanged"),
            "script": "dev-notes/repair_thrust_record.py",
            "thrust_samples_changed": int(changed.size),
            "budget_before": rep["budget_before"],
        })
        _write_json(folder / (case + ".manifest.json"), manifest)
        rep["written"] = True
    Path(report_path).write_text(json.dumps(rep, default=str), encoding="utf-8")


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="back up, then repair")
    ap.add_argument("--case", help=argparse.SUPPRESS)
    ap.add_argument("--report", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.case:
        child(args.case, args.write, args.report)
        return

    import run_results_matrix as rrm
    cases = [c["name"] for c in rrm.build_matrix()]
    missing = [c for c in cases if not (MATRIX / c / (c + ".npz")).exists()]
    if missing:
        raise SystemExit("archives missing: %s" % ", ".join(missing))

    if args.write:
        if BACKUP.exists():
            # Reused only as a backup of exactly the files about to be repaired.
            import hashlib
            digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
            live = sorted(p.relative_to(MATRIX) for p in MATRIX.rglob("*") if p.is_file())
            kept = sorted(p.relative_to(BACKUP) for p in BACKUP.rglob("*") if p.is_file())
            if live != kept or any(digest(MATRIX / f) != digest(BACKUP / f) for f in live):
                raise SystemExit("%s exists and does not match %s; refusing to proceed"
                                 % (BACKUP, MATRIX))
            print("backup %s already holds these files, byte for byte" % BACKUP)
        else:
            shutil.copytree(MATRIX, BACKUP)
            print("backed up %s -> %s" % (MATRIX, BACKUP))

    import tempfile
    reports = []
    tmp = Path(tempfile.mkdtemp(prefix="repair_thrust_"))
    for case in cases:
        report = tmp / ("_repair_report_%s.json" % case)
        cmd = [sys.executable, str(Path(__file__).resolve()), "--case", case,
               "--report", str(report)] + (["--write"] if args.write else [])
        proc = subprocess.run(cmd, env=dict(os.environ, PYTHONIOENCODING="utf-8"),
                              capture_output=True, text=True, encoding="utf-8")
        if proc.returncode != 0 or not report.exists():
            print("%-22s CRASHED (exit %d)\n%s" % (case, proc.returncode, proc.stderr[-2000:]))
            reports.append({"case": case, "ok": False, "why": "crashed"})
            continue
        rep = json.loads(report.read_text(encoding="utf-8"))
        reports.append(rep)
        b0, b1 = rep.get("budget_before", {}), rep.get("budget_after", {})
        d_ideal = (b1.get("dv_ideal") - b0.get("dv_ideal")
                   if b0.get("dv_ideal") is not None and b1.get("dv_ideal") is not None else None)
        other = [k for k in BUDGET_KEYS if k not in ("dv_ideal", "residual")
                 and not _same(b0.get(k), b1.get(k))]
        print("%-22s %-15s same_flight=%-5s thrust_samples=%-4s span=%s  "
              "dv_ideal %s  residual %s -> %s  other budget changed: %s  %s%s" % (
                  case, rep.get("architecture"), rep.get("same_flight"),
                  rep.get("thrust_samples_changed"), rep.get("changed_span_s"),
                  "n/a" if d_ideal is None else "%+.3f m/s" % d_ideal,
                  b0.get("residual"), b1.get("residual"), other or "none",
                  "OK" if rep["ok"] else "FAIL: " + rep.get("why", ""),
                  "  WRITTEN" if rep.get("written") else ""))
        if rep.get("non_budget_drift"):
            print("    drift:", rep["non_budget_drift"])

    n_ok = sum(1 for r in reports if r["ok"])
    print("\n%d / %d cases pass every check%s" % (
        n_ok, len(reports), " and were written" if args.write else " (dry run: nothing written)"))
    if args.write:
        log = BACKUP.parent / "results_matrix_repair_20260926.json"
        log.write_text(json.dumps(reports, indent=2, default=str), encoding="utf-8")
        print("per-case report: %s" % log)

    if args.write and n_ok == len(reports):
        rows = rrm._rows_for_csv([], MATRIX)
        rrm._write_csv(rows, MATRIX / "results_matrix.csv")
        print("rebuilt %s (%d rows)" % (MATRIX / "results_matrix.csv", len(rows)))


if __name__ == "__main__":
    main()
