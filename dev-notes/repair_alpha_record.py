"""Repair the alpha record of show_seg_fixed_alt (2026-09-30).

The Stage-2 alpha channel is interpolated from a log the ODE right-hand side
appends on every call, trial points included. At the segmented schedule's
altitude hand-off the ended integration had already probed past the crossing
with the old law (the gravity turn, alpha = 0), and those entries interleaved
with the new law's once the log was sorted: the recorded alpha dipped for single
samples over the ~9 s after the 120 km hand-off, and the steering loss read low.
Fixed at the source in GuidanceState.discard_logs_after, called at the crossing
in segmented_guidance_solver._thrust_phase (tests/test_alpha_record.py).

A scan of the 18 reported archives found the dips in this case only;
show_seg_opt_alt hands over in Stage 1, where there is no second integration to
interleave. As on 2026-09-26 (repair_thrust_record.py) the flight was never
affected, so the archive is repaired, not re-flown into a new one:

  1. re-fly the archived decision vector with the fixed code, configured from
     the archive's own manifest;
  2. require time, state and thrust to equal the archive BIT FOR BIT, and alpha
     to differ only within 15 s after the hand-off;
  3. rebuild the row through Archive.run_record.collect_row and require every
     field outside the delta-v budget to be unchanged;
  4. write the alpha channel into the .npz, the budget fields into the .json,
     a 'repairs' entry into the manifest, and rebuild results_matrix.csv.

With --write the case's folder and the CSV are first copied to
Output/results_matrix_prerepair_20260930/. Nothing is written if a check fails.

Usage, from the repository root:
    PY dev-notes/repair_alpha_record.py              # dry run: checks only
    PY dev-notes/repair_alpha_record.py --write      # back up, then repair
"""

import argparse
import contextlib
import datetime
import io
import json
import os
import shutil
import sys
from pathlib import Path

import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "Tese" / "src"
sys.path.insert(0, str(SRC))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import repair_thrust_record as rtr  # noqa: E402  (manifest configuration, re-flight)

CASE = "show_seg_fixed_alt"
MATRIX = SRC / "Output" / "results_matrix"
BACKUP = SRC / "Output" / "results_matrix_prerepair_20260930"
WINDOW_S = 15.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="back up, then repair")
    args = ap.parse_args()

    folder = MATRIX / CASE
    npz_path = folder / (CASE + ".npz")
    old = dict(np.load(npz_path, allow_pickle=True))
    row = json.loads((folder / (CASE + ".json")).read_text(encoding="utf-8"))
    manifest = json.loads((folder / (CASE + ".manifest.json")).read_text(encoding="utf-8"))

    sp = rtr._configure(manifest)
    x = np.asarray(old["decision_vector"], dtype=float)
    with contextlib.redirect_stdout(io.StringIO()):
        t, d, th, al, res = rtr._refly(sp, manifest["architecture"], x)
    t, d, th, al = (np.asarray(v, dtype=float) for v in (t, d, th, al))

    # 2. the same flight, bit for bit; alpha different only after the hand-off
    same = (t.shape == old["time"].shape and np.array_equal(t, old["time"])
            and d.shape == old["data"].shape and np.array_equal(d, old["data"])
            and np.array_equal(th, old["thrust"]))
    if not same:
        raise SystemExit("re-flight differs from the archive: nothing written")
    handoff_alt = float(old["segment_altitudes"][1])
    t_switch = float(t[np.argmax(d[1] - 6378000.0 >= handoff_alt)])
    changed = np.flatnonzero(al != old["alpha"])
    if not changed.size:
        raise SystemExit("alpha unchanged: nothing to repair")
    span = (float(t[changed[0]]), float(t[changed[-1]]))
    if span[0] < t_switch or span[1] > t_switch + WINDOW_S:
        raise SystemExit("alpha changed outside [%.1f, %.1f] s: %s"
                         % (t_switch, t_switch + WINDOW_S, span))
    max_change = float(np.degrees(np.max(np.abs(al[changed] - old["alpha"][changed]))))

    # 3. the row, with only the budget allowed to move
    from Archive import run_record
    new_row = run_record.collect_row(
        CASE, sp, t, d, th, al, res, row.get("J_prime"), None, row.get("wall_clock_s"),
        {'segment_laws': [str(v) for v in old["segment_laws"]]},
        tags={"section": row["section"], "factor": row["factor"]})
    drift = {k: (row.get(k), new_row.get(k)) for k in row
             if k in new_row and k not in rtr.BUDGET_KEYS and k not in rtr.NOT_COMPARED
             and not rtr._same(row.get(k), new_row.get(k))}
    if drift:
        raise SystemExit("fields outside the budget changed: %s" % drift)
    before = {k: row.get(k) for k in rtr.BUDGET_KEYS}
    after = {k: new_row.get(k) for k in rtr.BUDGET_KEYS}

    print("%s: same flight; hand-off at %.3f s" % (CASE, t_switch))
    print("  alpha samples changed: %d, over %.2f-%.2f s, by up to %.2f deg"
          % (changed.size, span[0], span[1], max_change))
    for k in rtr.BUDGET_KEYS:
        if not rtr._same(before[k], after[k]):
            print("  %-12s %s -> %s" % (k, before[k], after[k]))
    if not args.write:
        print("dry run: nothing written")
        return

    # 4. back up, then write
    BACKUP.mkdir(parents=True, exist_ok=False)
    shutil.copytree(folder, BACKUP / CASE)
    shutil.copy2(MATRIX / "results_matrix.csv", BACKUP / "results_matrix.csv")
    print("backed up %s and results_matrix.csv -> %s" % (CASE, BACKUP))

    from Archive.store import _write_json
    from Archive.run_record import git_info
    new = dict(old)
    new["alpha"] = al
    tmp_path = folder / (CASE + ".repair_tmp.npz")
    np.savez_compressed(tmp_path, **new)
    back = dict(np.load(tmp_path, allow_pickle=True))
    assert set(back) == set(old)
    for k in old:
        if k != "alpha":
            assert back[k].dtype == old[k].dtype and back[k].shape == old[k].shape, k
            assert np.array_equal(back[k], old[k],
                                  equal_nan=old[k].dtype.kind in "fc"), k
    assert np.array_equal(back["alpha"], al)
    os.replace(tmp_path, npz_path)

    for k in rtr.BUDGET_KEYS:
        row[k] = after[k]
    _write_json(folder / (CASE + ".json"), row)
    manifest.setdefault("repairs", []).append({
        "date": datetime.date.today().isoformat(),
        "git": git_info(),
        "what": ("alpha channel rebuilt from a bit-identical re-flight with "
                 "GuidanceState.discard_logs_after at the segmented hand-off (the "
                 "gravity turn's trial-point samples past the crossing had "
                 "interleaved with peg_new's); delta-v budget fields recomputed "
                 "through run_record.collect_row; trajectory, thrust and every other "
                 "row field unchanged"),
        "script": "dev-notes/repair_alpha_record.py",
        "alpha_samples_changed": int(changed.size),
        "alpha_changed_span_s": list(span),
        "budget_before": before,
    })
    _write_json(folder / (CASE + ".manifest.json"), manifest)

    import run_results_matrix as rrm
    rows = rrm._rows_for_csv([], MATRIX)
    rrm._write_csv(rows, MATRIX / "results_matrix.csv")
    print("written; rebuilt %s (%d rows)" % (MATRIX / "results_matrix.csv", len(rows)))


if __name__ == "__main__":
    main()
