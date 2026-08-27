"""
RESULTS MATRIX HARNESS

Runs the designed set of trajectories behind Chapter 6 and writes one tidy row
per run, so that the results chapter is assembled from a table rather than read
off twenty plots.

The design is one frozen baseline with **one factor changed at a time**, and
the factors are varied *within* a guidance law rather than across all nine. Two
laws are analysed in depth -- the gravity turn as a passive floor and peg_new as
the closed-loop ceiling -- against indirect_pmp as the optimal reference; the
remaining six are flown once each for breadth. Everything else in
simulation_parameters.py is a fixed condition of the experiment and is recorded
once, in the manifest, rather than swept. See Chapter 6 §6.1.

Usage
-----
Run the whole matrix (long — see --smoke first):

    python Tese/src/run_results_matrix.py

Prove every case dispatches, in a couple of minutes, before committing a night
to it:

    python Tese/src/run_results_matrix.py --smoke

One case, in-process, with solver output on screen:

    python Tese/src/run_results_matrix.py --case peg_baseline

A subset at a reduced budget, into its own root so the production set is not
touched (--only takes a comma-separated list of substrings; gt_,peg_ is exactly
the ten cases of Chapter 6 sections 6.2 and 6.3):

    python Tese/src/run_results_matrix.py --only gt_,peg_ --budget 50,100 \
        --out Output/results_matrix_r50x100

Note what --budget does NOT reach. COAST_METHOD="apogee_check" runs no PSO at
all -- its cost is the Ns=1000 brute grid in solver.py, and every grid point is
a complete ra.run() ascent -- so gt_apogee costs the same at any budget and is
usually the most expensive case in the set. PSO_DIRECT_* is already 50x100 in
the shipped config, so --budget 50,100 leaves the two "direct" cases at full
production fidelity.

Per-case output
---------------
Each case writes the standard three-file archive into its own folder --
Output/results_matrix/<case>/<case>.npz, .json and .manifest.json -- plus one
aggregate results_matrix.csv at the top of results_matrix/. The .npz holds the trajectory together with the channels the
Chapter 6 figures need: the PSO convergence curve, the arc boundary times, the
pseudo-force diagnostics, the theta and t_go histories and, for the segmented
cases, the schedule that flew. Those are solver state, not results: they are
gone the moment the subprocess exits, so anything not captured here costs a
re-run of the case to recover. The manifest records the configuration each case
was actually flown under, which is what makes a case comparable against a run
flown six months later.

The writing itself is Archive/store.save_run, the same call main.py makes for an
interactive run, so a batch case and a hand-flown run are the same kind of
object and load through the same Plots.results_figures._data.load. The figures themselves are built offline from these files
by Plots/results_figures/, which is why SAVE_PLOTS stays False -- firing the
per-run debugging suite for every case would write hundreds of PNGs the chapter
does not use.

Isolation
---------
Each case runs in its **own subprocess**. rocket_ascent.py keeps guidance and
ramp state in module globals, and the solvers reset what they know about; a
fresh interpreter per case removes the whole question rather than relying on
those resets covering every field. It costs a process start against a run
measured in tens of minutes.

Pseudo-forces are deliberately NOT set here. Every solver calls
ra.set_pseudo_forces_for_run() at its own entry point, and the segmented solver
does it per trajectory because its reference build legitimately differs from the
trajectories it then flies. Setting it from the harness would fight that.
"""

import argparse
import json
import os
import subprocess
import sys
import time as _time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np

from Archive import run_record, store

_HERE = Path(__file__).resolve().parent
OUTPUT_DIR = _HERE / "Output" / "results_matrix"


def _set_output_dir(path):
    """Point the batch at a different root.

    A rehearsal at a reduced budget must not be mistaken for, or overwrite, the
    production set: the case folder names are identical, and only the manifest
    records which budget flew. Giving the rehearsal its own root removes the
    question instead of relying on anyone reading the manifest.
    """
    global OUTPUT_DIR
    candidate = Path(path)
    OUTPUT_DIR = candidate if candidate.is_absolute() else _HERE / candidate
    return OUTPUT_DIR

# The two laws the chapter analyses in depth: a passive floor whose trajectory
# shape comes entirely from the kick and the arc timing, and the strongest
# closed-loop law, which is also one of only two that close under every
# architecture. indirect_pmp is neither -- it is its own architecture and enters
# as the reference the other two are measured against.
DEPTH_LAWS = ["gravity_turn", "peg_new"]

# Flown once each at the baseline for the capability section, reported as a
# single summary table with no per-law prose. The thesis claims nine guidance
# laws; this is what keeps that claim honest without giving six laws sections
# they do not earn.
SHOWCASE_LAWS = [
    "cpr",
    "linear_tangent",
    "bilinear_tangent",
    "apollo",
    "peg",
    "exp_shooting",
]

# Applied to every case before its own overrides. This is the frozen baseline.
BASELINE = {
    "MULTI_GUIDANCE_ENABLED": False,
    "COAST_METHOD": "pso_coast",
    "INCLUDE_DRAG": True,
    "INCLUDE_LIFT": True,
    "ENABLE_EARTH_ROTATION": True,
    "INCLUDE_PSEUDO_FORCES": True,
    "COMPUTE_CROSS_HEADING_COUNTER_FORCE": True,
    "TARGET_ORBITAL_ALTITUDE": 500e3,
    "TARGET_ORBIT_INCLINATION": 51.6,
    "LAUNCH_LATITUDE": 28.5,
    "ATMOSPHERE_EXIT_METHOD": "dynamic_pressure",
    "DYNAMIC_PRESSURE_THRESHOLD": 1000.0,
    # Pinned, and it has to be. Only the legacy run() honours this setting, so
    # under the config default of "triangular" the one apogee_check case would
    # fly a 45 s triangular pitch-over while every other case flew the
    # instantaneous gamma jump of ra.run_stage1() -- which ignores the setting
    # by design. The architecture comparison of Section 6.2 would then differ in
    # two factors rather than one, invisibly, since nothing downstream reports
    # the kick profile.
    "KICK_PROFILE_MODE": "instantaneous",
    "TGO_ESTIMATOR": "rocket_equation",
    "GUIDANCE_TGO_USE_PSO_PLAN": False,
    # The two halves of one nozzle model — see rocket_ascent._get_stage1_isp.
    # Required for the pressure loss of Auxiliary/losses.py to be meaningful.
    "ISP_1_MODE": "pressure",
    "THRUST_1_MODE": "pressure",
    "SAVE_PLOTS": False,
}

# Token budgets for --smoke. Enough to exercise every dispatch path and every
# collection branch; far too few to mean anything numerically.
#
# PMP_REFERENCE_CACHE is redirected, and that redirect is load-bearing. The two
# lines below it drop the reference PSO to 8x4, and those two settings are part
# of the reference CACHE KEY: without the redirect, a smoke run reaching either
# segmented case rebuilds the reference at a token budget and overwrites
# Tese/src/Output/pmp_reference.npz -- which is TRACKED IN GIT, took ~1 h to
# build at 250x500, and is the waypoint source every segmented run is measured
# against. That is the whole point of a smoke test destroying the one artefact a
# smoke test must not destroy, and it has happened before. Sending smoke's
# reference to its own file keeps the token build fast AND the tracked one
# untouched.
SMOKE_BUDGET = {
    "PMP_REFERENCE_CACHE": "Tese/src/Output/pmp_reference_smoke.npz",
    "PSO_N_PARTICLES": 8, "PSO_MAX_GENERATIONS": 4,
    "PSO_COAST_N_PARTICLES": 8, "PSO_COAST_MAX_GENERATIONS": 4,
    "PSO_DIRECT_N_PARTICLES": 8, "PSO_DIRECT_MAX_GENERATIONS": 4,
    "PSO_MG_N_PARTICLES": 8, "PSO_MG_MAX_GENERATIONS": 4,
    "PMP_REFERENCE_PSO_PARTICLES": 8, "PMP_REFERENCE_PSO_GENERATIONS": 4,
}


def budget_overrides(particles, generations):
    """A uniform PSO budget for every swarm-based architecture.

    This is how --budget applies a reduced budget WITHOUT hand-editing
    simulation_parameters.py: an edited config outlives the experiment and
    silently contaminates every later run, which is why nothing here writes to
    that file.

    PMP_REFERENCE_PSO_* is deliberately excluded. Those two are part of the PMP
    reference cache key, so changing them rebuilds and overwrites the tracked
    Output/pmp_reference.npz -- a ~1 h build, and one that has been triggered by
    accident before. A reduced budget is for the cases being flown, never for
    the cached reference they are measured against.
    """
    p, g = int(particles), int(generations)
    return {
        "PSO_N_PARTICLES": p, "PSO_MAX_GENERATIONS": g,
        "PSO_COAST_N_PARTICLES": p, "PSO_COAST_MAX_GENERATIONS": g,
        "PSO_DIRECT_N_PARTICLES": p, "PSO_DIRECT_MAX_GENERATIONS": g,
        "PSO_MG_N_PARTICLES": p, "PSO_MG_MAX_GENERATIONS": g,
    }


def _parse_budget(text):
    """'50,100' -> (50, 100)."""
    parts = text.split(",")
    if len(parts) != 2:
        raise SystemExit("--budget wants PARTICLES,GENERATIONS (e.g. 50,100), got %r" % text)
    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        raise SystemExit("--budget wants two integers, got %r" % text)


def build_matrix():
    """The 20 production cases, in chapter order.

    The design is one frozen baseline with **one factor changed at a time**, but
    the factors are varied *within* each guidance law rather than across all
    nine. Sections 6.2 and 6.3 each take one law and move it along the axes that
    apply to it; 6.4 is the reference; 6.7 shows breadth at reduced depth.
    Sections 6.5 (losses) and 6.6 (comparison) own no runs -- they re-read the
    rows below through a different lens, which is why the chapter is shorter
    than the matrix is wide.
    """
    cases = []

    # --- Section 6.2: gravity turn ---------------------------------------
    # The passive law carries the physics axes. With alpha = 0 after the kick
    # there is no steering logic to confound the response, which is precisely
    # the argument for isolating rotation and the engine model here rather than
    # on a closed-loop law.
    cases.append(dict(name="gt_baseline", section="6.2", factor="baseline",
                      overrides={"GUIDANCE_MODE": "gravity_turn"}))
    cases.append(dict(name="gt_apogee", section="6.2", factor="architecture",
                      overrides={"GUIDANCE_MODE": "gravity_turn",
                                 "COAST_METHOD": "apogee_check"}))
    # Expected to finish SUBORBITAL, and that is the result being collected, not
    # a failure to be fixed. direct is a single continuous Stage-2 burn with no
    # coast, which is delta-v-marginal and closes only for the explicit
    # terminal-constraint laws; the verdict is empirical (identical at 900 and
    # 5000 evaluations, so a true optimum rather than under-convergence). Paired
    # with peg_direct below it is the argument of the chapter for why coast arcs
    # exist. Reported via periapsis_km, which will be negative.
    cases.append(dict(name="gt_direct", section="6.2", factor="architecture",
                      overrides={"GUIDANCE_MODE": "gravity_turn",
                                 "COAST_METHOD": "direct"}))
    # INCLUDE_DRAG=False is the master no-atmosphere switch: it also drops the
    # fairing and zeroes ambient pressure, so this run flies vacuum thrust and
    # vacuum Isp. It differs from the baseline on two counts, not one.
    cases.append(dict(name="gt_vacuum", section="6.2", factor="atmosphere",
                      overrides={"GUIDANCE_MODE": "gravity_turn",
                                 "INCLUDE_DRAG": False}))
    cases.append(dict(name="gt_norot", section="6.2", factor="rotation",
                      overrides={"GUIDANCE_MODE": "gravity_turn",
                                 "ENABLE_EARTH_ROTATION": False,
                                 "INCLUDE_PSEUDO_FORCES": False,
                                 "COMPUTE_CROSS_HEADING_COUNTER_FORCE": False}))
    # Both halves of the nozzle model move together. Scaling thrust with
    # altitude while holding Isp fixed raises implied mass flow ~8% and burns
    # Stage 1 too fast. Under a constant mode losses.py reports
    # pressure_applicable=False rather than a number, and that absence is the
    # result: a deficit computed against a constant thrust would describe a
    # vehicle that was not simulated.
    cases.append(dict(name="gt_sea_level_engine", section="6.2", factor="engine_model",
                      overrides={"GUIDANCE_MODE": "gravity_turn",
                                 "ISP_1_MODE": "sea_level",
                                 "THRUST_1_MODE": "sea_level"}))

    # --- Section 6.3: powered explicit guidance ---------------------------
    # peg_new takes the architecture axis on the two swarm-based strategies,
    # and the environment is stripped in two steps rather than one: the vacuum
    # run removes the atmosphere from the baseline, and the run below it removes
    # the rotation from the vacuum run. Each comparison is still a single factor
    # against the case directly above it.
    cases.append(dict(name="peg_baseline", section="6.3", factor="baseline",
                      overrides={"GUIDANCE_MODE": "peg_new"}))
    cases.append(dict(name="peg_direct", section="6.3", factor="architecture",
                      overrides={"GUIDANCE_MODE": "peg_new",
                                 "COAST_METHOD": "direct"}))
    cases.append(dict(name="peg_vacuum", section="6.3", factor="atmosphere",
                      overrides={"GUIDANCE_MODE": "peg_new",
                                 "INCLUDE_DRAG": False}))
    # Read against peg_vacuum, not against peg_baseline: the one factor between
    # them is the rotation. Together they give the idealised case -- no air, no
    # rotating frame -- which is the closest the flyable law comes to the
    # conditions the indirect reference is derived on.
    cases.append(dict(name="peg_vacuum_norot", section="6.3", factor="rotation",
                      overrides={"GUIDANCE_MODE": "peg_new",
                                 "INCLUDE_DRAG": False,
                                 "ENABLE_EARTH_ROTATION": False,
                                 "INCLUDE_PSEUDO_FORCES": False,
                                 "COMPUTE_CROSS_HEADING_COUNTER_FORCE": False}))

    # --- Section 6.4: the reference ---------------------------------------
    # Needs the large PSO budget; a reduced one leaves it far from a closed
    # orbit and it is convergence-limited rather than broken. It is also exempt
    # from the rotating-frame pseudo-forces the other cases carry, because its
    # costate equations are derived on the drag-free EOM -- see
    # pseudo_forces_flown in the collected row.
    cases.append(dict(name="pmp_baseline", section="6.4", factor="reference",
                      overrides={"GUIDANCE_MODE": "indirect_pmp"}))
    cases.append(dict(name="pmp_vacuum", section="6.4", factor="reference",
                      overrides={"GUIDANCE_MODE": "indirect_pmp",
                                 "INCLUDE_DRAG": False}))

    # --- Section 6.7: capability showcase ---------------------------------
    for law in SHOWCASE_LAWS:
        cases.append(dict(name="show_" + law, section="6.7", factor="law_breadth",
                          overrides={"GUIDANCE_MODE": law}))
    # Two runs on ONE law combination, differing only in who picks the hand-off
    # altitude. The fixed run flies the schedule as written; the optimised run
    # appends the non-first activation altitudes to the PSO decision vector and
    # lets the swarm place them. The comparison is the point: whether the
    # optimiser agrees with the atmospheric/exoatmospheric division of
    # Chapter 4, and what the hand-off altitude is worth if it does not.
    cases.append(dict(name="show_seg_fixed_alt", section="6.7", factor="segmented",
                      overrides={"MULTI_GUIDANCE_ENABLED": True,
                                 "MULTI_GUIDANCE_OPTIMIZE_ALTITUDES": False,
                                 "GUIDANCE_SEGMENTS": [("gravity_turn", 0.0),
                                                       ("peg_new", 120e3)]}))
    cases.append(dict(name="show_seg_opt_alt", section="6.7", factor="segmented",
                      overrides={"MULTI_GUIDANCE_ENABLED": True,
                                 "MULTI_GUIDANCE_OPTIMIZE_ALTITUDES": True,
                                 "GUIDANCE_SEGMENTS": [("gravity_turn", 0.0),
                                                       ("peg_new", 120e3)]}))
    return cases


# =========================================================================
#  Worker — one case, in this process
# =========================================================================

def _apply(sim_params, overrides):
    """Set overrides on the config module, refusing to invent new settings.

    A typo in an override name would otherwise be a silent no-op that quietly
    ran the baseline instead of the case, which is the one failure this harness
    must not have.
    """
    for key, value in overrides.items():
        if not hasattr(sim_params, key):
            raise AttributeError(
                "%r is not a setting in simulation_parameters.py — refusing to "
                "create it, since an override that names nothing would silently "
                "run the baseline instead of this case." % key)
        setattr(sim_params, key, value)


def _architecture(sim_params):
    """The dispatch level actually in force, in main.py's precedence order."""
    return run_record.architecture(sim_params)


def _dispatch(sim_params):
    """Run one trajectory.

    Returns ``(time, data, thrust, alpha, result, J, history, extra)``.

    ``result`` is the solver's result dict where there is one; the apogee_check
    path has no such dict, so one is synthesised from the trajectory it returns
    to keep the collection code below uniform. ``extra`` carries whatever a
    particular architecture knows and the others do not; it is empty for most.
    """
    arch = _architecture(sim_params)

    # Solvers are imported here, AFTER the overrides are applied, so that any
    # module-level constant derived from the configuration sees the case value.
    if arch == "segmented":
        import Simulation.segmented_guidance_solver as seg
        out = seg.run_segmented(verbose=True)
        # The schedule is the identity of a segmented run and exists nowhere
        # else once this process exits: GUIDANCE_MODE is ignored entirely under
        # MULTI_GUIDANCE_ENABLED, so the config cannot be read back to recover
        # which laws actually flew. Flattened to plain types here because the
        # _Segments object is neither JSON- nor npz-serialisable.
        _sched = list(out['segs'].schedule)
        _alts = out.get('optimized_altitudes')
        extra = {
            'segment_laws': [str(m) for m, _a in _sched],
            'segment_altitudes': [float(a) for _m, a in _sched],
            'optimized_altitudes': ([float(a) for a in _alts] if _alts else []),
        }
        return (out['time'], out['data'], out['thrust'], out['alpha'],
                out['result'], out['best_f'], seg.LAST_PSO_MG_HISTORY, extra)

    if arch == "indirect_pmp":
        from Simulation.indirect_pso_solver import run_pso_optimization, run_indirect_full
        import Simulation.indirect_pso_solver as ips
        params, J = run_pso_optimization(verbose=True)
        time_a, data, thrust, alpha, _, result = run_indirect_full(params, verbose=True)
        return time_a, data, thrust, alpha, result, J, ips.LAST_PSO_HISTORY, {}

    if arch == "pso_coast":
        from Simulation.pso_coast_solver import (run_pso_coast_optimization,
                                                 run_pso_coast_full)
        import Simulation.pso_coast_solver as pcs
        params, J = run_pso_coast_optimization(verbose=True)
        time_a, data, thrust, alpha, _, result, _, _ = run_pso_coast_full(params, verbose=True)
        return time_a, data, thrust, alpha, result, J, pcs.LAST_PSO_COAST_HISTORY, {}

    if arch == "direct":
        from Simulation.direct_pso_solver import (run_pso_direct_optimization,
                                                  run_pso_direct_full)
        import Simulation.direct_pso_solver as dps
        params, J = run_pso_direct_optimization(verbose=True)
        time_a, data, thrust, alpha, _, result, _, _ = run_pso_direct_full(params, verbose=True)
        return time_a, data, thrust, alpha, result, J, dps.LAST_PSO_DIRECT_HISTORY, {}

    if arch == "apogee_check":
        from Simulation import solver
        from Simulation import rocket_ascent as ra
        kick = solver.find_initial_kick_angle_coast_single_burn()
        (time_a, data, _alt_stopped, delta_v, _m_prop, thrust, time_thrust,
         alpha, alpha_time, _cor, _cen) = ra.run(kick)
        # No solver result dict on this path; rebuild the same shape from the
        # final state so the collector does not need a special case.
        result = {'crashed': False, 'state_final': np.asarray(data)[:, -1],
                  'circularisation_dv': float(delta_v)}
        # Latitude is derived, not integrated, and ra.run() does not append it --
        # the PSO solvers do it themselves and main.py does it for this branch.
        # Without it the archived case carries five state rows where every other
        # case carries six, and gt_apogee would be the one case in Chapter 6
        # with no latitude channel to plot.
        data = ra.append_latitude_row(data)
        thrust = np.interp(time_a, time_thrust, thrust)
        alpha = np.interp(time_a, alpha_time, alpha)
        return time_a, data, thrust, alpha, result, None, None, {}

    raise ValueError("unrecognised architecture: %r" % arch)


# ---------------------------------------------------------------------------
# Collection
# ---------------------------------------------------------------------------
# The row, the channels and the manifest are built by Archive/run_record.py,
# which is where they now live so that main.py can write the same archive for an
# interactive run. This file used to own them, and owned them in a form only a
# matrix case could call: the row builder read case['section'] and
# case['factor'] straight off the case dict. Those two are the `tags` argument
# now, and nothing else changed -- a second copy of this logic that could drift
# from the harness would defeat the point of sharing it.

# Nothing is re-exported here on purpose: a wrapper kept "for compatibility" is
# the second copy this move was meant to remove. Call Archive.run_record
# directly.


def run_case(name, smoke=False, budget=None):
    """Run one case in this process and write its row and trajectory."""
    cases = {c['name']: c for c in build_matrix()}
    if name not in cases:
        raise SystemExit("unknown case %r — known: %s" % (name, ", ".join(sorted(cases))))
    case = cases[name]

    from Input_File import simulation_parameters as sim_params
    _apply(sim_params, BASELINE)
    _apply(sim_params, case['overrides'])
    if smoke:
        _apply(sim_params, SMOKE_BUDGET)
    elif budget:
        _apply(sim_params, budget_overrides(*budget))

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    started = _time.time()
    time_a, data, thrust, alpha, result, J, history, extra = _dispatch(sim_params)
    wall_clock = _time.time() - started

    # One writer for the whole simulator. The case name is passed explicitly, so
    # re-running a case replaces it -- the matrix case names ARE the identity of
    # the experiment, and a timestamped id per attempt would leave make_all.py
    # unable to find "gt_baseline". An interactive run gets the timestamped id
    # instead and never overwrites anything.
    # Each case gets its own folder. Twenty cases as sixty-one files in one
    # directory is unreadable, and a folder per case also means a single case
    # can be copied, compared or thrown away on its own.
    saved = store.save_run(
        sim_params, time_a, data, thrust, alpha, result,
        J=J, history=history, extra=extra, wall_clock=wall_clock,
        name=name, root=OUTPUT_DIR / name, source="matrix:" + name,
        label="%s / %s" % (case['section'], case['factor']),
        tags={'section': case['section'], 'factor': case['factor']},
        verbose=False)
    print("\n[harness] %s done in %.1f s" % (name, wall_clock))
    return saved['row']


# =========================================================================
#  Driver — every case, one subprocess each
# =========================================================================

def _write_csv(rows, path):
    """Write the rows with the union of their keys, stable column order."""
    preferred = ['case', 'section', 'factor', 'architecture', 'guidance_mode',
                 'include_drag', 'earth_rotation', 'thrust_1_mode',
                 'crashed', 'J_prime',
                 'insertion_alt_km', 'insertion_v_ms', 'insertion_fpa_deg',
                 'eccentricity', 'periapsis_km', 'apoapsis_km',
                 'prop_remaining_kg', 'dv_ideal', 'dv_gravity', 'dv_drag',
                 'dv_steering', 'dv_pressure', 'dv_losses', 'dv_gain',
                 'dv_achieved', 'residual', 't_meco', 't_seco',
                 'n_evaluations', 'wall_clock_s']
    seen = set(preferred)
    columns = preferred + sorted(k for row in rows for k in row if k not in seen)

    import csv
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _rows_for_csv(rows_run, root):
    """Every case archived under `root`, in matrix order.

    The CSV must describe the DIRECTORY, not the invocation that happened to
    write it last. A batch is naturally run in stages -- Sections 6.2 and 6.3
    one night, 6.4 and 6.7 the next -- and this used to be written from the
    current run's rows alone, so the second invocation silently truncated the
    file to its own cases and discarded the ones already on disk.

    Rows from this run win over what is on disk, so re-running a case replaces
    its row rather than duplicating it.
    """
    fresh = {row.get('case'): row for row in rows_run if row.get('case')}
    merged = []
    for case in build_matrix():
        name = case['name']
        if name in fresh:
            merged.append(fresh[name])
            continue
        path = root / name / (name + ".json")
        if path.exists():
            with open(path, encoding="utf-8") as fh:
                merged.append(json.load(fh))
    return merged


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--case", help="run a single case in this process")
    parser.add_argument("--only",
                        help="comma-separated substring filter over case names "
                             "(e.g. gt_,peg_ for sections 6.2 and 6.3)")
    parser.add_argument("--smoke", action="store_true",
                        help="token PSO budget — proves dispatch, means nothing numerically")
    parser.add_argument("--budget", metavar="P,G",
                        help="reduced PSO budget, PARTICLES,GENERATIONS (e.g. 50,100). "
                             "Applies to every swarm architecture but NOT to the PMP "
                             "reference cache. Trajectories are the right shape; the "
                             "numbers are not reportable.")
    parser.add_argument("--out", metavar="DIR",
                        help="write into this root instead of Output/results_matrix "
                             "(relative paths resolve against Tese/src)")
    args = parser.parse_args()

    if args.smoke and args.budget:
        raise SystemExit("--smoke and --budget both set the PSO budget; pick one.")
    budget = _parse_budget(args.budget) if args.budget else None
    if args.out:
        _set_output_dir(args.out)

    if args.case:
        run_case(args.case, smoke=args.smoke, budget=budget)
        return

    cases = build_matrix()
    if args.only:
        wanted = [s for s in args.only.split(",") if s]
        cases = [c for c in cases if any(s in c['name'] for s in wanted)]
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if args.smoke:
        tag = "  [SMOKE]"
    elif budget:
        tag = "  [budget %d x %d]" % budget
    else:
        tag = ""
    print("=" * 70)
    print("RESULTS MATRIX — %d case(s)%s" % (len(cases), tag))
    print("into %s" % OUTPUT_DIR)
    print("=" * 70)

    rows, failures = [], []
    for i, case in enumerate(cases, 1):
        name = case['name']
        print("\n[%d/%d] %s  (§%s)" % (i, len(cases), name, case['section']))
        cmd = [sys.executable, str(Path(__file__).resolve()), "--case", name]
        if args.smoke:
            cmd.append("--smoke")
        if args.budget:
            cmd += ["--budget", args.budget]
        # The child re-imports this module, so OUTPUT_DIR is back at its default
        # unless --out is forwarded. Passing the resolved path rather than the
        # user's string keeps parent and child writing to the same place however
        # the parent was invoked.
        if args.out:
            cmd += ["--out", str(OUTPUT_DIR)]
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        proc = subprocess.run(cmd, env=env)

        row_path = OUTPUT_DIR / name / (name + ".json")
        if proc.returncode != 0 or not row_path.exists():
            print("  FAILED (exit %d)" % proc.returncode)
            failures.append(name)
            rows.append({'case': name, 'section': case['section'],
                         'factor': case['factor'], 'crashed': True,
                         'harness_error': "exit %d" % proc.returncode})
            continue
        with open(row_path, encoding="utf-8") as fh:
            rows.append(json.load(fh))

    csv_path = OUTPUT_DIR / "results_matrix.csv"
    all_rows = _rows_for_csv(rows, OUTPUT_DIR)
    _write_csv(all_rows, csv_path)
    print("\n" + "=" * 70)
    if len(all_rows) > len(rows):
        print("wrote %s  (%d rows: %d from this run, %d already archived)"
              % (csv_path, len(all_rows), len(rows), len(all_rows) - len(rows)))
    else:
        print("wrote %s  (%d rows)" % (csv_path, len(all_rows)))
    if failures:
        print("FAILED: %s" % ", ".join(failures))
    print("=" * 70)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main() or 0)
