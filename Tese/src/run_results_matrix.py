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

_HERE = Path(__file__).resolve().parent
OUTPUT_DIR = _HERE / "Output" / "results_matrix"

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
SMOKE_BUDGET = {
    "PSO_N_PARTICLES": 8, "PSO_MAX_GENERATIONS": 4,
    "PSO_COAST_N_PARTICLES": 8, "PSO_COAST_MAX_GENERATIONS": 4,
    "PSO_DIRECT_N_PARTICLES": 8, "PSO_DIRECT_MAX_GENERATIONS": 4,
    "PSO_MG_N_PARTICLES": 8, "PSO_MG_MAX_GENERATIONS": 4,
    "PMP_REFERENCE_PSO_PARTICLES": 8, "PMP_REFERENCE_PSO_GENERATIONS": 4,
}


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
    if sim_params.MULTI_GUIDANCE_ENABLED:
        return "segmented"
    if sim_params.GUIDANCE_MODE == "indirect_pmp":
        return "indirect_pmp"
    return sim_params.COAST_METHOD


def _dispatch(sim_params):
    """Run one trajectory. Returns (time, data, thrust, alpha, result, J, history).

    ``result`` is the solver's result dict where there is one; the apogee_check
    path has no such dict, so one is synthesised from the trajectory it returns
    to keep the collection code below uniform.
    """
    arch = _architecture(sim_params)

    # Solvers are imported here, AFTER the overrides are applied, so that any
    # module-level constant derived from the configuration sees the case value.
    if arch == "segmented":
        import Simulation.segmented_guidance_solver as seg
        out = seg.run_segmented(verbose=True)
        return (out['time'], out['data'], out['thrust'], out['alpha'],
                out['result'], out['best_f'], None)

    if arch == "indirect_pmp":
        from Simulation.indirect_pso_solver import run_pso_optimization, run_indirect_full
        import Simulation.indirect_pso_solver as ips
        params, J = run_pso_optimization(verbose=True)
        time_a, data, thrust, alpha, _, result = run_indirect_full(params, verbose=True)
        return time_a, data, thrust, alpha, result, J, ips.LAST_PSO_HISTORY

    if arch == "pso_coast":
        from Simulation.pso_coast_solver import (run_pso_coast_optimization,
                                                 run_pso_coast_full)
        import Simulation.pso_coast_solver as pcs
        params, J = run_pso_coast_optimization(verbose=True)
        time_a, data, thrust, alpha, _, result, _, _ = run_pso_coast_full(params, verbose=True)
        return time_a, data, thrust, alpha, result, J, pcs.LAST_PSO_COAST_HISTORY

    if arch == "direct":
        from Simulation.direct_pso_solver import (run_pso_direct_optimization,
                                                  run_pso_direct_full)
        import Simulation.direct_pso_solver as dps
        params, J = run_pso_direct_optimization(verbose=True)
        time_a, data, thrust, alpha, _, result, _, _ = run_pso_direct_full(params, verbose=True)
        return time_a, data, thrust, alpha, result, J, dps.LAST_PSO_DIRECT_HISTORY

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
        thrust = np.interp(time_a, time_thrust, thrust)
        alpha = np.interp(time_a, alpha_time, alpha)
        return time_a, data, thrust, alpha, result, None, None

    raise ValueError("unrecognised architecture: %r" % arch)


def _collect(name, case, sim_params, time_a, data, thrust, alpha, result, J,
             history, wall_clock):
    """Reduce one trajectory to the scalars the results table needs."""
    from Auxiliary import constants as c
    from Auxiliary import losses as loss_mod
    from Auxiliary import rocket_specs as r_specs
    from Simulation import rocket_ascent as ra

    row = {
        'case': name,
        'section': case['section'],
        'factor': case['factor'],
        'architecture': _architecture(sim_params),
        'guidance_mode': sim_params.GUIDANCE_MODE,
        'include_drag': bool(sim_params.INCLUDE_DRAG),
        'earth_rotation': bool(sim_params.ENABLE_EARTH_ROTATION),
        'pseudo_forces_requested': bool(sim_params.INCLUDE_PSEUDO_FORCES),
        # What the architecture actually flew, which is not the same thing:
        # indirect_pmp is exempt and flies pseudo-force-free whatever the config
        # says. The budget residual is only interpretable against this column.
        'pseudo_forces_flown': bool(ra._PSEUDO_FORCES_THIS_RUN),
        # apogee_check inserts with an impulsive circularisation burn that is not
        # part of the integrated trajectory, so its dv_achieved excludes it and
        # its residual cannot be compared with the direct-insertion paths.
        'circularisation_dv': float(result.get('circularisation_dv', 0.0)),
        'wall_clock_s': round(wall_clock, 1),
        'J_prime': None if J is None else float(J),
        'crashed': bool(result.get('crashed', False)),
    }

    if row['crashed'] or result.get('state_final') is None:
        return row

    sf = np.asarray(result['state_final'], dtype=float)
    data = np.asarray(data)
    time_a = np.asarray(time_a, dtype=float)

    row['insertion_alt_km'] = float((sf[1] - c.R_EARTH) / 1e3)
    row['insertion_v_ms'] = float(sf[2])
    row['insertion_fpa_deg'] = float(np.rad2deg(sf[3]))

    try:
        v_in, g_in = ra.get_inertial_state_components(
            sf[1], sf[2], sf[3], np.deg2rad(sim_params.LAUNCH_LATITUDE))
        a, e, r_apo, r_peri, period = ra.get_orbital_elements(sf[1], v_in, g_in)
        row.update({
            'sma_km': float(a / 1e3),
            'eccentricity': float(e),
            'apoapsis_km': float((r_apo - c.R_EARTH) / 1e3),
            'periapsis_km': float((r_peri - c.R_EARTH) / 1e3),
            'period_min': float(period / 60.0),
        })
    except Exception as exc:                      # noqa: BLE001 — recorded, not raised
        row['orbit_error'] = str(exc)

    m_final = float(data[4, -1])
    row['prop_remaining_kg'] = max(
        0.0, m_final - (r_specs.M_STRUCTURE_2 + r_specs.M_PAYLOAD))

    # --- delta-v budget, over the powered ascent only ---------------------
    t_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    idx = len(time_a) if t_seco is None else int(np.searchsorted(time_a, t_seco, 'right'))
    idx = max(idx, 2)
    alt = data[1, :idx] - c.R_EARTH
    budget = loss_mod.delta_v_budget(
        time_a[:idx], alt, data[2, :idx], data[3, :idx], data[4, :idx],
        np.asarray(thrust, dtype=float)[:idx], np.asarray(alpha, dtype=float)[:idx],
        t_meco=ra.time_main_engine_cutoff,
        include_drag=sim_params.INCLUDE_DRAG,
        thrust_mode=sim_params.THRUST_1_MODE,
    )
    row.update({k: (v if isinstance(v, bool) else round(float(v), 3))
                for k, v in budget.items()})

    # Convergence history, the evidence that the budget was adequate. The
    # solvers record it as {'gen': array, 'gbest': array}; apogee_check has none.
    if isinstance(history, dict) and len(history.get('gbest', [])):
        gbest = np.asarray(history['gbest'], dtype=float)
        gens = np.asarray(history['gen'], dtype=float)
        row['pso_generations'] = int(gens[-1])
        row['pso_gbest_first'] = float(gbest[0])
        row['pso_gbest_last'] = float(gbest[-1])
        # Fraction of the total improvement still being made over the final
        # quarter of the run: near zero means converged, large means the budget
        # ran out before the swarm did.
        tail = max(1, len(gbest) // 4)
        span = gbest[0] - gbest[-1]
        row['pso_tail_improvement_frac'] = (
            float((gbest[-tail] - gbest[-1]) / span) if span > 0 else 0.0)

    return row


def run_case(name, smoke=False):
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

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    started = _time.time()
    time_a, data, thrust, alpha, result, J, history = _dispatch(sim_params)
    wall_clock = _time.time() - started

    row = _collect(name, case, sim_params, time_a, data, thrust, alpha,
                   result, J, history, wall_clock)

    np.savez_compressed(OUTPUT_DIR / (name + ".npz"),
                        time=time_a, data=data, thrust=thrust, alpha=alpha)
    with open(OUTPUT_DIR / (name + ".json"), "w", encoding="utf-8") as fh:
        json.dump(row, fh, indent=2)
    print("\n[harness] %s done in %.1f s" % (name, wall_clock))
    return row


# =========================================================================
#  Driver — every case, one subprocess each
# =========================================================================

def _write_csv(rows, path):
    """Write the rows with the union of their keys, stable column order."""
    preferred = ['case', 'section', 'factor', 'architecture', 'guidance_mode',
                 'include_drag', 'earth_rotation', 'crashed', 'J_prime',
                 'insertion_alt_km', 'insertion_v_ms', 'insertion_fpa_deg',
                 'eccentricity', 'periapsis_km', 'apoapsis_km',
                 'prop_remaining_kg', 'dv_ideal', 'dv_gravity', 'dv_drag',
                 'dv_steering', 'dv_pressure', 'dv_losses', 'dv_gain',
                 'dv_achieved', 'residual', 'wall_clock_s']
    seen = set(preferred)
    columns = preferred + sorted(k for row in rows for k in row if k not in seen)

    import csv
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--case", help="run a single case in this process")
    parser.add_argument("--only", help="substring filter over case names")
    parser.add_argument("--smoke", action="store_true",
                        help="token PSO budget — proves dispatch, means nothing numerically")
    args = parser.parse_args()

    if args.case:
        run_case(args.case, smoke=args.smoke)
        return

    cases = build_matrix()
    if args.only:
        cases = [c for c in cases if args.only in c['name']]
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("RESULTS MATRIX — %d case(s)%s" % (len(cases), "  [SMOKE]" if args.smoke else ""))
    print("=" * 70)

    rows, failures = [], []
    for i, case in enumerate(cases, 1):
        name = case['name']
        print("\n[%d/%d] %s  (§%s)" % (i, len(cases), name, case['section']))
        cmd = [sys.executable, str(Path(__file__).resolve()), "--case", name]
        if args.smoke:
            cmd.append("--smoke")
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        proc = subprocess.run(cmd, env=env)

        row_path = OUTPUT_DIR / (name + ".json")
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
    _write_csv(rows, csv_path)
    print("\n" + "=" * 70)
    print("wrote %s  (%d rows)" % (csv_path, len(rows)))
    if failures:
        print("FAILED: %s" % ", ".join(failures))
    print("=" * 70)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main() or 0)
