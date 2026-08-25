"""What one flown trajectory is reduced to: a scalar row, a channel set, a manifest.

This is the single implementation of collection, shared by ``main.py`` and
``run_results_matrix.py``. It used to live inside the harness, where the row
builder read ``case['section']`` and ``case['factor']`` directly and so could not
be called by anything that was not a matrix case; those two are now the optional
``tags`` argument and everything else is unchanged.

Three products, in increasing order of how much they know:

``channels``   the arrays a figure needs and the row cannot hold.
``collect_row`` the scalars the results table is built from.
``manifest``   what produced them -- every configuration value, the vehicle, the
               git commit, the wall clock. A trajectory with no record of its
               configuration cannot be compared against another six months later,
               which is the whole point of keeping it.
"""

import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

# Bumped when the meaning of a stored key changes, not when a key is added --
# the loaders all treat an absent key as absent rather than as an error, so
# additions are backward compatible by construction.
SCHEMA = 1

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent


# =========================================================================
#  Identity
# =========================================================================

def architecture(sim_params):
    """The dispatch level actually in force, in main.py's precedence order."""
    if sim_params.MULTI_GUIDANCE_ENABLED:
        return "segmented"
    if sim_params.GUIDANCE_MODE == "indirect_pmp":
        return "indirect_pmp"
    return sim_params.COAST_METHOD


def guidance_label(sim_params, extra):
    """What actually flew, which is not always GUIDANCE_MODE.

    Under MULTI_GUIDANCE_ENABLED the dispatcher ignores GUIDANCE_MODE outright,
    so recording it would label the segmented cases with whatever happens to be
    left in simulation_parameters.py -- indirect_pmp, as it stands -- in exactly
    the table Section 6.7 is built from.
    """
    if not sim_params.MULTI_GUIDANCE_ENABLED:
        return sim_params.GUIDANCE_MODE
    laws = (extra or {}).get('segment_laws')
    return " -> ".join(laws) if laws else "segmented"


# PSO budget attribute names per architecture, so the cost table of Section 6.7
# reports the evaluations spent and not only the wall clock. apogee_check runs
# no swarm and therefore has none.
_PSO_BUDGET_ATTRS = {
    'indirect_pmp': ('PSO_N_PARTICLES', 'PSO_MAX_GENERATIONS'),
    'pso_coast': ('PSO_COAST_N_PARTICLES', 'PSO_COAST_MAX_GENERATIONS'),
    'direct': ('PSO_DIRECT_N_PARTICLES', 'PSO_DIRECT_MAX_GENERATIONS'),
    'segmented': ('PSO_MG_N_PARTICLES', 'PSO_MG_MAX_GENERATIONS'),
}


def n_evaluations(sim_params):
    """The swarm budget this architecture was given, as function evaluations."""
    attrs = _PSO_BUDGET_ATTRS.get(architecture(sim_params))
    if attrs is None:
        return None
    return int(getattr(sim_params, attrs[0]) * getattr(sim_params, attrs[1]))


def as_float(value):
    """None-preserving float, for event times that may never have been set."""
    return None if value is None else float(value)


def nan(value):
    """npz has no None, so an event that never happened is stored as NaN."""
    return float('nan') if value is None else float(value)


# =========================================================================
#  Channels -- the arrays a figure needs and the row cannot hold
# =========================================================================

# The plot-suite channels, as ``main.py`` names them when it calls _emit_plots,
# mapped to the npz keys they are stored under. These are kept on whatever grid
# the architecture produced them on -- ODE-call cadence on the legacy path, the
# dense state grid on the PSO paths -- because that is what makes the twenty-plot
# suite replayable from an archive rather than only redrawable in approximation.
SUITE_KEYS = {
    'thrust_data': 'suite_thrust',
    'time_thrust': 'suite_time_thrust',
    'alpha_data': 'suite_alpha',
    'alpha_time_data': 'suite_alpha_time',
    'coriolis_mag_data': 'suite_coriolis',
    'centrifugal_mag_data': 'suite_centrifugal',
    'tgo_data': 'suite_tgo',
    'tgo_time_data': 'suite_tgo_time',
    'theta_data': 'suite_theta',
    'theta_time_data': 'suite_theta_time',
    'cross_heading_counter_force_data': 'suite_cross_force',
    'cross_heading_accel_data': 'suite_cross_accel',
}

# The reverse map, for replay: npz key -> the keyword run_new_plot_suite expects.
REPLAY_KEYS = {v: k for k, v in SUITE_KEYS.items()}

# Which time axis the suite plots each channel against. FIVE channels share
# ``time_thrust``, which is the trap: a channel and its axis can arrive from
# different grids and nothing notices until a replay months later dies on
# "x and y must have same first dimension". The archive reconciles them at write
# time instead, because that is the only moment both grids are still in hand.
SUITE_AXIS = {
    'thrust_data': 'time_thrust',
    'coriolis_mag_data': 'time_thrust',
    'centrifugal_mag_data': 'time_thrust',
    'cross_heading_counter_force_data': 'time_thrust',
    'cross_heading_accel_data': 'time_thrust',
    'alpha_data': 'alpha_time_data',
    'theta_data': 'theta_time_data',
    'tgo_data': 'tgo_time_data',
}


def _list_or_none(seq):
    """A history global as an array, or None when the run never appended to it."""
    if seq is None:
        return None
    arr = np.asarray(seq)
    return arr if arr.size else None


def _default_suite(sim_params, time_a, thrust, alpha, coriolis=None,
                   centrifugal=None):
    """The replay set as the architecture left it in the rocket_ascent globals.

    Every solver writes the full-flight theta / t_go / cross-heading channels
    back into those globals for main.py's plot block, so reading them here gives
    one collection path for all five architectures instead of five. A caller
    that has already massaged a channel onto the output grid passes it in and
    overrides what is found here.

    The results matrix passes nothing, so this is also what makes a batch case
    replayable: without a default the pseudo-force panel of the twenty-plot
    suite would simply be missing from every archive the harness writes.
    """
    from Simulation import rocket_ascent as ra

    suite = {
        'thrust_data': None if thrust is None else np.asarray(thrust, dtype=float),
        'time_thrust': None if thrust is None else np.asarray(time_a, dtype=float),
        'alpha_data': None if alpha is None else np.asarray(alpha, dtype=float),
        'alpha_time_data': None if alpha is None else np.asarray(time_a, dtype=float),
        'coriolis_mag_data': coriolis,
        'centrifugal_mag_data': centrifugal,
        'theta_data': _list_or_none(ra.theta_history),
        'theta_time_data': _list_or_none(ra.theta_time_history),
        'tgo_data': _list_or_none(ra.tgo_history),
        'tgo_time_data': _list_or_none(ra.tgo_time_history),
    }
    if sim_params.COMPUTE_CROSS_HEADING_COUNTER_FORCE:
        suite['cross_heading_counter_force_data'] = _list_or_none(
            ra.cross_heading_counter_force_history)
        suite['cross_heading_accel_data'] = _list_or_none(
            ra.cross_heading_accel_history)
    return suite


def _reconcile_suite(merged, time_a, grid_fallbacks):
    """Make every replay channel agree in length with the axis it is plotted on.

    The legacy suite plots thrust, both pseudo-force magnitudes and both
    cross-heading channels against one shared ``time_thrust``. Nothing upstream
    guarantees they came from the same grid: the results-matrix worker hands
    back thrust already interpolated onto the state grid while the cross-heading
    history is still at ODE-call cadence, so an apogee_check archive used to
    store a 2528-sample channel against a 40686-sample axis. It wrote and loaded
    perfectly and only failed at replay, which is the worst possible time to
    find out -- the run is long gone by then.

    Write time is the one moment both grids are in hand, so the fix belongs
    here. A mismatched channel is replaced by its state-grid equivalent where
    one exists, and dropped with a note where it does not. Dropping loses a
    panel; keeping would lose the whole replay.
    """
    for channel, axis_name in SUITE_AXIS.items():
        values = merged.get(channel)
        axis = merged.get(axis_name)
        if values is None or axis is None:
            continue
        if len(np.asarray(values)) == len(np.asarray(axis)):
            continue
        fallback = grid_fallbacks.get(channel)
        if (fallback is not None
                and len(np.asarray(fallback)) == len(np.asarray(axis))):
            merged[channel] = fallback
        else:
            merged[channel] = None
            print("  [archive] %s (%d samples) does not fit %s (%d); not stored"
                  % (channel, len(np.asarray(values)), axis_name,
                     len(np.asarray(axis))))


def channels(sim_params, time_a, data, extra, history,
             thrust=None, alpha=None, suite=None):
    """The per-run arrays the chapter figures need and the scalar row cannot hold.

    Everything here is either a solver global that dies with this subprocess or
    a quantity recomputable only under an exactly reconstructed configuration.
    A compressed archive runs from under 1 MB to about 15 MB, the spread being
    the trajectory length rather than the channel count -- an apogee_check run
    sampled at TIME_STEP over a 3000 s propagation is 300k samples on its own.
    Against a run measured in tens of minutes that is not a trade worth
    offering: not saving it costs the run again.
    """
    from Simulation import rocket_ascent as ra

    data = np.asarray(data)
    time_a = np.asarray(time_a, dtype=float)

    # Pseudo-force diagnostics, recomputed on the output grid rather than
    # spliced from ODE-call history, which is on no particular grid. The helper
    # gates on the same predicate the EOM uses, so an architecture that flew
    # without these terms -- indirect_pmp -- gets zeros rather than values that
    # were never applied. PROPAGATING_IN_INERTIAL_FRAME is left set by the final
    # ballistic coast and would zero the whole grid, so it is cleared for the
    # recomputation exactly as pso_coast_solver does.
    saved_frame_flag = ra.PROPAGATING_IN_INERTIAL_FRAME
    ra.PROPAGATING_IN_INERTIAL_FRAME = False
    try:
        cross_force, cross_accel, coriolis, centrifugal = \
            ra.pseudo_force_channels_on_grid(time_a, data[:5])
    finally:
        ra.PROPAGATING_IN_INERTIAL_FRAME = saved_frame_flag

    out = {
        'coriolis': coriolis,
        'centrifugal': centrifugal,
        'cross_heading_accel': cross_accel,
        # Arc boundaries, for the stage and coast markers on every time-axis
        # figure in the chapter.
        't_meco': np.array(nan(ra.time_main_engine_cutoff)),
        't_seco': np.array(nan(ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL)),
        't_coast_start': np.array(nan(ra.PSO_COAST_ARC2_START_TIME)),
        't_guidance_start': np.array(nan(ra.time_guidance_start)),
        # Two more the harness never captured and main.py prints in its event
        # timeline, so an archived run can reproduce that timeline as well.
        't_kick_start': np.array(nan(ra.time_kick_start)),
        't_atmosphere_exit': np.array(nan(ra.time_atmosphere_exit)),
    }

    # The convergence history, which the row reduces to three scalars. The full
    # curve is what shows a solve converged rather than merely stopped, and it
    # is discarded the moment this interpreter exits.
    if isinstance(history, dict) and len(history.get('gbest', [])):
        out['pso_gen'] = np.asarray(history['gen'], dtype=float)
        out['pso_gbest'] = np.asarray(history['gbest'], dtype=float)

    # The replay set, on its native grids -- minus whatever is bit-identical to
    # an array already being stored. On the PSO architectures the solvers hand
    # back thrust, alpha and the pseudo-force channels already on the state
    # grid, so six of the twelve would otherwise be exact duplicates and a
    # gravity-turn archive would be twice the size for nothing. What is dropped
    # is recorded in suite_aliases, so replay substitutes the original rather
    # than guessing, and an archive stays self-describing.
    merged = _default_suite(sim_params, time_a, thrust, alpha,
                            coriolis=coriolis, centrifugal=centrifugal)
    merged.update({k: v for k, v in (suite or {}).items() if v is not None})
    _reconcile_suite(merged, time_a, grid_fallbacks={
        'cross_heading_counter_force_data': cross_force,
        'cross_heading_accel_data': cross_accel,
        'coriolis_mag_data': coriolis,
        'centrifugal_mag_data': centrifugal,
    })
    duplicate_of = {
        'suite_thrust': ('thrust', None if thrust is None else np.asarray(thrust)),
        'suite_time_thrust': ('time', time_a),
        'suite_alpha': ('alpha', None if alpha is None else np.asarray(alpha)),
        'suite_alpha_time': ('time', time_a),
        'suite_coriolis': ('coriolis', coriolis),
        'suite_centrifugal': ('centrifugal', centrifugal),
        'suite_cross_accel': ('cross_heading_accel', cross_accel),
        'suite_theta_time': ('time', time_a),
        'suite_tgo_time': ('time', time_a),
    }
    aliases = []
    for kwarg, key in SUITE_KEYS.items():
        value = merged.get(kwarg)
        if value is None:
            continue
        value = np.asarray(value)
        target, reference = duplicate_of.get(key, (None, None))
        if reference is not None and np.array_equal(value, reference):
            aliases.append("%s=%s" % (key, target))
            continue
        out[key] = value
    if aliases:
        out['suite_aliases'] = np.array(sorted(aliases))
    freeze = (suite or {}).get('apollo_freeze_threshold')
    if freeze is None and sim_params.GUIDANCE_MODE == "apollo":
        freeze = getattr(sim_params, "APOLLO_FREEZE_THRESHOLD", None)
    if freeze is not None:
        out['apollo_freeze_threshold'] = np.array(float(freeze))

    out.update({k: np.asarray(v) for k, v in (extra or {}).items()})
    return out


# =========================================================================
#  The scalar row
# =========================================================================

def collect_row(name, sim_params, time_a, data, thrust, alpha, result, J,
                history, wall_clock, extra, tags=None):
    """Reduce one trajectory to the scalars the results table needs.

    ``tags`` is whatever the caller wants recorded alongside the run and cannot
    be derived from the configuration -- the results matrix passes its section
    and factor there. It used to be read straight off the matrix case, which is
    what stopped anything but the harness from calling this.
    """
    from Auxiliary import constants as c
    from Auxiliary import losses as loss_mod
    from Auxiliary import rocket_specs as r_specs
    from Simulation import rocket_ascent as ra

    row = {'case': name}
    row.update(tags or {})
    row.update({
        'architecture': architecture(sim_params),
        'guidance_mode': guidance_label(sim_params, extra),
        'include_drag': bool(sim_params.INCLUDE_DRAG),
        'earth_rotation': bool(sim_params.ENABLE_EARTH_ROTATION),
        # Both halves of the nozzle model, recorded because the pressure loss is
        # only defined under "pressure" and the figures must be able to say so
        # rather than plotting a zero that looks like a measurement.
        'kick_profile_mode': str(sim_params.KICK_PROFILE_MODE),
        'isp_1_mode': str(sim_params.ISP_1_MODE),
        'thrust_1_mode': str(sim_params.THRUST_1_MODE),
        # The mission the case was aiming at, so a figure can draw the target
        # without importing the configuration the run was flown under.
        'target_alt_km': float(sim_params.TARGET_ORBITAL_ALTITUDE) / 1e3,
        'pseudo_forces_requested': bool(sim_params.INCLUDE_PSEUDO_FORCES),
        # What the trajectory actually flew, which is not the same thing:
        # indirect_pmp is exempt and flies pseudo-force-free whatever the config
        # says, and a case with the rotation switched off carries no pseudo-
        # forces however willing its architecture was. The budget residual is
        # only interpretable against this column.
        #
        # This must mirror ra._pseudo_forces_active(), which is the gate the EOM
        # actually consults -- MINUS its PROPAGATING_IN_INERTIAL_FRAME term.
        # That term is a transient phase state (true only on the final ballistic
        # coast, once the state has been converted to the inertial frame), so
        # reading it here would report whichever phase happened to be current
        # when the run ended rather than a property of the run. Reading only
        # _PSEUDO_FORCES_THIS_RUN, as this line used to, reported the
        # architecture's willingness and labelled gt_norot -- the one case whose
        # entire purpose is having the rotation off -- as having flown them.
        'pseudo_forces_flown': bool(sim_params.ENABLE_EARTH_ROTATION
                                    and sim_params.INCLUDE_PSEUDO_FORCES
                                    and ra._PSEUDO_FORCES_THIS_RUN),
        # apogee_check inserts with an impulsive circularisation burn that is not
        # part of the integrated trajectory, so its dv_achieved excludes it and
        # its residual cannot be compared with the direct-insertion paths.
        'circularisation_dv': float(result.get('circularisation_dv', 0.0)),
        'wall_clock_s': round(wall_clock, 1),
        'n_evaluations': n_evaluations(sim_params),
        't_meco': as_float(ra.time_main_engine_cutoff),
        't_seco': as_float(ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL),
        'J_prime': None if J is None else float(J),
        'crashed': bool(result.get('crashed', False)),
    })

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


# =========================================================================
#  The manifest -- what produced the trajectory
# =========================================================================

def _jsonable(value):
    """Convert a configuration value to JSON, or report that it will not go.

    Returns ``(ok, converted)``. Anything that is not a plain scalar, a
    container of plain scalars, or a numpy equivalent is refused rather than
    stringified: a manifest entry that reads ``"<function foo at 0x...>"`` is
    worse than an absent one, because a diff would show two identical runs
    disagreeing on the address.
    """
    if value is None or isinstance(value, (bool, int, float, str)):
        return True, value
    if isinstance(value, np.generic):
        return True, value.item()
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, (list, tuple)):
        out = []
        for item in value:
            ok, converted = _jsonable(item)
            if not ok:
                return False, None
            out.append(converted)
        return True, out
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            if not isinstance(key, str):
                return False, None
            ok, converted = _jsonable(item)
            if not ok:
                return False, None
            out[key] = converted
        return True, out
    return False, None


def module_snapshot(module):
    """Every UPPERCASE module-level value, as JSON.

    Taken by iterating the module rather than from a hand-written list, so a
    setting added to simulation_parameters.py next month is archived without
    anyone remembering to add it here. The names are the interface; anything
    lowercase is an import or a helper.
    """
    snapshot = {}
    for name, value in vars(module).items():
        if not name or not name[0].isupper():
            continue
        if not all(ch.isupper() or ch.isdigit() or ch == "_" for ch in name):
            continue
        ok, converted = _jsonable(value)
        if ok:
            snapshot[name] = converted
    return dict(sorted(snapshot.items()))


def git_info():
    """The commit the run was flown at, and whether the tree was dirty.

    A dirty tree is recorded rather than refused: the whole point of archiving
    is that runs happen while the code is being worked on. Any git failure --
    no git on PATH, not a repository -- leaves the fields None instead of
    stopping the archive.
    """
    def _git(*args):
        try:
            proc = subprocess.run(["git"] + list(args), cwd=str(_PROJECT_ROOT),
                                  capture_output=True, text=True, timeout=10)
        except Exception:                          # noqa: BLE001 — best effort
            return None
        return proc.stdout.strip() if proc.returncode == 0 else None

    status = _git("status", "--porcelain")
    return {
        'commit': _git("rev-parse", "HEAD"),
        'branch': _git("rev-parse", "--abbrev-ref", "HEAD"),
        'describe': _git("describe", "--always", "--dirty"),
        'dirty': None if status is None else bool(status),
    }


def env_info():
    """Interpreter and library versions, because results move with them."""
    versions = {'python': sys.version.split()[0], 'platform': platform.platform()}
    for name in ("numpy", "scipy", "matplotlib", "pygmo"):
        try:
            versions[name] = __import__(name).__version__
        except Exception:                          # noqa: BLE001 — absent is a fact
            versions[name] = None
    return versions


def manifest(sim_params, run_id, wall_clock=None, source=None, label=None,
             extra=None):
    """Everything needed to know what this trajectory is, without the process.

    The configuration snapshot is the reason an archive is comparable at all: a
    trajectory with no record of the settings it was flown under can be plotted
    but not interpreted, and two of them cannot be told apart except by looking
    at the curves and guessing.
    """
    from Auxiliary import constants as c
    from Auxiliary import rocket_specs as r_specs

    now = datetime.now()
    return {
        'schema': SCHEMA,
        'run_id': run_id,
        'created_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
        'created_local': now.isoformat(timespec='seconds'),
        'wall_clock_s': None if wall_clock is None else round(float(wall_clock), 1),
        'source': source,
        'label': label or "",
        'architecture': architecture(sim_params),
        'guidance_mode': guidance_label(sim_params, extra),
        'n_evaluations': n_evaluations(sim_params),
        'git': git_info(),
        'env': env_info(),
        'config': module_snapshot(sim_params),
        'vehicle': module_snapshot(r_specs),
        'constants': module_snapshot(c),
    }
