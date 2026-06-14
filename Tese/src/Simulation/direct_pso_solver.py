"""
PSO Direct-Insertion Solver

2-variable PSO for COAST_METHOD == "direct".

Optimises jointly:
    x = [gamma_p,       pitch maneuver (kick) angle [rad], in [1.54, 1.57]
         t_burn_pct]    Stage-2 continuous burn duration as % of T_MAX_2 [%]

Trajectory structure — Stage 1 (instantaneous kick via ra.run_stage1) ->
pre-ignition ballistic coast -> ONE continuous Stage-2 thrust arc of duration
t_burn -> direct orbit insertion (no coast-to-apogee, no circularisation
burn). The selected guidance mode (simulation_parameters.GUIDANCE_MODE)
steers the single thrust arc.

Objective (4 terms, no transversality, no coast split — mirrors
pso_coast_solver):
    J = w_J * J_nd  +  w_alt * |Δh_nd|  +  w_vel * |ΔV_nd|  +  w_fpa * |Δγ_nd|
    + CRASH_PENALTY  (if trajectory crashed)
where J_nd = t_burn / T_MAX_2 (burn-time fraction) and Δh/ΔV/Δγ are the
altitude/velocity/FPA errors of the final state vs. the (rotating-frame)
circular-orbit target at TARGET_ORBITAL_ALTITUDE.

PyGMO is required (same as pso_coast_solver).
"""

import sys
import time
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent))

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r
from Input_File import simulation_parameters as sim_params
import Simulation.rocket_ascent as ra
from Simulation.pso_coast_solver import (
    _strip_to_pmp_state,
    _v_circular_rotating,
    GuidanceState,
    _stage2_ode_guidance,
    _event_crash,
    _T_MAX_2,
    _T_IGNITION_DELAY,
    _RTOL,
    _ATOL,
    _MAX_STEP,
    CRASH_PENALTY,
)

# Per-generation PSO convergence history (same format as LAST_PSO_COAST_HISTORY).
LAST_PSO_DIRECT_HISTORY = None


# ===========================================================================
# Full Stage-1 -> Stage-2 trajectory runner (PSO inner loop)
# ===========================================================================

def run_pso_direct_trajectory(gamma_p, t_burn_pct, verbose=False):
    """
    Simulate a Stage-1-kick + single-thrust-arc direct-insertion trajectory
    for one PSO particle.

    Parameters
    ----------
    gamma_p    : float  Pitch maneuver angle [rad]  (kick_angle = gamma_p - pi/2)
    t_burn_pct : float  Stage-2 continuous burn duration as % of T_MAX_2 [%]
    verbose    : bool

    Returns
    -------
    result : dict with keys
        crashed, state_final, t_burn, t_stage2_start, t_ignition,
        t_stage1, y_stage1
    """
    kick_angle = gamma_p - np.pi / 2.0
    t_burn = (t_burn_pct / 100.0) * _T_MAX_2

    # ---- Stage 1 ----
    t2_start, state2_init, _, t_stage1, y_stage1, crashed = ra.run_stage1(kick_angle)
    if crashed:
        return {
            'crashed': True, 'state_final': None,
            't_burn': t_burn, 't_stage2_start': 0.0, 't_ignition': 0.0,
            't_stage1': t_stage1, 'y_stage1': y_stage1,
        }

    # Strip to 5-element physical state (handles INCLUDE_PSEUDO_FORCES)
    state2_init = _strip_to_pmp_state(
        state2_init, np.deg2rad(sim_params.LAUNCH_LATITUDE))

    t_ignition = t2_start + _T_IGNITION_DELAY

    if verbose:
        h2 = state2_init[1] - c.R_EARTH
        print(f"  Stage 1 end: t={t2_start:.1f}s, h={h2/1e3:.1f}km, "
              f"v={state2_init[2]:.0f}m/s, gam={np.rad2deg(state2_init[3]):.2f}deg")

    # ---- Pre-ignition ballistic coast (stage sep -> ignition) ----
    sol_pre = solve_ivp(
        lambda t, y: _stage2_ode_guidance(t, y, 0.0, r.ISP_2, None),
        t_span=(t2_start, t_ignition),
        y0=state2_init[:5],
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
        events=_event_crash,
    )
    if len(sol_pre.t_events[0]) > 0:
        return {'crashed': True, 'state_final': None,
                't_burn': t_burn, 't_stage2_start': t2_start, 't_ignition': t_ignition,
                't_stage1': t_stage1, 'y_stage1': y_stage1}
    state_at_ign = sol_pre.y[:5, -1].copy()

    # ---- Single continuous thrust arc (t_ignition -> t_ignition + t_burn) ----
    gs = GuidanceState()
    t_burn_end = t_ignition + t_burn
    sol_burn = solve_ivp(
        lambda t, y: _stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs),
        t_span=(t_ignition, t_burn_end),
        y0=state_at_ign,
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
        events=_event_crash,
    )
    if len(sol_burn.t_events[0]) > 0:
        return {'crashed': True, 'state_final': None,
                't_burn': t_burn, 't_stage2_start': t2_start, 't_ignition': t_ignition,
                't_stage1': t_stage1, 'y_stage1': y_stage1}
    state_final = sol_burn.y[:5, -1].copy()

    if verbose:
        h_f = state_final[1] - c.R_EARTH
        print(f"  Stage 2 end: t={t_burn_end:.1f}s, h={h_f/1e3:.1f}km, "
              f"v={state_final[2]:.0f}m/s, gam={np.rad2deg(state_final[3]):.2f}deg")

    return {
        'crashed':        False,
        'state_final':    state_final,
        't_burn':         t_burn,
        't_stage2_start': t2_start,
        't_ignition':     t_ignition,
        't_stage1':       t_stage1,
        'y_stage1':       y_stage1,
    }


# ===========================================================================
# Objective function
# ===========================================================================

def _direct_objective_terms(result):
    """
    4-term non-dimensional objective, mirrors pso_coast_solver
    ._coast_objective_terms (no transversality term, no coast split).
    """
    state = result['state_final']
    r_val, v_f, g_f = state[1], state[2], state[3]

    r_target   = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    v_circular = _v_circular_rotating(r_target)
    gamma_ref  = np.deg2rad(sim_params.PSO_DIRECT_GAMMA_REF_DEG)

    J_nd  = result['t_burn'] / _T_MAX_2
    dh_nd = (r_val - r_target) / sim_params.TARGET_ORBITAL_ALTITUDE
    dv_nd = (v_f - v_circular) / v_circular
    dg_nd = g_f / gamma_ref

    return {
        'J'  : sim_params.PSO_DIRECT_W_J        * J_nd,
        'alt': sim_params.PSO_DIRECT_W_ALTITUDE * abs(dh_nd),
        'vel': sim_params.PSO_DIRECT_W_VELOCITY * abs(dv_nd),
        'fpa': sim_params.PSO_DIRECT_W_FPA      * abs(dg_nd),
    }


def compute_direct_objective(result):
    """Augmented objective value J for PSO minimisation."""
    if result['crashed'] or result['state_final'] is None:
        return CRASH_PENALTY
    state = result['state_final']
    C = 0.0
    if (state[1] - c.R_EARTH) < 0 or state[2] < 0:
        C = CRASH_PENALTY
    return float(sum(_direct_objective_terms(result).values()) + C)


def breakdown_direct_objective(result):
    """Decompose J into individual (weighted, non-dimensional) terms."""
    if result['crashed'] or result['state_final'] is None:
        return {'J': 1e20, 'alt': 1e20, 'vel': 1e20, 'fpa': 1e20}
    return _direct_objective_terms(result)


# ===========================================================================
# PyGMO-compatible problem class
# ===========================================================================

class DirectPSOProblem:
    """
    UDP for PyGMO's PSO algorithm.
    Decision vector: [gamma_p, t_burn_pct]
    """

    def fitness(self, x):
        gamma_p, t_burn_pct = x
        result = run_pso_direct_trajectory(gamma_p, t_burn_pct)
        return [compute_direct_objective(result)]

    def get_bounds(self):
        return (sim_params.PSO_DIRECT_LB, sim_params.PSO_DIRECT_UB)

    def get_nobj(self):
        return 1


# ===========================================================================
# PSO runner
# ===========================================================================

def run_pso_direct_optimization(verbose=True):
    """
    Run the 2-variable PSO direct-insertion optimisation.

    Returns
    -------
    optimal_params : list  [gamma_p, t_burn_pct]
    J_optimal      : float  Best objective value
    """
    global LAST_PSO_DIRECT_HISTORY

    n_particles = sim_params.PSO_DIRECT_N_PARTICLES
    n_gen       = sim_params.PSO_DIRECT_MAX_GENERATIONS
    lb          = sim_params.PSO_DIRECT_LB
    ub          = sim_params.PSO_DIRECT_UB

    if verbose:
        print("\n" + "=" * 60)
        print(f"PSO DIRECT-INSERTION OPTIMISATION — {sim_params.GUIDANCE_MODE.upper()}")
        print("=" * 60)
        print("  Optimising 2 variables: gamma_p, t_burn%")
        print(f"  Particles : {n_particles}")
        print(f"  Max gen.  : {n_gen}")
        print(f"  Bounds    : {list(zip(lb, ub))}")
        print("=" * 60 + "\n")

    t_start = time.time()

    try:
        import pygmo as pg

        prob = pg.problem(DirectPSOProblem())
        algo = pg.algorithm(pg.pso(
            gen     = n_gen,
            omega   = sim_params.PSO_DIRECT_OMEGA,
            eta1    = sim_params.PSO_DIRECT_C1,
            eta2    = sim_params.PSO_DIRECT_C2,
            max_vel = sim_params.PSO_DIRECT_VMAX,
            seed    = sim_params.PSO_DIRECT_SEED,
        ))
        if verbose:
            algo.set_verbosity(25)

        pop = pg.population(prob, size=n_particles, seed=sim_params.PSO_DIRECT_SEED)
        pop = algo.evolve(pop)

        best_x = list(pop.champion_x)
        best_f = float(pop.champion_f[0])

        uda = algo.extract(pg.pso)
        log = uda.get_log() if uda is not None else []
        if log:
            gens  = [row[0] for row in log]
            gbest = [row[2] for row in log]
            if gens[-1] != n_gen:
                gens.append(n_gen)
                gbest.append(best_f)
            LAST_PSO_DIRECT_HISTORY = {
                'gen': np.array(gens), 'gbest': np.array(gbest)}
        else:
            LAST_PSO_DIRECT_HISTORY = None

        if verbose:
            print(f"\n[PSO direct] Finished in {time.time() - t_start:.1f}s")
            print(f"  Best J = {best_f:.6f}")
            _print_direct_solution(best_x, best_f)

        return best_x, best_f

    except ImportError:
        raise ImportError(
            "pygmo is required for PSO direct-insertion optimisation. "
            "Install it with: conda install -c conda-forge pygmo"
        )


# ===========================================================================
# Full trajectory re-run for plotting
# ===========================================================================

def run_pso_direct_full(optimal_params, verbose=True):
    """
    Re-run the optimal PSO direct-insertion trajectory with dense output for
    plotting.

    Sets ``rocket_ascent.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL`` to the
    thrust-arc end time and ``rocket_ascent.LAST_DIRECT_MECO`` /
    ``rocket_ascent.LAST_DIRECT_INSERTION_REACHED`` from the achieved final
    state (mirrors run()'s direct-mode reporting).

    Returns
    -------
    time_full            : ndarray  Combined time array [s]
    data_full            : ndarray  State data (5 or 6 x N; row 5 = latitude
                                     when Earth rotation is enabled)
    thrust_full          : ndarray  Thrust [N] at each time step
    alpha_full           : ndarray  Angle of attack [rad] at each time step
    t_ignition           : float    Stage-2 engine ignition time [s]
    result               : dict     Same keys as run_pso_direct_trajectory
    coriolis_mag_data    : ndarray  Coriolis accel magnitude [m/s^2] (Stage-1
                                     real + Stage-2 zeros)
    centrifugal_mag_data : ndarray  Centrifugal accel magnitude [m/s^2]

    Also writes ra.theta_*_history, ra.tgo_*_history and (when enabled)
    ra.cross_heading_*_history so the shared plot block in main.py renders
    guidance/Earth-rotation plots.
    """
    gamma_p, t_burn_pct = optimal_params
    kick_angle = gamma_p - np.pi / 2.0

    # ---- Stage 1 ----
    t2_start, state2_init, _, t_stage1, y_stage1, crashed = ra.run_stage1(kick_angle)
    if crashed:
        raise RuntimeError("Stage 1 crashed during PSO direct full-trajectory run.")

    t_ignition  = t2_start + _T_IGNITION_DELAY
    state2_init = _strip_to_pmp_state(
        state2_init, np.deg2rad(sim_params.LAUNCH_LATITUDE))

    t_burn     = (t_burn_pct / 100.0) * _T_MAX_2
    t_burn_end = t_ignition + t_burn

    _dt = 0.5

    def _make_teval(t0, t1):
        pts = np.arange(t0, t1, _dt)
        if len(pts) == 0 or pts[-1] < t1:
            pts = np.append(pts, t1)
        return pts

    # ---- Pre-ignition coast (dense) ----
    sol_pre = solve_ivp(
        lambda t, y: _stage2_ode_guidance(t, y, 0.0, r.ISP_2, None),
        t_span=(t2_start, t_ignition),
        y0=state2_init[:5],
        t_eval=_make_teval(t2_start, t_ignition),
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
        events=_event_crash,
    )
    state_at_ign = sol_pre.y[:5, -1].copy()

    # ---- Single thrust arc (dense) ----
    gs_full = GuidanceState()
    sol_burn = solve_ivp(
        lambda t, y: _stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs_full),
        t_span=(t_ignition, t_burn_end),
        y0=state_at_ign,
        t_eval=_make_teval(t_ignition, t_burn_end),
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
        events=_event_crash,
    )
    state_insertion = sol_burn.y[:5, -1].copy()

    # ---- Post-insertion orbit coast (thrust off) ----
    # Propagate the achieved orbit so altitude/trajectory plots show the full
    # orbit and Final Orbital Elements are meaningful (mirrors pso_coast).
    # The trajectory velocity is rotating-frame, but orbital propagation needs
    # the INERTIAL velocity, so convert the insertion state to inertial here.
    post_init = state_insertion.copy()
    if sim_params.ENABLE_EARTH_ROTATION:
        lat_ins = np.deg2rad(sim_params.LAUNCH_LATITUDE)
        v_in, g_in = ra.get_inertial_state_components(
            state_insertion[1], state_insertion[2], state_insertion[3], lat_ins)
        post_init[2], post_init[3] = v_in, g_in

    t_post_start = t_burn_end
    t_post_end   = t_post_start + sim_params.DURATION_AFTER_SIMULATION
    sol_post = solve_ivp(
        lambda t, y: _stage2_ode_guidance(t, y, 0.0, r.ISP_2, None),
        t_span=(t_post_start, t_post_end),
        y0=post_init,
        t_eval=_make_teval(t_post_start, t_post_end),
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
        events=_event_crash,
    )

    # ---- Assemble Stage-2 arrays ----
    sols2_list = [sol_pre, sol_burn]
    thrusts2   = [0.0, r.F_THRUST_2]
    if sol_post is not None and len(sol_post.t) > 0:
        sols2_list.append(sol_post); thrusts2.append(0.0)

    t_s2_parts, y_s2_parts, th_s2_parts = [], [], []
    for sol, F in zip(sols2_list, thrusts2):
        if sol is None or len(sol.t) == 0:
            continue
        t_s2_parts.append(sol.t)
        y_s2_parts.append(sol.y[:5, :])
        th_s2_parts.append(np.full(len(sol.t), F))

    t_stage2_full = np.concatenate(t_s2_parts)
    y_stage2_full = np.concatenate(y_s2_parts, axis=1)
    thrust_stage2 = np.concatenate(th_s2_parts)

    # Alpha: interpolate gs_full log onto the Stage-2 output grid
    from Plots.plot_state_utils import interpolate_to_time
    if gs_full.time_log:
        alpha_stage2 = interpolate_to_time(
            gs_full.time_log, gs_full.alpha_log, t_stage2_full)
    else:
        alpha_stage2 = np.zeros(len(t_stage2_full))

    # ---- Combine Stage 1 + Stage 2 ----
    y1        = y_stage1[:5, :]
    time_full = np.concatenate([t_stage1, t_stage2_full])
    data_full = np.concatenate([y1, y_stage2_full], axis=1)

    thrust_stage1 = interpolate_to_time(ra.time_history, ra.thrust_history, t_stage1)
    alpha_stage1  = interpolate_to_time(
        ra.alpha_time_history, ra.alpha_history, t_stage1)

    thrust_full = np.concatenate([thrust_stage1, thrust_stage2])
    alpha_full  = np.concatenate([alpha_stage1,  alpha_stage2])

    n_stage1 = len(t_stage1)
    n_stage2 = len(t_stage2_full)

    # ---- Latitude row (6th state row) so the latitude plot renders ----
    if sim_params.ENABLE_EARTH_ROTATION:
        lat_row = np.array([ra.get_latitude_from_downrange(s) for s in data_full[0]])
        data_full = np.vstack([data_full, lat_row])   # rows: s, r, v, gamma, m, lat

    # ---- Assemble full-trajectory history channels for the plot suite ----
    theta_full = alpha_full + data_full[3]            # pitch theta = alpha + gamma
    ra.theta_history      = list(theta_full)
    ra.theta_time_history = list(time_full)

    # t_go: guidance runs in Stage 2 only (apollo / linear_tangent / bilinear_tangent modes)
    if gs_full.tgo_time_log:
        ra.tgo_time_history = list(gs_full.tgo_time_log)
        ra.tgo_history      = list(gs_full.tgo_log)

    # Pseudo-force / cross-heading: Stage-1 real (already in ra.*_history),
    # Stage-2 zeros appended.
    coriolis_stage1    = np.asarray(ra.coriolis_mag_history, dtype=float)
    centrifugal_stage1 = np.asarray(ra.centrifugal_mag_history, dtype=float)
    cor_s1 = interpolate_to_time(ra.time_history, coriolis_stage1, t_stage1) \
        if len(coriolis_stage1) else np.zeros(n_stage1)
    cen_s1 = interpolate_to_time(ra.time_history, centrifugal_stage1, t_stage1) \
        if len(centrifugal_stage1) else np.zeros(n_stage1)
    coriolis_mag_data    = np.concatenate([cor_s1, np.zeros(n_stage2)])
    centrifugal_mag_data = np.concatenate([cen_s1, np.zeros(n_stage2)])

    if sim_params.COMPUTE_CROSS_HEADING_COUNTER_FORCE:
        chf_s1 = np.asarray(ra.cross_heading_counter_force_history, dtype=float)
        cha_s1 = np.asarray(ra.cross_heading_accel_history, dtype=float)
        chf_s1 = interpolate_to_time(ra.time_history, chf_s1, t_stage1) \
            if len(chf_s1) else np.zeros(n_stage1)
        cha_s1 = interpolate_to_time(ra.time_history, cha_s1, t_stage1) \
            if len(cha_s1) else np.zeros(n_stage1)
        ra.cross_heading_counter_force_history = list(
            np.concatenate([chf_s1, np.zeros(n_stage2)]))
        ra.cross_heading_accel_history = list(
            np.concatenate([cha_s1, np.zeros(n_stage2)]))

    # ---- Direct-insertion reporting (mirrors run()'s direct-mode block) ----
    r_target   = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    v_circular = np.sqrt(c.MU_EARTH / r_target)
    if sim_params.ENABLE_EARTH_ROTATION:
        v_insertion, _ = ra.get_inertial_state_components(
            state_insertion[1], state_insertion[2], state_insertion[3],
            np.deg2rad(sim_params.LAUNCH_LATITUDE))
    else:
        v_insertion = state_insertion[2]
    box_margin = ra.interrupt_direct_insertion(0.0, state_insertion)

    ra.LAST_DIRECT_MECO              = bool(v_insertion >= v_circular)
    ra.LAST_DIRECT_INSERTION_REACHED = bool(box_margin <= 0.0)
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = t_burn_end

    if verbose:
        sf = data_full[:, -1]
        print(f"\n[PSO direct full run] t_end={time_full[-1]:.1f}s, "
              f"h={(sf[1]-c.R_EARTH)/1e3:.1f}km, "
              f"v={sf[2]:.0f}m/s, gam={np.rad2deg(sf[3]):.2f}deg")

    # ---- Build result dict from the dense run (no extra re-integration) ----
    result = {
        'crashed':        False,
        'state_final':    state_insertion,   # state at orbit insertion (burn end)
        't_burn':         t_burn,
        't_stage2_start': t2_start,
        't_ignition':     t_ignition,
        't_stage1':       t_stage1,
        'y_stage1':       y_stage1,
    }

    return (time_full, data_full, thrust_full, alpha_full, t_ignition, result,
            coriolis_mag_data, centrifugal_mag_data)


# ===========================================================================
# Diagnostic helper
# ===========================================================================

def _print_direct_solution(x, J):
    """Pretty-print the optimal PSO direct-insertion parameters (compact)."""
    gamma_p, t_burn_pct = x
    kick_angle = gamma_p - np.pi / 2.0
    t_burn = (t_burn_pct / 100.0) * _T_MAX_2

    print("\nOptimal PSO direct-insertion parameters:")
    print(f"  Pitch angle gamma_p = {np.rad2deg(gamma_p):.4f} deg  ({gamma_p:.6f} rad)")
    print(f"  Kick angle          = {np.rad2deg(kick_angle):.4f} deg")
    print(f"  Burn duration       = {t_burn:.2f} s  ({t_burn_pct:.2f} % of T_max = {_T_MAX_2:.1f} s)")
    print(f"  J                   = {J:.6f}")
