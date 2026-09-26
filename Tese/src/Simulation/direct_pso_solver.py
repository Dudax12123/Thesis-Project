"""
Direct-Insertion Solver

Optimiser for COAST_METHOD == "direct", over
    x = [gamma_p,       pitch maneuver (kick) angle [rad]
         t_burn_pct]    Stage-2 continuous burn duration as % of T_MAX_2 [%]
by the 2-variable PSO (DIRECT_OPTIMIZER = "pso", bounds PSO_DIRECT_LB/UB) or by
an exhaustive grid refined with Brent's method ("grid_brent", bounds DIRECT_GRID_*).

Under DIRECT_LAW_TERMINATED_CUTOFF with GUIDANCE_MODE = "peg_new" the burn ends at
peg_new's own t_go instead, and x = [gamma_p] (see _fly_law_terminated_burn).

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

PyGMO is required by the PSO only; "grid_brent" needs SciPy alone.
"""

import sys
import time
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize_scalar

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent))

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r
from Input_File import simulation_parameters as sim_params
import Simulation.rocket_ascent as ra
import Guidance.peg_guidance_new as peg_new_mod
from Simulation.pso_coast_solver import (
    _strip_to_pmp_state,
    _v_circular_rotating,
    GuidanceState,
    _stage2_ode_guidance,
    solve_guided_arc,
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
# grid_brent only: the grid it searched and the evaluations it spent (None after a PSO).
LAST_DIRECT_GRID = None
LAST_DIRECT_N_EVALUATIONS = None


def law_terminated():
    """True when peg_new, not the optimiser, ends the burn (DIRECT_LAW_TERMINATED_CUTOFF).

    Only peg_new has a cutoff of its own to hand over; for every other law the
    switch is inert and the burn time stays a decision variable."""
    return (bool(getattr(sim_params, "DIRECT_LAW_TERMINATED_CUTOFF", False))
            and sim_params.GUIDANCE_MODE == "peg_new")


def _decision_bounds():
    """(lb, ub) of the PSO's decision vector: [gamma_p, t_burn_pct], or [gamma_p]
    when the law ends the burn."""
    if law_terminated():
        return [sim_params.PSO_DIRECT_LB[0]], [sim_params.PSO_DIRECT_UB[0]]
    return sim_params.PSO_DIRECT_LB, sim_params.PSO_DIRECT_UB


def _fly_law_terminated_burn(t_ignition, y0, gs, t_eval_fn=None):
    """peg_new's single burn, ended where peg_new's own t_go says.

    The major loop runs HERE, at guidance-cycle boundaries (PEG_MAJOR_LOOP_RATE) on
    accepted states, and the ODE right-hand side only reads the coefficients
    (gs.peg_new_external): no guidance update and no cutoff is ever decided at one
    of solve_ivp's speculative evaluations. Each cycle is its own solve_ivp call,
    ending exactly on its boundary. Once t_go <= max(APOLLO_FREEZE_THRESHOLD, cycle)
    the coefficients freeze and the last cycle is flown to exactly t_go. Propellant
    exhaustion (_T_MAX_2 after ignition) ends the burn regardless.

    The target is the one the pso_coast dispatcher resolves for a single-law run
    (gs.target is None): r_T at TARGET_ORBITAL_ALTITUDE, the rotating-frame circular
    speed, v_r = 0.

    Returns (sols, t_end, y_end, crashed); sols are the per-cycle solutions.
    """
    gs.peg_new_external = True
    r_tgt     = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    v_theta_T = _v_circular_rotating(r_tgt)
    Ve        = r.ISP_2 * c.G_0
    cycle     = float(sim_params.PEG_MAJOR_LOOP_RATE)
    last      = max(float(sim_params.APOLLO_FREEZE_THRESHOLD), cycle)
    t_exhaust = t_ignition + _T_MAX_2
    t, y = t_ignition, np.asarray(y0, dtype=float).copy()
    sols = []
    while t < t_exhaust:
        (gs.peg_new_vgo_r, gs.peg_new_vgo_theta,
         gs.peg_new_L0,    gs.peg_new_tgo,
         gs.peg_new_t_lambda, gs.peg_new_lambda_r) = peg_new_mod.peg_new_major_loop(
             y[:5], r_tgt, c.MU_EARTH, Ve, r.F_THRUST_2,
             v_theta_T=v_theta_T, v_r_T=0.0)
        gs.peg_new_t_epoch = t
        tgo = gs.peg_new_tgo
        gs.tgo_time_log.append(t)
        gs.tgo_log.append(tgo)
        if not np.isfinite(tgo) or tgo <= 0.0:
            break                            # the law asks for no more burn
        final = tgo <= last
        gs.peg_new_frozen = final
        t_next = min(t + (tgo if final else cycle), t_exhaust)
        sol = solve_ivp(
            lambda tt, yy: _stage2_ode_guidance(tt, yy, r.F_THRUST_2, r.ISP_2, gs),
            t_span=(t, t_next),
            y0=y,
            t_eval=None if t_eval_fn is None else t_eval_fn(t, t_next),
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        if len(sol.t_events[0]) > 0:
            return sols, t, y, True
        sols.append(sol)
        t, y = t_next, sol.y[:5, -1].copy()
        if final:
            break
    return sols, t, y, False


# ===========================================================================
# Full Stage-1 -> Stage-2 trajectory runner (PSO inner loop)
# ===========================================================================

def run_pso_direct_trajectory(gamma_p, t_burn_pct=None, verbose=False):
    """
    Simulate a Stage-1-kick + single-thrust-arc direct-insertion trajectory
    for one PSO particle.

    Parameters
    ----------
    gamma_p    : float  Pitch maneuver angle [rad]  (kick_angle = gamma_p - pi/2)
    t_burn_pct : float  Stage-2 continuous burn duration as % of T_MAX_2 [%];
                        ignored (and may be omitted) when law_terminated()
    verbose    : bool

    Returns
    -------
    result : dict with keys
        crashed, state_final, t_burn, t_stage2_start, t_ignition,
        t_stage1, y_stage1
    """
    ra.set_pseudo_forces_for_run(True)   # carried for the whole ascent
    kick_angle = gamma_p - np.pi / 2.0
    t_burn = None if law_terminated() else (t_burn_pct / 100.0) * _T_MAX_2

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
    # run_stage1 hands Stage 2 a state that still carries the fairing;
    # shed it here if the jettison criterion is already met.
    state2_init = ra.shed_fairing_if_due(t2_start, state2_init)

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
    # Second chance, for a trajectory that staged below the criterion:
    # the pre-ignition coast climbs fast, so the crossing lands here.
    state_at_ign = ra.shed_fairing_if_due(t_ignition, state_at_ign)

    # ---- Single continuous thrust arc (t_ignition -> t_ignition + t_burn) ----
    gs = GuidanceState()
    if law_terminated():
        # peg_new ends the burn at its own t_go; the burn time is an OUTPUT here.
        _, t_burn_end, state_final, crashed = _fly_law_terminated_burn(
            t_ignition, state_at_ign, gs)
        t_burn = t_burn_end - t_ignition
        if crashed:
            return {'crashed': True, 'state_final': None,
                    't_burn': t_burn, 't_stage2_start': t2_start, 't_ignition': t_ignition,
                    't_stage1': t_stage1, 'y_stage1': y_stage1}
    else:
        t_burn_end = t_ignition + t_burn
        gs.tgo_deadline = t_burn_end
        sol_burn = solve_guided_arc(
            lambda t, y: _stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs), gs,
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
    Decision vector: [gamma_p, t_burn_pct], or [gamma_p] when law_terminated()
    """

    def fitness(self, x):
        result = run_pso_direct_trajectory(*x)
        return [compute_direct_objective(result)]

    def get_bounds(self):
        return _decision_bounds()

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
    lb, ub      = _decision_bounds()

    # A single continuous burn (no coast) is delta-v-marginal to reach the target
    # circular orbit, so only the explicit terminal-constraint laws close it.
    # Empirically (PSO converges to the SAME optimum at 900 and 5000 evals, i.e.
    # budget-independently), the other laws settle on a SUBORBITAL insertion under
    # "direct" — warn so the result is not mistaken for a convergence failure.
    # Use "pso_coast"/"apogee_check" (which have a coast) for those laws instead.
    if verbose and sim_params.GUIDANCE_MODE not in ("apollo", "peg", "peg_new"):
        print("\n" + "!" * 60)
        print(f"WARNING: COAST_METHOD='direct' with GUIDANCE_MODE="
              f"'{sim_params.GUIDANCE_MODE}' is not a reliable pairing.")
        print("  'direct' is one continuous Stage-2 burn with no coast; only")
        print("  {apollo, peg, peg_new} fly the near-optimal lofting steering that")
        print("  reaches the target circular orbit. Other laws converge to a")
        print("  SUBORBITAL insertion here (more PSO budget does NOT help).")
        print("  Use COAST_METHOD='pso_coast' or 'apogee_check' for this mode.")
        print("!" * 60)

    if verbose:
        print("\n" + "=" * 60)
        print(f"PSO DIRECT-INSERTION OPTIMISATION — {sim_params.GUIDANCE_MODE.upper()}")
        print("=" * 60)
        print("  Optimising 1 variable: gamma_p (peg_new ends the burn)" if law_terminated()
              else "  Optimising 2 variables: gamma_p, t_burn%")
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
# Grid + Brent runner (DIRECT_OPTIMIZER = "grid_brent")
# ===========================================================================

def _grid_then_brent(f, lo, hi, n, xatol, label=None):
    """Minimise f over one bounded variable: an exhaustive grid of n points, then
    Brent's bounded method on the bracket formed by the best grid point and its two
    neighbours (scipy minimize_scalar, method="bounded").

    Nothing here assumes a single minimum: the grid picks the basin and Brent only
    refines inside it. Brent's point is kept only if it beats the grid's, since the
    bounded method never evaluates the ends of its bracket.

    Returns (x_best, f_best, xs, fs, on_bound); on_bound is True when the best grid
    point is an end of the grid -- an optimum the box may be cutting off.
    """
    xs = np.linspace(float(lo), float(hi), int(n))
    fs = np.empty(len(xs))
    step = max(1, len(xs) // 10)
    for k, x in enumerate(xs):
        fs[k] = f(x)
        if label and (k + 1) % step == 0:
            print(f"  [{label}] grid {k + 1}/{len(xs)}  best so far "
                  f"{fs[:k + 1].min():.6f} at {xs[int(np.argmin(fs[:k + 1]))]:.6f}", flush=True)
    i = int(np.argmin(fs))
    x_best, f_best = float(xs[i]), float(fs[i])
    a, b = xs[max(i - 1, 0)], xs[min(i + 1, len(xs) - 1)]
    if b > a:
        res = minimize_scalar(f, bounds=(a, b), method="bounded",
                              options={"xatol": float(xatol)})
        if res.fun < f_best:
            x_best, f_best = float(res.x), float(res.fun)
    return x_best, f_best, xs, fs, bool(i in (0, len(xs) - 1))


def run_direct_grid_optimization(verbose=True):
    """Deterministic alternative to the PSO for the same objective.

    law_terminated() : x = [gamma_p]; one grid + Brent over gamma_p.
    otherwise        : x = [gamma_p, t_burn_pct]; an outer grid + Brent over gamma_p
                       whose value at each gamma_p is an inner grid + Brent over
                       t_burn_pct (cached, so Brent's re-visits cost nothing).

    Sets LAST_DIRECT_GRID (the outer grid, its J, and for the nested search the inner
    optimum of every grid point) and LAST_DIRECT_N_EVALUATIONS (trajectories flown).

    Returns (best_x, J_best) like run_pso_direct_optimization.
    """
    global LAST_PSO_DIRECT_HISTORY, LAST_DIRECT_GRID, LAST_DIRECT_N_EVALUATIONS
    n_eval = [0]

    def J_of(x):
        n_eval[0] += 1
        return compute_direct_objective(run_pso_direct_trajectory(*x))

    g_lo, g_hi = sim_params.DIRECT_GRID_GAMMA_P_BOUNDS
    g_n, g_tol = sim_params.DIRECT_GRID_GAMMA_P_POINTS, sim_params.DIRECT_BRENT_XATOL_GAMMA_P
    t_start = time.time()
    if verbose:
        print("\n" + "=" * 60)
        print(f"GRID + BRENT DIRECT-INSERTION OPTIMISATION — {sim_params.GUIDANCE_MODE.upper()}")
        print("=" * 60)
        print(f"  gamma_p    : {g_n} points over [{g_lo}, {g_hi}] rad, Brent xatol {g_tol}")
        if law_terminated():
            print("  t_burn     : ended by peg_new's own t_go (DIRECT_LAW_TERMINATED_CUTOFF)")
        else:
            print(f"  t_burn_pct : {sim_params.DIRECT_GRID_T_BURN_PCT_POINTS} points over "
                  f"{sim_params.DIRECT_GRID_T_BURN_PCT_BOUNDS} %, Brent xatol "
                  f"{sim_params.DIRECT_BRENT_XATOL_T_BURN_PCT}, per gamma_p")
        print("=" * 60 + "\n", flush=True)

    grid = {}
    if law_terminated():
        gp, J, xs, fs, edge = _grid_then_brent(
            lambda g: J_of([g]), g_lo, g_hi, g_n, g_tol,
            label="gamma_p" if verbose else None)
        best_x = [gp]
    else:
        t_lo, t_hi = sim_params.DIRECT_GRID_T_BURN_PCT_BOUNDS
        t_n, t_tol = (sim_params.DIRECT_GRID_T_BURN_PCT_POINTS,
                      sim_params.DIRECT_BRENT_XATOL_T_BURN_PCT)
        inner = {}

        def best_burn(g):
            if g not in inner:
                tb, Jt, _, _, t_edge = _grid_then_brent(
                    lambda tb: J_of([g, tb]), t_lo, t_hi, t_n, t_tol)
                inner[g] = (tb, Jt, t_edge)
            return inner[g]

        gp, J, xs, fs, edge = _grid_then_brent(
            lambda g: best_burn(g)[1], g_lo, g_hi, g_n, g_tol,
            label="gamma_p" if verbose else None)
        best_x = [gp, best_burn(gp)[0]]
        grid['t_burn_pct'] = np.array([inner[g][0] for g in xs])
        grid['t_burn_pct_on_bound'] = bool(best_burn(gp)[2])
    grid.update({'gamma_p': xs, 'J': fs, 'gamma_p_on_bound': edge})

    LAST_PSO_DIRECT_HISTORY = None
    LAST_DIRECT_GRID = grid
    LAST_DIRECT_N_EVALUATIONS = n_eval[0]
    if verbose:
        print(f"\n[grid+Brent direct] Finished in {time.time() - t_start:.1f}s, "
              f"{n_eval[0]} trajectories")
        print(f"  Best J = {J:.6f}")
        if edge:
            print("  NOTE: the optimum gamma_p sits on the edge of DIRECT_GRID_GAMMA_P_BOUNDS "
                  "-- widen the box before reading it as an optimum.")
        if grid.get('t_burn_pct_on_bound'):
            print("  NOTE: the optimum t_burn_pct sits on the edge of "
                  "DIRECT_GRID_T_BURN_PCT_BOUNDS.")
        _print_direct_solution(best_x, J)
    return best_x, J


def run_direct_optimization(verbose=True):
    """The direct architecture's optimiser, as DIRECT_OPTIMIZER selects it."""
    global LAST_DIRECT_GRID, LAST_DIRECT_N_EVALUATIONS
    method = getattr(sim_params, "DIRECT_OPTIMIZER", "pso")
    if method == "pso":
        LAST_DIRECT_GRID = None
        LAST_DIRECT_N_EVALUATIONS = None
        return run_pso_direct_optimization(verbose=verbose)
    if method == "grid_brent":
        return run_direct_grid_optimization(verbose=verbose)
    raise ValueError(f"DIRECT_OPTIMIZER must be 'pso' or 'grid_brent', got {method!r}")


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
    ra.set_pseudo_forces_for_run(True)   # carried for the whole ascent
    gamma_p = optimal_params[0]
    t_burn_pct = None if law_terminated() else optimal_params[1]
    kick_angle = gamma_p - np.pi / 2.0

    # ---- Stage 1 ----
    t2_start, state2_init, _, t_stage1, y_stage1, crashed = ra.run_stage1(kick_angle)
    if crashed:
        raise RuntimeError("Stage 1 crashed during PSO direct full-trajectory run.")

    t_ignition  = t2_start + _T_IGNITION_DELAY
    state2_init = _strip_to_pmp_state(
        state2_init, np.deg2rad(sim_params.LAUNCH_LATITUDE))
    # run_stage1 hands Stage 2 a state that still carries the fairing;
    # shed it here if the jettison criterion is already met.
    state2_init = ra.shed_fairing_if_due(t2_start, state2_init)

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
    # Second chance, for a trajectory that staged below the criterion:
    # the pre-ignition coast climbs fast, so the crossing lands here.
    state_at_ign = ra.shed_fairing_if_due(t_ignition, state_at_ign)

    # ---- Single thrust arc (dense) ----
    gs_full = GuidanceState()
    if law_terminated():
        # Same cycles as the optimiser's inner loop; t_eval only samples the dense
        # output, so the insertion state is the one the optimiser scored. After the
        # first cycle each grid starts one sample past the boundary the previous
        # cycle ended on.
        def _cycle_teval(t0, t1):
            pts = _make_teval(t0, t1)
            return pts if t0 == t_ignition else pts[1:]
        burn_sols, t_burn_end, state_insertion, crashed = _fly_law_terminated_burn(
            t_ignition, state_at_ign, gs_full, t_eval_fn=_cycle_teval)
        if crashed:
            raise RuntimeError("Stage 2 crashed during the direct full-trajectory run.")
        t_burn = t_burn_end - t_ignition
    else:
        t_burn     = (t_burn_pct / 100.0) * _T_MAX_2
        t_burn_end = t_ignition + t_burn
        gs_full.tgo_deadline = t_burn_end
        sol_burn = solve_guided_arc(
            lambda t, y: _stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs_full), gs_full,
            t_span=(t_ignition, t_burn_end),
            y0=state_at_ign,
            t_eval=_make_teval(t_ignition, t_burn_end),
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        state_insertion = sol_burn.y[:5, -1].copy()
        burn_sols = [sol_burn]

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

    # State is now INERTIAL — suppress the rotating-frame pseudo-forces for the
    # orbit propagation below (mirrors pso_coast_solver and run()).
    ra.PROPAGATING_IN_INERTIAL_FRAME = True

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
    sols2_list = [sol_pre] + burn_sols
    thrusts2   = [0.0] + [r.F_THRUST_2] * len(burn_sols)
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

    thrust_stage1 = ra.thrust_on_grid(t_stage1)
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

    # Pseudo-force / cross-heading channels, recomputed on the full dense grid —
    # Stage 2 now carries the same frame terms as Stage 1. See the equivalent
    # block in pso_coast_solver for why the inertial flag is cleared here and the
    # post-insertion tail zeroed instead.
    _n_post = len(sol_post.t) if (sol_post is not None and len(sol_post.t) > 0) else 0
    _saved_inertial_flag = ra.PROPAGATING_IN_INERTIAL_FRAME
    ra.PROPAGATING_IN_INERTIAL_FRAME = False
    try:
        chf_grid, cha_grid, coriolis_mag_data, centrifugal_mag_data = \
            ra.pseudo_force_channels_on_grid(time_full, data_full[:5])
    finally:
        ra.PROPAGATING_IN_INERTIAL_FRAME = _saved_inertial_flag
    if _n_post:
        for _ch in (chf_grid, cha_grid, coriolis_mag_data, centrifugal_mag_data):
            _ch[-_n_post:] = 0.0

    if sim_params.COMPUTE_CROSS_HEADING_COUNTER_FORCE:
        ra.cross_heading_counter_force_history = list(chf_grid)
        ra.cross_heading_accel_history         = list(cha_grid)

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
    """Pretty-print the optimal direct-insertion parameters (compact)."""
    gamma_p = x[0]
    kick_angle = gamma_p - np.pi / 2.0

    print("\nOptimal direct-insertion parameters:")
    print(f"  Pitch angle gamma_p = {np.rad2deg(gamma_p):.4f} deg  ({gamma_p:.6f} rad)")
    print(f"  Kick angle          = {np.rad2deg(kick_angle):.4f} deg")
    if len(x) > 1:
        t_burn_pct = x[1]
        t_burn = (t_burn_pct / 100.0) * _T_MAX_2
        print(f"  Burn duration       = {t_burn:.2f} s  ({t_burn_pct:.2f} % of T_max = {_T_MAX_2:.1f} s)")
    else:
        print("  Burn duration       = set by peg_new's own t_go (see the full run)")
    print(f"  J                   = {J:.6f}")
