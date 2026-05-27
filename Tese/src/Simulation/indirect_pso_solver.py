"""
Indirect Trajectory Optimization — PSO Solver

Implements the outer PSO (Particle Swarm Optimisation) loop described in
Sect. 4.2.2 of the thesis paper.  The PSO simultaneously finds:

    x = [lambda0_r, lambda0_v, lambda0_g,    ← initial costate values  ([-1,1])
         delta_tc,                            ← coast duration [s]
         delta_tr_pct,                        ← Stage-2 burn as % of T_max [%]
         coast_start_pct,                     ← coast start as % of burn time [%]
         gamma_p]                             ← pitch maneuver angle [rad]

Each PSO evaluation runs a two-phase trajectory simulation:
  Phase 1 – Stage 1 gravity turn  (via rocket_ascent.run_stage1)
  Phase 2 – Stage 2 with PMP guidance:
              Arc 1: thrust for t_coast_start seconds
              Arc 2: coast for delta_tc seconds
              Arc 3: thrust for (T_burn_total − t_coast_start) seconds
  The augmented state  [s, r, v, γ, m, λ_r, λ_v, λ_γ]  is propagated by
  scipy.solve_ivp, so costates are integrated with the same RK45 accuracy as
  the physical state.

The objective function (Eq. 39) penalises:
  • altitude, velocity, and FPA terminal constraint violations
  • transversality condition violation (Eq. 38)
  • trajectories that crash or deplete all propellant before reaching orbit

PyGMO is used when available; otherwise scipy.optimize.differential_evolution
provides an equivalent (though parameter-incompatible) fallback.
"""

import sys
import time
import warnings
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
from Guidance.indirect_pmp_guidance import (
    pmp_control_law,
    costate_derivatives,
    compute_hamiltonian,
)
import Simulation.rocket_ascent as ra


# ---------------------------------------------------------------------------
# Derived Stage-2 constants (computed once at import time)
# ---------------------------------------------------------------------------
_MDOT_2 = r.F_THRUST_2 / (r.ISP_2 * c.G_0)           # Stage-2 mass flow rate [kg/s]
_T_MAX_2 = r.M_PROP_2 / _MDOT_2                        # Time to deplete ALL Stage-2 propellant [s]
# Engine-ignition delay measured from stage separation:
_T_IGNITION_DELAY = r.TIME_SECOND_ENGINE_IGNITION - r.TIME_First_STAGE_SEPARATION


# ===========================================================================
# Stage-2 augmented ODE
# ===========================================================================

def _stage2_ode(t, aug_state, thrust, Isp):
    """
    Right-hand side for the augmented Stage-2 ODE.

    aug_state = [s, r, v, γ, m, λ_r, λ_v, λ_γ]
    (indices 0-4 = physical state, 5-7 = costates)

    The PMP control law computes α from the current costates.
    Drag-free dynamics are used (consistent with the costate equations).

    Parameters
    ----------
    t         : float   Current time [s]  (required by solve_ivp but unused here)
    aug_state : array   Augmented state (8 elements)
    thrust    : float   Current thrust force [N]  (0 during coast arcs)
    Isp       : float   Specific impulse [s]

    Returns
    -------
    derivatives : list  d(aug_state)/dt  (8 elements)
    """
    s, r_val, v, gamma, m = aug_state[:5]
    lam_r, lam_v, lam_g   = aug_state[5], aug_state[6], aug_state[7]

    _EPS = 1e-10
    mu = c.MU_EARTH

    # Angle of attack from PMP
    alpha = pmp_control_law(lam_v, lam_g, v)

    cg = np.cos(gamma)
    sg = np.sin(gamma)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    g_local = mu / r_val ** 2
    T_over_m = (thrust / m) if m > _EPS else 0.0

    # --- Physical state derivatives (drag-free) ---
    dsdt    = (c.R_EARTH / r_val) * v * cg
    drdt    = v * sg
    dvdt    = T_over_m * ca - g_local * sg
    if abs(v) < _EPS:
        dgdt = 0.0
    else:
        dgdt = (1.0 / v) * (T_over_m * sa - (g_local - v ** 2 / r_val) * cg)
    dmdt    = -thrust / (Isp * c.G_0) if thrust > 0 and m > _EPS else 0.0

    # --- Costate derivatives ---
    dlams = costate_derivatives(r_val, v, gamma, thrust, m, lam_r, lam_v, lam_g, alpha)

    return [dsdt, drdt, dvdt, dgdt, dmdt] + dlams


# ---------------------------------------------------------------------------
# Ground-collision event (terminal)
# ---------------------------------------------------------------------------
def _event_crash(t, y, *args):
    return y[1] - c.R_EARTH

_event_crash.terminal  = True
_event_crash.direction = -1


# ===========================================================================
# Full two-phase trajectory simulation
# ===========================================================================

def run_indirect_trajectory(lambda0_r, lambda0_v, lambda0_g,
                             delta_tc, delta_tr_pct, coast_start_pct,
                             gamma_p, verbose=False):
    """
    Two-phase trajectory simulation for a single PSO particle evaluation.

    Phase 1  — Stage 1 gravity turn with kick angle ``gamma_p`` (mapped to
               the existing ``run_stage1`` function).

    Phase 2  — Stage 2 with PMP guidance split into three sub-arcs:
      Arc 1 (thrust) : duration = t_coast_start = coast_start_pct/100 * T_burn_total
      Arc 2 (coast)  : duration = delta_tc  [s]   (F_T = 0, costates still propagated)
      Arc 3 (thrust) : duration = T_burn_total − t_coast_start

    Where  T_burn_total = delta_tr_pct/100 * T_MAX_2  (total burn time, excl. coast).
    The engine-ignition delay (_T_IGNITION_DELAY s) is prepended as an un-controlled
    ballistic arc at the start of Stage 2.

    Costates are initialised at the START of the PMP-guided burn (after ignition delay)
    and propagated continuously through all three arcs.  The Weierstrass–Erdmann
    condition (costate continuity across arc junctions) is automatically satisfied.

    Parameters
    ----------
    lambda0_r, lambda0_v, lambda0_g : float
        Initial costate values at Stage-2 engine ignition  (paper bounds: [-1, 1])
    delta_tc        : float   Coast phase duration [s]          (bounds: [0, 2000])
    delta_tr_pct    : float   Stage-2 burn as % of T_MAX_2 [%] (bounds: [0, 100])
    coast_start_pct : float   Coast start as % of burn time [%] (bounds: [0, 100])
    gamma_p         : float   Pitch maneuver (kick) angle [rad] (bounds: [1.54, 1.57])
    verbose         : bool    If True, print intermediate results

    Returns
    -------
    result : dict with keys
        'crashed'          : bool
        'state_final'      : ndarray  [s, r, v, γ, m]  at end of Stage 2
        'H_burn_start'     : float    H at beginning of last-stage burn (Arc 1 start)
        'H_coast_end'      : float    H at end of coast arc (Arc 2 end)
        'H_burn_end'       : float    H at end of last-stage burn (Arc 3 end)
        't_f'              : float    Final time [s]
        't_cf'             : float    Coast start time [s]  (= t_f − arc3_duration)
        't_stage2_start'   : float    Time of stage separation [s]
        't_ignition'       : float    Time of Stage-2 engine ignition [s]
        't_stage1'         : ndarray  Stage-1 time array
        'y_stage1'         : ndarray  Stage-1 state data
    """
    # -----------------------------------------------------------------
    # Phase 1: Stage 1 gravity turn
    # -----------------------------------------------------------------
    # gamma_p is the pitch maneuver angle in [1.54, 1.57] rad (≈ 88–90°).
    # In the existing code, the kick angle is a small negative offset from
    # vertical; we convert: kick_angle = gamma_p − π/2  (negative for pitchover)
    kick_angle = gamma_p - np.pi / 2.0   # maps [1.54, 1.57] → [−0.031, −0.001] rad

    t2_start, state2_init, t_meco, t_stage1, y_stage1, crashed = ra.run_stage1(kick_angle)

    if crashed:
        return {
            'crashed': True,
            'state_final': None,
            'H_burn_start': 0.0, 'H_coast_end': 0.0, 'H_burn_end': 0.0,
            't_f': 0.0, 't_cf': 0.0,
            't_stage2_start': 0.0, 't_ignition': 0.0,
            't_stage1': t_stage1, 'y_stage1': y_stage1,
        }

    # Strip optional lat/heading states — the PMP ODE uses drag-free vacuum
    # dynamics and requires exactly [s, r, v, gamma, m] (5 elements).
    state2_init = state2_init[:5]

    if verbose:
        h2 = state2_init[1] - c.R_EARTH
        print(f"  Stage 1 end: t={t2_start:.1f}s, h={h2/1e3:.1f}km, "
              f"v={state2_init[2]:.0f}m/s, gam={np.rad2deg(state2_init[3]):.2f}deg, "
              f"m={state2_init[4]:.0f}kg")

    # -----------------------------------------------------------------
    # Timing calculations for Stage 2
    # -----------------------------------------------------------------
    T_burn_total   = (delta_tr_pct  / 100.0) * _T_MAX_2        # total burn time [s]
    t_coast_start  = (coast_start_pct / 100.0) * T_burn_total   # thrust before coast [s]
    t_arc3_burn    = T_burn_total - t_coast_start                # thrust after coast [s]

    t_ignition     = t2_start + _T_IGNITION_DELAY               # absolute ignition time

    # -----------------------------------------------------------------
    # Pre-ignition ballistic coast (ignition delay arc)
    # -----------------------------------------------------------------
    # Propagate physical state only (no costates yet, no thrust).
    n_state = 5   # always 5 after the [:5] strip above
    aug0_preig = list(state2_init) + [0.0, 0.0, 0.0]   # costates = 0 (unused)

    sol_pre = solve_ivp(
        lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2),
        t_span=(t2_start, t_ignition + 0.5),
        y0=aug0_preig,
        max_step=0.5,
        events=_event_crash,
        atol=1e-8,
    )

    if len(sol_pre.t_events[0]) > 0:           # crash during ignition delay
        return {
            'crashed': True,
            'state_final': None,
            'H_burn_start': 0.0, 'H_coast_end': 0.0, 'H_burn_end': 0.0,
            't_f': 0.0, 't_cf': 0.0,
            't_stage2_start': t2_start, 't_ignition': t_ignition,
            't_stage1': t_stage1, 'y_stage1': y_stage1,
        }

    state_at_ignition = sol_pre.y[:n_state, -1].copy()

    # -----------------------------------------------------------------
    # Initialise augmented state with PSO-provided costates
    # -----------------------------------------------------------------
    aug_state_ign = list(state_at_ignition) + [lambda0_r, lambda0_v, lambda0_g]

    # Record H at start of guided burn (for transversality condition, Eq. 38)
    H_burn_start = compute_hamiltonian(
        state_at_ignition[1], state_at_ignition[2], state_at_ignition[3],
        r.F_THRUST_2, state_at_ignition[4],
        pmp_control_law(lambda0_v, lambda0_g, state_at_ignition[2]),
        lambda0_r, lambda0_v, lambda0_g,
    )

    # ------------------------------------------------------------------
    # Arc 1: thrust  (t_ignition → t_ignition + t_coast_start)
    # ------------------------------------------------------------------
    t_arc1_end = t_ignition + t_coast_start

    if t_coast_start > 0.01:
        sol_arc1 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2),
            t_span=(t_ignition, t_arc1_end + 0.5),
            y0=aug_state_ign,
            max_step=0.5,
            events=_event_crash,
            atol=1e-8,
        )
        if len(sol_arc1.t_events[0]) > 0:
            return {
                'crashed': True,
                'state_final': None,
                'H_burn_start': H_burn_start,
                'H_coast_end': 0.0, 'H_burn_end': 0.0,
                't_f': sol_arc1.t_events[0][0], 't_cf': 0.0,
                't_stage2_start': t2_start, 't_ignition': t_ignition,
                't_stage1': t_stage1, 'y_stage1': y_stage1,
            }
        aug_state_arc2 = list(sol_arc1.y[:, -1])
        t_arc2_start = float(sol_arc1.t[-1])
    else:
        aug_state_arc2 = aug_state_ign
        t_arc2_start = t_ignition

    # Record t_cf (coast start time) for objective J = t_f − t_cf
    t_cf = t_arc2_start

    # ------------------------------------------------------------------
    # Arc 2: coast  (t_arc2_start → t_arc2_start + delta_tc)
    # ------------------------------------------------------------------
    t_arc2_end = t_arc2_start + delta_tc

    if delta_tc > 0.01:
        sol_arc2 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2),
            t_span=(t_arc2_start, t_arc2_end + 0.5),
            y0=aug_state_arc2,
            max_step=0.5,
            events=_event_crash,
            atol=1e-8,
        )
        if len(sol_arc2.t_events[0]) > 0:
            return {
                'crashed': True,
                'state_final': None,
                'H_burn_start': H_burn_start,
                'H_coast_end': 0.0, 'H_burn_end': 0.0,
                't_f': sol_arc2.t_events[0][0], 't_cf': t_cf,
                't_stage2_start': t2_start, 't_ignition': t_ignition,
                't_stage1': t_stage1, 'y_stage1': y_stage1,
            }
        aug_state_arc3 = list(sol_arc2.y[:, -1])
        t_arc3_start = float(sol_arc2.t[-1])

        # H at end of coast arc (for transversality)
        H_coast_end = compute_hamiltonian(
            aug_state_arc3[1], aug_state_arc3[2], aug_state_arc3[3],
            0.0, aug_state_arc3[4],
            pmp_control_law(aug_state_arc3[6], aug_state_arc3[7], aug_state_arc3[2]),
            aug_state_arc3[5], aug_state_arc3[6], aug_state_arc3[7],
        )
    else:
        aug_state_arc3 = aug_state_arc2
        t_arc3_start = t_arc2_start
        H_coast_end = H_burn_start   # no coast → same Hamiltonian

    # ------------------------------------------------------------------
    # Arc 3: thrust  (t_arc3_start → t_arc3_start + t_arc3_burn)
    # ------------------------------------------------------------------
    t_arc3_end = t_arc3_start + t_arc3_burn

    if t_arc3_burn > 0.01:
        sol_arc3 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2),
            t_span=(t_arc3_start, t_arc3_end + 0.5),
            y0=aug_state_arc3,
            max_step=0.5,
            events=_event_crash,
            atol=1e-8,
        )
        if len(sol_arc3.t_events[0]) > 0:
            return {
                'crashed': True,
                'state_final': None,
                'H_burn_start': H_burn_start,
                'H_coast_end': H_coast_end, 'H_burn_end': 0.0,
                't_f': sol_arc3.t_events[0][0], 't_cf': t_cf,
                't_stage2_start': t2_start, 't_ignition': t_ignition,
                't_stage1': t_stage1, 'y_stage1': y_stage1,
            }
        aug_final = sol_arc3.y[:, -1]
        t_f = float(sol_arc3.t[-1])
    else:
        aug_final = np.array(aug_state_arc3)
        t_f = t_arc3_start

    state_final = aug_final[:5]

    # H at end of burn (for transversality)
    H_burn_end = compute_hamiltonian(
        aug_final[1], aug_final[2], aug_final[3],
        r.F_THRUST_2, aug_final[4],
        pmp_control_law(aug_final[6], aug_final[7], aug_final[2]),
        aug_final[5], aug_final[6], aug_final[7],
    )

    if verbose:
        h_f = state_final[1] - c.R_EARTH
        print(f"  Stage 2 end: t={t_f:.1f}s, h={h_f/1e3:.1f}km, "
              f"v={state_final[2]:.0f}m/s, gam={np.rad2deg(state_final[3]):.2f}deg")
        print(f"  H_burn_start={H_burn_start:.4f}  H_coast_end={H_coast_end:.4f}  "
              f"H_burn_end={H_burn_end:.4f}")

    return {
        'crashed': False,
        'state_final': state_final,
        'H_burn_start': H_burn_start,
        'H_coast_end':  H_coast_end,
        'H_burn_end':   H_burn_end,
        't_f':  t_f,
        't_cf': t_cf,
        't_stage2_start': t2_start,
        't_ignition':     t_ignition,
        't_stage1': t_stage1,
        'y_stage1': y_stage1,
    }


# ===========================================================================
# Augmented objective function  (Eq. 39)
# ===========================================================================

def compute_augmented_objective(result):
    """
    Augmented objective J' (Eq. 39 of the paper):

        J' = J  +  s1|Δh|  +  s2|ΔV|  +  s3|Δγ|
               +  s4|H_f^burn + H_f^coast − H_0^burn|  +  C

    where:
        J         = t_f − t_cf                (impulse duration — minimised)
        Δh        = h_f − h_target
        ΔV        = V_f − V_circular(h_f)
        Δγ        = γ_f − 0  (target: horizontal at injection)
        C         = 10^20 if trajectory constraints violated, else 0

    The penalty weights s1…s4 come from ``simulation_parameters``.

    Parameters
    ----------
    result : dict   Output of ``run_indirect_trajectory``

    Returns
    -------
    J_prime : float  Augmented objective value
    """
    CRASH_PENALTY = 1e20

    if result['crashed'] or result['state_final'] is None:
        return CRASH_PENALTY

    state  = result['state_final']
    r_val  = state[1]
    v_f    = state[2]
    g_f    = state[3]   # flight path angle at final time
    t_f    = result['t_f']
    t_cf   = result['t_cf']

    # --- Primary objective: burn duration ---
    J = t_f - t_cf    # Eq. 27

    # --- Target orbital parameters ---
    r_target  = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    v_circular = np.sqrt(c.MU_EARTH / r_target)
    gamma_target = 0.0   # horizontal injection

    # --- Terminal constraint errors (Eq. 37) ---
    delta_h = r_val  - r_target           # altitude error  [m]
    delta_v = v_f    - v_circular         # velocity error  [m/s]
    delta_g = g_f    - gamma_target       # FPA error       [rad]

    # --- Transversality condition (Eq. 38) ---
    H0  = result['H_burn_start']
    Hfc = result['H_coast_end']
    Hfb = result['H_burn_end']
    transversality = Hfb + Hfc - H0      # should be 0 at optimum

    # --- Trajectory constraint penalty C (Eq. 40) ---
    h_final = r_val - c.R_EARTH
    C = 0.0
    if h_final < 0:                       # below ground
        C = CRASH_PENALTY
    elif v_f < 0:                         # negative velocity (unphysical)
        C = CRASH_PENALTY

    # --- Augmented objective (Eq. 39) ---
    J_prime = (J
               + sim_params.PENALTY_W_ALTITUDE  * abs(delta_h)
               + sim_params.PENALTY_W_VELOCITY  * abs(delta_v)
               + sim_params.PENALTY_W_FPA       * abs(delta_g)
               + sim_params.PENALTY_W_TRANSVERS * abs(transversality)
               + C)

    return float(J_prime)


# ===========================================================================
# PyGMO-compatible problem class
# ===========================================================================

class IndirectTPBVPProblem:
    """
    User-defined problem (UDP) for PyGMO's PSO algorithm.

    Decision vector  x (7 variables):
        [lambda0_r, lambda0_v, lambda0_g, delta_tc, delta_tr_pct,
         coast_start_pct, gamma_p]

    Objective: minimise J' (augmented objective, Eq. 39).
    """

    def fitness(self, x):
        (lambda0_r, lambda0_v, lambda0_g,
         delta_tc, delta_tr_pct, coast_start_pct, gamma_p) = x
        result = run_indirect_trajectory(
            lambda0_r, lambda0_v, lambda0_g,
            delta_tc, delta_tr_pct, coast_start_pct, gamma_p,
        )
        return [compute_augmented_objective(result)]

    def get_bounds(self):
        return (sim_params.PSO_LB, sim_params.PSO_UB)

    def get_nobj(self):
        return 1


# ===========================================================================
# PSO runner
# ===========================================================================

def run_pso_optimization(verbose=True):
    """
    Run the PSO optimisation as described in the paper (Sect. 4.2.2).

    Attempts to use PyGMO (``pygmo``) first.  If not installed, falls back to
    ``scipy.optimize.differential_evolution`` with a comparable population size
    and generation count.

    Parameters
    ----------
    verbose : bool   Print progress and final result if True.

    Returns
    -------
    optimal_params : list  [lambda0_r, lambda0_v, lambda0_g, delta_tc,
                             delta_tr_pct, coast_start_pct, gamma_p]
    J_optimal      : float  Best augmented objective value achieved
    """
    n_particles = sim_params.PSO_N_PARTICLES
    n_gen       = sim_params.PSO_MAX_GENERATIONS
    lb          = sim_params.PSO_LB
    ub          = sim_params.PSO_UB
    bounds_list = list(zip(lb, ub))

    if verbose:
        print("\n" + "=" * 60)
        print("INDIRECT PMP TRAJECTORY OPTIMISATION — PSO")
        print("=" * 60)
        print(f"  Particles : {n_particles}")
        print(f"  Max gen.  : {n_gen}")
        print(f"  Bounds    : {bounds_list}")
        print("=" * 60 + "\n")

    t_start = time.time()

    # ------------------------------------------------------------------
    # Try PyGMO first (paper's algorithm)
    # ------------------------------------------------------------------
    try:
        import pygmo as pg  # type: ignore

        prob = pg.problem(IndirectTPBVPProblem())
        algo = pg.algorithm(pg.pso(
            gen      = n_gen,
            omega    = sim_params.PSO_OMEGA,
            eta1     = sim_params.PSO_C1,
            eta2     = sim_params.PSO_C2,
            max_vel  = sim_params.PSO_VMAX,
        ))
        if verbose:
            algo.set_verbosity(50)   # print every 50 generations

        pop = pg.population(prob, size=n_particles)
        pop = algo.evolve(pop)

        best_x = list(pop.champion_x)
        best_f = float(pop.champion_f[0])

        if verbose:
            print(f"\n[PyGMO PSO] Finished in {time.time()-t_start:.1f}s")
            print(f"  Best J' = {best_f:.4f}")
            _print_solution(best_x, best_f)

        return best_x, best_f

    except ImportError:
        warnings.warn(
            "pygmo not found - falling back to scipy.differential_evolution. "
            "PSO-specific parameters (c1, c2, omega) are not reproduced exactly.",
            RuntimeWarning,
            stacklevel=2,
        )

    # ------------------------------------------------------------------
    # Fallback: scipy Differential Evolution
    # ------------------------------------------------------------------
    from scipy.optimize import differential_evolution  # type: ignore

    def _obj(x):
        result = run_indirect_trajectory(*x)
        return compute_augmented_objective(result)

    de_result = differential_evolution(
        _obj,
        bounds=bounds_list,
        maxiter=n_gen,
        popsize=max(1, n_particles // 7),   # DE popsize = multiplier of n_vars
        seed=42,
        disp=verbose,
        tol=1e-9,
        mutation=(0.5, 1.0),
        recombination=0.9,
        workers=1,
    )

    best_x = list(de_result.x)
    best_f = float(de_result.fun)

    if verbose:
        print(f"\n[scipy DE] Finished in {time.time()-t_start:.1f}s")
        print(f"  Best J' = {best_f:.4f}")
        _print_solution(best_x, best_f)

    return best_x, best_f


# ===========================================================================
# Full trajectory runner for plotting
# ===========================================================================

def run_indirect_full(optimal_params, verbose=True):
    """
    Re-run the optimal indirect PMP trajectory with dense output suitable for
    plotting.  Returns data in the same format as ``rocket_ascent.run()``.

    Parameters
    ----------
    optimal_params : list/tuple  7-element vector from ``run_pso_optimization``
    verbose        : bool

    Returns
    -------
    time_full   : ndarray  Combined time array [s]
    data_full   : ndarray  State data (5 × N)   [s, r, v, γ, m]
    thrust_full : ndarray  Thrust force at each time step [N]
    alpha_full  : ndarray  Angle of attack at each time step [rad]
    t_ignition  : float    Absolute time of Stage-2 engine ignition [s]
    result      : dict     Same as ``run_indirect_trajectory`` return value
    """
    (lambda0_r, lambda0_v, lambda0_g,
     delta_tc, delta_tr_pct, coast_start_pct, gamma_p) = optimal_params

    kick_angle = gamma_p - np.pi / 2.0

    # --- Stage 1 ---
    t2_start, state2_init, t_meco, t_stage1, y_stage1, crashed = ra.run_stage1(kick_angle)

    if crashed:
        raise RuntimeError("Stage 1 crashed during full-trajectory plotting run.")

    t_ignition = t2_start + _T_IGNITION_DELAY

    # Strip optional lat/heading — PMP ODE needs exactly [s, r, v, gamma, m]
    state2_init = state2_init[:5]

    # timing
    T_burn_total  = (delta_tr_pct   / 100.0) * _T_MAX_2
    t_coast_start = (coast_start_pct / 100.0) * T_burn_total
    t_arc3_burn   = T_burn_total - t_coast_start

    n_state = 5   # always 5 after stripping
    aug0_preig = list(state2_init) + [0.0, 0.0, 0.0]

    _dt = 0.5   # output step for plotting

    def _make_teval(t0, t1):
        return np.arange(t0, t1, _dt)

    # --- Pre-ignition coast ---
    sol_pre = solve_ivp(
        lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2),
        t_span=(t2_start, t_ignition + 0.5),
        y0=aug0_preig,
        t_eval=_make_teval(t2_start, t_ignition),
        max_step=0.5, events=_event_crash, atol=1e-8,
    )
    state_at_ign = sol_pre.y[:n_state, -1].copy()
    aug_ign = list(state_at_ign) + [lambda0_r, lambda0_v, lambda0_g]

    # --- Arc 1 (thrust) ---
    t_arc1_end = t_ignition + t_coast_start
    if t_coast_start > 0.01:
        sol1 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2),
            t_span=(t_ignition, t_arc1_end + 0.5),
            y0=aug_ign,
            t_eval=_make_teval(t_ignition, t_arc1_end),
            max_step=0.5, events=_event_crash, atol=1e-8,
        )
        aug_arc2 = list(sol1.y[:, -1])
        t_arc2_start = float(sol1.t[-1])
    else:
        sol1 = None
        aug_arc2 = aug_ign
        t_arc2_start = t_ignition

    # --- Arc 2 (coast) ---
    t_arc2_end = t_arc2_start + delta_tc
    if delta_tc > 0.01:
        sol2 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2),
            t_span=(t_arc2_start, t_arc2_end + 0.5),
            y0=aug_arc2,
            t_eval=_make_teval(t_arc2_start, t_arc2_end),
            max_step=0.5, events=_event_crash, atol=1e-8,
        )
        aug_arc3 = list(sol2.y[:, -1])
        t_arc3_start = float(sol2.t[-1])
    else:
        sol2 = None
        aug_arc3 = aug_arc2
        t_arc3_start = t_arc2_start

    # --- Arc 3 (thrust) ---
    t_arc3_end = t_arc3_start + t_arc3_burn
    if t_arc3_burn > 0.01:
        sol3 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2),
            t_span=(t_arc3_start, t_arc3_end + 0.5),
            y0=aug_arc3,
            t_eval=_make_teval(t_arc3_start, t_arc3_end),
            max_step=0.5, events=_event_crash, atol=1e-8,
        )
    else:
        sol3 = None

    # --- Assemble Stage 2 data ---
    sols2_list  = [sol_pre]
    thrusts2    = [0.0]   # per-arc thrust value
    if sol1  is not None: sols2_list.append(sol1);  thrusts2.append(r.F_THRUST_2)
    if sol2  is not None: sols2_list.append(sol2);  thrusts2.append(0.0)
    if sol3  is not None: sols2_list.append(sol3);  thrusts2.append(r.F_THRUST_2)

    t_s2_parts  = []
    y_s2_parts  = []   # shape [5 × n_i] (physical state only)
    th_s2_parts = []
    al_s2_parts = []

    for sol, F in zip(sols2_list, thrusts2):
        if sol is None or len(sol.t) == 0:
            continue
        t_s2_parts.append(sol.t)
        y_s2_parts.append(sol.y[:5, :])       # physical state rows 0-4

        # alpha from costates at each point
        alphas = np.array([
            pmp_control_law(sol.y[6, i], sol.y[7, i], sol.y[2, i])
            for i in range(sol.y.shape[1])
        ])
        al_s2_parts.append(alphas)
        th_s2_parts.append(np.full(len(sol.t), F))

    t_stage2_full  = np.concatenate(t_s2_parts)
    y_stage2_full  = np.concatenate(y_s2_parts, axis=1)
    thrust_stage2  = np.concatenate(th_s2_parts)
    alpha_stage2   = np.concatenate(al_s2_parts)

    # --- Combine Stage 1 and Stage 2 ---
    # Stage 1 state has n_state columns; pad or trim to 5 rows
    y1 = y_stage1[:5, :]

    time_full   = np.concatenate([t_stage1, t_stage2_full])
    data_full   = np.concatenate([y1, y_stage2_full], axis=1)
    thrust_full = np.concatenate([
        np.array(ra.thrust_history)[:len(t_stage1)]
        if len(ra.thrust_history) >= len(t_stage1)
        else np.zeros(len(t_stage1)),
        thrust_stage2,
    ])
    alpha_full  = np.concatenate([
        np.array(ra.alpha_history)[:len(t_stage1)]
        if len(ra.alpha_history) >= len(t_stage1)
        else np.zeros(len(t_stage1)),
        alpha_stage2,
    ])

    if verbose:
        sf = y_stage2_full[:, -1]
        print(f"\n[Full run] t_end={time_full[-1]:.1f}s, "
              f"h={(sf[1]-c.R_EARTH)/1e3:.1f}km, "
              f"v={sf[2]:.0f}m/s, gam={np.rad2deg(sf[3]):.2f}deg")

    # Also return the result dict (for transversality / objective info)
    result = run_indirect_trajectory(
        lambda0_r, lambda0_v, lambda0_g,
        delta_tc, delta_tr_pct, coast_start_pct, gamma_p,
        verbose=verbose,
    )

    return time_full, data_full, thrust_full, alpha_full, t_ignition, result


def _print_solution(x, J_prime):
    """Pretty-print the optimal PSO solution."""
    (lambda0_r, lambda0_v, lambda0_g,
     delta_tc, delta_tr_pct, coast_start_pct, gamma_p) = x
    print("\nOptimal parameters:")
    print(f"  lam0_r        = {lambda0_r:.6f}")
    print(f"  lam0_v        = {lambda0_v:.6f}")
    print(f"  lam0_gam      = {lambda0_g:.6f}")
    print(f"  Coast time    = {delta_tc:.2f} s")
    print(f"  Burn %        = {delta_tr_pct:.2f} %  of T_max = {_T_MAX_2:.1f} s")
    print(f"  Coast start % = {coast_start_pct:.2f} %")
    print(f"  Pitch angle   = {np.rad2deg(gamma_p):.4f} deg  ({gamma_p:.6f} rad)")
    print(f"  J_prime       = {J_prime:.4f}")

    # Verify final trajectory
    result = run_indirect_trajectory(*x, verbose=True)
    if not result['crashed'] and result['state_final'] is not None:
        sf = result['state_final']
        h_f = (sf[1] - c.R_EARTH) / 1e3
        v_f = sf[2]
        g_f = np.rad2deg(sf[3])
        r_t = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
        v_c = np.sqrt(c.MU_EARTH / r_t)
        dh  = h_f - sim_params.TARGET_ORBITAL_ALTITUDE / 1e3
        dv  = v_f - v_c
        print(f"\nFinal state vs. target:")
        print(f"  Altitude : {h_f:.2f} km  (target {sim_params.TARGET_ORBITAL_ALTITUDE/1e3:.0f} km, delta={dh:.2f} km)")
        print(f"  Velocity : {v_f:.2f} m/s (circular {v_c:.2f} m/s, delta={dv:.2f} m/s)")
        print(f"  FPA      : {g_f:.4f} deg  (target 0.0 deg)")
        H_trans = result['H_burn_end'] + result['H_coast_end'] - result['H_burn_start']
        print(f"  Transversality: {H_trans:.6f}  (target ~0)")
