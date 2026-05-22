""" ===============================================
    PSO PAPER-MODE TRAJECTORY OPTIMIZER

    Particle Swarm Optimization wrapping the simulator's rocket_ascent.run(...)
    for the indirect optimal-control approach from Morgado, Marta, Gil (2022).

    Design vector x (length 7):
        x[0] = lam_h0          initial costate (paper eq. 30)        in [-1, 1]
        x[1] = lam_V0          initial costate                       in [-1, 1]
        x[2] = lam_g0          initial costate                       in [-1, 1]
        x[3] = gamma_p         initial pitch angle [rad]             in [1.54, 1.57]
        x[4] = dt_coast        Stage-2 mid-burn coast duration [s]   in [0, 3000]
        x[5] = coast_start_pct fraction of Δt_T before coast         in [0, 1]
        x[6] = last_burn_pct   fraction of m_prop_S2/ṁ to burn       in [0.70, 0.95]

    Penalized objective (paper eq. 39, sign-corrected to a minimisation):
        J' = (t_seco - t_coast)
             + s_alt   * |h_f - h_target|/h_target
             + s_vel   * |V_f - V_target|/V_target
             + s_gamma * |gamma_f|
             + s_ham   * |H_f_last + H_f_coast - H_0_last|
             + 1e20    if (crash | NaN | no valid trajectory)
=============================================== """

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import time
import numpy as np
import pyswarms as ps
from pyswarms.single import GlobalBestPSO

from Auxiliary import constants as c
from Input_File import simulation_parameters as sim_params
from Simulation import rocket_ascent as ra
import Guidance.pso_paper_guidance as pso_paper_mod


# ─── Bounds helper ────────────────────────────────────────────────────────────

def _make_bounds():
    """Build the (lb, ub) tuple for pyswarms from simulation_parameters.py."""
    lam_lo, lam_hi = sim_params.PSO_PAPER_LAMBDA_BOUNDS
    gp_lo,  gp_hi  = sim_params.PSO_PAPER_GAMMA_P_BOUNDS
    dc_lo,  dc_hi  = sim_params.PSO_PAPER_COAST_DURATION_BOUNDS
    cs_lo,  cs_hi  = sim_params.PSO_PAPER_COAST_START_PCT
    lb_lo,  lb_hi  = sim_params.PSO_PAPER_LAST_BURN_PCT
    lb = np.array([lam_lo, lam_lo, lam_lo, gp_lo, dc_lo, cs_lo, lb_lo])
    ub = np.array([lam_hi, lam_hi, lam_hi, gp_hi, dc_hi, cs_hi, lb_hi])
    return lb, ub


def _gamma_p_to_kick_angle(gamma_p):
    """Map the paper's initial pitch (rad above horizon, ≈ 1.55) to the
    simulator's kick-angle convention.

    The simulator's `current_kick_angle` is the AOA pulse magnitude during the
    pitch-over maneuver.  A small negative kick produces a small negative
    flight-path-angle deviation, which corresponds to the rocket pitching from
    vertical (π/2) down to gamma_p.  Mapping: kick = -(π/2 - gamma_p).
    """
    return -(np.pi / 2.0 - float(gamma_p))


# ─── Per-particle objective ───────────────────────────────────────────────────

def _evaluate_particle(x):
    """Run one rocket trajectory simulation with design vector x; return J'.

    Side-effect: writes the design vector into ra module globals before
    calling ra.run(...) so that the simulator's per-step EOM picks them up.
    """
    lam_h0, lam_V0, lam_g0, gamma_p, dt_coast, coast_pct, burn_pct = [float(v) for v in x]

    # --- Push design vector into rocket_ascent module globals ---
    ra.pso_paper_lam0      = (lam_h0, lam_V0, lam_g0)
    ra.pso_paper_gamma_p   = gamma_p
    ra.pso_paper_dt_coast  = dt_coast
    ra.pso_paper_coast_pct = coast_pct
    ra.pso_paper_burn_pct  = burn_pct

    kick_angle = _gamma_p_to_kick_angle(gamma_p)

    # --- Run the simulator in optimization mode (no post-SECO circularization) ---
    ra.SINGLE_BURN_FULL_SIMULATION = False
    try:
        (time_arr, data, alt_stopped, delta_v, m_prop_used,
         _thrust_d, _time_t, _alpha_d, _alpha_t,
         _cor, _cent) = ra.run(kick_angle)
    except Exception:
        return sim_params.PSO_PAPER_PENALTY_HARD

    # Hard failure: only on actual crash or missing data.  The simulator's
    # apogee-matching check returns m_prop_used = 9999999.0 whenever the
    # achievable apogee deviates from target by more than ±0.2% — for paper
    # mode that is a normal off-target case, not a failure, and is handled
    # by the soft penalty weights on alt/vel/gamma below.
    if ra.CRASH_DETECTED:
        return sim_params.PSO_PAPER_PENALTY_HARD
    if data is None or data.shape[1] == 0:
        return sim_params.PSO_PAPER_PENALTY_HARD

    # --- Extract final state ---
    r_f     = float(data[1, -1])
    v_f     = float(data[2, -1])
    gamma_f = float(data[3, -1])
    h_f     = r_f - c.R_EARTH

    if not np.all(np.isfinite([r_f, v_f, gamma_f])):
        return sim_params.PSO_PAPER_PENALTY_HARD

    # --- Target circular orbit state ---
    h_T = sim_params.TARGET_ORBITAL_ALTITUDE
    r_T = c.R_EARTH + h_T
    V_T = float(np.sqrt(c.MU_EARTH / r_T))

    alt_err_frac = abs(h_f - h_T) / max(h_T, 1.0)
    vel_err_frac = abs(v_f - V_T) / max(V_T, 1.0)
    gamma_err    = abs(gamma_f)

    # --- Transversality residual (paper eq. 38) — best-effort, optional ---
    # H_0_last : Hamiltonian at the start of the *final* thrusting arc
    # H_f_last : at SECO
    # H_f_coast : at start of coast (≡ end of pre-coast arc)
    ham_residual = 0.0
    try:
        idx_seco       = int(np.searchsorted(time_arr, ra.pso_paper_seco_t))        if ra.pso_paper_seco_t        is not None else -1
        idx_coast_end  = int(np.searchsorted(time_arr, ra.pso_paper_coast_end_t))   if ra.pso_paper_coast_end_t   is not None else -1
        idx_coast_start= int(np.searchsorted(time_arr, ra.pso_paper_coast_start_t)) if ra.pso_paper_coast_start_t is not None else -1

        n_t = data.shape[1]
        cs_off = ra._paper_costate_offset(data.shape[0])
        if (cs_off is not None
                and 0 <= idx_seco        < n_t
                and 0 <= idx_coast_end   < n_t
                and 0 <= idx_coast_start < n_t):
            def _ham_at(idx, thrust):
                r_i, v_i, g_i, m_i = data[1, idx], data[2, idx], data[3, idx], data[4, idx]
                lh, lv, lg = data[cs_off, idx], data[cs_off+1, idx], data[cs_off+2, idx]
                a_i = pso_paper_mod.steering_from_costates(lv, lg, v_i)
                return pso_paper_mod.hamiltonian(v_i, g_i, r_i, lh, lv, lg,
                                                 a_i, thrust, m_i,
                                                 c.MU_EARTH, c.R_EARTH)
            H_f_last  = _ham_at(idx_seco,        0.0)  # at SECO thrust is just cut
            H_f_coast = _ham_at(idx_coast_start, 0.0)  # entering coast
            H_0_last  = _ham_at(idx_coast_end,   0.0)  # reigniting after coast
            ham_residual = abs(H_f_last + H_f_coast - H_0_last)
    except Exception:
        ham_residual = 0.0

    # --- Impulse-duration component (paper eq. 27) ---
    # J = t_f - t_cf ≈ duration of thrusting arcs (post-coast portion)
    j_impulse = 0.0
    if ra.pso_paper_seco_t is not None and ra.pso_paper_coast_end_t is not None:
        j_impulse = max(0.0, ra.pso_paper_seco_t - ra.pso_paper_coast_end_t)

    J_prime = (j_impulse
               + sim_params.PSO_PAPER_PENALTY_ALT   * alt_err_frac
               + sim_params.PSO_PAPER_PENALTY_VEL   * vel_err_frac
               + sim_params.PSO_PAPER_PENALTY_GAMMA * gamma_err
               + sim_params.PSO_PAPER_PENALTY_HAM   * ham_residual)
    return float(J_prime)


def _swarm_objective(X):
    """Vectorised objective dispatch required by pyswarms (shape (n_particles, n_dims))."""
    out = np.empty(X.shape[0], dtype=float)
    for i in range(X.shape[0]):
        out[i] = _evaluate_particle(X[i])
    return out


# ─── Public entry point ───────────────────────────────────────────────────────

def find_pso_paper_trajectory(verbose=True):
    """Run PSO over the paper's 7-dim design space and return the best particle.

    Returns
    -------
    best_x : np.ndarray of shape (7,)
        Optimal (lam_h0, lam_V0, lam_g0, gamma_p, dt_coast, coast_pct, burn_pct).
    best_cost : float
        Value of the penalized objective at best_x.
    """
    # --- Pseudo-force override (paper sec 3.2.2) ---
    saved_pseudo = sim_params.INCLUDE_PSEUDO_FORCES
    if getattr(sim_params, "PSO_PAPER_FORCE_DISABLE_PSEUDO", True):
        sim_params.INCLUDE_PSEUDO_FORCES = False

    saved_events_print = sim_params.EVENTS_PRINT
    sim_params.EVENTS_PRINT = False  # avoid spamming during swarm evaluation

    try:
        lb, ub = _make_bounds()
        bounds = (lb, ub)

        options = {
            "c1": sim_params.PSO_PAPER_C1,
            "c2": sim_params.PSO_PAPER_C2,
            "w":  sim_params.PSO_PAPER_W,
        }

        if verbose:
            print("\n" + "="*60)
            print("PSO PAPER-MODE TRAJECTORY OPTIMIZATION")
            print("="*60)
            print(f"  Particles:  {sim_params.PSO_PAPER_POPULATION}")
            print(f"  Iterations: {sim_params.PSO_PAPER_ITERATIONS}")
            print(f"  Bounds (lo): {np.array2string(lb, precision=4)}")
            print(f"  Bounds (hi): {np.array2string(ub, precision=4)}")
            print(f"  Pseudo-forces forcibly OFF: "
                  f"{getattr(sim_params, 'PSO_PAPER_FORCE_DISABLE_PSEUDO', True)}")

        optimizer = GlobalBestPSO(
            n_particles=sim_params.PSO_PAPER_POPULATION,
            dimensions=7,
            options=options,
            bounds=bounds,
        )

        t0 = time.time()
        best_cost, best_pos = optimizer.optimize(
            _swarm_objective,
            iters=sim_params.PSO_PAPER_ITERATIONS,
            verbose=verbose,
        )
        wall = time.time() - t0

        if verbose:
            print("\n" + "-"*60)
            print(f"PSO finished in {wall:.2f}s")
            print(f"  Best cost: {best_cost:.6e}")
            print(f"  Best x:    {np.array2string(best_pos, precision=6)}")
            names = ["lam_h0", "lam_V0", "lam_g0", "gamma_p", "dt_coast",
                     "coast_pct", "burn_pct"]
            for n, v in zip(names, best_pos):
                print(f"    {n:>10} = {v: .6f}")

        # --- Inject the winner into ra globals so the downstream full-sim run
        #     uses the same design vector ---
        ra.pso_paper_lam0      = (float(best_pos[0]),
                                  float(best_pos[1]),
                                  float(best_pos[2]))
        ra.pso_paper_gamma_p   = float(best_pos[3])
        ra.pso_paper_dt_coast  = float(best_pos[4])
        ra.pso_paper_coast_pct = float(best_pos[5])
        ra.pso_paper_burn_pct  = float(best_pos[6])

        return np.array(best_pos), float(best_cost)
    finally:
        sim_params.INCLUDE_PSEUDO_FORCES = saved_pseudo
        sim_params.EVENTS_PRINT = saved_events_print


def kick_angle_from_best(best_pos):
    """Convenience: extract simulator kick angle from a PSO winning vector."""
    return _gamma_p_to_kick_angle(best_pos[3])
