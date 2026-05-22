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

    Penalized objective (normalised, well-conditioned):
        J' = (t_seco - t_coast) / t_max_s2          ← j_impulse fraction [0,1]
             + s_alt   * |h_f - h_target|/h_target   ← alt error fraction [0,1]
             + s_vel   * |V_f - V_target|/V_target   ← vel error fraction [0,1]
             + s_gamma * |gamma_f| / (π/2)            ← gamma fraction [0,1]
             + s_ham   * |H_f_last + H_f_coast - H_0_last|
             + 1e20    if (crash | NaN | no valid trajectory)

    All four primary terms are normalised to [0, 1] so PSO_PAPER_PENALTY_GAMMA = 1e3
    (same as alt/vel) is valid; the old 1e5 weight on raw radians over-penalised gamma
    by 100-1000x and caused the swarm to stagnate on the gamma constraint alone.
=============================================== """

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import time
import numpy as np
import pyswarms as ps
from pyswarms.single import GlobalBestPSO

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r_specs
from Input_File import simulation_parameters as sim_params
from Simulation import rocket_ascent as ra
import Guidance.pso_paper_guidance as pso_paper_mod

# Max Stage-2 burn duration (used to normalise j_impulse to [0, 1])
_MDOT_S2       = r_specs.F_THRUST_2 / (r_specs.ISP_2 * c.G_0)
_T_MAX_S2_BURN = r_specs.M_PROP_2 / _MDOT_S2   # ≈ 338.6 s for Falcon 9 specs


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

    # pitch_program_linear is short-circuited in paper mode — the pitch maneuver
    # is an instantaneous gamma state-jump inside run() at PSO_PAPER_T_PITCHOVER.
    # The kick-angle argument is therefore irrelevant; pass 0.0 for clarity.
    ra.SINGLE_BURN_FULL_SIMULATION = False
    try:
        (time_arr, data, alt_stopped, delta_v, m_prop_used,
         _thrust_d, _time_t, _alpha_d, _alpha_t,
         _cor, _cent) = ra.run(0.0)
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

    # --- Extract state AT SECO, not at the end of the long post-SECO coast ---
    # After paper-mode SECO the simulation coasts for thousands of seconds
    # (interrupt_stage_2_burnt does not fire when propellant reserve remains).
    # Reading data[:,-1] would give a random orbital-coast state; instead find
    # the time-array index nearest to the PSO-commanded SECO time.
    if ra.pso_paper_seco_t is not None and len(time_arr) > 0:
        idx_eval = int(np.searchsorted(time_arr, ra.pso_paper_seco_t))
        idx_eval = min(idx_eval, data.shape[1] - 1)
    else:
        idx_eval = -1

    r_f     = float(data[1, idx_eval])
    v_f     = float(data[2, idx_eval])
    gamma_f = float(data[3, idx_eval])
    h_f     = r_f - c.R_EARTH

    if not np.all(np.isfinite([r_f, v_f, gamma_f])):
        return sim_params.PSO_PAPER_PENALTY_HARD

    # --- Target circular orbit state ---
    h_T = sim_params.TARGET_ORBITAL_ALTITUDE
    r_T = c.R_EARTH + h_T
    V_T = float(np.sqrt(c.MU_EARTH / r_T))

    alt_err_frac = abs(h_f - h_T) / max(h_T, 1.0)
    vel_err_frac = abs(v_f - V_T) / max(V_T, 1.0)
    gamma_err    = abs(gamma_f) / (np.pi / 2.0)   # normalised to [0, 1]

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

    # --- Impulse-duration component (paper eq. 27), normalised to [0, 1] ---
    # J = (t_seco - t_coast_end) / t_max_s2  — terminal burn fraction
    j_impulse_frac = 0.0
    if ra.pso_paper_seco_t is not None and ra.pso_paper_coast_end_t is not None:
        j_impulse_raw = max(0.0, ra.pso_paper_seco_t - ra.pso_paper_coast_end_t)
        j_impulse_frac = j_impulse_raw / max(_T_MAX_S2_BURN, 1.0)

    J_prime = (j_impulse_frac
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


# ─── Post-run diagnostic ──────────────────────────────────────────────────────

def _diagnose_best(best_pos):
    """Re-run the best particle and return a dict of trajectory diagnostics.

    Used post-PSO to expose *why* J' settled where it did — SECO state, per-term
    penalty breakdown, crash flag.  Not part of the optimisation loop.
    """
    lam_h0, lam_V0, lam_g0, gamma_p, dt_coast, coast_pct, burn_pct = [float(v) for v in best_pos]
    ra.pso_paper_lam0      = (lam_h0, lam_V0, lam_g0)
    ra.pso_paper_gamma_p   = gamma_p
    ra.pso_paper_dt_coast  = dt_coast
    ra.pso_paper_coast_pct = coast_pct
    ra.pso_paper_burn_pct  = burn_pct
    ra.SINGLE_BURN_FULL_SIMULATION = False
    try:
        (time_arr, data, *_rest) = ra.run(0.0)
    except Exception as exc:
        return {"error": str(exc)}

    if ra.CRASH_DETECTED:
        return {"crashed": True, "crash_t": ra.CRASH_TIME}
    if data is None or data.shape[1] == 0:
        return {"error": "empty trajectory"}

    idx_eval = int(np.searchsorted(time_arr, ra.pso_paper_seco_t)) if ra.pso_paper_seco_t is not None else -1
    idx_eval = min(idx_eval, data.shape[1] - 1)
    r_f      = float(data[1, idx_eval])
    v_f      = float(data[2, idx_eval])
    gamma_f  = float(data[3, idx_eval])
    h_f      = r_f - c.R_EARTH

    h_T = sim_params.TARGET_ORBITAL_ALTITUDE
    r_T = c.R_EARTH + h_T
    V_T = float(np.sqrt(c.MU_EARTH / r_T))

    alt_err_frac   = abs(h_f - h_T) / max(h_T, 1.0)
    vel_err_frac   = abs(v_f - V_T) / max(V_T, 1.0)
    gamma_err_norm = abs(gamma_f) / (np.pi / 2.0)
    j_raw          = (max(0.0, ra.pso_paper_seco_t - ra.pso_paper_coast_end_t)
                      if ra.pso_paper_seco_t is not None and ra.pso_paper_coast_end_t is not None
                      else 0.0)
    j_frac         = j_raw / max(_T_MAX_S2_BURN, 1.0)

    return {
        "crashed":        False,
        "t_seco":         ra.pso_paper_seco_t,
        "h_f_km":         h_f / 1e3,
        "v_f":            v_f,
        "gamma_f_deg":    np.rad2deg(gamma_f),
        "h_T_km":         h_T / 1e3,
        "V_T":            V_T,
        "alt_err_pct":    alt_err_frac * 100,
        "vel_err_pct":    vel_err_frac * 100,
        "gamma_err_norm": gamma_err_norm,
        "gamma_err_rad":  abs(gamma_f),
        "j_impulse_s":    j_raw,
        "j_impulse_frac": j_frac,
        "term_alt":       sim_params.PSO_PAPER_PENALTY_ALT   * alt_err_frac,
        "term_vel":       sim_params.PSO_PAPER_PENALTY_VEL   * vel_err_frac,
        "term_gamma":     sim_params.PSO_PAPER_PENALTY_GAMMA * gamma_err_norm,
        "term_j":         j_frac,
    }


def _print_diagnostic(diag):
    """Pretty-print the dict returned by _diagnose_best."""
    print("\n" + "-"*60)
    print("BEST PARTICLE DIAGNOSTIC")
    print("-"*60)
    if diag is None:
        print("  (diagnostic unavailable)")
        return
    if "error" in diag:
        print(f"  trajectory error: {diag['error']}")
        return
    if diag.get("crashed"):
        print(f"  CRASHED at t = {diag.get('crash_t')}")
        return
    print(f"  SECO time:        {diag['t_seco']:.2f} s")
    print(f"  Altitude at SECO: {diag['h_f_km']:.3f} km   (target {diag['h_T_km']:.1f} km)")
    print(f"  Velocity at SECO: {diag['v_f']:.2f} m/s    (target {diag['V_T']:.2f} m/s)")
    print(f"  γ at SECO:        {diag['gamma_f_deg']:.4f}°")
    print(f"  Terminal burn:    {diag['j_impulse_s']:.2f} s")
    print()
    print("  Penalty term breakdown:")
    print(f"    j_impulse_frac  = {diag['term_j']:.4f}")
    print(f"    s_alt   × frac  = {diag['term_alt']:.4f}   (alt err {diag['alt_err_pct']:.3f} %)")
    print(f"    s_vel   × frac  = {diag['term_vel']:.4f}   (vel err {diag['vel_err_pct']:.3f} %)")
    print(f"    s_gamma × norm  = {diag['term_gamma']:.4f}   (|γ_f| {diag['gamma_err_rad']:.4f} rad)")
    print()
    alt_ok   = diag['alt_err_pct']/100   <= sim_params.PSO_PAPER_EARLY_STOP_ALT_TOL
    vel_ok   = diag['vel_err_pct']/100   <= sim_params.PSO_PAPER_EARLY_STOP_VEL_TOL
    gamma_ok = diag['gamma_err_rad']     <= sim_params.PSO_PAPER_EARLY_STOP_GAMMA_TOL
    _mark = lambda ok: "OK " if ok else "XX "
    print(f"  Orbit tolerance:  alt {_mark(alt_ok)} vel {_mark(vel_ok)} γ {_mark(gamma_ok)}")


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

        # --- Post-mortem diagnostic of the winning particle ---
        if verbose:
            diag = _diagnose_best(best_pos)
            _print_diagnostic(diag)

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
