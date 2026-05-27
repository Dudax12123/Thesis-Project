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
from pyswarms.single import GlobalBestPSO, LocalBestPSO

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r_specs
from Input_File import simulation_parameters as sim_params
from Simulation import rocket_ascent as ra
import Guidance.pso_paper_guidance as pso_paper_mod
import Guidance.peg_guidance_new as peg_new_mod

# Max Stage-2 burn duration (used to normalise j_impulse to [0, 1])
_MDOT_S2       = r_specs.F_THRUST_2 / (r_specs.ISP_2 * c.G_0)
_T_MAX_S2_BURN = r_specs.M_PROP_2 / _MDOT_S2   # ≈ 338.6 s for Falcon 9 specs


# ─── Bounds helper ────────────────────────────────────────────────────────────

def _make_bounds():
    """Build the (lb, ub) tuple for pyswarms from simulation_parameters.py."""
    lh_lo, lh_hi = sim_params.PSO_PAPER_LAM_H_BOUNDS
    lv_lo, lv_hi = sim_params.PSO_PAPER_LAM_V_BOUNDS
    lg_lo, lg_hi = sim_params.PSO_PAPER_LAM_G_BOUNDS
    gp_lo, gp_hi = sim_params.PSO_PAPER_GAMMA_P_BOUNDS
    dc_lo, dc_hi = sim_params.PSO_PAPER_COAST_DURATION_BOUNDS
    cs_lo, cs_hi = sim_params.PSO_PAPER_COAST_START_PCT
    lb_lo, lb_hi = sim_params.PSO_PAPER_LAST_BURN_PCT
    lb = np.array([lh_lo, lv_lo, lg_lo, gp_lo, dc_lo, cs_lo, lb_lo])
    ub = np.array([lh_hi, lv_hi, lg_hi, gp_hi, dc_hi, cs_hi, lb_hi])
    return lb, ub


def _make_init_pos(lb, ub, n_particles):
    """Return an (n_particles, 7) initial-position matrix, or None for default init.

    Population layout (all rows clipped to [lb, ub]):
      [0 : n_hand]              Gaussian cloud around PSO_PAPER_WARM_START_SEED
      [n_hand : n_hand+n_peg]   Gaussian cloud around peg_new-derived costate seed
      [n_hand+n_peg : end]      Uniform random within bounds

    The peg_new seed block is only built when PSO_PAPER_PEG_SEED_ENABLED is True
    and the trial run successfully reaches Stage-2 ignition.  If the trial fails,
    the slot falls back to uniform random without raising an error.
    """
    if not getattr(sim_params, "PSO_PAPER_WARM_START_ENABLED", False):
        return None

    rng_seed = getattr(sim_params, "PSO_PAPER_WARM_START_SEED_RNG", None)
    rng = np.random.default_rng(rng_seed)

    init = np.empty((n_particles, lb.size), dtype=float)
    cursor = 0  # next unfilled row

    # ── Hand-tuned Gaussian cloud ──────────────────────────────────────────────
    seed   = np.array(sim_params.PSO_PAPER_WARM_START_SEED, dtype=float)
    n_hand = min(int(sim_params.PSO_PAPER_WARM_START_N_SEEDS), n_particles)
    if n_hand > 0:
        sigma = float(sim_params.PSO_PAPER_WARM_START_JITTER) * (ub - lb)
        init[:n_hand] = np.clip(
            seed + rng.normal(0.0, sigma, size=(n_hand, lb.size)),
            lb, ub,
        )
        cursor = n_hand

    # ── peg_new-derived Gaussian cloud ─────────────────────────────────────────
    n_peg = min(
        int(getattr(sim_params, "PSO_PAPER_PEG_SEED_N_PARTICLES", 0)),
        n_particles - cursor,
    )
    if getattr(sim_params, "PSO_PAPER_PEG_SEED_ENABLED", False) and n_peg > 0:
        # Trial run uses the same γ_p as the hand-tuned seed for consistency.
        gamma_p_trial = float(sim_params.PSO_PAPER_WARM_START_SEED[3])
        snap    = _run_peg_new_trial(gamma_p_trial)
        peg_vec = _peg_to_pso_seed(snap, gamma_p_trial)
        if peg_vec is not None:
            sigma_peg = float(getattr(sim_params, "PSO_PAPER_PEG_SEED_JITTER", 0.03)) * (ub - lb)
            init[cursor : cursor + n_peg] = np.clip(
                peg_vec + rng.normal(0.0, sigma_peg, size=(n_peg, lb.size)),
                lb, ub,
            )
            cursor += n_peg
        else:
            print("[peg_new seed] Trial failed — falling back to uniform random for this block.")

    # ── Remainder: uniform random ──────────────────────────────────────────────
    if n_particles - cursor > 0:
        init[cursor:] = rng.uniform(lb, ub, size=(n_particles - cursor, lb.size))

    return init


def _make_restart_init_pos(elite_pos, lb, ub, n_particles, rng=None):
    """Build init_pos for a stagnation-kick restart.

    Row 0 is the current best (clipped to bounds); the remaining rows are
    uniform-random within bounds.  LocalBestPSO's ring topology will pull the
    elite's neighbours toward it while distant particles explore freely.
    """
    if rng is None:
        rng = np.random.default_rng()
    init = np.empty((n_particles, lb.size), dtype=float)
    init[0]  = np.clip(np.asarray(elite_pos, dtype=float), lb, ub)
    if n_particles > 1:
        init[1:] = rng.uniform(lb, ub, size=(n_particles - 1, lb.size))
    return init


def _gamma_p_to_kick_angle(gamma_p):
    """Map the paper's initial pitch (rad above horizon, ≈ 1.55) to the
    simulator's kick-angle convention.

    The simulator's `current_kick_angle` is the AOA pulse magnitude during the
    pitch-over maneuver.  A small negative kick produces a small negative
    flight-path-angle deviation, which corresponds to the rocket pitching from
    vertical (π/2) down to gamma_p.  Mapping: kick = -(π/2 - gamma_p).
    """
    return -(np.pi / 2.0 - float(gamma_p))


# ─── peg_new-derived warm-start helpers ───────────────────────────────────────

def _run_peg_new_trial(gamma_p_seed):
    """Run a pso_paper trial trajectory and evaluate peg_new at the SEI state.

    Strategy
    --------
    We need the rocket state at Stage-2 ignition (SEI) produced by the same
    instantaneous pitch-over that the PSO paper mode uses.  Running in pure
    "peg_new" mode would use the standard 45-second triangular kick, giving a
    very different Stage-1 trajectory.  Instead we:

      1. Run ra.run(0.0) in "pso_paper" mode with the requested γ_p and
         gravity-turn costates (lam_V = −1, others = 0) so Stage 2 coasts
         ballistically — we only care about the state *at* Stage-2 ignition.
      2. Extract the state vector at t_SEI from the returned trajectory array.
      3. Call peg_new_major_loop() directly on that state.

    Returns a snapshot dict (same schema as peg_new_sei_snapshot in
    rocket_ascent) or None if Stage-2 ignition was not reached.
    """
    # --- Save state that will be overwritten ---
    prev_mode      = sim_params.GUIDANCE_MODE
    prev_pseudo    = getattr(sim_params, "INCLUDE_PSEUDO_FORCES", True)
    prev_lam0      = ra.pso_paper_lam0
    prev_gamma_p   = ra.pso_paper_gamma_p
    prev_dt_coast  = ra.pso_paper_dt_coast
    prev_coast_pct = ra.pso_paper_coast_pct
    prev_burn_pct  = ra.pso_paper_burn_pct

    try:
        sim_params.GUIDANCE_MODE = "pso_paper"
        if getattr(sim_params, "PSO_PAPER_FORCE_DISABLE_PSEUDO", True):
            sim_params.INCLUDE_PSEUDO_FORCES = False

        # Gravity-turn costates: α = atan2(0/V, 1) = 0 throughout Stage 2.
        # Stage 2 trajectory doesn't matter — only the SEI state does.
        ra.pso_paper_lam0      = (0.0, -1.0, 0.0)
        ra.pso_paper_gamma_p   = gamma_p_seed
        # Use the existing warm-start seed for burn scheduling so the run
        # terminates cleanly rather than timing out.
        ws = sim_params.PSO_PAPER_WARM_START_SEED
        ra.pso_paper_dt_coast  = float(ws[4])
        ra.pso_paper_coast_pct = float(ws[5])
        ra.pso_paper_burn_pct  = float(ws[6])
        ra.SINGLE_BURN_FULL_SIMULATION = False

        try:
            result = ra.run(0.0)   # kick angle unused in paper mode
        except Exception:
            return None

        time_arr, data = result[0], result[1]
        if data is None or data.shape[1] == 0:
            return None
        if ra.time_main_engine_cutoff is None:
            return None

        # --- Find state at Stage-2 ignition time ---
        t_sei = ra.time_main_engine_cutoff + r_specs.TIME_SECOND_ENGINE_IGNITION
        idx   = int(np.searchsorted(time_arr, t_sei))
        idx   = min(idx, data.shape[1] - 1)
        state_sei = data[:5, idx].copy()   # [s, r, v, gamma, m]

        if not np.all(np.isfinite(state_sei)):
            return None

        # --- Call peg_new_major_loop() directly at the SEI state ---
        Ve    = r_specs.ISP_2 * c.G_0
        r_tgt = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
        (vgo_r, vgo_theta, L0, tgo,
         t_lambda, lambda_r) = peg_new_mod.peg_new_major_loop(
             state_sei, r_tgt, c.MU_EARTH, Ve, r_specs.F_THRUST_2)

        return {
            "state":     state_sei,
            "vgo_r":     vgo_r,
            "vgo_theta": vgo_theta,
            "L0":        L0,
            "tgo":       tgo,
            "t_lambda":  t_lambda,
            "lambda_r":  lambda_r,
        }

    finally:
        sim_params.GUIDANCE_MODE         = prev_mode
        sim_params.INCLUDE_PSEUDO_FORCES = prev_pseudo
        ra.pso_paper_lam0                = prev_lam0
        ra.pso_paper_gamma_p             = prev_gamma_p
        ra.pso_paper_dt_coast            = prev_dt_coast
        ra.pso_paper_coast_pct           = prev_coast_pct
        ra.pso_paper_burn_pct            = prev_burn_pct


def _peg_to_pso_seed(snapshot, gamma_p_trial):
    """Convert a peg_new SEI snapshot to a 7-element PSO design vector.

    Costate mapping (paper velocity-normalization convention):
        V_ref = sqrt(μ / r_T)          ← circular-orbit speed at target altitude
        V̄    = V_SEI / V_ref           ← dimensionless speed at Stage-2 ignition
        α_peg                          ← peg_new steering angle at t_epoch
        λ_V0  = −cos(α_peg)            ← ∈ [−1, 0] for |α| < 90°
        λ_γ0  = −V̄ · sin(α_peg)       ← ∈ [−1, 1] (requires expanded bounds)
        λ_h0  ≈ λ'_r · tgo · V̄ / 1e4  ← small dimensionless estimate

    Burn-scheduling estimate:
        burn_pct = clip(tgo / T_max_S2, 0.70, 0.95)
        dt_coast seeded at 0 (peg_new has no coast); PSO searches around it.
        coast_pct seeded at 0.5 (bounds midpoint).

    Returns None if snapshot is None or contains degenerate values.
    """
    if snapshot is None:
        return None

    state     = snapshot["state"]
    gamma_sei = float(state[3])
    V_sei     = float(state[2])
    tgo       = snapshot["tgo"]

    if tgo is None or tgo <= 0 or V_sei <= 0:
        return None

    V_ref = np.sqrt(c.MU_EARTH / (c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE))
    V_bar = V_sei / V_ref   # ≈ 0.4–0.6 at Stage-2 ignition

    alpha_peg = peg_new_mod.peg_new_alpha(
        t_since_epoch=0.0,
        vgo_r=snapshot["vgo_r"],
        vgo_theta=snapshot["vgo_theta"],
        L0=snapshot["L0"],
        lambda_r_prime=snapshot["lambda_r"],
        t_lambda=snapshot["t_lambda"],
        gamma=gamma_sei,
    )

    lam_V0   = float(-np.cos(alpha_peg))
    lam_g0   = float(-V_bar * np.sin(alpha_peg))
    lam_h0   = float(snapshot["lambda_r"]) * float(tgo) * V_bar / 1e4

    burn_pct = float(np.clip(float(tgo) / _T_MAX_S2_BURN, 0.70, 0.95))

    print(f"[peg_new seed] alpha_peg={np.degrees(alpha_peg):.2f} deg  "
          f"V_bar={V_bar:.3f}  burn_pct={burn_pct:.3f}  "
          f"(lam_h,lam_V,lam_g)=({lam_h0:.4f},{lam_V0:.4f},{lam_g0:.4f})")

    return np.array([lam_h0, lam_V0, lam_g0,
                     float(gamma_p_trial),
                     0.0,    # dt_coast: no coast in peg_new baseline
                     0.5,    # coast_pct: midpoint of bounds
                     burn_pct])


def _hamiltonian_residual_breakdown(time_arr, data, verbose=False):
    """Return (H_f_last, H_f_coast, H_0_last, ham_residual, H_scale, ham_residual_norm).

    Paper eq. (38) sample Hamiltonians at SECO, coast-start, coast-end
    (re-ignition) plus the natural reference scale
        H_scale = max(|H_f_last|, |H_f_coast|, |H_0_last|, 1.0)
    and the dimensionless transversality residual
        ham_residual_norm = |H_f_last + H_f_coast - H_0_last| / H_scale .

    The PSO cost uses the *normalised* residual so it is commensurate with the
    other [0, 1]-ish penalty fractions (alt_err_frac, vel_err_frac, gamma_err).
    The raw residual is kept for diagnostic transparency.

    Returns (0.0, 0.0, 0.0, 0.0, 1.0, 0.0) when the required switching times or
    costate offset are unavailable (e.g. Stage 2 never ignited).

    When verbose=True, prints the specific reason the breakdown was skipped.
    Defaults to False so the PSO inner loop is silent; the post-PSO diagnostic
    passes verbose=True so we can see what is wrong with the winning particle.
    """
    def _bail(reason):
        if verbose:
            print(f"  [H-residual breakdown skipped] {reason}")
        return 0.0, 0.0, 0.0, 0.0, 1.0, 0.0

    try:
        seco_t        = ra.pso_paper_seco_t
        coast_end_t   = ra.pso_paper_coast_end_t
        coast_start_t = ra.pso_paper_coast_start_t
        missing = []
        if seco_t        is None: missing.append("pso_paper_seco_t")
        if coast_end_t   is None: missing.append("pso_paper_coast_end_t")
        if coast_start_t is None: missing.append("pso_paper_coast_start_t")
        if missing:
            return _bail(f"switching time(s) None: {', '.join(missing)}")

        n_t = data.shape[1]
        # Cap each index at n_t - 1.  np.searchsorted returns n_t when the
        # target time is at or past time_arr[-1] — exactly what happens when
        # the trajectory log ends at SECO (the common case).  Without the cap
        # the helper bails for every such particle and term_ham silently
        # drops out of J', so the PSO never sees pressure from paper eq. 38.
        idx_seco        = min(int(np.searchsorted(time_arr, seco_t)),        n_t - 1)
        idx_coast_end   = min(int(np.searchsorted(time_arr, coast_end_t)),   n_t - 1)
        idx_coast_start = min(int(np.searchsorted(time_arr, coast_start_t)), n_t - 1)
        cs_off = ra._paper_costate_offset(data.shape[0])
        if cs_off is None:
            return _bail(f"costate offset is None (state has {data.shape[0]} rows)")
        if idx_seco < 0:
            return _bail(f"idx_seco={idx_seco} negative for seco_t={seco_t:.3f}")
        if idx_coast_end < 0:
            return _bail(f"idx_coast_end={idx_coast_end} negative for coast_end_t={coast_end_t:.3f}")
        if idx_coast_start < 0:
            return _bail(f"idx_coast_start={idx_coast_start} negative for coast_start_t={coast_start_t:.3f}")

        def _ham_at(idx, thrust):
            r_i, v_i, g_i, m_i = data[1, idx], data[2, idx], data[3, idx], data[4, idx]
            lh, lv, lg = data[cs_off, idx], data[cs_off + 1, idx], data[cs_off + 2, idx]
            a_i = pso_paper_mod.steering_from_costates(lv, lg, v_i)
            return pso_paper_mod.hamiltonian(v_i, g_i, r_i, lh, lv, lg,
                                             a_i, thrust, m_i, c.MU_EARTH)

        H_f_last  = _ham_at(idx_seco,        0.0)              # SECO — engine cut
        H_f_coast = _ham_at(idx_coast_start, 0.0)              # entering coast — engine cut
        H_0_last  = _ham_at(idx_coast_end,   r_specs.F_THRUST_2)  # re-ignition — engine on
        ham_residual = abs(H_f_last + H_f_coast - H_0_last)
        H_scale = max(abs(H_f_last), abs(H_f_coast), abs(H_0_last), 1.0)
        ham_residual_norm = ham_residual / H_scale
        return (float(H_f_last), float(H_f_coast), float(H_0_last),
                float(ham_residual), float(H_scale), float(ham_residual_norm))
    except Exception as exc:
        return _bail(f"exception: {type(exc).__name__}: {exc}")


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
    # Stage 2 must have ignited (otherwise no SECO time is set) — without it
    # the trajectory state is meaningless for orbit insertion evaluation.
    if ra.pso_paper_seco_t is None or len(time_arr) == 0:
        return sim_params.PSO_PAPER_PENALTY_HARD
    idx_eval = int(np.searchsorted(time_arr, ra.pso_paper_seco_t))
    idx_eval = min(idx_eval, data.shape[1] - 1)

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

    # Rotating-frame v_f at SECO vs rotating-frame target.
    # Earth's surface rotation provides a free ΔV boost; subtract it so the
    # PSO is not penalised for the velocity the rocket gets for free.
    V_rot = float(ra.LAUNCH_ROTATION_SPEED) if sim_params.ENABLE_EARTH_ROTATION else 0.0
    V_T   = V_T - V_rot

    alt_err_frac = abs(h_f - h_T) / max(h_T, 1.0)
    vel_err_frac = abs(v_f - V_T) / max(V_T, 1.0)
    gamma_err    = abs(gamma_f) / (np.pi / 2.0)   # normalised to [0, 1]
    # Asymmetric extra penalty for γ < 0 (descending at SECO → periapsis underground).
    # A positive γ at SECO is acceptable (ascending arc); negative γ means the rocket
    # already overshot its apoapsis, making the orbit suborbital.
    if gamma_f < 0.0:
        gamma_err += getattr(sim_params, "PSO_PAPER_PENALTY_GAMMA_NEG", 5.0) * abs(gamma_f) / (np.pi / 2.0)

    # --- Periapsis penalty: guard against suborbital / highly-elliptic orbits ---
    # Compute the osculating periapsis altitude from the SECO state.
    # Penalises any orbit whose periapsis falls below the target altitude, regardless
    # of whether γ_f is positive.  A circular orbit at h_T has peri_err = 0.
    v_t_seco = v_f * np.cos(gamma_f)           # tangential speed at SECO
    h_mom    = r_f * v_t_seco                   # specific angular momentum [m²/s]
    E_orb    = 0.5 * v_f**2 - c.MU_EARTH / r_f # specific orbital energy [m²/s²]
    if E_orb < -1.0:                            # bound orbit (E < 0)
        a_orb   = -c.MU_EARTH / (2.0 * E_orb)
        ecc_orb = np.sqrt(max(0.0, 1.0 - h_mom**2 / (c.MU_EARTH * a_orb)))
        r_peri  = a_orb * (1.0 - ecc_orb)
        h_peri  = r_peri - c.R_EARTH
        peri_err = max(0.0, h_T - h_peri) / max(h_T, 1.0)
    else:
        peri_err = 10.0   # hyperbolic trajectory — treat as maximally bad

    # --- Transversality residual (paper eq. 38) — best-effort, optional ---
    # Computation factored into _hamiltonian_residual_breakdown so the same
    # samples can be re-used by the post-PSO diagnostic.  We use the *normalised*
    # residual (|Δ|/H_scale) in J' so it is dimensionally commensurate with the
    # other [0, 1]-ish penalty fractions (alt_err_frac, vel_err_frac, gamma_err).
    (_H_f_last, _H_f_coast, _H_0_last,
     _ham_resid_raw, _H_scale, ham_residual_norm) = _hamiltonian_residual_breakdown(time_arr, data)

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
               + sim_params.PSO_PAPER_PENALTY_PERI  * peri_err
               + sim_params.PSO_PAPER_PENALTY_HAM   * ham_residual_norm)
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

    if ra.pso_paper_seco_t is None:
        return {"error": "Stage 2 SECO time not set (Stage 2 likely never ignited)"}
    idx_eval = int(np.searchsorted(time_arr, ra.pso_paper_seco_t))
    idx_eval = min(idx_eval, data.shape[1] - 1)
    r_f      = float(data[1, idx_eval])
    v_f      = float(data[2, idx_eval])
    gamma_f  = float(data[3, idx_eval])
    h_f      = r_f - c.R_EARTH

    h_T   = sim_params.TARGET_ORBITAL_ALTITUDE
    r_T   = c.R_EARTH + h_T
    V_rot = float(ra.LAUNCH_ROTATION_SPEED) if sim_params.ENABLE_EARTH_ROTATION else 0.0
    V_T   = float(np.sqrt(c.MU_EARTH / r_T)) - V_rot   # rotating-frame target

    alt_err_frac   = abs(h_f - h_T) / max(h_T, 1.0)
    vel_err_frac   = abs(v_f - V_T) / max(V_T, 1.0)
    gamma_err_norm = abs(gamma_f) / (np.pi / 2.0)
    j_raw          = (max(0.0, ra.pso_paper_seco_t - ra.pso_paper_coast_end_t)
                      if ra.pso_paper_seco_t is not None and ra.pso_paper_coast_end_t is not None
                      else 0.0)
    j_frac         = j_raw / max(_T_MAX_S2_BURN, 1.0)

    # Osculating periapsis from SECO state (same formula as in _evaluate_particle)
    v_t_seco_d = v_f * np.cos(gamma_f)
    h_mom_d    = r_f * v_t_seco_d
    E_orb_d    = 0.5 * v_f**2 - c.MU_EARTH / r_f
    if E_orb_d < -1.0:
        a_orb_d   = -c.MU_EARTH / (2.0 * E_orb_d)
        ecc_orb_d = np.sqrt(max(0.0, 1.0 - h_mom_d**2 / (c.MU_EARTH * a_orb_d)))
        r_peri_d  = a_orb_d * (1.0 - ecc_orb_d)
        h_peri_d  = r_peri_d - c.R_EARTH
        peri_err_d = max(0.0, h_T - h_peri_d) / max(h_T, 1.0)
    else:
        h_peri_d   = float("nan")
        peri_err_d = 10.0

    # --- TPBVP diagnostics (paper eq. 38 + costate end-points) ---
    # verbose=True so the post-PSO best-particle re-run reports exactly which
    # guard fires if the breakdown bails out (the PSO inner-loop call passes
    # verbose=False by default and stays silent).
    (H_f_last, H_f_coast, H_0_last,
     ham_residual, H_scale, ham_residual_norm) = _hamiltonian_residual_breakdown(time_arr, data, verbose=True)
    term_ham_val = sim_params.PSO_PAPER_PENALTY_HAM * ham_residual_norm

    lam0 = (tuple(float(v) for v in ra.pso_paper_lam0)
            if ra.pso_paper_lam0 is not None else (float("nan"),) * 3)
    cs_off = ra._paper_costate_offset(data.shape[0])
    if cs_off is not None and 0 <= idx_eval < data.shape[1]:
        lam_seco = (float(data[cs_off,     idx_eval]),
                    float(data[cs_off + 1, idx_eval]),
                    float(data[cs_off + 2, idx_eval]))
    else:
        lam_seco = (float("nan"),) * 3

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
        "h_peri_km":      h_peri_d / 1e3,
        "peri_err":       peri_err_d,
        "term_peri":      getattr(sim_params, "PSO_PAPER_PENALTY_PERI", 0.0) * peri_err_d,
        # TPBVP
        "H_f_last":       H_f_last,
        "H_f_coast":      H_f_coast,
        "H_0_last":       H_0_last,
        "ham_resid":      ham_residual,         # raw |Δ|
        "H_scale":        H_scale,              # max(|H_*|, 1)
        "ham_resid_norm": ham_residual_norm,    # |Δ| / H_scale
        "term_ham":       term_ham_val,         # s_ham × |Δ|/H_scale (what PSO sees)
        "lam0":           lam0,
        "lam_seco":       lam_seco,
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
    print(f"    s_peri  × err   = {diag['term_peri']:.4f}   (h_peri {diag['h_peri_km']:.3f} km, peri_err {diag['peri_err']:.4f})")
    print(f"    s_ham   × |Δ|/H = {diag['term_ham']:.4f}   "
          f"(|H_residual| {diag['ham_resid']:.4e}, "
          f"H_scale {diag['H_scale']:.4e}, "
          f"|Δ|/H = {diag['ham_resid_norm']:.4e})")
    print()
    print("  Hamiltonian samples (paper eq. 38; expect H_f_last + H_f_coast ≈ H_0_last):")
    print(f"    H_f_last  (SECO, F_T=0)        = {diag['H_f_last']: .4e}")
    print(f"    H_f_coast (coast start, F_T=0) = {diag['H_f_coast']: .4e}")
    print(f"    H_0_last  (re-ignition, F_T>0) = {diag['H_0_last']: .4e}")
    print()
    print("  Costates:")
    print(f"    initial (from PSO):  λ_h0={diag['lam0'][0]: .4f}  λ_V0={diag['lam0'][1]: .4f}  λ_γ0={diag['lam0'][2]: .4f}")
    print(f"    at SECO (evolved):   λ_h ={diag['lam_seco'][0]: .4e}  λ_V ={diag['lam_seco'][1]: .4e}  λ_γ ={diag['lam_seco'][2]: .4e}")
    print()
    alt_ok   = diag['alt_err_pct']/100   <= sim_params.PSO_PAPER_EARLY_STOP_ALT_TOL
    vel_ok   = diag['vel_err_pct']/100   <= sim_params.PSO_PAPER_EARLY_STOP_VEL_TOL
    gamma_ok = diag['gamma_err_rad']     <= sim_params.PSO_PAPER_EARLY_STOP_GAMMA_TOL
    _mark = lambda ok: "OK " if ok else "XX "
    print(f"  Orbit tolerance:  alt {_mark(alt_ok)} vel {_mark(vel_ok)} γ {_mark(gamma_ok)}")

    terms = {
        "alt":   diag['term_alt'],
        "vel":   diag['term_vel'],
        "gamma": diag['term_gamma'],
        "peri":  diag['term_peri'],
        "ham":   diag['term_ham'],
        "j":     diag['term_j'],
    }
    dom_term = max(terms, key=terms.get)
    verdict = ("TPBVP necessary conditions violated"
               if dom_term == "ham"
               else "PSO boundary-condition convergence (TPBVP locally consistent)")
    print(f"  Dominant J' term: {dom_term}   →   {verdict}")


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

        topo = str(getattr(sim_params, "PSO_PAPER_TOPOLOGY", "local")).lower()
        if topo == "global":
            pso_cls   = GlobalBestPSO
            topo_name = "GlobalBestPSO (star topology)"
        elif topo == "local":
            pso_cls   = LocalBestPSO
            topo_name = "LocalBestPSO (ring topology)"
        else:
            raise ValueError(
                f"PSO_PAPER_TOPOLOGY must be 'local' or 'global'; got {topo!r}"
            )

        options = {
            "c1": sim_params.PSO_PAPER_C1,
            "c2": sim_params.PSO_PAPER_C2,
            "w":  sim_params.PSO_PAPER_W,
        }
        if pso_cls is LocalBestPSO:
            options["k"] = sim_params.PSO_PAPER_K_NEIGHBORS
            options["p"] = sim_params.PSO_PAPER_P_NORM

        n_particles = sim_params.PSO_PAPER_POPULATION
        init_pos = _make_init_pos(lb, ub, n_particles)
        warm_on  = init_pos is not None

        if verbose:
            print("\n" + "="*60)
            print(f"PSO PAPER-MODE TRAJECTORY OPTIMIZATION  ({topo_name})")
            print("="*60)
            print(f"  Particles:  {n_particles}")
            print(f"  Iterations: {sim_params.PSO_PAPER_ITERATIONS}")
            if pso_cls is LocalBestPSO:
                print(f"  Neighbors (k): {sim_params.PSO_PAPER_K_NEIGHBORS}   p-norm: {sim_params.PSO_PAPER_P_NORM}")
            print(f"  Bounds (lo): {np.array2string(lb, precision=4)}")
            print(f"  Bounds (hi): {np.array2string(ub, precision=4)}")
            print(f"  Pseudo-forces forcibly OFF: "
                  f"{getattr(sim_params, 'PSO_PAPER_FORCE_DISABLE_PSEUDO', True)}")
            if warm_on:
                n_seed = min(int(sim_params.PSO_PAPER_WARM_START_N_SEEDS), n_particles)
                print(f"  Warm-start: {n_seed}/{n_particles} particles seeded "
                      f"(jitter={sim_params.PSO_PAPER_WARM_START_JITTER:g} × bound range)")
                print(f"    seed = {np.array2string(np.asarray(sim_params.PSO_PAPER_WARM_START_SEED), precision=4)}")
            else:
                print("  Warm-start: OFF (uniform-random init)")

        total_iters  = sim_params.PSO_PAPER_ITERATIONS
        kick_enabled = bool(getattr(sim_params, "PSO_PAPER_STAGNATION_ENABLED", False))
        patience     = int(getattr(sim_params, "PSO_PAPER_STAGNATION_PATIENCE", total_iters))
        rtol         = float(getattr(sim_params, "PSO_PAPER_STAGNATION_RTOL", 0.0))

        if verbose:
            if kick_enabled:
                print(f"  Stagnation kick: ON  (patience={patience} iters, ftol={rtol:g})")
            else:
                print("  Stagnation kick: OFF")

        best_cost    = np.inf
        best_pos     = None
        iters_used   = 0
        n_restarts   = 0
        current_init = init_pos   # warm-start (or None) for the very first chunk

        t0 = time.time()
        while iters_used < total_iters:
            remaining = total_iters - iters_used

            optimizer = pso_cls(
                n_particles=n_particles,
                dimensions=7,
                options=options,
                bounds=bounds,
                init_pos=current_init,
                ftol      = rtol     if kick_enabled else -np.inf,
                ftol_iter = patience if kick_enabled else 1,
            )
            cost, pos = optimizer.optimize(
                _swarm_objective,
                iters=remaining,
                verbose=verbose,
            )

            iters_this_chunk = len(optimizer.cost_history)
            iters_used += iters_this_chunk

            if cost < best_cost:
                best_cost = float(cost)
                best_pos  = np.array(pos, dtype=float)

            # Budget consumed or kicks disabled: we're done.
            if iters_this_chunk >= remaining or not kick_enabled:
                break

            # ftol fired before the budget — restart with elite.
            n_restarts += 1
            if verbose:
                print(f"\n  [STAGNATION KICK #{n_restarts}] best cost {best_cost:.4e} "
                      f"stagnated within ftol={rtol:g} for {patience} iters "
                      f"(after {iters_used}/{total_iters} iters); restarting with elite.")
            current_init = _make_restart_init_pos(best_pos, lb, ub, n_particles)

        wall = time.time() - t0

        if verbose and n_restarts > 0:
            print(f"\n  Total stagnation kicks: {n_restarts}")

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
