"""
PSO Coast Solver

4-variable PSO for all guidance modes except ``indirect_pmp``.

Optimises jointly:
    x = [delta_tc,          coast phase duration [s]
         delta_tr_pct,      Stage-2 burn as % of T_MAX_2 [%]
         coast_start_pct,   coast start as % of Stage-2 burn time [%]
         gamma_p]           pitch maneuver (kick) angle [rad]

Trajectory structure — Thrust → Coast → Thrust with direct orbit insertion
(no separate circularisation burn), identical to the indirect_pmp arc layout
but without PMP costates.

During thrust arcs the steering angle alpha is computed by the guidance mode
selected in simulation_parameters.GUIDANCE_MODE.  All guidance state that
would normally live in rocket_ascent module-level globals is carried in a
``GuidanceState`` dataclass created fresh per particle evaluation so that
PSO particle evaluations do not interfere with each other.

Objective (4 terms, no transversality):
    J' = w_J * J_nd  +  w_alt * |Δh_nd|  +  w_vel * |ΔV_nd|  +  w_fpa * |Δγ_nd|
    + CRASH_PENALTY  (if trajectory crashed)

PyGMO is required (same as indirect_pso_solver).
"""

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np
from scipy.integrate import solve_ivp

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent))

from Auxiliary import constants as c
from Auxiliary import gravity as grav
from Auxiliary import rocket_specs as r
from Input_File import simulation_parameters as sim_params
import Simulation.rocket_ascent as ra
import Guidance.apollo_guidance as apollo_guidance_module
import Guidance.simple_polynomial as simple_poly_guidance
import Guidance.linear_tangent_steering as lts_guidance
import Guidance.bilinear_tangent_steering as bts_guidance
import Guidance.cpr_guidance as cpr_guidance_module
import Guidance.peg_guidance as peg_guidance_mod
import Guidance.peg_guidance_new as peg_new_mod
import Guidance.exp_shooting_guidance as exp_shoot_mod

# ---------------------------------------------------------------------------
# State-strip helper (mirrors indirect_pso_solver._strip_to_pmp_state)
# ---------------------------------------------------------------------------

def _strip_to_pmp_state(state, lat_fallback_rad):
    """Return [s, r, v, γ, m]; velocity stays in the rotating (ground-relative)
    frame.

    Earth rotation is accounted for ONCE — in the objective velocity target
    (``v_circular = √(μ/r) − v_rot``).  No ECEF→ECI conversion is applied here,
    so the trajectory velocity and the objective target are both rotating-frame
    and the rotation credit is not double-counted.  (``lat_fallback_rad`` is
    retained for call-site compatibility.)
    """
    return np.array(state[:5], dtype=float)


# ---------------------------------------------------------------------------
# Derived Stage-2 constants (computed once at import time, same as indirect_pso_solver)
# ---------------------------------------------------------------------------
_MDOT_2 = r.F_THRUST_2 / (r.ISP_2 * c.G_0)
_T_MAX_2 = r.M_PROP_2 / _MDOT_2
_T_IGNITION_DELAY = r.TIME_SECOND_ENGINE_IGNITION - r.TIME_First_STAGE_SEPARATION

# Dry mass threshold: thrust is cut when mass drops at or below this.
_DRY_MASS_2 = r.M_STRUCTURE_2 + r.M_PAYLOAD

# ---------------------------------------------------------------------------
# Integration tolerances (same as indirect_pso_solver for consistency)
# ---------------------------------------------------------------------------
_RTOL = 1e-9
_ATOL = 1e-9
_MAX_STEP = 10.0

# Per-generation PSO convergence history (same format as LAST_PSO_HISTORY).
LAST_PSO_COAST_HISTORY = None

CRASH_PENALTY = 1e20


# ===========================================================================
# GuidanceState — per-particle mutable guidance state
# ===========================================================================

@dataclass
class GuidanceState:
    """
    Carries guidance state for one trajectory evaluation.

    Created fresh per PSO particle so evaluations do not share module globals.
    The same instance is passed through Arc 1, coast, and Arc 3, so guidance
    coefficients computed before the coast are available after it.

    ``atmosphere_exited`` is pre-set to True because Stage 2 always ignites
    above the atmosphere; this ensures guidance activates immediately when
    GUIDANCE_START_MODE == "after_atmosphere_exit".
    """
    # Common
    atmosphere_exited: bool = True
    guidance_phase_active: bool = False
    time_guidance_start: float = 0.0
    last_guidance_update_time: float = 0.0
    guidance_initial_tgo: Optional[float] = None
    guidance_coefficients: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0])

    # Apollo
    apollo_coefficients_frozen: bool = False
    apollo_freeze_time: Optional[float] = None
    apollo_previous_tgo: Optional[float] = None

    # LTS / bilinear
    lts_previous_tgo: Optional[float] = None

    # PEG
    peg_A: float = 0.0
    peg_B: float = 0.0
    peg_T: Optional[float] = None
    peg_t_epoch: Optional[float] = None
    peg_frozen: bool = False

    # PEG_new
    peg_new_vgo_r: float = 0.0
    peg_new_vgo_theta: float = 0.0
    peg_new_L0: float = 1.0
    peg_new_tgo: Optional[float] = None
    peg_new_t_lambda: float = 0.0
    peg_new_lambda_r: float = 0.0
    peg_new_t_epoch: Optional[float] = None
    peg_new_frozen: bool = False

    # exp_shooting
    exp_shoot_a: Optional[float] = None
    exp_shoot_b: Optional[float] = None
    exp_shoot_epoch: Optional[float] = None

    # CPR
    cpr_theta_initial: Optional[float] = None
    cpr_theta_dot: Optional[float] = None
    cpr_t_start: Optional[float] = None

    # Alpha / time log for dense-output post-processing
    alpha_log: List[float] = field(default_factory=list)
    time_log: List[float] = field(default_factory=list)

    # t_go log (apollo / linear_tangent / bilinear_tangent modes)
    tgo_log: List[float] = field(default_factory=list)
    tgo_time_log: List[float] = field(default_factory=list)


# ===========================================================================
# t_go helpers (Stage-2 context — main_engine_cutoff = True always here)
# ===========================================================================

def _estimate_tto_altitude(state, target_altitude):
    """Simple altitude-based t_go: Δh / v_radial."""
    r_val, v, gamma = state[1], state[2], state[3]
    v_radial = v * np.sin(gamma)
    if abs(v_radial) < 1e-6:
        return 100.0
    return max((target_altitude - (r_val - c.R_EARTH)) / v_radial, 0.1)


def _compute_tgo_stage2(state, F_T, Isp, previous_tgo=None):
    """
    Stage-2 rocket-equation t_go.
    Mirrors the Stage-2 branch of rocket_ascent._compute_apollo_tgo.
    """
    r_val, v, gamma, m = state[1], state[2], state[3], state[4]
    remaining_prop = m - _DRY_MASS_2
    if remaining_prop <= 0.0:
        return 0.0
    Ve = Isp * c.G_0
    if F_T <= 0.0 or Ve <= 0.0:
        return float(previous_tgo) if previous_tgo is not None else 0.0

    mdot  = F_T / Ve
    T_BUP = remaining_prop / mdot

    r_target   = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    vx_current = v * np.cos(gamma)
    vy_current = v * np.sin(gamma)
    vx_target  = np.sqrt(c.MU_EARTH / r_target)
    VG = np.sqrt((vx_target - vx_current) ** 2 + vy_current ** 2)

    if VG > 0.0:
        tgo = T_BUP * (1.0 - np.exp(-VG / Ve))
    else:
        tgo = float(previous_tgo) if previous_tgo is not None else 0.0

    return float(np.clip(tgo, 0.1, T_BUP))


# ===========================================================================
# Per-step guidance alpha dispatch (Stage 2 only)
# ===========================================================================

def _compute_alpha_stage2(t, state, F_T, Isp, gs):
    """
    Compute steering angle alpha for Stage 2 and update gs.

    Called at every ODE evaluation during thrust arcs.  The GuidanceState gs
    is mutated in place (initialization and per-step updates).  (t, alpha) are
    appended to gs.time_log / gs.alpha_log for later interpolation.

    Returns alpha [rad].
    """
    s, r_val, v, gamma, m = state[:5]
    mode  = sim_params.GUIDANCE_MODE
    r_tgt = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    Ve    = Isp * c.G_0

    guidance_start_ready = (
        gs.atmosphere_exited
        or sim_params.GUIDANCE_START_MODE == "after_kick"
    )

    # -----------------------------------------------------------------------
    # Initialization — runs once when guidance becomes active
    # -----------------------------------------------------------------------
    if not gs.guidance_phase_active and F_T > 0 and guidance_start_ready:
        gs.guidance_phase_active     = True
        gs.time_guidance_start       = t
        gs.last_guidance_update_time = t

        if mode == "cpr":
            gs.cpr_theta_initial = gamma
            if sim_params.CPR_THETA_DOT_MODE == "manual":
                gs.cpr_theta_dot = np.deg2rad(sim_params.CPR_THETA_DOT)
            else:
                tgo = _compute_tgo_stage2(state, F_T, Isp, None)
                gs.cpr_theta_dot = gs.cpr_theta_initial / max(tgo, 0.1)
            gs.cpr_t_start = t

        elif mode in ("simple_poly", "linear_tangent", "bilinear_tangent", "apollo"):
            if mode in ("linear_tangent", "bilinear_tangent") and \
                    sim_params.LTS_TGO_METHOD == "propellant":
                tgo = _compute_tgo_stage2(state, F_T, Isp, None)
                gs.lts_previous_tgo = tgo
            elif mode == "apollo" and sim_params.APOLLO_TGO_METHOD != "altitude":
                tgo = _compute_tgo_stage2(state, F_T, Isp, None)
                gs.apollo_previous_tgo = tgo
            else:
                tgo = _estimate_tto_altitude(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            gs.guidance_initial_tgo = tgo

            if mode == "simple_poly":
                gs.guidance_coefficients = \
                    simple_poly_guidance.compute_polynomial_coefficients(
                        state, sim_params.TARGET_ORBITAL_ALTITUDE, tgo)
            elif mode == "linear_tangent":
                gs.guidance_coefficients = lts_guidance.compute_lts_coefficients(
                    state, sim_params.TARGET_ORBITAL_ALTITUDE, tgo)
            elif mode == "bilinear_tangent":
                gs.guidance_coefficients = bts_guidance.compute_bilinear_coefficients(
                    state, sim_params.TARGET_ORBITAL_ALTITUDE, tgo)
            elif mode == "apollo":
                gs.guidance_coefficients = \
                    apollo_guidance_module.compute_apollo_coefficients(
                        state, sim_params.TARGET_ORBITAL_ALTITUDE, tgo,
                        use_downrange_constraint=(
                            sim_params.GUIDANCE_START_MODE == "after_atmosphere_exit"))
                gs.apollo_freeze_time        = t
                gs.apollo_coefficients_frozen = False

        elif mode == "peg":
            dry    = _DRY_MASS_2
            T_seed = max(m - dry, 0.1) / (F_T / Ve)
            _damp  = (sim_params.PEG_CONVERGENCE_DAMPING
                      if sim_params.PEG_CONVERGENCE_MODE == "damped" else 1.0)
            _tol   = (sim_params.PEG_CONVERGENCE_TOL
                      if sim_params.PEG_CONVERGENCE_MODE == "damped" else 0.0)
            gs.peg_A, gs.peg_B, gs.peg_T = peg_guidance_mod.converge_peg(
                state[:5], T_seed, Ve, F_T, r_tgt, c.MU_EARTH,
                max_iter=sim_params.PEG_CONVERGENCE_MAX_ITER,
                tol=_tol, damping=_damp)
            gs.peg_t_epoch = t
            gs.peg_frozen  = False

        elif mode == "peg_new":
            (gs.peg_new_vgo_r, gs.peg_new_vgo_theta,
             gs.peg_new_L0,    gs.peg_new_tgo,
             gs.peg_new_t_lambda, gs.peg_new_lambda_r) = peg_new_mod.peg_new_major_loop(
                 state[:5], r_tgt, c.MU_EARTH, Ve, F_T)
            gs.peg_new_t_epoch = t
            gs.peg_new_frozen  = False

        elif mode == "exp_shooting":
            gs.exp_shoot_a, gs.exp_shoot_b = exp_shoot_mod.optimize_exp_pitch(
                state[:5], r_tgt, c.MU_EARTH, F_T, Isp, r.M_STRUCTURE_2, c.G_0)
            gs.exp_shoot_epoch = t

    # -----------------------------------------------------------------------
    # Per-step alpha computation
    # -----------------------------------------------------------------------
    alpha = 0.0

    if gs.guidance_phase_active and F_T > 0:

        if mode == "gravity_turn":
            alpha = 0.0

        elif mode == "cpr":
            alpha = cpr_guidance_module.cpr_alpha(
                t, gs.cpr_t_start, gs.cpr_theta_initial,
                gs.cpr_theta_dot, gamma)

        elif mode == "simple_poly":
            if (t - gs.last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
                tgo = _estimate_tto_altitude(state, sim_params.TARGET_ORBITAL_ALTITUDE)
                gs.guidance_coefficients = \
                    simple_poly_guidance.compute_polynomial_coefficients(
                        state, sim_params.TARGET_ORBITAL_ALTITUDE, tgo)
                gs.last_guidance_update_time = t
            tgo   = _estimate_tto_altitude(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            alpha = simple_poly_guidance.polynomial_guidance(
                t, tgo, state, gs.guidance_coefficients)

        elif mode == "linear_tangent":
            if (not sim_params.GUIDANCE_COEFFICIENTS_FIXED
                    and (t - gs.last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE):
                tgo = _estimate_tto_altitude(state, sim_params.TARGET_ORBITAL_ALTITUDE)
                gs.guidance_coefficients = lts_guidance.compute_lts_coefficients(
                    state, sim_params.TARGET_ORBITAL_ALTITUDE, tgo)
                gs.last_guidance_update_time = t
            if sim_params.GUIDANCE_TGO_FIXED:
                tgo = gs.guidance_initial_tgo
            elif sim_params.LTS_TGO_METHOD == "propellant":
                tgo = _compute_tgo_stage2(state, F_T, Isp, gs.lts_previous_tgo)
                gs.lts_previous_tgo = tgo
            else:
                tgo = _estimate_tto_altitude(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            gs.tgo_log.append(tgo)
            gs.tgo_time_log.append(t)
            alpha = lts_guidance.linear_tangent_steering(
                t, tgo, state, gs.guidance_coefficients)

        elif mode == "bilinear_tangent":
            if (not sim_params.GUIDANCE_COEFFICIENTS_FIXED
                    and (t - gs.last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE):
                tgo = _estimate_tto_altitude(state, sim_params.TARGET_ORBITAL_ALTITUDE)
                gs.guidance_coefficients = bts_guidance.compute_bilinear_coefficients(
                    state, sim_params.TARGET_ORBITAL_ALTITUDE, tgo)
                gs.last_guidance_update_time = t
            if sim_params.GUIDANCE_TGO_FIXED:
                tgo = gs.guidance_initial_tgo
            elif sim_params.LTS_TGO_METHOD == "propellant":
                tgo = _compute_tgo_stage2(state, F_T, Isp, gs.lts_previous_tgo)
                gs.lts_previous_tgo = tgo
            else:
                tgo = _estimate_tto_altitude(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            gs.tgo_log.append(tgo)
            gs.tgo_time_log.append(t)
            alpha = bts_guidance.bilinear_tangent_steering(
                t, tgo, state, gs.guidance_coefficients)

        elif mode == "apollo":
            if sim_params.APOLLO_TGO_METHOD == "altitude":
                tgo = _estimate_tto_altitude(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            else:
                tgo = _compute_tgo_stage2(state, F_T, Isp, gs.apollo_previous_tgo)
            gs.apollo_previous_tgo = tgo
            gs.tgo_log.append(tgo)
            gs.tgo_time_log.append(t)

            if tgo < sim_params.APOLLO_FREEZE_THRESHOLD and not gs.apollo_coefficients_frozen:
                gs.apollo_coefficients_frozen = True
                gs.apollo_freeze_time         = t

            if (not gs.apollo_coefficients_frozen
                    and (t - gs.last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE):
                gs.guidance_coefficients = \
                    apollo_guidance_module.compute_apollo_coefficients(
                        state, sim_params.TARGET_ORBITAL_ALTITUDE, tgo,
                        use_downrange_constraint=(
                            sim_params.GUIDANCE_START_MODE == "after_atmosphere_exit"))
                gs.apollo_freeze_time        = t
                gs.last_guidance_update_time = t

            alpha, _ = apollo_guidance_module.apollo_guidance(
                t, gs.apollo_freeze_time, state, gs.guidance_coefficients)

        elif mode == "peg":
            if (not gs.peg_frozen
                    and (t - gs.last_guidance_update_time) >= sim_params.PEG_MAJOR_LOOP_RATE):
                dt       = t - gs.last_guidance_update_time
                gs.peg_T = max(gs.peg_T - dt, 0.1)
                if gs.peg_T < sim_params.APOLLO_FREEZE_THRESHOLD:
                    gs.peg_frozen = True
                else:
                    _damp = (sim_params.PEG_CONVERGENCE_DAMPING
                             if sim_params.PEG_CONVERGENCE_MODE == "damped" else 1.0)
                    _tol  = (sim_params.PEG_CONVERGENCE_TOL
                             if sim_params.PEG_CONVERGENCE_MODE == "damped" else 0.0)
                    gs.peg_A, gs.peg_B, gs.peg_T = peg_guidance_mod.converge_peg(
                        state[:5], gs.peg_T, Ve, F_T, r_tgt, c.MU_EARTH,
                        max_iter=sim_params.PEG_CONVERGENCE_MAX_ITER,
                        tol=_tol, damping=_damp)
                    gs.peg_t_epoch = t
                gs.last_guidance_update_time = t
            t_since = t - gs.peg_t_epoch if gs.peg_t_epoch is not None else 0.0
            alpha   = peg_guidance_mod.peg_alpha(t_since, gs.peg_A, gs.peg_B, gamma)

        elif mode == "peg_new":
            if (not gs.peg_new_frozen
                    and (t - gs.last_guidance_update_time) >= sim_params.PEG_MAJOR_LOOP_RATE):
                if (gs.peg_new_tgo is not None
                        and gs.peg_new_tgo < sim_params.APOLLO_FREEZE_THRESHOLD):
                    gs.peg_new_frozen = True
                else:
                    (gs.peg_new_vgo_r, gs.peg_new_vgo_theta,
                     gs.peg_new_L0,    gs.peg_new_tgo,
                     gs.peg_new_t_lambda, gs.peg_new_lambda_r
                     ) = peg_new_mod.peg_new_major_loop(
                         state[:5], r_tgt, c.MU_EARTH, Ve, F_T)
                    gs.peg_new_t_epoch = t
                gs.last_guidance_update_time = t
            t_since = t - gs.peg_new_t_epoch if gs.peg_new_t_epoch is not None else 0.0
            alpha   = peg_new_mod.peg_new_alpha(
                t_since,
                gs.peg_new_vgo_r, gs.peg_new_vgo_theta,
                gs.peg_new_L0,    gs.peg_new_lambda_r,
                gs.peg_new_t_lambda, gamma)

        elif mode == "exp_shooting":
            if gs.exp_shoot_a is None:
                gs.exp_shoot_a, gs.exp_shoot_b = exp_shoot_mod.optimize_exp_pitch(
                    state[:5], r_tgt, c.MU_EARTH, F_T, Isp,
                    r.M_STRUCTURE_2, c.G_0)
                gs.exp_shoot_epoch = t
            alpha = exp_shoot_mod.exp_pitch_alpha(
                t - gs.exp_shoot_epoch,
                gs.exp_shoot_a, gs.exp_shoot_b, gamma)

    gs.time_log.append(t)
    gs.alpha_log.append(alpha)
    return alpha


# ===========================================================================
# Stage-2 ODE (vacuum dynamics, F_L = F_D = 0)
# ===========================================================================

def _stage2_ode_guidance(t, y, thrust, Isp, gs):
    """
    Stage-2 ODE with guidance steering. Vacuum dynamics (F_L = F_D = 0).

    gs = None  →  ballistic (pre-ignition or pure coast).
    """
    s, r_val, v, gamma, m = y[:5]

    # Auto-cutoff when propellant is depleted (prevents F_T/m blow-up)
    F_T = thrust if m > _DRY_MASS_2 + 0.1 else 0.0

    if gs is not None and F_T > 0:
        alpha = _compute_alpha_stage2(t, y, F_T, Isp, gs)
    else:
        alpha = 0.0

    a_grav = grav.gravitational_acceleration(r_val)
    return ra.diff_eom_base(s, r_val, v, gamma, m, 0.0, 0.0, F_T, a_grav, alpha, Isp)


# Ground-collision event (terminal; identical to indirect_pso_solver)
def _event_crash(t, y, *args):
    return y[1] - c.R_EARTH

_event_crash.terminal  = True
_event_crash.direction = -1


# ===========================================================================
# Full Stage-1 → Stage-2 trajectory runner (PSO inner loop)
# ===========================================================================

def run_pso_coast_trajectory(delta_tc, delta_tr_pct, coast_start_pct, gamma_p,
                              verbose=False):
    """
    Simulate a thrust–coast–thrust Stage-2 trajectory for one PSO particle.

    Parameters
    ----------
    delta_tc        : float  Coast duration [s]
    delta_tr_pct    : float  Stage-2 burn as % of T_MAX_2 [%]
    coast_start_pct : float  Coast start as % of Stage-2 burn time [%]
    gamma_p         : float  Pitch maneuver angle [rad]  (kick_angle = gamma_p - pi/2)
    verbose         : bool

    Returns
    -------
    result : dict with keys
        crashed, state_final, t_f, t_cf,
        t_stage2_start, t_ignition, t_arc2_start, t_arc3_end,
        t_stage1, y_stage1
    """
    kick_angle = gamma_p - np.pi / 2.0

    # ---- Stage 1 ----
    t2_start, state2_init, _, t_stage1, y_stage1, crashed = ra.run_stage1(kick_angle)
    if crashed:
        return {
            'crashed': True, 'state_final': None,
            't_f': 0.0, 't_cf': 0.0,
            't_stage2_start': 0.0, 't_ignition': 0.0,
            't_arc2_start': 0.0, 't_arc3_end': 0.0,
            't_stage1': t_stage1, 'y_stage1': y_stage1,
        }

    # Strip to 5-element physical state (handles INCLUDE_PSEUDO_FORCES)
    state2_init = _strip_to_pmp_state(
        state2_init, np.deg2rad(sim_params.LAUNCH_LATITUDE))

    if verbose:
        h2 = state2_init[1] - c.R_EARTH
        print(f"  Stage 1 end: t={t2_start:.1f}s, h={h2/1e3:.1f}km, "
              f"v={state2_init[2]:.0f}m/s, gam={np.rad2deg(state2_init[3]):.2f}deg")

    # ---- Timing ----
    T_burn_total  = (delta_tr_pct   / 100.0) * _T_MAX_2
    t_coast_start = (coast_start_pct / 100.0) * T_burn_total
    t_arc3_burn   = T_burn_total - t_coast_start
    t_ignition    = t2_start + _T_IGNITION_DELAY

    # ---- Pre-ignition ballistic coast (stage sep → ignition) ----
    sol_pre = solve_ivp(
        lambda t, y: _stage2_ode_guidance(t, y, 0.0, r.ISP_2, None),
        t_span=(t2_start, t_ignition),
        y0=state2_init[:5],
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
        events=_event_crash,
    )
    if len(sol_pre.t_events[0]) > 0:
        return {'crashed': True, 'state_final': None,
                't_f': 0.0, 't_cf': 0.0,
                't_stage2_start': t2_start, 't_ignition': t_ignition,
                't_arc2_start': 0.0, 't_arc3_end': 0.0,
                't_stage1': t_stage1, 'y_stage1': y_stage1}
    state_at_ign = sol_pre.y[:5, -1].copy()

    # Fresh GuidanceState: persists across Arc 1 → coast → Arc 3
    gs = GuidanceState()

    # ---- Arc 1: Thrust (t_ignition → t_ignition + t_coast_start) ----
    t_arc1_end = t_ignition + t_coast_start
    if t_coast_start > 0.01:
        sol_arc1 = solve_ivp(
            lambda t, y: _stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs),
            t_span=(t_ignition, t_arc1_end),
            y0=state_at_ign,
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        if len(sol_arc1.t_events[0]) > 0:
            return {'crashed': True, 'state_final': None,
                    't_f': 0.0, 't_cf': 0.0,
                    't_stage2_start': t2_start, 't_ignition': t_ignition,
                    't_arc2_start': 0.0, 't_arc3_end': 0.0,
                    't_stage1': t_stage1, 'y_stage1': y_stage1}
        state_arc2   = sol_arc1.y[:5, -1].copy()
        t_arc2_start = float(sol_arc1.t[-1])
    else:
        state_arc2   = state_at_ign.copy()
        t_arc2_start = t_ignition

    # Planned values avoid ODE endpoint overshoot (same approach as indirect_pso_solver)
    t_f_result  = T_burn_total + delta_tc
    t_cf_result = delta_tc

    # ---- Arc 2: Coast (t_arc2_start → t_arc2_start + delta_tc) ----
    t_arc2_end = t_arc2_start + delta_tc
    if delta_tc > 0.01:
        sol_arc2 = solve_ivp(
            lambda t, y: _stage2_ode_guidance(t, y, 0.0, r.ISP_2, None),
            t_span=(t_arc2_start, t_arc2_end),
            y0=state_arc2,
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        if len(sol_arc2.t_events[0]) > 0:
            return {'crashed': True, 'state_final': None,
                    't_f': 0.0, 't_cf': 0.0,
                    't_stage2_start': t2_start, 't_ignition': t_ignition,
                    't_arc2_start': t_arc2_start, 't_arc3_end': 0.0,
                    't_stage1': t_stage1, 'y_stage1': y_stage1}
        state_arc3   = sol_arc2.y[:5, -1].copy()
        t_arc3_start = float(sol_arc2.t[-1])
    else:
        state_arc3   = state_arc2.copy()
        t_arc3_start = t_arc2_start

    # ---- Arc 3: Thrust (t_arc3_start → t_arc3_start + t_arc3_burn) ----
    t_arc3_end = t_arc3_start + t_arc3_burn
    if t_arc3_burn > 0.01:
        sol_arc3 = solve_ivp(
            lambda t, y: _stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs),
            t_span=(t_arc3_start, t_arc3_end),
            y0=state_arc3,
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        if len(sol_arc3.t_events[0]) > 0:
            return {'crashed': True, 'state_final': None,
                    't_f': 0.0, 't_cf': 0.0,
                    't_stage2_start': t2_start, 't_ignition': t_ignition,
                    't_arc2_start': t_arc2_start, 't_arc3_end': 0.0,
                    't_stage1': t_stage1, 'y_stage1': y_stage1}
        state_final = sol_arc3.y[:5, -1].copy()
    else:
        state_final = state_arc3.copy()

    if verbose:
        h_f = state_final[1] - c.R_EARTH
        print(f"  Stage 2 end: t={t_ignition + t_f_result:.1f}s, h={h_f/1e3:.1f}km, "
              f"v={state_final[2]:.0f}m/s, gam={np.rad2deg(state_final[3]):.2f}deg")

    return {
        'crashed':       False,
        'state_final':   state_final,
        't_f':           t_f_result,
        't_cf':          t_cf_result,
        't_stage2_start': t2_start,
        't_ignition':    t_ignition,
        't_arc2_start':  t_arc2_start,
        't_arc3_end':    t_arc3_end,
        't_stage1':      t_stage1,
        'y_stage1':      y_stage1,
    }


# ===========================================================================
# Objective function (no transversality term)
# ===========================================================================

def _coast_objective_terms(result):
    """
    4-term non-dimensional objective.  Same normalisation as indirect_pso_solver
    _objective_terms, minus the transversality term.
    """
    state = result['state_final']
    r_val, v_f, g_f = state[1], state[2], state[3]

    r_target   = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    # Rotating-frame circular target: the trajectory velocity is ground-relative,
    # so credit Earth's surface rotation speed once here. Zero when rotation off.
    if sim_params.ENABLE_EARTH_ROTATION:
        v_rot = (c.OMEGA_EARTH * c.R_EARTH
                 * np.cos(np.deg2rad(sim_params.LAUNCH_LATITUDE)))
    else:
        v_rot = 0.0
    v_circular = np.sqrt(c.MU_EARTH / r_target) - v_rot
    gamma_ref  = np.deg2rad(sim_params.PSO_COAST_GAMMA_REF_DEG)

    J_nd  = (result['t_f'] - result['t_cf']) / _T_MAX_2
    dh_nd = (r_val - r_target) / sim_params.TARGET_ORBITAL_ALTITUDE
    dv_nd = (v_f - v_circular) / v_circular
    dg_nd = g_f / gamma_ref

    return {
        'J'  : sim_params.PSO_COAST_W_J        * J_nd,
        'alt': sim_params.PSO_COAST_W_ALTITUDE * abs(dh_nd),
        'vel': sim_params.PSO_COAST_W_VELOCITY * abs(dv_nd),
        'fpa': sim_params.PSO_COAST_W_FPA      * abs(dg_nd),
    }


def compute_coast_objective(result):
    """Augmented objective value J' for PSO minimisation."""
    if result['crashed'] or result['state_final'] is None:
        return CRASH_PENALTY
    state = result['state_final']
    C = 0.0
    if (state[1] - c.R_EARTH) < 0 or state[2] < 0:
        C = CRASH_PENALTY
    return float(sum(_coast_objective_terms(result).values()) + C)


def breakdown_coast_objective(result):
    """Decompose J' into individual (weighted, non-dimensional) terms."""
    if result['crashed'] or result['state_final'] is None:
        return {'J': 1e20, 'alt': 1e20, 'vel': 1e20, 'fpa': 1e20}
    return _coast_objective_terms(result)


# ===========================================================================
# PyGMO-compatible problem class
# ===========================================================================

class CoastPSOProblem:
    """
    UDP for PyGMO's PSO algorithm.
    Decision vector: [delta_tc, delta_tr_pct, coast_start_pct, gamma_p]
    """

    def fitness(self, x):
        delta_tc, delta_tr_pct, coast_start_pct, gamma_p = x
        result = run_pso_coast_trajectory(
            delta_tc, delta_tr_pct, coast_start_pct, gamma_p)
        return [compute_coast_objective(result)]

    def get_bounds(self):
        return (sim_params.PSO_COAST_LB, sim_params.PSO_COAST_UB)

    def get_nobj(self):
        return 1


# ===========================================================================
# PSO runner
# ===========================================================================

def run_pso_coast_optimization(verbose=True):
    """
    Run the PSO coast optimisation (Sect. 4.2.2 of the paper, adapted for
    guidance-based modes without costates).

    Returns
    -------
    optimal_params : list  [delta_tc, delta_tr_pct, coast_start_pct, gamma_p]
    J_optimal      : float  Best augmented objective value
    """
    global LAST_PSO_COAST_HISTORY

    n_particles = sim_params.PSO_COAST_N_PARTICLES
    n_gen       = sim_params.PSO_COAST_MAX_GENERATIONS
    lb          = sim_params.PSO_COAST_LB
    ub          = sim_params.PSO_COAST_UB

    if verbose:
        print("\n" + "=" * 60)
        print(f"PSO COAST OPTIMISATION — {sim_params.GUIDANCE_MODE.upper()}")
        print("=" * 60)
        print("  Optimising 4 variables: Δt_c, Δt_r%, coast_start%, γ_p")
        print(f"  Particles : {n_particles}")
        print(f"  Max gen.  : {n_gen}")
        print(f"  Bounds    : {list(zip(lb, ub))}")
        print("=" * 60 + "\n")

    t_start = time.time()

    try:
        import pygmo as pg

        prob = pg.problem(CoastPSOProblem())
        algo = pg.algorithm(pg.pso(
            gen     = n_gen,
            omega   = sim_params.PSO_COAST_OMEGA,
            eta1    = sim_params.PSO_COAST_C1,
            eta2    = sim_params.PSO_COAST_C2,
            max_vel = sim_params.PSO_COAST_VMAX,
            seed    = sim_params.PSO_COAST_SEED,
        ))
        if verbose:
            algo.set_verbosity(25)

        pop = pg.population(prob, size=n_particles, seed=sim_params.PSO_COAST_SEED)
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
            LAST_PSO_COAST_HISTORY = {
                'gen': np.array(gens), 'gbest': np.array(gbest)}
        else:
            LAST_PSO_COAST_HISTORY = None

        if verbose:
            print(f"\n[PSO coast] Finished in {time.time() - t_start:.1f}s")
            print(f"  Best J' = {best_f:.4f}")
            _print_coast_solution(best_x, best_f)

        return best_x, best_f

    except ImportError:
        raise ImportError(
            "pygmo is required for PSO coast optimisation. "
            "Install it with: conda install -c conda-forge pygmo"
        )


# ===========================================================================
# Full trajectory re-run for plotting
# ===========================================================================

def run_pso_coast_full(optimal_params, verbose=True):
    """
    Re-run the optimal PSO coast trajectory with dense output for plotting.

    Sets ``rocket_ascent.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL`` to the Arc-3
    end time and ``rocket_ascent.PSO_COAST_ARC2_START_TIME`` to the coast start
    so that event-marker code in plot_state_utils picks them up.

    Returns
    -------
    time_full            : ndarray  Combined time array [s]
    data_full            : ndarray  State data (5 or 6 × N; row 5 = latitude
                                     when Earth rotation is enabled)
    thrust_full          : ndarray  Thrust [N] at each time step
    alpha_full           : ndarray  Angle of attack [rad] at each time step
    t_ignition           : float    Stage-2 engine ignition time [s]
    result               : dict     Same keys as run_pso_coast_trajectory
    coriolis_mag_data    : ndarray  Coriolis accel magnitude [m/s²] (Stage-1
                                     real + Stage-2 zeros)
    centrifugal_mag_data : ndarray  Centrifugal accel magnitude [m/s²]

    Also writes ra.theta_*_history, ra.tgo_*_history, ra.cross_heading_*_history
    so the shared plot block in main.py renders guidance/Earth-rotation plots.
    """
    delta_tc, delta_tr_pct, coast_start_pct, gamma_p = optimal_params
    kick_angle = gamma_p - np.pi / 2.0

    # ---- Stage 1 ----
    t2_start, state2_init, _, t_stage1, y_stage1, crashed = ra.run_stage1(kick_angle)
    if crashed:
        raise RuntimeError("Stage 1 crashed during PSO coast full-trajectory run.")

    t_ignition  = t2_start + _T_IGNITION_DELAY
    state2_init = _strip_to_pmp_state(
        state2_init, np.deg2rad(sim_params.LAUNCH_LATITUDE))

    T_burn_total  = (delta_tr_pct   / 100.0) * _T_MAX_2
    t_coast_start = (coast_start_pct / 100.0) * T_burn_total
    t_arc3_burn   = T_burn_total - t_coast_start

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

    # Single GuidanceState persists through Arc 1 → coast → Arc 3
    gs_full = GuidanceState()

    # ---- Arc 1 (thrust, dense) ----
    t_arc1_end = t_ignition + t_coast_start
    if t_coast_start > 0.01:
        sol1 = solve_ivp(
            lambda t, y: _stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs_full),
            t_span=(t_ignition, t_arc1_end),
            y0=state_at_ign,
            t_eval=_make_teval(t_ignition, t_arc1_end),
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        state_arc2   = sol1.y[:5, -1].copy()
        t_arc2_start = float(sol1.t[-1])
    else:
        sol1         = None
        state_arc2   = state_at_ign.copy()
        t_arc2_start = t_ignition

    # ---- Arc 2 (coast, dense) ----
    t_arc2_end = t_arc2_start + delta_tc
    if delta_tc > 0.01:
        sol2 = solve_ivp(
            lambda t, y: _stage2_ode_guidance(t, y, 0.0, r.ISP_2, None),
            t_span=(t_arc2_start, t_arc2_end),
            y0=state_arc2,
            t_eval=_make_teval(t_arc2_start, t_arc2_end),
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        state_arc3   = sol2.y[:5, -1].copy()
        t_arc3_start = float(sol2.t[-1])
    else:
        sol2         = None
        state_arc3   = state_arc2.copy()
        t_arc3_start = t_arc2_start

    # ---- Arc 3 (thrust, dense) ----
    t_arc3_end = t_arc3_start + t_arc3_burn
    if t_arc3_burn > 0.01:
        sol3 = solve_ivp(
            lambda t, y: _stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs_full),
            t_span=(t_arc3_start, t_arc3_end),
            y0=state_arc3,
            t_eval=_make_teval(t_arc3_start, t_arc3_end),
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        state_insertion = sol3.y[:5, -1].copy()
    else:
        sol3 = None
        state_insertion = state_arc3.copy()

    # ---- Post-insertion orbit coast (thrust off) ----
    # Propagate the achieved orbit so altitude/trajectory plots show the full
    # orbit and Final Orbital Elements are meaningful (mirrors apogee_check).
    t_post_start = t_arc3_end
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
    sols2_list  = [sol_pre]
    thrusts2    = [0.0]
    if sol1 is not None: sols2_list.append(sol1); thrusts2.append(r.F_THRUST_2)
    if sol2 is not None: sols2_list.append(sol2); thrusts2.append(0.0)
    if sol3 is not None: sols2_list.append(sol3); thrusts2.append(r.F_THRUST_2)
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
    y1         = y_stage1[:5, :]
    time_full  = np.concatenate([t_stage1, t_stage2_full])
    data_full  = np.concatenate([y1, y_stage2_full], axis=1)

    thrust_stage1 = interpolate_to_time(ra.time_history, ra.thrust_history, t_stage1)
    alpha_stage1  = interpolate_to_time(
        ra.alpha_time_history, ra.alpha_history, t_stage1)

    thrust_full = np.concatenate([thrust_stage1, thrust_stage2])
    alpha_full  = np.concatenate([alpha_stage1,  alpha_stage2])

    n_stage1 = len(t_stage1)
    n_stage2 = len(t_stage2_full)

    # ---- Latitude row (6th state row) so the latitude plot renders ----
    # Derived from downrange via great-circle geometry (Earth rotation only).
    if sim_params.ENABLE_EARTH_ROTATION:
        lat_row = np.array([ra.get_latitude_from_downrange(s) for s in data_full[0]])
        data_full = np.vstack([data_full, lat_row])   # rows: s, r, v, γ, m, lat

    # ---- Assemble full-trajectory history channels for the plot suite ----
    # The shared plot block in main.py reads these from ra.*_history globals.
    # Stage 1 already logged real values; Stage 2 is inertial vacuum, so its
    # pseudo-force / cross-heading contributions are zero (physically honest).
    theta_full = alpha_full + data_full[3]            # pitch θ = α + γ
    ra.theta_history      = list(theta_full)
    ra.theta_time_history = list(time_full)

    # t_go: guidance runs in Stage 2 only for pso_coast, so report the Stage-2
    # guidance log directly (apollo / linear_tangent / bilinear_tangent modes).
    if gs_full.tgo_time_log:
        ra.tgo_time_history = list(gs_full.tgo_time_log)
        ra.tgo_history      = list(gs_full.tgo_log)

    # Pseudo-force / cross-heading: Stage-1 real (already in ra.*_history),
    # Stage-2 zeros appended.
    coriolis_stage1    = np.asarray(ra.coriolis_mag_history, dtype=float)
    centrifugal_stage1 = np.asarray(ra.centrifugal_mag_history, dtype=float)
    cor_s1  = interpolate_to_time(ra.time_history, coriolis_stage1, t_stage1) \
        if len(coriolis_stage1) else np.zeros(n_stage1)
    cen_s1  = interpolate_to_time(ra.time_history, centrifugal_stage1, t_stage1) \
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

    # ---- Set event markers used by plot_state_utils ----
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = t_arc3_end
    ra.PSO_COAST_ARC2_START_TIME              = t_arc2_start

    if verbose:
        sf = data_full[:, -1]
        print(f"\n[PSO coast full run] t_end={time_full[-1]:.1f}s, "
              f"h={(sf[1]-c.R_EARTH)/1e3:.1f}km, "
              f"v={sf[2]:.0f}m/s, gam={np.rad2deg(sf[3]):.2f}deg")

    # ---- Build result dict from the dense run (no extra re-integration) ----
    result = {
        'crashed':        False,
        'state_final':    state_insertion,   # state at orbit insertion (Arc-3 end)
        't_f':            T_burn_total + delta_tc,
        't_cf':           delta_tc,
        't_stage2_start': t2_start,
        't_ignition':     t_ignition,
        't_arc2_start':   t_arc2_start,
        't_arc3_end':     t_arc3_end,
        't_stage1':       t_stage1,
        'y_stage1':       y_stage1,
    }

    return (time_full, data_full, thrust_full, alpha_full, t_ignition, result,
            coriolis_mag_data, centrifugal_mag_data)


# ===========================================================================
# Diagnostic helper
# ===========================================================================

def _print_coast_solution(x, J_prime):
    """Pretty-print the optimal PSO coast parameters (compact).

    The full final-state / J' breakdown is printed once by main.py's results
    block, so this helper intentionally does NOT re-run the trajectory.
    """
    delta_tc, delta_tr_pct, coast_start_pct, gamma_p = x
    kick_angle    = gamma_p - np.pi / 2.0
    T_burn_total  = (delta_tr_pct   / 100.0) * _T_MAX_2
    t_coast_start = (coast_start_pct / 100.0) * T_burn_total

    print("\nOptimal PSO coast parameters:")
    print(f"  Coast duration   = {delta_tc:.2f} s")
    print(f"  Burn fraction    = {delta_tr_pct:.2f} %  (T_max = {_T_MAX_2:.1f} s)")
    print(f"  Coast start      = {coast_start_pct:.2f} %  (= {t_coast_start:.1f} s into burn)")
    print(f"  Pitch angle γ_p  = {np.rad2deg(gamma_p):.4f}°  ({gamma_p:.6f} rad)")
    print(f"  Kick angle       = {np.rad2deg(kick_angle):.4f}°")
    print(f"  J'               = {J_prime:.4f}")
