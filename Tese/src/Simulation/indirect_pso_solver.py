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

PyGMO is used.
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
    drag_specific_force,
)
import Simulation.rocket_ascent as ra


# ---------------------------------------------------------------------------
# Derived Stage-2 constants (computed once at import time)
# ---------------------------------------------------------------------------
_MDOT_2 = r.F_THRUST_2 / (r.ISP_2 * c.G_0)           # Stage-2 mass flow rate [kg/s]
_T_MAX_2 = r.M_PROP_2 / _MDOT_2                        # Time to deplete ALL Stage-2 propellant [s]
# Engine-ignition delay measured from stage separation:
_T_IGNITION_DELAY = r.TIME_SECOND_ENGINE_IGNITION - r.TIME_First_STAGE_SEPARATION

# ---------------------------------------------------------------------------
# Derived Stage-1 constants (only used by the full-ascent PMP extension)
# ---------------------------------------------------------------------------
# The augmented PMP ODE is a constant-(thrust, Isp) point-mass model, so Stage 1
# is represented with a single representative thrust/Isp — the mean of the
# sea-level and vacuum values (the true engine ramps between them with ambient
# pressure). This is the same order of simplification the indirect method already
# makes for Stage 2, and it only affects the opt-in full-ascent mode.
_F_THRUST_1 = 0.5 * (r.F_THRUST_1_SL + r.F_THRUST_1_VAC)   # representative Stage-1 thrust [N]
_ISP_1      = 0.5 * (r.ISP_1_SL + r.ISP_1_VAC)             # representative Stage-1 Isp [s]
_MDOT_1     = _F_THRUST_1 / (_ISP_1 * c.G_0)               # Stage-1 mass flow [kg/s]
_T_MAX_1    = r.M_PROP_1 / _MDOT_1                          # Stage-1 full-burn time [s]

# Full stacked launch mass (fairing is part of M_STRUCTURE_1).
_M_LAUNCH = (r.M_STRUCTURE_1 + r.M_PROP_1 + r.M_STRUCTURE_2
             + r.M_PROP_2 + r.M_PAYLOAD)


def _resolve_pmp_options():
    """Read the full-ascent PMP config knobs into a normalised tuple.

    Returns (full_ascent, include_drag, alpha_max_rad):
      full_ascent   : bool   INDIRECT_PMP_FULL_ASCENT
      include_drag  : bool   INDIRECT_PMP_INCLUDE_DRAG, or full_ascent if None
      alpha_max_rad : float or None   INDIRECT_PMP_ALPHA_MAX_DEG in radians

    All three default so that the Stage-2-only mode is byte-for-byte the original
    drag-free, unconstrained behaviour (full_ascent=False ⇒ include_drag=False,
    alpha_max=None regardless of the drag/alpha knobs — the caller only applies
    them on the full-ascent path).
    """
    full_ascent = bool(getattr(sim_params, "INDIRECT_PMP_FULL_ASCENT", False))
    inc = getattr(sim_params, "INDIRECT_PMP_INCLUDE_DRAG", None)
    include_drag = full_ascent if inc is None else bool(inc)
    a_deg = getattr(sim_params, "INDIRECT_PMP_ALPHA_MAX_DEG", None)
    alpha_max_rad = None if a_deg is None else np.deg2rad(float(a_deg))
    return full_ascent, include_drag, alpha_max_rad

# ---------------------------------------------------------------------------
# Integration tolerances (shared by all solve_ivp calls)
# ---------------------------------------------------------------------------
# Tight rtol/atol give good relative accuracy on the mixed-scale state
# (r~1e7, v~1e3, m~1e4, costates~1); the default rtol=1e-3 tolerated ~km-level
# error in r, which fed straight into the altitude penalty. max_step is relaxed
# from 0.5 s — the dynamics are smooth, so the adaptive stepper takes large
# steps on long coasts while the crash event is still bracketed reliably.
_RTOL = 1e-9
_ATOL = 1e-9
_MAX_STEP = 10.0

# Per-generation PSO convergence history, populated by run_pso_optimization.
# Dict with keys 'gen' and 'gbest' (best J' so far), or None if unavailable.
LAST_PSO_HISTORY = None


# ===========================================================================
# Stage-1 → Stage-2 state handoff
# ===========================================================================

def _normalize_costates(lam_r, lam_v, lam_g):
    """Return the initial costate vector scaled to unit norm.

    The trajectory depends only on the costate direction (control law and
    linear costate ODEs are invariant to positive scaling), while every
    Hamiltonian scales linearly with the costate magnitude. Pinning ‖λ‖=1
    fixes that gauge so the transversality residual is a meaningful constraint.
    If the vector is ~0 it is returned unchanged (degenerate → α≈0).
    """
    norm = np.sqrt(lam_r ** 2 + lam_v ** 2 + lam_g ** 2)
    if norm > 1e-12:
        return lam_r / norm, lam_v / norm, lam_g / norm
    return lam_r, lam_v, lam_g


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


# ===========================================================================
# Stage-2 augmented ODE
# ===========================================================================

def _stage2_ode(t, aug_state, thrust, Isp, include_drag=False, alpha_max=None,
                force_alpha=None, alpha_cap_qmin=None):
    """
    Right-hand side for the augmented PMP ODE (Stage 2, and Stage 1 in
    full-ascent mode — the equations are engine-agnostic, parameterised by
    ``thrust`` / ``Isp``).

    aug_state = [s, r, v, γ, m, λ_r, λ_v, λ_γ]
    (indices 0-4 = physical state, 5-7 = costates)

    The PMP control law computes α from the current costates.

    Parameters
    ----------
    t         : float   Current time [s]  (required by solve_ivp but unused here)
    aug_state : array   Augmented state (8 elements)
    thrust    : float   Current thrust force [N]  (0 during coast arcs)
    Isp       : float   Specific impulse [s]
    include_drag : bool
        Couple aerodynamic drag into BOTH the physical EOM (V̇ -= a_D) and the
        costate ODEs. Default False ⇒ drag-free (byte-for-byte the original
        Stage-2 dynamics; a_D → 0 in vacuum anyway).
    alpha_max : float or None
        Angle-of-attack constraint [rad] passed to the control law. None ⇒
        unconstrained (original behaviour).
    force_alpha : float or None
        If not None, override the PMP control with this fixed α [rad] (used for
        the vertical-rise arc, where α ≡ 0 keeps the vehicle vertical until the
        pitch-over kick). The same α feeds the costate ODEs so they stay
        consistent with the flown control.

    Returns
    -------
    derivatives : list  d(aug_state)/dt  (8 elements)
    """
    s, r_val, v, gamma, m = aug_state[:5]
    lam_r, lam_v, lam_g   = aug_state[5], aug_state[6], aug_state[7]

    _EPS = 1e-10
    mu = c.MU_EARTH

    # Angle of attack: fixed override (vertical rise) or PMP (optionally clamped,
    # with the clamp q-gated so it is lifted in near-vacuum).
    if force_alpha is not None:
        alpha = force_alpha
    else:
        alpha = pmp_control_law(lam_v, lam_g, v, alpha_max=alpha_max,
                                alpha_cap_qmin=alpha_cap_qmin, r_val=r_val)

    cg = np.cos(gamma)
    sg = np.sin(gamma)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    g_local = mu / r_val ** 2
    T_over_m = (thrust / m) if m > _EPS else 0.0
    a_D = drag_specific_force(r_val, v, m) if include_drag else 0.0

    # --- Physical state derivatives (drag included only when requested) ---
    dsdt    = (c.R_EARTH / r_val) * v * cg
    drdt    = v * sg
    dvdt    = T_over_m * ca - g_local * sg - a_D
    if abs(v) < _EPS:
        dgdt = 0.0
    else:
        dgdt = (1.0 / v) * (T_over_m * sa - (g_local - v ** 2 / r_val) * cg)
    dmdt    = -thrust / (Isp * c.G_0) if thrust > 0 and m > _EPS else 0.0

    # --- Costate derivatives (r, v, γ) ---
    dlams = costate_derivatives(r_val, v, gamma, thrust, m, lam_r, lam_v, lam_g,
                                alpha, include_drag=include_drag)

    # --- Optional mass costate λ_m (9-state, full-ascent only) ---
    # λ̇_m = −∂H/∂m = λ_v(T·cosα/m² − a_D/m) + λ_γ·T·sinα/(m²·v).  ṁ is independent
    # of r,v,γ, so λ_m never feeds back into the state/other costates (passive).
    # 8-state calls (Stage-2-only) return exactly as before.
    if len(aug_state) >= 9:
        if m > _EPS:
            v_safe = v if abs(v) >= _EPS else _EPS
            T_over_m2 = thrust / (m * m)
            dlam_m = (lam_v * (T_over_m2 * ca - a_D / m)
                      + lam_g * (T_over_m2 * sa / v_safe))
        else:
            dlam_m = 0.0
        return [dsdt, drdt, dvdt, dgdt, dmdt] + dlams + [dlam_m]

    return [dsdt, drdt, dvdt, dgdt, dmdt] + dlams


# ---------------------------------------------------------------------------
# Ground-collision event (terminal)
# ---------------------------------------------------------------------------
def _event_crash(t, y, *args):
    return y[1] - c.R_EARTH

_event_crash.terminal  = True
_event_crash.direction = -1


def _make_mass_event(m_threshold):
    """Terminal solve_ivp event that fires when mass decreases to ``m_threshold``
    (used to end the Stage-1 burn arc at propellant depletion / MECO)."""
    def _event(t, y, *args):
        return y[4] - m_threshold
    _event.terminal  = True
    _event.direction = -1
    return _event


# ===========================================================================
# Full-ascent PMP (modular arc engine)  —  opt-in via INDIRECT_PMP_FULL_ASCENT
# ===========================================================================
#
# When enabled, the PMP costate law steers the WHOLE powered ascent instead of
# just Stage 2. The trajectory is expressed as an ordered list of arcs threaded
# through a single integrator that carries the 8-element augmented state
# [s, r, v, γ, m, λ_r, λ_v, λ_γ] — including across the staging mass drop, where
# the reduced (no-λ_m) costates are continuous (Weierstrass–Erdmann), so no jump
# condition is needed.
#
#   1. vrise   Stage-1 thrust, α ≡ 0 (vertical) until the pitch-over kick time
#              → kick: γ jump AND costate initialization (PMP starts here)
#   2. s1burn  Stage-1 thrust, PMP-steered, until MECO (mass-depletion event)
#              → staging mass drop
#   3. coast   inter-stage ballistic coast (thrust = 0)
#   4. s2arc1  Stage-2 thrust, PMP-steered
#   5. s2coast Stage-2 coast
#   6. s2arc3  Stage-2 thrust, PMP-steered
#
# Costates are initialised at the POST-KICK state (start of the PMP region), not
# at liftoff: the vertical rise has no steering freedom (α≡0) and the unit-norm λ
# gauge would otherwise drift through it. This mirrors the multistage indirect
# formulation of Pontani (Acta Astronautica 2014) / Gath & Calise (JGCD 2001):
# a sequence of propelled + coast arcs with inert-mass separation at burnout,
# costates continuous across staging and coast corners (Weierstrass–Erdmann).
# Stage-1 burns to depletion (fixed duration), so the only FREE-timing
# transversality conditions are on the Stage-2 coast/final time — hence the
# objective's transversality residual stays anchored on the Stage-2 boundaries,
# unchanged. A verbose H-at-corners diagnostic confirms H-constancy.

_FA_DENSE_STEP = 0.5   # output step [s] for the dense (plotting/reference) run


def _apply_post(aug, post):
    """Apply an inter-arc discontinuity.

    post = ('mass_drop', dm)                 staging inert-mass separation
         | ('kick_init', (dγ, (λr, λv, λg))) pitch-over kick AND costate init.

    ``kick_init`` both applies the instantaneous pitch-over (γ += dγ) and SEEDS
    the costates at the post-kick state — i.e. the PMP optimization starts after
    the kick, not at liftoff. The pre-kick vertical rise carries zero costates
    (α is forced to 0 there, so they never steer), and the unit-norm gauge is
    fixed here, where the transversality residual is actually evaluated.
    """
    if post is None:
        return aug
    kind, val = post
    aug = list(aug)
    if kind == 'mass_drop':
        aug[4] -= val
    elif kind == 'kick_init':
        gamma_delta, lam0 = val
        aug[3] += gamma_delta
        aug[5], aug[6], aug[7] = lam0[0], lam0[1], lam0[2]
    return aug


def _fa_crashed(result_extra=None):
    res = {
        'crashed': True, 'state_final': None,
        'H_burn_start': 0.0, 'H_coast_end': 0.0, 'H_burn_end': 0.0,
        't_f': 0.0, 't_cf': 0.0,
        't_stage2_start': 0.0, 't_ignition': 0.0,
        't_stage1': np.array([]), 'y_stage1': np.array([]),
    }
    if result_extra:
        res.update(result_extra)
    return res


def _integrate_full_ascent(lambda0_r, lambda0_v, lambda0_g,
                           delta_tc, delta_tr_pct, coast_start_pct, gamma_p,
                           include_drag, alpha_max, dense=False, verbose=False):
    """Integrate the full-ascent PMP trajectory (Stage 1 → insertion).

    Returns a dict with the same keys as ``run_indirect_trajectory`` under
    ``'result'``; when ``dense`` is True it additionally carries the concatenated
    plotting arrays 'time_full', 'data_full' (5×N), 'thrust_full', 'alpha_full'
    and 't_ignition'.
    """
    lam0 = _normalize_costates(lambda0_r, lambda0_v, lambda0_g)

    # Dynamic-pressure floor below which the α clamp is lifted (vacuum → interior
    # PMP steering). None ⇒ the clamp applies everywhere.
    alpha_cap_qmin = getattr(sim_params, "INDIRECT_PMP_ALPHA_CAP_QMIN", None)

    # --- Mass bookkeeping (mirror run_stage1's fairing convention) ---
    if include_drag:
        m0 = _M_LAUNCH                       # carry the fairing (folded into staging)
        stage_drop = r.M_STRUCTURE_1
    else:
        m0 = _M_LAUNCH - r.M_FAIRING         # no atmosphere ⇒ no fairing (as run_stage1)
        stage_drop = r.M_STRUCTURE_1 - r.M_FAIRING
    meco_mass = m0 - r.M_PROP_1              # Stage-1 propellant-depletion threshold

    # --- Stage-2 arc timing (identical to the Stage-2-only mode) ---
    T_burn_total  = (delta_tr_pct   / 100.0) * _T_MAX_2
    t_coast_start = (coast_start_pct / 100.0) * T_burn_total
    t_arc3_burn   = T_burn_total - t_coast_start

    plan = [
        # vertical rise: PMP does NOT steer here (α≡0) and carries no costates —
        # the kick both pitches over AND seeds λ₀, so the optimization begins at
        # the post-kick state (costate init AFTER the kick, unit norm fixed there).
        dict(label='vrise',   thrust=_F_THRUST_1, Isp=_ISP_1, term=('time', sim_params.TIME_TO_START_KICK),
             force_alpha=0.0,  post=('kick_init', (gamma_p - np.pi / 2.0, lam0))),
        dict(label='s1burn',  thrust=_F_THRUST_1, Isp=_ISP_1, term=('mass', meco_mass),
             force_alpha=None, post=('mass_drop', stage_drop)),
        dict(label='coast',   thrust=0.0,         Isp=_ISP_1, term=('time', r.TIME_SECOND_ENGINE_IGNITION),
             force_alpha=None, post=None),
        dict(label='s2arc1',  thrust=r.F_THRUST_2, Isp=r.ISP_2, term=('time', t_coast_start),
             force_alpha=None, post=None),
        dict(label='s2coast', thrust=0.0,          Isp=r.ISP_2, term=('time', delta_tc),
             force_alpha=None, post=None),
        dict(label='s2arc3',  thrust=r.F_THRUST_2, Isp=r.ISP_2, term=('time', t_arc3_burn),
             force_alpha=None, post=None),
    ]

    # Liftoff with ZERO costates: the costate ODE is homogeneous in λ, so they
    # stay 0 through the vertical rise and are seeded to λ₀ by the kick_init post.
    # The 9th element is the passive mass costate λ_m (seeded 0 at the kick; its
    # additive integration constant is fixed afterwards by the transversality
    # λ_m(t_f)=0, applied as a constant shift below).
    aug = [0.0, c.R_EARTH, 0.0, np.pi / 2.0, m0, 0.0, 0.0, 0.0, 0.0]
    t = 0.0
    endpoints = {}     # label -> (t_end, aug_end_before_post)
    arc_starts = {}    # label -> aug at arc start (post previous discontinuity)
    sols = []          # (label, thrust, force_alpha, sol) for dense reconstruction
    max_step = _FA_DENSE_STEP if dense else _MAX_STEP

    for arc in plan:
        arc_starts[arc['label']] = list(aug)
        thrust, Isp, fa = arc['thrust'], arc['Isp'], arc['force_alpha']
        rhs = (lambda tt, y, thrust=thrust, Isp=Isp, fa=fa:
               _stage2_ode(tt, y, thrust, Isp, include_drag=include_drag,
                           alpha_max=alpha_max, force_alpha=fa,
                           alpha_cap_qmin=alpha_cap_qmin))
        kind, val = arc['term']

        if kind == 'time':
            if val <= 1e-9:                       # zero-duration arc — skip
                endpoints[arc['label']] = (t, list(aug))
                aug = _apply_post(aug, arc['post'])
                continue
            sol = solve_ivp(rhs, (t, t + val), aug, rtol=_RTOL, atol=_ATOL,
                            max_step=max_step, events=_event_crash)
        else:                                     # ('mass', threshold) — MECO
            sol = solve_ivp(rhs, (t, t + _T_MAX_1 * 2.0), aug,
                            rtol=_RTOL, atol=_ATOL, max_step=max_step,
                            events=[_event_crash, _make_mass_event(val)])
            if len(sol.t_events[1]) == 0:         # never reached MECO (unphysical)
                return {'result': _fa_crashed()}

        if len(sol.t_events[0]) > 0:              # ground collision
            return {'result': _fa_crashed()}

        sols.append((arc['label'], thrust, fa, sol))
        t = float(sol.t[-1])
        aug = list(sol.y[:, -1])
        endpoints[arc['label']] = (t, list(aug))
        aug = _apply_post(aug, arc['post'])

    aug_ign      = endpoints['coast'][1]     # Stage-2 ignition (end of inter-stage coast)
    aug_coast_end = endpoints['s2coast'][1]  # end of Stage-2 coast
    aug_burn_end  = endpoints['s2arc3'][1]   # end of Stage-2 burn (final)

    # Mass-costate transversality: λ_m(t_f) = ∂Φ/∂m = 0 (no terminal-mass cost).
    # λ_m is a passive, additive quadrature, so the physically-correct λ_m is the
    # integrated value shifted by −λ_m(t_f). One constant fixes the whole history.
    lam_m_shift = aug_burn_end[8] if len(aug_burn_end) >= 9 else 0.0

    # OBJECTIVE Hamiltonians use λ_m = 0 (identical to the pre-λ_m full-ascent):
    # λ_m is carried for RIGOR/diagnostics only, so the tuned transversality
    # residual — and hence the optimization — is left unchanged.
    def _H_obj(aug_pt, thrust):
        a = pmp_control_law(aug_pt[6], aug_pt[7], aug_pt[2], alpha_max=alpha_max,
                            alpha_cap_qmin=alpha_cap_qmin, r_val=aug_pt[1])
        return compute_hamiltonian(aug_pt[1], aug_pt[2], aug_pt[3], thrust, aug_pt[4],
                                   a, aug_pt[5], aug_pt[6], aug_pt[7],
                                   include_drag=include_drag)

    # DIAGNOSTIC Hamiltonians include the shifted λ_m·ṁ term (so H is conserved
    # along each powered arc — the check that the arcs are rigorous extremals).
    def _H_diag(aug_pt, thrust, Isp):
        a = pmp_control_law(aug_pt[6], aug_pt[7], aug_pt[2], alpha_max=alpha_max,
                            alpha_cap_qmin=alpha_cap_qmin, r_val=aug_pt[1])
        lam_m = (aug_pt[8] - lam_m_shift) if len(aug_pt) >= 9 else 0.0
        return compute_hamiltonian(aug_pt[1], aug_pt[2], aug_pt[3], thrust, aug_pt[4],
                                   a, aug_pt[5], aug_pt[6], aug_pt[7],
                                   include_drag=include_drag, lam_m=lam_m, Isp=Isp)

    H_burn_start = _H_obj(aug_ign,      r.F_THRUST_2)
    H_coast_end  = _H_obj(aug_coast_end, 0.0)
    H_burn_end   = _H_obj(aug_burn_end,  r.F_THRUST_2)

    state_final = np.array(aug_burn_end[:5])
    t_meco      = endpoints['s1burn'][0]
    t_ignition  = endpoints['coast'][0]

    # Stage-1 physical trajectory (vrise + s1burn) for downstream plotting.
    s1_sols = [s for s in sols if s[0] in ('vrise', 's1burn')]
    if s1_sols:
        t_stage1 = np.concatenate([s[3].t for s in s1_sols])
        y_stage1 = np.concatenate([s[3].y[:5, :] for s in s1_sols], axis=1)
    else:
        t_stage1, y_stage1 = np.array([]), np.array([])

    result = {
        'crashed': False,
        'state_final': state_final,
        'H_burn_start': H_burn_start,
        'H_coast_end':  H_coast_end,
        'H_burn_end':   H_burn_end,
        't_f':  T_burn_total + delta_tc,   # Eq. 27 (Stage-2 powered + coast)
        't_cf': delta_tc,
        't_stage2_start': t_meco,
        't_ignition':     t_ignition,
        't_stage1': t_stage1,
        'y_stage1': y_stage1,
    }

    if verbose:
        h_f = state_final[1] - c.R_EARTH
        print(f"  [full-ascent] MECO t={t_meco:.1f}s, ignition t={t_ignition:.1f}s")
        print(f"  Ascent end: t={t_ignition + result['t_f']:.1f}s, h={h_f/1e3:.1f}km, "
              f"v={state_final[2]:.0f}m/s, gam={np.rad2deg(state_final[3]):.2f}deg")
        print(f"  H_burn_start={H_burn_start:.4f}  H_coast_end={H_coast_end:.4f}  "
              f"H_burn_end={H_burn_end:.4f}   (objective H, lam_m=0)")
        print(f"  lam_m carried for rigor; lam_m(t_f)=0 enforced "
              f"(shift={lam_m_shift:+.4e})")
        # Hamiltonian at the PMP-region corners (Weierstrass–Erdmann check). With
        # the mass costate λ_m carried, H is now conserved ALONG each powered arc
        # (s1 start ≈ s1 end) and continuous across the FREE Stage-2 coast corners.
        # Staging is a forced (fixed-duration) corner, so a small H step there
        # remains (from the drag a_D=D/m term across the mass drop).
        corners = [
            ("s1 burn start (post-kick)", arc_starts['s1burn'],  _F_THRUST_1, _ISP_1),
            ("s1 burn end (MECO)",        endpoints['s1burn'][1], _F_THRUST_1, _ISP_1),
            ("coast start (post-staging)", arc_starts['coast'],   0.0,         _ISP_1),
            ("s2 ignition",               aug_ign,                r.F_THRUST_2, r.ISP_2),
            ("s2 coast end",              aug_coast_end,          0.0,          r.ISP_2),
            ("s2 burn end",               aug_burn_end,           r.F_THRUST_2, r.ISP_2),
        ]
        print("  Hamiltonian at arc corners (transversality diagnostic, lam_m-aware):")
        for name, a_pt, th, isp in corners:
            print(f"    {name:28s} H={_H_diag(a_pt, th, isp):+.5f}")

    out = {'result': result}

    if dense:
        t_parts, y_parts, th_parts, al_parts = [], [], [], []
        for label, thrust, fa, sol in sols:
            if sol is None or len(sol.t) == 0:
                continue
            t_parts.append(sol.t)
            y_parts.append(sol.y[:5, :])
            th_parts.append(np.full(len(sol.t), thrust))
            if fa is not None:
                al_parts.append(np.full(len(sol.t), fa))
            else:
                al_parts.append(np.array([
                    pmp_control_law(sol.y[6, i], sol.y[7, i], sol.y[2, i],
                                    alpha_max=alpha_max, alpha_cap_qmin=alpha_cap_qmin,
                                    r_val=sol.y[1, i])
                    for i in range(sol.y.shape[1])
                ]))
        out['time_full']   = np.concatenate(t_parts)
        out['data_full']   = np.concatenate(y_parts, axis=1)
        out['thrust_full'] = np.concatenate(th_parts)
        out['alpha_full']  = np.concatenate(al_parts)
        out['t_ignition']  = t_ignition

    return out


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
    # Full-ascent mode (opt-in): PMP steers Stage 1 → insertion. Routed through
    # the modular arc engine; the Stage-2-only path below is left untouched.
    # -----------------------------------------------------------------
    full_ascent, include_drag, alpha_max = _resolve_pmp_options()
    if full_ascent:
        return _integrate_full_ascent(
            lambda0_r, lambda0_v, lambda0_g,
            delta_tc, delta_tr_pct, coast_start_pct, gamma_p,
            include_drag=include_drag, alpha_max=alpha_max,
            dense=False, verbose=verbose,
        )['result']

    # -----------------------------------------------------------------
    # Phase 1: Stage 1 gravity turn
    # -----------------------------------------------------------------
    # gamma_p is the pitch maneuver angle in [1.54, 1.57] rad (~ 88–90 deg).
    # With the instantaneous pitch-over now in place, the kick is a discontinuous
    # gamma jump applied exactly at TIME_TO_START_KICK:  gamma_post = pi/2 + kick_angle.
    # Setting kick_angle = gamma_p - pi/2 therefore makes gamma_post == gamma_p
    # exactly — gamma_p is literally the post-kick flight-path angle (and pitch
    # angle, since alpha = 0 in the subsequent gravity turn).
    kick_angle = gamma_p - np.pi / 2.0   # maps [1.54, 1.57] -> [-0.031, -0.001] rad

    # Normalize the initial costate vector to unit norm. The trajectory depends
    # only on the costate DIRECTION (the control law and linear costate ODEs are
    # invariant to positive scaling), while the Hamiltonian — and hence the
    # transversality residual — scales linearly with the costate magnitude.
    # Pinning ‖λ₀‖=1 fixes that free gauge so the transversality penalty cannot
    # be driven to zero by simply shrinking the costates.
    lambda0_r, lambda0_v, lambda0_g = _normalize_costates(
        lambda0_r, lambda0_v, lambda0_g
    )

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
    state2_init = _strip_to_pmp_state(
        state2_init, np.deg2rad(sim_params.LAUNCH_LATITUDE)
    )

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
        t_span=(t2_start, t_ignition),
        y0=aug0_preig,
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
        events=_event_crash,
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
            t_span=(t_ignition, t_arc1_end),
            y0=aug_state_ign,
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        if len(sol_arc1.t_events[0]) > 0:
            return {
                'crashed': True,
                'state_final': None,
                'H_burn_start': H_burn_start,
                'H_coast_end': 0.0, 'H_burn_end': 0.0,
                't_f': 0.0, 't_cf': 0.0,
                't_stage2_start': t2_start, 't_ignition': t_ignition,
                't_stage1': t_stage1, 'y_stage1': y_stage1,
            }
        aug_state_arc2 = list(sol_arc1.y[:, -1])
        t_arc2_start = float(sol_arc1.t[-1])
    else:
        aug_state_arc2 = aug_state_ign
        t_arc2_start = t_ignition

    # Paper Eq. 27: J = t_f - t_cf = total powered time = T_burn_total
    # t_f  = total Stage-2 flight time (powered + coast), computed from plan
    # t_cf = coast duration only
    # Using planned values avoids ODE endpoint overshoot artifacts.
    t_f_result  = T_burn_total + delta_tc
    t_cf_result = delta_tc

    # ------------------------------------------------------------------
    # Arc 2: coast  (t_arc2_start → t_arc2_start + delta_tc)
    # ------------------------------------------------------------------
    t_arc2_end = t_arc2_start + delta_tc

    if delta_tc > 0.01:
        sol_arc2 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2),
            t_span=(t_arc2_start, t_arc2_end),
            y0=aug_state_arc2,
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        if len(sol_arc2.t_events[0]) > 0:
            return {
                'crashed': True,
                'state_final': None,
                'H_burn_start': H_burn_start,
                'H_coast_end': 0.0, 'H_burn_end': 0.0,
                't_f': 0.0, 't_cf': 0.0,
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
        # Honest H at the (zero-duration) coast endpoint: same state as end
        # of Arc 1, but evaluated with thrust=0 so the thrust contribution to
        # H_geom is removed (matches the > 0.01 branch in the limit
        # delta_tc → 0). The previous shortcut H_coast_end = H_burn_start
        # under-counted the residual by (T/m)·D and biased PSO toward
        # near-zero-coast solutions.
        H_coast_end = compute_hamiltonian(
            aug_state_arc3[1], aug_state_arc3[2], aug_state_arc3[3],
            0.0, aug_state_arc3[4],
            pmp_control_law(aug_state_arc3[6], aug_state_arc3[7], aug_state_arc3[2]),
            aug_state_arc3[5], aug_state_arc3[6], aug_state_arc3[7],
        )

    # ------------------------------------------------------------------
    # Arc 3: thrust  (t_arc3_start → t_arc3_start + t_arc3_burn)
    # ------------------------------------------------------------------
    t_arc3_end = t_arc3_start + t_arc3_burn

    if t_arc3_burn > 0.01:
        sol_arc3 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2),
            t_span=(t_arc3_start, t_arc3_end),
            y0=aug_state_arc3,
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP,
            events=_event_crash,
        )
        if len(sol_arc3.t_events[0]) > 0:
            return {
                'crashed': True,
                'state_final': None,
                'H_burn_start': H_burn_start,
                'H_coast_end': H_coast_end, 'H_burn_end': 0.0,
                't_f': 0.0, 't_cf': 0.0,
                't_stage2_start': t2_start, 't_ignition': t_ignition,
                't_stage1': t_stage1, 'y_stage1': y_stage1,
            }
        aug_final = sol_arc3.y[:, -1]
    else:
        aug_final = np.array(aug_state_arc3)

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
        t_end_abs = t_ignition + t_f_result  # absolute end time for display
        print(f"  Stage 2 end: t={t_end_abs:.1f}s, h={h_f/1e3:.1f}km, "
              f"v={state_final[2]:.0f}m/s, gam={np.rad2deg(state_final[3]):.2f}deg")
        print(f"  H_burn_start={H_burn_start:.4f}  H_coast_end={H_coast_end:.4f}  "
              f"H_burn_end={H_burn_end:.4f}")

    return {
        'crashed': False,
        'state_final': state_final,
        'H_burn_start': H_burn_start,
        'H_coast_end':  H_coast_end,
        'H_burn_end':   H_burn_end,
        't_f':  t_f_result,
        't_cf': t_cf_result,
        't_stage2_start': t2_start,
        't_ignition':     t_ignition,
        't_stage1': t_stage1,
        'y_stage1': y_stage1,
    }


# ===========================================================================
# Augmented objective function  (Eq. 39)
# ===========================================================================

CRASH_PENALTY = 1e20


def _objective_terms(result):
    """Weighted, non-dimensional contributions to J' — single source of truth.

    Every term is non-dimensionalised so the weights in
    ``simulation_parameters`` are unitless and directly comparable:

        J_nd  = (t_f − t_cf) / T_MAX_2          burn time as a fraction of the
                                                propellant-limited maximum  ∈ [0, 1]
        Δh_nd = (r_f − r_target) / h_target     relative altitude error
        ΔV_nd = (V_f − V_circular) / V_circular relative velocity error
        Δγ_nd = γ_f / γ_ref                     FPA error in units of γ_ref (deg)
        tv_nd = (H_be + H_ce − H_bs) / V_circ   transversality residual; H scales
                                                like ṙ (velocity), so divide by V_circ

    Both ``compute_augmented_objective`` and ``breakdown_objective`` consume
    this, so they cannot drift out of sync.
    """
    state = result['state_final']
    r_val, v_f, g_f = state[1], state[2], state[3]

    r_target   = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    # Rotating-frame circular target: the trajectory velocity is ground-relative,
    # so credit Earth's surface rotation speed once here. Zero when rotation off.
    if sim_params.ENABLE_EARTH_ROTATION:
        v_rot = c.OMEGA_EARTH * r_target * np.cos(np.deg2rad(sim_params.LAUNCH_LATITUDE))
    else:
        v_rot = 0.0
    v_circular = np.sqrt(c.MU_EARTH / r_target) - v_rot
    gamma_ref  = np.deg2rad(sim_params.GAMMA_REF_DEG)

    J_nd  = (result['t_f'] - result['t_cf']) / _T_MAX_2
    dh_nd = (r_val - r_target) / sim_params.TARGET_ORBITAL_ALTITUDE
    dv_nd = (v_f - v_circular) / v_circular
    dg_nd = g_f / gamma_ref
    transv = result['H_burn_end'] + result['H_coast_end'] - result['H_burn_start']
    tv_nd  = transv / v_circular

    return {
        'J'     : sim_params.PENALTY_W_J         * J_nd,
        'alt'   : sim_params.PENALTY_W_ALTITUDE  * abs(dh_nd),
        'vel'   : sim_params.PENALTY_W_VELOCITY  * abs(dv_nd),
        'fpa'   : sim_params.PENALTY_W_FPA       * abs(dg_nd),
        'transv': sim_params.PENALTY_W_TRANSVERS * abs(tv_nd),
    }


def compute_augmented_objective(result):
    """
    Augmented objective J' (Eq. 39 of the paper), non-dimensional form:

        J' = w_J·J_nd + s1|Δh_nd| + s2|ΔV_nd| + s3|Δγ_nd| + s4|tv_nd| + C

    where C = 10^20 if the trajectory crashed / is unphysical, else 0.
    See ``_objective_terms`` for the term definitions.

    Parameters
    ----------
    result : dict   Output of ``run_indirect_trajectory``

    Returns
    -------
    J_prime : float  Augmented objective value
    """
    if result['crashed'] or result['state_final'] is None:
        return CRASH_PENALTY

    # --- Trajectory constraint penalty C (Eq. 40) ---
    state = result['state_final']
    r_val, v_f = state[1], state[2]
    C = 0.0
    if (r_val - c.R_EARTH) < 0:           # below ground
        C = CRASH_PENALTY
    elif v_f < 0:                         # negative velocity (unphysical)
        C = CRASH_PENALTY

    return float(sum(_objective_terms(result).values()) + C)


def breakdown_objective(result):
    """Decompose J' into its individual (weighted, non-dimensional) terms.

    Returns a dict with keys: J, alt, vel, fpa, transv.  For a non-crashed
    trajectory the values sum to the same J' as ``compute_augmented_objective``
    (ignoring the crash penalty C).
    """
    if result['crashed'] or result['state_final'] is None:
        return {'J': 1e20, 'alt': 1e20, 'vel': 1e20, 'fpa': 1e20, 'transv': 1e20}

    return _objective_terms(result)


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
        # Full-ascent mode may widen the γ_p (kick-angle) range: the kick now
        # seeds the whole PMP ascent, so the Stage-2-only range [1.54, 1.57] is
        # often too tight. Overrides ONLY element 6 and ONLY when full-ascent is
        # on, so the shared Stage-2-only bounds are untouched.
        lb = list(sim_params.PSO_LB)
        ub = list(sim_params.PSO_UB)
        full_ascent, _, _ = _resolve_pmp_options()
        if full_ascent:
            gp = getattr(sim_params, "INDIRECT_PMP_FULL_ASCENT_GAMMA_P_BOUNDS", None)
            if gp is not None:
                lb[6], ub[6] = float(gp[0]), float(gp[1])
        return (lb, ub)

    def get_nobj(self):
        return 1


# ===========================================================================
# PSO runner
# ===========================================================================

def run_pso_optimization(verbose=True, n_particles=None, n_gen=None):
    """
    Run the PSO optimisation as described in the paper (Sect. 4.2.2).

    Attempts to use PyGMO (``pygmo``) first.

    Parameters
    ----------
    verbose : bool   Print progress and final result if True.
    n_particles, n_gen : int or None
        Swarm size / generation count overrides. None ⇒ use the configured
        ``PSO_N_PARTICLES`` / ``PSO_MAX_GENERATIONS``. The segmented PMP-reference
        build passes higher values here to raise the reference fidelity without
        touching the indirect_pmp mode's settings.

    Returns
    -------
    optimal_params : list  [lambda0_r, lambda0_v, lambda0_g, delta_tc,
                             delta_tr_pct, coast_start_pct, gamma_p]
    J_optimal      : float  Best augmented objective value achieved
    """
    n_particles = sim_params.PSO_N_PARTICLES if n_particles is None else int(n_particles)
    n_gen       = sim_params.PSO_MAX_GENERATIONS if n_gen is None else int(n_gen)
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
            seed     = sim_params.PSO_SEED,
        ))
        if verbose:
            algo.set_verbosity(25)   # print every 25 generations

        pop = pg.population(prob, size=n_particles, seed=sim_params.PSO_SEED)
        pop = algo.evolve(pop)

        best_x = list(pop.champion_x)
        best_f = float(pop.champion_f[0])

        # Capture the per-generation convergence log (best J' over generations).
        # PyGMO's pso log rows are (gen, fevals, gbest, mean_vel, mean_lbest,
        # avg_dist); only populated when verbosity was set (verbose path). The
        # log samples every `set_verbosity` generations, so append the final
        # (n_gen, best_f) point if it isn't already the last logged generation.
        global LAST_PSO_HISTORY
        uda = algo.extract(pg.pso)
        log = uda.get_log() if uda is not None else []
        if log:
            gens  = [row[0] for row in log]
            gbest = [row[2] for row in log]
            if gens[-1] != n_gen:
                gens.append(n_gen)
                gbest.append(best_f)
            LAST_PSO_HISTORY = {'gen': np.array(gens), 'gbest': np.array(gbest)}
        else:
            LAST_PSO_HISTORY = None

        if verbose:
            print(f"\n[PyGMO PSO] Finished in {time.time()-t_start:.1f}s")
            print(f"  Best J' = {best_f:.4f}")
            _print_solution(best_x, best_f)

        return best_x, best_f

    except ImportError:
        raise ImportError(
            "pygmo is required for the indirect PMP optimisation. "
            "Install it with: conda install -c conda-forge pygmo"
        )


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

    # Full-ascent mode: reconstruct the dense trajectory from the same modular
    # arc engine used by the objective, so plots/reference match the optimum.
    full_ascent, include_drag, alpha_max = _resolve_pmp_options()
    if full_ascent:
        out = _integrate_full_ascent(
            lambda0_r, lambda0_v, lambda0_g,
            delta_tc, delta_tr_pct, coast_start_pct, gamma_p,
            include_drag=include_drag, alpha_max=alpha_max,
            dense=True, verbose=verbose,
        )
        if out['result']['crashed']:
            raise RuntimeError("Full-ascent PMP trajectory crashed during plotting run.")
        return (out['time_full'], out['data_full'], out['thrust_full'],
                out['alpha_full'], out['t_ignition'], out['result'])

    # Match run_indirect_trajectory: the trajectory uses the unit-norm costates.
    lambda0_r, lambda0_v, lambda0_g = _normalize_costates(
        lambda0_r, lambda0_v, lambda0_g
    )

    kick_angle = gamma_p - np.pi / 2.0

    # --- Stage 1 ---
    t2_start, state2_init, t_meco, t_stage1, y_stage1, crashed = ra.run_stage1(kick_angle)

    if crashed:
        raise RuntimeError("Stage 1 crashed during full-trajectory plotting run.")

    t_ignition = t2_start + _T_IGNITION_DELAY

    # Strip optional lat/heading — PMP ODE needs exactly [s, r, v, gamma, m]
    state2_init = _strip_to_pmp_state(
        state2_init, np.deg2rad(sim_params.LAUNCH_LATITUDE)
    )

    # Sanity check: _T_MAX_2 is built from M_PROP_2, so a full burn assumes the
    # mass handed over by Stage 1 equals the Stage-2 wet mass. Warn if it drifts.
    if verbose:
        m_stage2_expected = r.M_STRUCTURE_2 + r.M_PROP_2 + r.M_PAYLOAD
        m_handoff = state2_init[4]
        if abs(m_handoff - m_stage2_expected) > 0.01 * m_stage2_expected:
            print(f"  [warn] Stage-2 handoff mass {m_handoff:.0f} kg differs from "
                  f"expected wet mass {m_stage2_expected:.0f} kg by "
                  f"{100*(m_handoff-m_stage2_expected)/m_stage2_expected:+.1f}% — "
                  f"_T_MAX_2 (propellant cap) may be inconsistent.")

    # timing
    T_burn_total  = (delta_tr_pct   / 100.0) * _T_MAX_2
    t_coast_start = (coast_start_pct / 100.0) * T_burn_total
    t_arc3_burn   = T_burn_total - t_coast_start

    n_state = 5   # always 5 after stripping
    aug0_preig = list(state2_init) + [0.0, 0.0, 0.0]

    _dt = 0.5   # output step for plotting

    def _make_teval(t0, t1):
        # Include the exact endpoint so each arc is sampled at its planned end
        # (np.arange excludes the stop value). This keeps the plotted/reported
        # terminal state identical to the objective's run_indirect_trajectory.
        pts = np.arange(t0, t1, _dt)
        if len(pts) == 0 or pts[-1] < t1:
            pts = np.append(pts, t1)
        return pts

    # --- Pre-ignition coast ---
    sol_pre = solve_ivp(
        lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2),
        t_span=(t2_start, t_ignition),
        y0=aug0_preig,
        t_eval=_make_teval(t2_start, t_ignition),
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP, events=_event_crash,
    )
    state_at_ign = sol_pre.y[:n_state, -1].copy()
    aug_ign = list(state_at_ign) + [lambda0_r, lambda0_v, lambda0_g]

    # --- Arc 1 (thrust) ---
    t_arc1_end = t_ignition + t_coast_start
    if t_coast_start > 0.01:
        sol1 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2),
            t_span=(t_ignition, t_arc1_end),
            y0=aug_ign,
            t_eval=_make_teval(t_ignition, t_arc1_end),
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP, events=_event_crash,
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
            t_span=(t_arc2_start, t_arc2_end),
            y0=aug_arc2,
            t_eval=_make_teval(t_arc2_start, t_arc2_end),
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP, events=_event_crash,
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
            t_span=(t_arc3_start, t_arc3_end),
            y0=aug_arc3,
            t_eval=_make_teval(t_arc3_start, t_arc3_end),
            rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP, events=_event_crash,
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

    # Stage-1 thrust/alpha are recorded inside the RHS (rocket_dynamics) at the
    # solver's RK evaluations, NOT on the t_stage1 output grid — so they have a
    # different length and time mapping. Interpolate each history onto t_stage1
    # using its own paired timestamps before concatenating with Stage 2.
    # (A previous index-slice zeroed all of Stage 1 whenever the grid was denser
    # than the RHS-eval count, leaving plots showing only Stage 2.)
    from Plots.plot_state_utils import interpolate_to_time
    thrust_stage1 = interpolate_to_time(ra.time_history, ra.thrust_history, t_stage1)
    alpha_stage1  = interpolate_to_time(ra.alpha_time_history, ra.alpha_history, t_stage1)

    thrust_full = np.concatenate([thrust_stage1, thrust_stage2])
    alpha_full  = np.concatenate([alpha_stage1,  alpha_stage2])

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
    # The trajectory uses the unit-norm costates; report those (the raw PSO
    # values only matter through their direction).
    n_lr, n_lv, n_lg = _normalize_costates(lambda0_r, lambda0_v, lambda0_g)
    print("\nOptimal parameters:")
    print(f"  lam0_r        = {lambda0_r:.6f}  (normalised {n_lr:.6f})")
    print(f"  lam0_v        = {lambda0_v:.6f}  (normalised {n_lv:.6f})")
    print(f"  lam0_gam      = {lambda0_g:.6f}  (normalised {n_lg:.6f})")
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
        bd = breakdown_objective(result)
        burn_s = result['t_f'] - result['t_cf']
        print(f"\nJ prime breakdown:")
        print(f"  J term (burn frac):   {bd['J']:.4f}  (burn time {burn_s:.1f} s of {_T_MAX_2:.1f} s max)")
        print(f"  Altitude penalty:     {bd['alt']:.4f}")
        print(f"  Velocity penalty:     {bd['vel']:.4f}")
        print(f"  FPA penalty:          {bd['fpa']:.4f}")
        print(f"  Transversality:       {bd['transv']:.4f}")
        print(f"  Total J prime:        {sum(bd.values()):.4f}")
        if result['t_cf'] < 1.0:
            print(f"  (coast = {result['t_cf']:.2f} s -- direct insertion trajectory)")
