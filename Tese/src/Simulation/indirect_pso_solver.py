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

Stage 2 is flown, by default, in the rotating frame with the same Coriolis and
centrifugal terms every other architecture carries, the costate equations kept as
published (INDIRECT_PMP_STAGE2_FRAME, see the "Stage-2 frame and force model"
block below for the measured size of what they omit and for the two older forms).

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
)
import Simulation.rocket_ascent as ra
from Auxiliary import earth_rotation as earth_rot


# ---------------------------------------------------------------------------
# Derived Stage-2 constants (computed once at import time)
# ---------------------------------------------------------------------------
_MDOT_2 = r.F_THRUST_2 / (r.ISP_2 * c.G_0)           # Stage-2 mass flow rate [kg/s]
_T_MAX_2 = r.M_PROP_2 / _MDOT_2                        # Time to deplete ALL Stage-2 propellant [s]
# Engine-ignition delay measured from stage separation:
_T_IGNITION_DELAY = r.TIME_SECOND_ENGINE_IGNITION - r.TIME_First_STAGE_SEPARATION

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
    """Return [s, r, v, γ, m] as Stage 1 hands it over — ground-relative.

    Any frame change is ``_to_stage2_frame``'s job, applied after the fairing
    check.  (``lat_fallback_rad`` is retained for call-site compatibility.)
    """
    return np.array(state[:5], dtype=float)


# ===========================================================================
# Stage-2 frame and force model
# ===========================================================================
# The Stage-2 costate equations (Eqs. 30b-30d) are -(dH/dx)^T of the drag-free,
# rotation-free EOM. Three forms of the arc exist (INDIRECT_PMP_STAGE2_FRAME):
#
#   "rotating_pseudo_forces" (default since 2026-09-16, decision 7d): the
#       ground-relative state is propagated in the rotating frame WITH the same
#       Coriolis/centrifugal terms every other architecture carries -- the very
#       call pso_coast_solver._stage2_ode_guidance makes, latitude from downrange,
#       heading held at the launch azimuth -- against the laws' own target
#       sqrt(mu/r) - v_rot. The costate equations are kept as published and so
#       omit the partial derivatives of those terms: measured along the arc they
#       are 0.01-0.4 % of the retained gravity/kinematic partials (one entry
#       1.4 % where the retained term crosses zero), and the dH/ds they would
#       give lambda_s integrates to ~1.7e-6 over a 2000 s coast for unit costates
#       (tests/test_pmp_stage1_pseudo_forces.py keeps that measured). The control
#       law, Eq. 34, is exact -- alpha does not appear in the pseudo-forces --
#       and every Hamiltonian the transversality penalty reads is evaluated with
#       the rates actually flown (_hamiltonian_at). With the terms in the state
#       equations the level-flight condition at the target IS the laws' target
#       (exactly so for a due-east launch at the equator: gamma_dot = 0 at
#       v = sqrt(mu/r) - omega*r), so the ellipse defect of the legacy form does
#       not arise. One force model and one credit convention for all five
#       architectures.
#   "inertial" (2026-09-13 to 2026-09-16): the hand-off converted at separation
#       with the exact planar transform, propagated rotation-free against
#       sqrt(mu/r), converted back for every consumer. The published formulation
#       to the letter (pallone2016: rotating lower stages, inertial costate arc),
#       but not the physics the other architectures fly except for a due-east
#       launch at the equator: the transform credits the full omega*r*cos(lat)
#       along-track (413.8 m/s at hand-off) where the pseudo-force terms credit
#       the azimuth-projected part along the drifting latitude (292.5 m/s) --
#       121 m/s, about 944 kg of Stage-2 propellant, and 125 km / 296 m/s of
#       divergence after an 1883 s coast. Kept to reproduce archived rows.
#   "rotating" (until 2026-09-13): ground-relative state, rotation-free
#       equations, target sqrt(mu/r) - v_rot. In those equations that target is
#       the apoapsis of an ellipse whose periapsis is ~890 km below the surface,
#       and a local refinement reached it ballistically by deleting the
#       circularisation burn (dev-notes/pmp_local_refine.py). Kept only to
#       reproduce archived rows.
#
# Everything reported outward is ground-relative in every form (_from_stage2_frame).

_STAGE2_FRAMES = ("rotating_pseudo_forces", "inertial", "rotating")


def _stage2_frame():
    """The validated INDIRECT_PMP_STAGE2_FRAME setting."""
    frame = getattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "rotating_pseudo_forces")
    if frame not in _STAGE2_FRAMES:
        raise ValueError("INDIRECT_PMP_STAGE2_FRAME must be one of "
                         f"{_STAGE2_FRAMES}, got {frame!r}")
    return frame


def _stage2_inertial():
    """True when the Stage-2 arc is propagated and targeted in the inertial frame."""
    return _stage2_frame() == "inertial" and bool(sim_params.ENABLE_EARTH_ROTATION)


def _stage2_pseudo_forces():
    """True when the Stage-2 arc carries the rotating-frame pseudo-forces in its
    state equations (the "rotating_pseudo_forces" form). Inert, like the terms
    themselves, with the rotation or INCLUDE_PSEUDO_FORCES off."""
    return (_stage2_frame() == "rotating_pseudo_forces"
            and bool(sim_params.ENABLE_EARTH_ROTATION)
            and bool(sim_params.INCLUDE_PSEUDO_FORCES))


def _stage2_frame_flown(pseudo_forces):
    """Label of the form the arc was actually propagated in."""
    if _stage2_inertial():
        return "inertial"
    return "rotating_pseudo_forces" if pseudo_forces else "rotating"


def _stage1_pseudo_forces():
    """Whether the PMP's Stage 1 carries the rotating-frame pseudo-forces.

    Stage 1 is run_stage1's gravity turn, flown before any costate exists, so the
    terms can be carried there without touching the Stage-2 formulation. What is
    refused is one force model before staging and a different one after it, the
    mix ra.set_pseudo_forces_for_run exists to prevent: Stage 1 with the terms
    over the legacy pseudo-force-free "rotating" Stage 2, or Stage 1 without them
    under a "rotating_pseudo_forces" Stage 2. The inertial Stage 2 has no such
    term either way and accepts both. Inert, like the terms themselves, with the
    rotation or INCLUDE_PSEUDO_FORCES off."""
    on = bool(getattr(sim_params, "INDIRECT_PMP_STAGE1_PSEUDO_FORCES", True))
    terms = bool(sim_params.ENABLE_EARTH_ROTATION) and bool(sim_params.INCLUDE_PSEUDO_FORCES)
    frame = _stage2_frame()
    if terms and on and frame == "rotating":
        raise ValueError(
            "INDIRECT_PMP_STAGE1_PSEUDO_FORCES=True carries the pseudo-forces through "
            "Stage 1, and the legacy INDIRECT_PMP_STAGE2_FRAME='rotating' form is "
            "pseudo-force-free: two force models in one ascent. Use "
            "'rotating_pseudo_forces' or 'inertial' for Stage 2, or set "
            "INDIRECT_PMP_STAGE1_PSEUDO_FORCES=False to reproduce the runs flown under it.")
    if terms and not on and frame == "rotating_pseudo_forces":
        raise ValueError(
            "INDIRECT_PMP_STAGE2_FRAME='rotating_pseudo_forces' carries the pseudo-forces "
            "through Stage 2, and INDIRECT_PMP_STAGE1_PSEUDO_FORCES=False leaves them out "
            "of Stage 1: two force models in one ascent. Set the Stage-1 flag True, or "
            "use the 'inertial' form to reproduce the runs flown under the exemption.")
    return on


def terminal_speed_target(r_target=None):
    """Terminal speed the Stage-2 arc is scored against, in the frame it is flown in.

    √(μ/r) when inertial (or with the rotation off); √(μ/r) − v_rot -- the target
    every other architecture flies to -- under the two rotating-frame forms."""
    if r_target is None:
        r_target = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    if _stage2_inertial():
        return float(np.sqrt(c.MU_EARTH / r_target))
    return float(earth_rot.v_circular_rotating(
        r_target, np.deg2rad(sim_params.LAUNCH_LATITUDE), sim_params.ENABLE_EARTH_ROTATION))


def _to_stage2_frame(state):
    """Ground-relative hand-off state -> the frame the Stage-2 arc is flown in.

    Exact planar transform, rotation credit ω·r·cos(LAUNCH_LATITUDE) along-track:
    the credit convention of the target and the archive, but with the flight-path
    angle rotated too (``ecef_to_eci_velocity`` keeps it, harmless at insertion and
    not at a ~27° hand-off). Identity under the legacy "rotating" form."""
    if not _stage2_inertial():
        return state
    out = np.array(state, dtype=float)
    out[2], out[3] = earth_rot.rotating_to_inertial_planar(
        out[2], out[3], np.deg2rad(sim_params.LAUNCH_LATITUDE), out[1])
    return out


def _from_stage2_frame(state, t, t_stage2_start):
    """Stage-2 state(s) -> the ground-relative convention every consumer reads.

    Takes one state or a (5, N) array with times ``t``. Downrange is corrected too:
    ṡ differs between the frames by (R_E/r)·ω·r·cos(lat) = R_E·ω·cos(lat), a
    constant. Identity under the legacy "rotating" form."""
    if not _stage2_inertial():
        return state
    lat_rad = np.deg2rad(sim_params.LAUNCH_LATITUDE)
    out = np.array(state, dtype=float)
    out[2], out[3] = earth_rot.inertial_to_rotating_planar(out[2], out[3], lat_rad, out[1])
    out[0] = out[0] - (c.R_EARTH * c.OMEGA_EARTH * np.cos(lat_rad)
                       * (np.asarray(t, dtype=float) - t_stage2_start))
    return out


# ===========================================================================
# Stage-2 augmented ODE
# ===========================================================================

def _stage2_pseudo_rates(s, r_val, v, gamma):
    """(delta_dvdt, delta_dgammadt) of the rotating-frame pseudo-forces at a
    ground-relative state: the same call, with the same latitude-from-downrange
    and the heading held at the launch azimuth, as
    pso_coast_solver._stage2_ode_guidance makes for every other architecture."""
    lat = ra.get_latitude_from_downrange(s)
    delta_dvdt, delta_dgammadt, *_ = earth_rot.rotating_frame_pseudoforce_rates(
        v, gamma, ra.LAUNCH_AZIMUTH, lat, r_val)
    return float(delta_dvdt), float(delta_dgammadt)


def _stage2_state_rates(s, r_val, v, gamma, m, thrust, Isp, alpha, pseudo_forces):
    """d[s, r, v, gamma, m]/dt of the Stage-2 arc: drag-free, inverse-square
    gravity, thrust at angle of attack ``alpha``, plus the rotating-frame
    pseudo-forces when ``pseudo_forces`` (the "rotating_pseudo_forces" form)."""
    _EPS = 1e-10
    mu = c.MU_EARTH

    cg = np.cos(gamma)
    sg = np.sin(gamma)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    g_local = mu / r_val ** 2
    T_over_m = (thrust / m) if m > _EPS else 0.0

    dsdt    = (c.R_EARTH / r_val) * v * cg
    drdt    = v * sg
    dvdt    = T_over_m * ca - g_local * sg
    if abs(v) < _EPS:
        dgdt = 0.0
    else:
        dgdt = (1.0 / v) * (T_over_m * sa - (g_local - v ** 2 / r_val) * cg)
    dmdt    = -thrust / (Isp * c.G_0) if thrust > 0 and m > _EPS else 0.0

    if pseudo_forces:
        d_v, d_g = _stage2_pseudo_rates(s, r_val, v, gamma)
        dvdt += d_v
        dgdt += d_g

    return [dsdt, drdt, dvdt, dgdt, dmdt]


def _stage2_ode(t, aug_state, thrust, Isp, pseudo_forces=False):
    """
    Right-hand side for the augmented Stage-2 ODE.

    aug_state = [s, r, v, γ, m, λ_r, λ_v, λ_γ]
    (indices 0-4 = physical state, 5-7 = costates)

    The PMP control law computes α from the current costates. The state is
    drag-free; it carries the rotating-frame pseudo-forces when ``pseudo_forces``
    (see the frame block above). The costate equations are the published ones in
    every form.

    Parameters
    ----------
    t             : float   Current time [s]  (required by solve_ivp but unused here)
    aug_state     : array   Augmented state (8 elements)
    thrust        : float   Current thrust force [N]  (0 during coast arcs)
    Isp           : float   Specific impulse [s]
    pseudo_forces : bool    Add Coriolis/centrifugal to the state rates

    Returns
    -------
    derivatives : list  d(aug_state)/dt  (8 elements)
    """
    s, r_val, v, gamma, m = aug_state[:5]
    lam_r, lam_v, lam_g   = aug_state[5], aug_state[6], aug_state[7]

    # Angle of attack from PMP
    alpha = pmp_control_law(lam_v, lam_g, v)

    dx = _stage2_state_rates(s, r_val, v, gamma, m, thrust, Isp, alpha, pseudo_forces)

    # --- Costate derivatives ---
    dlams = costate_derivatives(r_val, v, gamma, thrust, m, lam_r, lam_v, lam_g, alpha)

    return dx + dlams


def _hamiltonian_at(state, lams, thrust, pseudo_forces):
    """H = λ·f at a state [s, r, v, γ, m] with costates (λ_r, λ_v, λ_γ), the
    control law applied, f being the rates actually flown: the pseudo-force
    contributions are included when the arc carries them."""
    s, r_val, v, gamma, m = (float(x) for x in state[:5])
    lam_r, lam_v, lam_g = (float(x) for x in lams)
    alpha = pmp_control_law(lam_v, lam_g, v)
    d_v, d_g = _stage2_pseudo_rates(s, r_val, v, gamma) if pseudo_forces else (0.0, 0.0)
    return compute_hamiltonian(r_val, v, gamma, thrust, m, alpha, lam_r, lam_v, lam_g,
                               delta_dvdt=d_v, delta_dgammadt=d_g)


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
    # gamma_p is the pitch maneuver angle in [1.54, 1.57] rad (~ 88–90 deg).
    # With the instantaneous pitch-over now in place, the kick is a discontinuous
    # gamma jump applied exactly at TIME_TO_START_KICK:  gamma_post = pi/2 + kick_angle.
    # Setting kick_angle = gamma_p - pi/2 therefore makes gamma_post == gamma_p
    # exactly — gamma_p is literally the post-kick flight-path angle (and pitch
    # angle, since alpha = 0 in the subsequent gravity turn).
    # Stage 1 is run_stage1's fixed gravity turn, flown before any costate exists,
    # so it carries the rotating-frame pseudo-forces like every other architecture
    # (INDIRECT_PMP_STAGE1_PSEUDO_FORCES; False reproduces the fully exempt runs
    # flown until 2026-09-16, which handed Stage 2 a state 9.1 km lower, 44 m/s
    # faster and 4.5 deg shallower than the identical Stage 1 of every other
    # case). Stage 2 follows INDIRECT_PMP_STAGE2_FRAME (frame block above): the
    # default carries the same terms in its state equations with the published
    # costate equations, the inertial form has no such term, and the legacy
    # rotating form is refused alongside a Stage 1 that has them.
    # _stage1_pseudo_forces() enforces the one-force-model-per-ascent rule of
    # ra.set_pseudo_forces_for_run.
    ra.set_pseudo_forces_for_run(_stage1_pseudo_forces())
    # Whether the Stage-2 state equations add the terms: only the
    # "rotating_pseudo_forces" form, and only with the run switch on -- tied to
    # the switch the driving solver set, never read off the config alone.
    pf2 = _stage2_pseudo_forces() and bool(ra._PSEUDO_FORCES_THIS_RUN)
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
    # run_stage1 hands Stage 2 a state that still carries the fairing;
    # shed it here if the jettison criterion is already met.
    state2_init = ra.shed_fairing_if_due(t2_start, state2_init)

    if verbose:
        h2 = state2_init[1] - c.R_EARTH
        print(f"  Stage 1 end: t={t2_start:.1f}s, h={h2/1e3:.1f}km, "
              f"v={state2_init[2]:.0f}m/s, gam={np.rad2deg(state2_init[3]):.2f}deg, "
              f"m={state2_init[4]:.0f}kg")

    # Into the frame the Stage-2 arc is flown in (identity unless inertial).
    state2_init = _to_stage2_frame(state2_init)

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
        lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2, pf2),
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

    # Second chance, for a trajectory that staged below the criterion:

    # the pre-ignition coast climbs fast, so the crossing lands here.

    state_at_ignition = ra.shed_fairing_if_due(t_ignition, state_at_ignition)

    # -----------------------------------------------------------------
    # Initialise augmented state with PSO-provided costates
    # -----------------------------------------------------------------
    aug_state_ign = list(state_at_ignition) + [lambda0_r, lambda0_v, lambda0_g]

    # Record H at start of guided burn (for transversality condition, Eq. 38)
    H_burn_start = _hamiltonian_at(state_at_ignition, (lambda0_r, lambda0_v, lambda0_g),
                                   r.F_THRUST_2, pf2)

    # ------------------------------------------------------------------
    # Arc 1: thrust  (t_ignition → t_ignition + t_coast_start)
    # ------------------------------------------------------------------
    t_arc1_end = t_ignition + t_coast_start

    if t_coast_start > 0.01:
        sol_arc1 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2, pf2),
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

    # H at the END of the first thrust arc, engine still on. Diagnostic only:
    # stationarity of the burn time in the first-burn duration equates it with
    # H_last_burn_start (dev-notes/pmp_duration_conditions.py).
    H_burn1_end = _hamiltonian_at(aug_state_arc2[:5], aug_state_arc2[5:8], r.F_THRUST_2, pf2)

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
            lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2, pf2),
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
        H_coast_end = _hamiltonian_at(aug_state_arc3[:5], aug_state_arc3[5:8], 0.0, pf2)
    else:
        aug_state_arc3 = aug_state_arc2
        t_arc3_start = t_arc2_start
        # Honest H at the (zero-duration) coast endpoint: same state as end
        # of Arc 1, but evaluated with thrust=0 so the thrust contribution to
        # H_geom is removed (matches the > 0.01 branch in the limit
        # delta_tc → 0). The previous shortcut H_coast_end = H_burn_start
        # under-counted the residual by (T/m)·D and biased PSO toward
        # near-zero-coast solutions.
        H_coast_end = _hamiltonian_at(aug_state_arc3[:5], aug_state_arc3[5:8], 0.0, pf2)

    # H at the start of the LAST thrust arc: the coast-end state with the engine
    # on. Diagnostic only -- Pontani (2014) writes the transversality condition
    # with this instant as H_0^last, while the objective below uses Stage-2
    # ignition (H_burn_start).
    H_last_burn_start = _hamiltonian_at(aug_state_arc3[:5], aug_state_arc3[5:8],
                                        r.F_THRUST_2, pf2)

    # ------------------------------------------------------------------
    # Arc 3: thrust  (t_arc3_start → t_arc3_start + t_arc3_burn)
    # ------------------------------------------------------------------
    t_arc3_end = t_arc3_start + t_arc3_burn

    if t_arc3_burn > 0.01:
        sol_arc3 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2, pf2),
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
        t_final_abs = float(sol_arc3.t[-1])
    else:
        aug_final = np.array(aug_state_arc3)
        t_final_abs = t_arc3_start

    state_final = aug_final[:5]

    # H at end of burn (for transversality)
    H_burn_end = _hamiltonian_at(aug_final[:5], aug_final[5:8], r.F_THRUST_2, pf2)

    if verbose:
        h_f = state_final[1] - c.R_EARTH
        t_end_abs = t_ignition + t_f_result  # absolute end time for display
        print(f"  Stage 2 end: t={t_end_abs:.1f}s, h={h_f/1e3:.1f}km, "
              f"v={state_final[2]:.0f}m/s, gam={np.rad2deg(state_final[3]):.2f}deg "
              f"({_stage2_frame_flown(pf2)} form)")
        print(f"  H_burn_start={H_burn_start:.4f}  H_coast_end={H_coast_end:.4f}  "
              f"H_burn_end={H_burn_end:.4f}")

    return {
        'crashed': False,
        # Ground-relative, like every other PSO architecture's burn-end state, so
        # the archive's insertion columns and orbit conversion read it unchanged.
        'state_final': _from_stage2_frame(state_final, t_final_abs, t2_start),
        # As propagated (and scored): inertial only under the inertial form.
        'state_final_propagated': state_final,
        'stage2_frame': _stage2_frame_flown(pf2),
        'H_burn_start': H_burn_start,
        'H_coast_end':  H_coast_end,
        'H_burn_end':   H_burn_end,
        'H_last_burn_start': H_last_burn_start,
        'H_burn1_end': H_burn1_end,
        'costates_final': np.array(aug_final[5:8], dtype=float),   # λ_r, λ_v, λ_γ at t_f
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


def transversality_residual(result):
    """Non-negative transversality residual [H units] for INDIRECT_PMP_TRANSVERSALITY.

    "duration_stationarity": stationarity of the burn time in this solver's own
    decision variables — burn D1, coast Dc, burn D3 — written with the reduced
    Hamiltonian. No mass costate is needed: control variations drop out because
    the PMP steering makes ∂H/∂α = 0, and the shift of the mass profile in the
    last burn integrates to H_burn_end − H_last_burn_start.

        ∂J'/∂Dc : H_coast_end = 0     (≤ 0 with Dc at its upper bound, ≥ 0 at its lower)
        ∂J'/∂D3 : H_burn_end = −λ0 < 0                     (hinge)
        ∂J'/∂D1 : H_burn1_end = H_last_burn_start          (both engine on)

    Assumes both burns have positive duration. Verified by finite differences and
    satisfiable (dev-notes/pmp_duration_conditions.py, 2026-09-13).

    "pontani_eq38": |H_burn_end + H_coast_end − H_burn_start|, flown until
    2026-09-13. H_burn_start is taken at Stage-2 ignition, an instant no condition
    involves, and no point near any solution satisfies it together with the orbit
    constraints. Kept only to reproduce archived rows.
    """
    mode = getattr(sim_params, "INDIRECT_PMP_TRANSVERSALITY", "duration_stationarity")
    if mode == "pontani_eq38":
        return abs(result['H_burn_end'] + result['H_coast_end'] - result['H_burn_start'])
    if mode != "duration_stationarity":
        raise ValueError("INDIRECT_PMP_TRANSVERSALITY must be 'duration_stationarity' or "
                         f"'pontani_eq38', got {mode!r}")
    h_ce = result['H_coast_end']
    if result['t_cf'] >= sim_params.PSO_UB[3]:
        coast = max(0.0, h_ce)
    elif result['t_cf'] <= sim_params.PSO_LB[3]:
        coast = max(0.0, -h_ce)
    else:
        coast = abs(h_ce)
    return (coast
            + abs(result['H_burn1_end'] - result['H_last_burn_start'])
            + max(0.0, result['H_burn_end']))


def transversality_residual_nd(result):
    """``transversality_residual`` in the units J' prices it in: divided by the
    terminal speed target, since H scales like a velocity. UNWEIGHTED, so it
    stays a measurement of the duration conditions when PENALTY_W_TRANSVERS is
    0 and the 'transv' term of ``_objective_terms`` is identically zero."""
    r_target = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    return transversality_residual(result) / terminal_speed_target(r_target)


def _objective_terms(result):
    """Weighted, non-dimensional contributions to J' — single source of truth.

    Every term is non-dimensionalised so the weights in
    ``simulation_parameters`` are unitless and directly comparable:

        J_nd  = (t_f − t_cf) / T_MAX_2          burn time as a fraction of the
                                                propellant-limited maximum  ∈ [0, 1]
        Δh_nd = (r_f − r_target) / h_target     relative altitude error
        ΔV_nd = (V_f − V_circular) / V_circular relative velocity error
        Δγ_nd = γ_f / γ_ref                     FPA error in units of γ_ref (deg)
        tv_nd = transversality_residual / V_circ   see transversality_residual; H
                                                scales like ṙ (velocity), so divide by V_circ

    Both ``compute_augmented_objective`` and ``breakdown_objective`` consume
    this, so they cannot drift out of sync.
    """
    # Scored in the frame the arc was flown in (see _stage2_inertial).
    state = result.get('state_final_propagated', result['state_final'])
    r_val, v_f, g_f = state[1], state[2], state[3]

    r_target   = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    v_circular = terminal_speed_target(r_target)
    gamma_ref  = np.deg2rad(sim_params.GAMMA_REF_DEG)

    J_nd  = (result['t_f'] - result['t_cf']) / _T_MAX_2
    dh_nd = (r_val - r_target) / sim_params.TARGET_ORBITAL_ALTITUDE
    dv_nd = (v_f - v_circular) / v_circular
    dg_nd = g_f / gamma_ref
    tv_nd  = transversality_residual_nd(result)

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
        return (sim_params.PSO_LB, sim_params.PSO_UB)

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

    Also writes the full-flight pitch history into ``rocket_ascent.theta_history``
    / ``theta_time_history`` and sets ``TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL``
    (end of the last burn) and ``PSO_COAST_ARC2_START_TIME`` (start of the coast),
    the globals the plot suite and the archive read -- the same side effects as
    ``run_pso_coast_full``.

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

    # Match run_indirect_trajectory: the trajectory uses the unit-norm costates.
    lambda0_r, lambda0_v, lambda0_g = _normalize_costates(
        lambda0_r, lambda0_v, lambda0_g
    )

    ra.set_pseudo_forces_for_run(_stage1_pseudo_forces())   # see run_indirect_trajectory
    pf2 = _stage2_pseudo_forces() and bool(ra._PSEUDO_FORCES_THIS_RUN)
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
    # run_stage1 hands Stage 2 a state that still carries the fairing;
    # shed it here if the jettison criterion is already met.
    state2_init = ra.shed_fairing_if_due(t2_start, state2_init)
    state2_init = _to_stage2_frame(state2_init)   # as run_indirect_trajectory

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
        lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2, pf2),
        t_span=(t2_start, t_ignition),
        y0=aug0_preig,
        t_eval=_make_teval(t2_start, t_ignition),
        rtol=_RTOL, atol=_ATOL, max_step=_MAX_STEP, events=_event_crash,
    )
    state_at_ign = sol_pre.y[:n_state, -1].copy()
    # Second chance, for a trajectory that staged below the criterion:
    # the pre-ignition coast climbs fast, so the crossing lands here.
    state_at_ign = ra.shed_fairing_if_due(t_ignition, state_at_ign)
    aug_ign = list(state_at_ign) + [lambda0_r, lambda0_v, lambda0_g]

    # --- Arc 1 (thrust) ---
    t_arc1_end = t_ignition + t_coast_start
    if t_coast_start > 0.01:
        sol1 = solve_ivp(
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2, pf2),
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
            lambda t, y: _stage2_ode(t, y, 0.0, r.ISP_2, pf2),
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
            lambda t, y: _stage2_ode(t, y, r.F_THRUST_2, r.ISP_2, pf2),
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

    # Report Stage 2 in the ground-relative convention the archive, the figures
    # and the segmented waypoints read. Pitch is frame-independent, so alpha is
    # re-referenced to the ground-relative gamma rather than kept.
    gamma_flown   = y_stage2_full[3].copy()
    y_stage2_full = _from_stage2_frame(y_stage2_full, t_stage2_full, t2_start)
    alpha_stage2  = alpha_stage2 + gamma_flown - y_stage2_full[3]

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
    thrust_stage1 = ra.thrust_on_grid(t_stage1)
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

    # ---- Full-flight channels and event markers for the plot suite ----
    # Same contract as run_pso_coast_full: main.py reads the pitch history and
    # the arc boundaries from the rocket_ascent globals, and until this block
    # existed a PMP run left them at what Stage 1 had written -- a pitch plot
    # that stopped at separation, and no SECO marker on any figure.
    #
    # It sits AFTER run_indirect_trajectory above on purpose: that call re-runs
    # Stage 1, and run_stage1's reset block empties theta_history and sets
    # TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL back to None. Written any earlier,
    # everything here is silently undone before the function returns.
    #
    # Pitch is built on the dense output grid, NOT from the ODE right-hand side.
    # solve_ivp evaluates the RHS at speculative times past a terminal event, so
    # the RHS-cadence theta_history carried five samples at theta = 90 deg
    # timestamped up to 0.9 s AFTER the T+7.5 s kick, interleaved with the real
    # post-kick samples: sorting by time cannot remove them and the pitch plot
    # drew a sawtooth. On the output grid every sample is an accepted step.
    theta_full = alpha_full + data_full[3]            # pitch theta = alpha + gamma
    ra.theta_history      = list(theta_full)
    ra.theta_time_history = list(time_full)

    # SECO is the end of the planned Stage-2 sequence, which is also the last
    # dense sample (_make_teval includes each arc's endpoint), so the archive's
    # budget window -- searchsorted(t_seco, 'right') -- still spans the whole
    # trajectory exactly as it did when the marker was None.
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = t_arc3_end
    ra.PSO_COAST_ARC2_START_TIME              = t_arc2_start

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
        sf = result.get('state_final_propagated', result['state_final'])
        h_f = (sf[1] - c.R_EARTH) / 1e3
        v_f = sf[2]
        g_f = np.rad2deg(sf[3])
        r_t = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
        v_c = terminal_speed_target(r_t)
        dh  = h_f - sim_params.TARGET_ORBITAL_ALTITUDE / 1e3
        dv  = v_f - v_c
        print(f"\nFinal state vs. target ({result.get('stage2_frame', 'rotating')} frame):")
        print(f"  Altitude : {h_f:.2f} km  (target {sim_params.TARGET_ORBITAL_ALTITUDE/1e3:.0f} km, delta={dh:.2f} km)")
        print(f"  Velocity : {v_f:.2f} m/s (target {v_c:.2f} m/s, delta={dv:.2f} m/s)")
        print(f"  FPA      : {g_f:.4f} deg  (target 0.0 deg)")
        H_trans = transversality_residual(result)
        _tv_mode = getattr(sim_params, 'INDIRECT_PMP_TRANSVERSALITY', 'duration_stationarity')
        print(f"  Transversality ({_tv_mode}): {H_trans:.6f}  (target 0)")
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
