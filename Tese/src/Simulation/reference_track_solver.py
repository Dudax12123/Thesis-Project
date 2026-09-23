"""
Reference-Track Solver -- COAST_METHOD = "reference_track"

peg_new or apollo flies the indirect-PMP reference's plan. There is no
optimiser: the reference (Simulation/segment_reference.py, the tracked
pmp_reference.npz) supplies three things and nothing else,

    gamma_p        the kick, so Stage 1 is the reference's own
    arc-1 target   the reference's state where its first Stage-2 burn ends
                   (h, v, gamma -- a SegmentTarget, as the segmented mode uses)
    coast length   the reference's delta_tc, coasted from wherever arc 1 ended

and everything else is an output: the burn durations, SECO, the mass delivered.
Arc 3 aims at the objective orbit exactly as a single-law run does.

peg_new. Both Stage-2 burns end where peg_new's OWN time-to-go says. Its major loop runs
outside the ODE right-hand side, on accepted states, every PEG_MAJOR_LOOP_RATE;
once t_go <= max(freeze threshold, cycle) the coefficients freeze and the last
cycle is flown to exactly t_go. That is direct_pso_solver's
_fly_law_terminated_burn, generalised to an intermediate target and to a second
burn. The cutoff time being free is what makes the plan flyable at all: a law
told to reach the waypoint AND to stop at the reference's instant is
over-determined, which is what sank the swarm-timed arc-1 waypoint of
2026-09-22.

apollo. A fixed-time law: it cannot find the instant at which a waypoint is
reached, so arc 1 is handed the reference's own arc-1 cutoff instant and ends
there -- time-terminated, on the reference's clock, where peg_new's arc 1 is
law-terminated. Arc 3 ends when apollo's own t_go to the orbit
(_compute_tgo_stage2: the rocket equation, or TGO_ESTIMATOR) expires, the same
rule as peg_new's. The coefficients are refreshed OUTSIDE the ODE on accepted
states every GUIDANCE_UPDATE_RATE (GuidanceState.apollo_external): the
pso_coast in-RHS refresh fires on solve_ivp's speculative trial points and
dates the coefficients from a time the integrator then steps back from, and on
this flight that ended arc 1 966 m/s short (measured 2026-09-23). Apollo steers
altitude and vertical speed first and gives the horizontal channel what thrust
is left (Luminary P12), so its arc-1 speed miss (-3.7 m/s) is larger than
peg_new's while its altitude miss is not.

What it measures is the tracking loss of a closed-loop law handed the optimum's
plan -- compare against pmp_baseline, not against peg_baseline / show_apollo,
since against those the arc-1 target, the kick, the coast and the cutoff rule
all change at once.

Only peg_new and apollo: they are the two laws that take a full terminal state
(altitude, speed, flight-path angle). Classical peg's call sites hardcode
circular speed, and the open-loop and passive laws have nothing to aim.

Freeze. Both laws freeze at APOLLO_FREEZE_THRESHOLD (10 s), the value every
peg_new and apollo final burn uses. Below ~10 s the realigned peg_new's endgame
does not terminate -- lambda'_r grows like r_go / t_go^3 -- and at 2 s arc 1 was
measured burning to propellant exhaustion (2026-09-23). Apollo's t_go is
prescribed in arc 1, so it has no such limit there.

Written 2026-09-23 from dev-notes/arc1_reference_track.py, which now imports it;
apollo added the same day.
"""

import sys
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent))

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r
from Input_File import simulation_parameters as sim_params
import Simulation.rocket_ascent as ra
import Simulation.pso_coast_solver as pcs
import Guidance.peg_guidance_new as peg_new_mod
import Guidance.apollo_guidance as apollo_mod

LAWS = ("peg_new", "apollo")

DT = 0.5   # output step [s], as run_pso_coast_full

# Stage 1 is shared code flown at the reference's own gamma_p, so the ignition
# state must reproduce the reference's to round-off. Anything more means the
# configuration is not the reference's and nothing downstream is meaningful.
STAGE1_TOL = {"t": 1e-3, "h": 1.0, "v": 0.01, "gamma": 1e-5, "m": 0.1}

# Everything the last flight knew beyond the trajectory: the arc boundaries, the
# arc-1 target and miss, the t_go traces. Read by archive_extra().
LAST_REFERENCE_TRACK = None


# ---------------------------------------------------------------------------
# The plan
# ---------------------------------------------------------------------------

def plan_from_reference(time_full, data_full, decision_vector, source=None):
    """The plan an indirect_pmp trajectory flew, from its decision vector.

    ``decision_vector`` has the indirect layout ``[lambda0_r, lambda0_v,
    lambda0_g, delta_tc, delta_tr_pct, coast_start_pct, gamma_p]``; the arc-1
    duration is computed exactly as indirect_pso_solver.run_indirect_full times
    it, so the waypoint instant lands on the reference's own grid stamp.
    """
    x = np.asarray(decision_vector, dtype=float)
    if x.shape != (7,):
        raise ValueError("expected the 7-element indirect_pmp decision vector, got shape %s"
                         % (x.shape,))
    delta_tc, delta_tr_pct, coast_start_pct, gamma_p = (float(v) for v in x[3:7])
    t_burn = (delta_tr_pct / 100.0) * pcs._T_MAX_2
    arc1 = (coast_start_pct / 100.0) * t_burn
    return {
        "x": x,
        "gamma_p": gamma_p,
        "delta_tc": delta_tc,
        "arc1": arc1,
        "arc3": t_burn - arc1,
        "time": np.asarray(time_full, dtype=float),
        "data": np.asarray(data_full, dtype=float)[:5],
        "source": source,
    }


def load_plan(verbose=True):
    """The plan of the reference cache in force (built if absent, as for segmented)."""
    import Simulation.segment_reference as segref
    time_full, data_full, x, source = segref.get_pmp_reference_plan(verbose=verbose)
    return plan_from_reference(time_full, data_full, x, source)


def reference_state_at(plan, t, tol=1e-6):
    """The reference's [s, r, v, gamma, m] at one of its grid stamps.

    An arc boundary is stamped twice (the end of one arc, the start of the
    next), with the same state; the later copy is returned. Raises when no
    stamp lies within ``tol`` of ``t``.
    """
    tg = plan["time"]
    j = int(np.searchsorted(tg, t - tol, side="left"))
    if j >= len(tg) or tg[j] > t + tol:
        raise ValueError("the reference has no sample within %.1e s of t = %.6f s" % (tol, t))
    while j + 1 < len(tg) and tg[j + 1] <= t + tol:
        j += 1
    return plan["data"][:, j].copy()


# ---------------------------------------------------------------------------
# Flight primitives
# ---------------------------------------------------------------------------

def _teval(t0, t1):
    pts = np.arange(t0, t1, DT)
    if len(pts) == 0 or pts[-1] < t1:
        pts = np.append(pts, t1)
    return pts


def _ivp(t0, t1, y0, thrust, gs, t_eval):
    return solve_ivp(
        lambda t, y: pcs._stage2_ode_guidance(t, y, thrust, r.ISP_2, gs),
        t_span=(t0, t1), y0=y0, t_eval=t_eval,
        rtol=pcs._RTOL, atol=pcs._ATOL, max_step=pcs._MAX_STEP,
        events=pcs._event_crash)


def fly_law_terminated_arc(t0, y0, gs, target):
    """One peg_new burn, ended where peg_new's own t_go says.

    ``target=None`` aims at the objective orbit exactly as _compute_alpha_stage2
    resolves it for a single-law run; a SegmentTarget aims at
    (r, v cos gamma, v sin gamma). Propellant exhaustion caps the burn, computed
    from the mass in hand -- arc 3 starts with whatever arc 1 left.

    Returns ``(sols, t_end, y_end, crashed)``; on a crash the last solution is
    the one that crashed and ``t_end``/``y_end`` are where it stopped.
    """
    gs.peg_new_external = True
    gs.target = target
    if target is not None:
        r_tgt, v_theta_T, v_r_T = target.r, target.v_theta_T, target.v_r_T
        freeze = (target.freeze_threshold if target.freeze_threshold is not None
                  else sim_params.APOLLO_FREEZE_THRESHOLD)
    else:
        r_tgt = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
        v_theta_T, v_r_T = pcs._v_circular_rotating(r_tgt), 0.0
        freeze = sim_params.APOLLO_FREEZE_THRESHOLD
    ve = r.ISP_2 * c.G_0
    cycle = float(sim_params.PEG_MAJOR_LOOP_RATE)
    last = max(float(freeze), cycle)
    t, y = float(t0), np.asarray(y0[:5], dtype=float).copy()
    t_exhaust = t + max(y[4] - pcs._DRY_MASS_2, 0.0) / pcs._MDOT_2
    sols = []
    while t < t_exhaust:
        (gs.peg_new_vgo_r, gs.peg_new_vgo_theta,
         gs.peg_new_L0, gs.peg_new_tgo,
         gs.peg_new_t_lambda, gs.peg_new_lambda_r) = peg_new_mod.peg_new_major_loop(
             y, r_tgt, c.MU_EARTH, ve, r.F_THRUST_2, v_theta_T=v_theta_T, v_r_T=v_r_T)
        gs.peg_new_t_epoch = t
        tgo = gs.peg_new_tgo
        gs.tgo_time_log.append(t)
        gs.tgo_log.append(tgo)
        if not np.isfinite(tgo) or tgo <= 0.0:
            break
        final = tgo <= last
        gs.peg_new_frozen = final
        t_next = min(t + (tgo if final else cycle), t_exhaust)
        grid = _teval(t, t_next)
        sol = _ivp(t, t_next, y, r.F_THRUST_2, gs, grid if not sols else grid[1:])
        sols.append(sol)
        if len(sol.t_events[0]) > 0:
            return sols, float(sol.t_events[0][0]), sol.y_events[0][0][:5].copy(), True
        t, y = t_next, sol.y[:5, -1].copy()
        if final:
            break
    return sols, t, y, False


def fly_apollo_arc(t0, y0, gs, target, deadline=None):
    """One apollo burn with the coefficients refreshed outside the ODE.

    ``deadline`` given: t_go = deadline - t and the burn ends at the deadline (the
    arc-1 rule, apollo being a fixed-time law). ``deadline=None``: t_go is apollo's
    own estimate to the objective orbit (_compute_tgo_stage2) and the burn ends
    when it expires, frozen for the last cycle exactly as fly_law_terminated_arc
    does for peg_new. ``target`` as there. Returns ``(sols, t_end, y_end, crashed)``.
    """
    gs.apollo_external = True
    gs.guidance_phase_active = True
    gs.target = target
    if target is not None:
        alt = target.alt
        kw = dict(terminal_velocity=target.v, terminal_gamma=target.gamma,
                  terminal_altitude=target.alt)
        freeze = (target.freeze_threshold if target.freeze_threshold is not None
                  else sim_params.APOLLO_FREEZE_THRESHOLD)
    else:
        alt, kw = sim_params.TARGET_ORBITAL_ALTITUDE, {}
        freeze = sim_params.APOLLO_FREEZE_THRESHOLD
    cycle = float(sim_params.GUIDANCE_UPDATE_RATE)
    last = max(float(freeze), cycle)
    t, y = float(t0), np.asarray(y0[:5], dtype=float).copy()
    t_exhaust = t + max(y[4] - pcs._DRY_MASS_2, 0.0) / pcs._MDOT_2
    sols = []
    while t < t_exhaust:
        tgo = (deadline - t if deadline is not None
               else pcs._compute_tgo_stage2(y, r.F_THRUST_2, r.ISP_2))
        gs.tgo_time_log.append(t)
        gs.tgo_log.append(tgo)
        if not np.isfinite(tgo) or tgo <= 1e-9:
            break
        gs.guidance_coefficients = apollo_mod.compute_apollo_coefficients(
            pcs._state_with_lat(y), alt, tgo, use_downrange_constraint=False, **kw)
        gs.apollo_freeze_time = t
        final = tgo <= last
        gs.apollo_coefficients_frozen = final
        t_next = min(t + (tgo if final else cycle), t_exhaust)
        grid = _teval(t, t_next)
        sol = _ivp(t, t_next, y, r.F_THRUST_2, gs, grid if not sols else grid[1:])
        sols.append(sol)
        if len(sol.t_events[0]) > 0:
            return sols, float(sol.t_events[0][0]), sol.y_events[0][0][:5].copy(), True
        t, y = t_next, sol.y[:5, -1].copy()
        if final:
            break
    return sols, t, y, False


def fly_stage1(gamma_p):
    """Stage 1 and the pre-ignition coast, exactly as run_pso_coast_full flies them.

    Returns ``(t2_start, t_meco, t_ignition, y_ignition, t_stage1, y_stage1, sol_pre)``.
    """
    t2_start, s2, t_meco, t_st1, y_st1, crashed = ra.run_stage1(gamma_p - np.pi / 2.0)
    if crashed:
        raise RuntimeError("Stage 1 crashed at the reference's gamma_p = %.10g rad" % gamma_p)
    t_ign = t2_start + pcs._T_IGNITION_DELAY
    s2 = pcs._strip_to_pmp_state(s2, np.deg2rad(sim_params.LAUNCH_LATITUDE))
    # Both jettison checks run_pso_coast_full makes: the reference stages below
    # the 65 km criterion with the fairing still on.
    s2 = ra.shed_fairing_if_due(t2_start, s2)
    sol_pre = _ivp(t2_start, t_ign, s2[:5], 0.0, None, _teval(t2_start, t_ign))
    y_ign = ra.shed_fairing_if_due(t_ign, sol_pre.y[:5, -1].copy())
    return t2_start, t_meco, t_ign, y_ign, t_st1, y_st1, sol_pre


def check_stage1(plan, t_ign, y_ign):
    """Differences between this Stage-2 ignition and the reference's, per STAGE1_TOL key.

    Returns ``(diffs, failed_keys)``. The time check is whether the reference
    has a sample within STAGE1_TOL['t'] of this ignition at all.
    """
    try:
        ref = reference_state_at(plan, t_ign, tol=STAGE1_TOL["t"])
    except ValueError:
        return {"t": float("nan")}, ["t"]
    diffs = {"h": y_ign[1] - ref[1], "v": y_ign[2] - ref[2],
             "gamma": y_ign[3] - ref[3], "m": y_ign[4] - ref[4]}
    failed = [k for k, d in diffs.items() if not abs(d) <= STAGE1_TOL[k]]
    return diffs, failed


# ---------------------------------------------------------------------------
# Assembly (run_pso_coast_full's bookkeeping, so the archive reads it alike)
# ---------------------------------------------------------------------------

def _assemble(t_st1, y_st1, segments, gs, y_insertion, t_insertion, t_coast_start,
              propagate_orbit=True):
    """``segments``: [(sol, thrust), ...] for Stage 2 up to insertion. Adds the
    post-insertion orbit coast, builds the full arrays, writes the ra.* channels
    the archive and the plot suite read. Returns (time, data, thrust, alpha,
    coriolis_mag, centrifugal_mag)."""
    from Plots.plot_state_utils import interpolate_to_time

    sol_post = None
    if propagate_orbit:
        post_init = np.asarray(y_insertion, dtype=float).copy()
        if sim_params.ENABLE_EARTH_ROTATION:
            v_in, g_in = ra.get_inertial_state_components(
                y_insertion[1], y_insertion[2], y_insertion[3],
                np.deg2rad(sim_params.LAUNCH_LATITUDE))
            post_init[2], post_init[3] = v_in, g_in
        # Inertial from here on; reset by run_stage1 at the next trajectory.
        ra.PROPAGATING_IN_INERTIAL_FRAME = True
        t_post_end = t_insertion + sim_params.DURATION_AFTER_SIMULATION
        sol_post = _ivp(t_insertion, t_post_end, post_init, 0.0, None,
                        _teval(t_insertion, t_post_end))
        segments = list(segments) + [(sol_post, 0.0)]

    t_parts, y_parts, th_parts = [], [], []
    for sol, F in segments:
        if sol is None or len(sol.t) == 0:
            continue
        t_parts.append(sol.t)
        y_parts.append(sol.y[:5, :])
        th_parts.append(np.full(len(sol.t), F))
    t2 = np.concatenate(t_parts)
    y2 = np.concatenate(y_parts, axis=1)
    th2 = np.concatenate(th_parts)
    alpha2 = (interpolate_to_time(gs.time_log, gs.alpha_log, t2) if gs.time_log
              else np.zeros(len(t2)))

    time_full = np.concatenate([t_st1, t2])
    data_full = np.concatenate([y_st1[:5, :], y2], axis=1)
    thrust_full = np.concatenate(
        [interpolate_to_time(ra.time_history, ra.thrust_history, t_st1), th2])
    alpha_full = np.concatenate(
        [interpolate_to_time(ra.alpha_time_history, ra.alpha_history, t_st1), alpha2])

    if sim_params.ENABLE_EARTH_ROTATION:
        lat_row = np.array([ra.get_latitude_from_downrange(s) for s in data_full[0]])
        data_full = np.vstack([data_full, lat_row])

    ra.theta_history = list(alpha_full + data_full[3])
    ra.theta_time_history = list(time_full)
    if gs.tgo_time_log:
        ra.tgo_time_history = list(gs.tgo_time_log)
        ra.tgo_history = list(gs.tgo_log)

    n_post = len(sol_post.t) if sol_post is not None else 0
    saved = ra.PROPAGATING_IN_INERTIAL_FRAME
    ra.PROPAGATING_IN_INERTIAL_FRAME = False
    try:
        chf, cha, cor, cen = ra.pseudo_force_channels_on_grid(time_full, data_full[:5])
    finally:
        ra.PROPAGATING_IN_INERTIAL_FRAME = saved
    if n_post:
        for ch in (chf, cha, cor, cen):
            ch[-n_post:] = 0.0
    if sim_params.COMPUTE_CROSS_HEADING_COUNTER_FORCE:
        ra.cross_heading_counter_force_history = list(chf)
        ra.cross_heading_accel_history = list(cha)

    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = t_insertion
    ra.PSO_COAST_ARC2_START_TIME = t_coast_start
    return time_full, data_full, thrust_full, alpha_full, cor, cen


# ---------------------------------------------------------------------------
# The flight
# ---------------------------------------------------------------------------

def run_reference_track(plan=None, verbose=True, waypoint=True, arc1_freeze=None,
                        coast_mode="duration", check_stage1_state=True):
    """Fly the reference's plan with peg_new or apollo (GUIDANCE_MODE).

    ``plan``         from plan_from_reference / load_plan; None loads the cache in force.
    ``waypoint``     False aims arc 1 at the final orbit instead (a control: it
                     isolates the waypoint, all else equal).
    ``arc1_freeze``  arc-1 freeze threshold [s]; None = APOLLO_FREEZE_THRESHOLD.
                     peg_new's arc 1 ends on its own t_go, apollo's at the
                     reference's arc-1 cutoff instant (see the module docstring).
    ``coast_mode``   "duration": coast the reference's delta_tc from wherever arc 1
                     ends (the ballistic arc the reference traced); "seco": coast
                     until the reference's own arc-3 ignition instant.
    ``check_stage1_state``  raise if Stage 1 does not reproduce the reference's.

    Returns the tuple of run_pso_coast_full: ``(time, data, thrust, alpha,
    t_ignition, result, coriolis_mag, centrifugal_mag)``; the extra detail is in
    LAST_REFERENCE_TRACK.
    """
    global LAST_REFERENCE_TRACK
    law = sim_params.GUIDANCE_MODE
    if law not in LAWS:
        raise ValueError(
            "COAST_METHOD='reference_track' flies peg_new or apollo only "
            "(GUIDANCE_MODE=%r): they are the two laws that take a full terminal state "
            "(altitude, speed, flight-path angle) as their target." % law)
    if coast_mode not in ("duration", "seco"):
        raise ValueError("coast_mode must be 'duration' or 'seco', got %r" % coast_mode)
    if plan is None:
        plan = load_plan(verbose=verbose)
    # After load_plan: a reference build runs the indirect solver, which sets
    # its own pseudo-force switch.
    ra.set_pseudo_forces_for_run(True)   # carried for the whole ascent

    t2_start, t_meco, t_ign, y_ign, t_st1, y_st1, sol_pre = fly_stage1(plan["gamma_p"])
    diffs, failed = check_stage1(plan, t_ign, y_ign)
    if failed and check_stage1_state:
        raise ValueError(
            "Stage 1 does not reproduce the reference's ignition state (%s): this "
            "configuration is not the one the reference was flown under."
            % ", ".join("%s %+.3e" % (k, diffs.get(k, float("nan"))) for k in failed))

    t_ref_arc1_end = t_ign + plan["arc1"]
    wp = reference_state_at(plan, t_ref_arc1_end)
    freeze1 = (sim_params.APOLLO_FREEZE_THRESHOLD if arc1_freeze is None
               else float(arc1_freeze))
    target = None
    if waypoint:
        target = pcs.SegmentTarget(r=float(wp[1]), alt=float(wp[1]) - c.R_EARTH,
                                   v=float(wp[2]), gamma=float(wp[3]),
                                   freeze_threshold=freeze1)

    gs = pcs.GuidanceState()
    segments = [(sol_pre, 0.0)]
    crashed_in = None

    # ---- Arc 1 ----
    if law == "apollo":
        sols1, t_a1, y_a1, crashed = fly_apollo_arc(t_ign, y_ign, gs, target,
                                                   deadline=t_ref_arc1_end)
    else:
        sols1, t_a1, y_a1, crashed = fly_law_terminated_arc(t_ign, y_ign, gs, target)
    segments += [(s, r.F_THRUST_2) for s in sols1]
    n_tgo_arc1 = len(gs.tgo_log)
    t_c_end, t_a3, y_ins = t_a1, t_a1, y_a1
    if crashed:
        crashed_in = "arc 1"

    # ---- Coast ----
    if crashed_in is None:
        if coast_mode == "duration":
            t_c_end = t_a1 + plan["delta_tc"]
        else:
            t_c_end = max(t_ref_arc1_end + plan["delta_tc"], t_a1)
        y_a3 = y_a1
        if t_c_end - t_a1 > 1e-9:
            sol_c = _ivp(t_a1, t_c_end, y_a1, 0.0, None, _teval(t_a1, t_c_end))
            segments.append((sol_c, 0.0))
            if len(sol_c.t_events[0]) > 0:
                crashed_in = "coast"
                t_c_end = float(sol_c.t_events[0][0])
                y_a3 = sol_c.y_events[0][0][:5].copy()
            else:
                y_a3 = sol_c.y[:5, -1].copy()
        t_a3, y_ins = t_c_end, y_a3

    # ---- Arc 3 ----
    if crashed_in is None:
        gs.restart_for_new_burn()
        if law == "apollo":
            sols3, t_a3, y_ins, crashed = fly_apollo_arc(t_c_end, y_a3, gs, None)
        else:
            sols3, t_a3, y_ins, crashed = fly_law_terminated_arc(t_c_end, y_a3, gs, None)
        segments += [(s, r.F_THRUST_2) for s in sols3]
        if crashed:
            crashed_in = "arc 3"

    time_a, data, thrust, alpha, cor, cen = _assemble(
        t_st1, y_st1, segments, gs, y_ins, t_a3, t_a1,
        propagate_orbit=crashed_in is None)

    arc1, coast, arc3 = t_a1 - t_ign, t_c_end - t_a1, t_a3 - t_c_end
    burn = arc1 + arc3
    result = {
        "crashed": crashed_in is not None,
        "state_final": np.asarray(y_ins, dtype=float),
        "t_f": burn + coast, "t_cf": coast,
        "t_stage2_start": t2_start, "t_ignition": t_ign, "t_arc2_start": t_a1,
        "t_arc3_end": t_a3, "t_stage1": t_st1, "y_stage1": y_st1,
    }
    LAST_REFERENCE_TRACK = {
        "law": law,
        "arc1_cutoff": ("reference instant" if law == "apollo" else "law t_go"),
        "plan": plan,
        "t_meco": t_meco,
        "t_ignition": t_ign,
        "stage1_diffs": diffs,
        "t_arc1_end": t_a1,
        "t_arc3_start": t_c_end,
        "t_arc3_end": t_a3,
        "t_reference_arc1_end": t_ref_arc1_end,
        "waypoint": wp,
        "target": target,
        "arc1_freeze": freeze1 if waypoint else float(sim_params.APOLLO_FREEZE_THRESHOLD),
        "arc3_freeze": float(sim_params.APOLLO_FREEZE_THRESHOLD),
        "coast_mode": coast_mode,
        "y_arc1_end": np.asarray(y_a1, dtype=float),
        "y_insertion": np.asarray(y_ins, dtype=float),
        "crashed_in": crashed_in,
        # The flight in pso_coast's layout [delta_tc, delta_tr_pct, coast_start_pct,
        # gamma_p], so it can be set beside a swarm's point.
        "realised_schedule": [coast, 100.0 * burn / pcs._T_MAX_2,
                              100.0 * arc1 / burn if burn > 0 else 0.0, plan["gamma_p"]],
        "tgo_arc1": list(zip(gs.tgo_time_log[:n_tgo_arc1], gs.tgo_log[:n_tgo_arc1])),
        "tgo_arc3": list(zip(gs.tgo_time_log[n_tgo_arc1:], gs.tgo_log[n_tgo_arc1:])),
    }

    if verbose:
        _print_flight(LAST_REFERENCE_TRACK)
    return time_a, data, thrust, alpha, t_ign, result, cor, cen


def archive_extra():
    """The last flight's detail as flat, archivable values (for store.save_run's extra).

    ``decision_vector`` is the reference's -- the plan this flight was handed,
    which is what re-flies it -- in the indirect layout; ``realised_schedule`` is
    what the flight did, in pso_coast's.
    """
    info = LAST_REFERENCE_TRACK
    if info is None:
        raise RuntimeError("no reference_track flight has run in this process")
    wp, y1 = info["waypoint"], info["y_arc1_end"]
    tgt = info["target"]
    nan3 = [float("nan")] * 3
    return {
        "decision_vector": [float(v) for v in info["plan"]["x"]],
        "realised_schedule": [float(v) for v in info["realised_schedule"]],
        "arc1_target": nan3 if tgt is None else [float(tgt.r), float(tgt.v), float(tgt.gamma)],
        "arc1_achieved": [float(v) for v in y1[1:5]],
        "arc1_miss_vs_reference": [float(y1[k] - wp[k]) for k in (1, 2, 3, 4)],
        "arc1_freeze_threshold": float(info["arc1_freeze"]),
        "arc3_freeze_threshold": float(info["arc3_freeze"]),
        "t_arc1_end": float(info["t_arc1_end"]),
        "t_arc3_start": float(info["t_arc3_start"]),
        "t_arc3_end": float(info["t_arc3_end"]),
        "reference_t_arc1_end": float(info["t_reference_arc1_end"]),
        "coast_mode": info["coast_mode"],
        "arc1_cutoff_rule": info["arc1_cutoff"],
        "reference_source": info["plan"]["source"] or "",
    }


def _print_flight(info):
    def hvgm(y):
        return ("h %9.3f km   v %9.3f m/s   gamma %8.4f deg   m %10.2f kg"
                % ((y[1] - c.R_EARTH) / 1e3, y[2], np.rad2deg(y[3]), y[4]))
    wp, y1 = info["waypoint"], info["y_arc1_end"]
    plan = info["plan"]
    print("\n" + "=" * 60)
    print("REFERENCE TRACK -- %s flies the PMP plan (no optimiser; arc 1 ends on %s)"
          % (info["law"], info["arc1_cutoff"]))
    print("=" * 60)
    print("  gamma_p %.10g rad   coast %.4f s   reference arc 1 %.4f s"
          % (plan["gamma_p"], plan["delta_tc"], plan["arc1"]))
    print("  Stage-1 check vs reference: " + ", ".join(
        "%s %+.2e" % kv for kv in info["stage1_diffs"].items()))
    print("  Arc 1  %9.3f -> %9.3f s   (reference ends %9.3f)"
          % (info["t_ignition"], info["t_arc1_end"], info["t_reference_arc1_end"]))
    print("    achieved   " + hvgm(y1))
    print("    reference  " + hvgm(wp))
    print("    miss       dh %+.3f km   dv %+.3f m/s   dgamma %+.4f deg   dm %+.2f kg"
          % ((y1[1] - wp[1]) / 1e3, y1[2] - wp[2], np.rad2deg(y1[3] - wp[3]), y1[4] - wp[4]))
    print("  Coast  %9.3f -> %9.3f s" % (info["t_arc1_end"], info["t_arc3_start"]))
    print("  Arc 3  %9.3f -> %9.3f s" % (info["t_arc3_start"], info["t_arc3_end"]))
    print("    insertion  " + hvgm(info["y_insertion"]))
    if info["crashed_in"]:
        print("  CRASHED in %s" % info["crashed_in"])
