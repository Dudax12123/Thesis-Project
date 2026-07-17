"""
Segmented Guidance Solver — multi-law, altitude-triggered ascent guidance.

Flies the ordered ``simulation_parameters.GUIDANCE_SEGMENTS`` schedule instead of a
single guidance law: the FIRST chosen law takes over right after the kick maneuver
(gravity turn is one selectable option, no longer forced), then each subsequent law
at its activation altitude. Each non-final segment aims at the indirect-PMP optimal
(alt, v, gamma) waypoint at the NEXT activation altitude; the final segment aims at
orbit insertion. Time-to-go is a planned-deadline countdown (deadline - t) sourced
from the PMP reference, so it never collapses across the stage boundary — this is
what lets the t_go-dependent laws fly DURING Stage 1.

Architecture (per the chosen "pso_coast only" path):
  * Stage 1 reuses the validated rocket_ascent physics via ``ra.run_stage1`` with
    an isolated steering hook (``ra._SEGMENTED_ALPHA_HOOK``) so a guidance law can
    fly sub-MECO without duplicating the atmospheric/thrust-ramp/MECO physics.
  * Stage 2 reuses pso_coast's vacuum ODE / GuidanceState machinery
    (thrust-coast-thrust, direct insertion), with the active law switched at
    activation altitudes via terminal solve_ivp events.
  * One GuidanceState persists across the whole ascent; it is re-initialised
    (``restart_for_new_burn``) at every segment / thrust-phase boundary.

Nothing here runs unless ``MULTI_GUIDANCE_ENABLED`` is set and main.py dispatches
to this module; the single-law paths are untouched.

PyGMO is required (same as pso_coast). The PMP reference is built once and cached.
"""

import sys
import time
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
import Simulation.segment_reference as segref


# ---------------------------------------------------------------------------
# Altitude-crossing event factory (terminal, upward crossing)
# ---------------------------------------------------------------------------

def _make_alt_event(target_alt):
    def ev(t, y, *a):
        return (y[1] - c.R_EARTH) - target_alt
    ev.terminal = True
    ev.direction = 1
    return ev


# ---------------------------------------------------------------------------
# Segment schedule + per-segment targets / deadlines
# ---------------------------------------------------------------------------

class _Segments:
    """The resolved guidance schedule and its PMP-derived targets/timing.

    schedule[i] is the user's GUIDANCE_SEGMENTS[i] (law, activation altitude); the
    FIRST entry flies right after the kick (its altitude is normalised to 0.0) and
    is no longer forced to be a gravity turn. For segment i, target_alt[i] is the
    activation altitude of segment i+1 (or the orbit altitude for the final
    segment), and target[i] is the SegmentTarget aiming at that waypoint (or None ⇒
    circular orbit for the final segment).
    """

    def __init__(self, time_full, data_full, alpha_full=None):
        self.schedule = [
            (str(mode), float(alt)) for (mode, alt) in sim_params.GUIDANCE_SEGMENTS
        ]
        # The first segment flies right after the kick — its law is the user's
        # choice (gravity turn no longer forced) and its activation altitude is the
        # floor, so normalise it to 0.0 for a clean strictly-increasing schedule.
        if self.schedule:
            self.schedule[0] = (self.schedule[0][0], 0.0)
        self.n = len(self.schedule)

        # Ascent prefix for monotonic altitude->time / waypoint interpolation
        alt_col = np.asarray(data_full[1], dtype=float) - c.R_EARTH
        i_top = int(np.argmax(alt_col))
        if i_top < 1:
            i_top = len(alt_col) - 1
        self._alt_asc = alt_col[: i_top + 1]
        self._t_asc = np.asarray(time_full, dtype=float)[: i_top + 1]

        # Keep the raw reference arrays so the per-segment targets can be rebuilt
        # when activation altitudes change (e.g. the altitude-optimising PSO).
        self._time_full = np.asarray(time_full, dtype=float)
        self._data_full = np.asarray(data_full, dtype=float)

        # Optional replay of the stored indirect-PMP optimal control: a segment
        # whose law is "indirect_pmp" commands the reference α at the current
        # altitude instead of running a live guidance law. None ⇒ no reference
        # control available (only needed when an indirect_pmp segment is used).
        self.replay_alpha = None
        if alpha_full is not None:
            self._alpha_asc = np.asarray(alpha_full, dtype=float)[: i_top + 1]
            _alt, _al = self._alt_asc, self._alpha_asc
            self.replay_alpha = lambda state: float(np.interp(state[1] - c.R_EARTH, _alt, _al))

        self._build_targets()

    def _build_targets(self):
        """(Re)compute each segment's aim point from the PMP reference.

        Non-final segment i aims at the PMP waypoint at segment i+1's activation
        altitude; the final segment aims at the circular orbit (target None). Split
        out of __init__ so it can be re-run when activation altitudes change.
        """
        orbit_alt = float(sim_params.TARGET_ORBITAL_ALTITUDE)
        ft = float(getattr(sim_params, "SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD", 2.0))
        self.target_alt = []
        self.target = []
        for i in range(self.n):
            if i + 1 < self.n:
                t_alt = self.schedule[i + 1][1]
                wp = segref.waypoint_at_altitude(self._data_full, self._time_full, t_alt)
                self.target_alt.append(t_alt)
                self.target.append(pcs.SegmentTarget.from_waypoint(wp, freeze_threshold=ft))
            else:
                self.target_alt.append(orbit_alt)
                self.target.append(None)   # final segment -> circular orbit

    def set_activation_altitudes(self, alts):
        """Update schedule[1:] activation altitudes and rebuild targets.

        ``alts`` has length n-1 (segment 0 stays at 0.0 / "after the kick"). Used by
        the altitude-optimising PSO to evaluate a candidate altitude vector; PSO
        evaluation is serial here, so mutating the shared schedule in place is safe.
        """
        for i, a in enumerate(alts, start=1):
            self.schedule[i] = (self.schedule[i][0], float(a))
        self._build_targets()

    def mode(self, idx):
        return self.schedule[idx][0]

    def t_ref(self, alt):
        return float(np.interp(alt, self._alt_asc, self._t_asc))

    def index_for_alt(self, alt):
        idx = 0
        for i in range(self.n):
            if alt >= self.schedule[i][1] - 1e-9:
                idx = i
        return idx

    def next_activation_alt(self, idx):
        if idx + 1 < self.n:
            return self.schedule[idx + 1][1]
        return None

    def apply(self, gs, idx):
        """Point the GuidanceState at segment ``idx``."""
        gs.mode_override = self.schedule[idx][0]
        gs.target = self.target[idx]

    def set_deadline(self, gs, idx, t_now, alt_now):
        """Planned-deadline countdown from the PMP reference timing.

        deadline = t_now + (t_ref(target_alt) − t_ref(alt_now)), i.e. the optimal
        remaining time from the current altitude to this segment's target. Robust
        to coast (recomputed at each thrust (re)start) and never saturates.
        """
        dur = self.t_ref(self.target_alt[idx]) - self.t_ref(alt_now)
        gs.tgo_deadline = t_now + max(dur, 1.0)


# ---------------------------------------------------------------------------
# Stage-1 steering hook
# ---------------------------------------------------------------------------

def _make_stage1_hook(gs, segs, mgr):
    """rocket_dynamics steering override for the Stage-1 portion.

    Switches to the next law when its activation altitude is crossed (idx is
    latched monotone so RK45 trial/rejected steps cannot switch backward), and
    returns alpha from the shared Stage-2 dispatcher so Stage 1 and Stage 2 use
    the identical law implementation.
    """
    def hook(t, state, F_T, Isp):
        alt = state[1] - c.R_EARTH
        # Apply the FIRST segment once, on the first post-kick call. Nothing ever
        # switches *into* index 0, so with the gravity-turn prefix gone an active
        # segment 0 would otherwise never get its target/deadline set. The hook only
        # runs once kick_performed, so this is exactly "right after the kick".
        if not mgr.get("seg0_applied"):
            mgr["seg0_applied"] = True
            gs.restart_for_new_burn()
            segs.apply(gs, mgr["idx"])
            if segs.mode(mgr["idx"]) != "gravity_turn":
                segs.set_deadline(gs, mgr["idx"], t, alt)
        new_idx = segs.index_for_alt(alt)
        if new_idx > mgr["idx"]:
            mgr["idx"] = new_idx
            gs.restart_for_new_burn()
            segs.apply(gs, new_idx)
            if segs.mode(new_idx) != "gravity_turn":
                segs.set_deadline(gs, new_idx, t, alt)
        idx = mgr["idx"]
        if segs.mode(idx) == "gravity_turn":
            return 0.0
        return pcs._compute_alpha_stage2(t, state, F_T, Isp, gs)
    return hook


# ---------------------------------------------------------------------------
# Stage-2 thrust phase (altitude-segmented)
# ---------------------------------------------------------------------------

def _thrust_phase(t0, duration, y0, gs, segs, mgr, teval_fn=None):
    """Integrate a Stage-2 thrust phase, switching segments at activation altitudes.

    Re-initialises guidance at the phase start (post-coast / post-ignition) and at
    each altitude crossing. Returns (t_end, y_end, crashed, sol_pieces).
    """
    t_target = t0 + duration

    # Phase (re)start: fresh guidance epoch, catch the segment index up to the
    # current altitude (in case a coast crossed a boundary), set the deadline.
    gs.restart_for_new_burn()
    mgr["idx"] = max(mgr["idx"], segs.index_for_alt(y0[1] - c.R_EARTH))
    segs.apply(gs, mgr["idx"])
    if segs.mode(mgr["idx"]) != "gravity_turn":
        segs.set_deadline(gs, mgr["idx"], t0, y0[1] - c.R_EARTH)

    t_cur = float(t0)
    y_cur = np.asarray(y0[:5], dtype=float).copy()
    pieces = []

    while t_cur < t_target - 1e-6:
        nxt = segs.next_activation_alt(mgr["idx"])
        events = [pcs._event_crash]
        if nxt is not None:
            events.append(_make_alt_event(nxt))
        sol = solve_ivp(
            lambda t, y: pcs._stage2_ode_guidance(t, y, r.F_THRUST_2, r.ISP_2, gs),
            t_span=(t_cur, t_target),
            y0=y_cur,
            t_eval=(teval_fn(t_cur, t_target) if teval_fn is not None else None),
            rtol=pcs._RTOL, atol=pcs._ATOL, max_step=pcs._MAX_STEP,
            events=events,
        )
        pieces.append(sol)
        if len(sol.t_events[0]) > 0:                 # ground collision
            return t_cur, y_cur, True, pieces
        t_cur = float(sol.t[-1])
        y_cur = sol.y[:5, -1].copy()

        crossed = (nxt is not None and len(sol.t_events[1]) > 0)
        if crossed:
            mgr["idx"] += 1
            gs.restart_for_new_burn()
            segs.apply(gs, mgr["idx"])
            if segs.mode(mgr["idx"]) != "gravity_turn":
                segs.set_deadline(gs, mgr["idx"], t_cur, y_cur[1] - c.R_EARTH)
        else:
            break                                    # reached t_target
    return t_cur, y_cur, False, pieces


def _ballistic(t0, t1, y0, teval_fn=None):
    """Thrust-off ballistic arc (pre-ignition / coast)."""
    return solve_ivp(
        lambda t, y: pcs._stage2_ode_guidance(t, y, 0.0, r.ISP_2, None),
        t_span=(t0, t1), y0=np.asarray(y0[:5], dtype=float),
        t_eval=(teval_fn(t0, t1) if teval_fn is not None else None),
        rtol=pcs._RTOL, atol=pcs._ATOL, max_step=pcs._MAX_STEP,
        events=pcs._event_crash,
    )


# ---------------------------------------------------------------------------
# Trajectory runner (PSO inner loop)
# ---------------------------------------------------------------------------

def run_segmented_trajectory(delta_tc, delta_tr_pct, coast_start_pct, gamma_p,
                             segs, teval_fn=None, collect=False, verbose=False):
    """Simulate one segmented thrust-coast-thrust trajectory.

    Returns a result dict with the same keys pso_coast's objective consumes
    (crashed, state_final, t_f, t_cf, ...). When ``collect`` is True also returns
    the dense Stage-1 and Stage-2 solution pieces for plotting (key 'pieces').
    """
    kick_angle = gamma_p - np.pi / 2.0
    gs = pcs.GuidanceState()
    gs.force_planned_tgo = True
    # Replay the stored indirect-PMP optimal control for any "indirect_pmp"
    # segment (None ⇒ feature unused / no reference control available).
    gs.replay_alpha = getattr(segs, "replay_alpha", None)
    mgr = {"idx": 0}

    crashed_result = lambda **kw: {
        'crashed': True, 'state_final': None, 't_f': 0.0, 't_cf': 0.0,
        't_stage2_start': 0.0, 't_ignition': 0.0, 't_arc2_start': 0.0,
        't_arc3_end': 0.0, **kw}

    # ---- Stage 1 (first chosen law flies right after the kick via the hook) ----
    hook = _make_stage1_hook(gs, segs, mgr)
    ra._SEGMENTED_ALPHA_HOOK = hook
    try:
        t2_start, state2_init, _t_meco, t_stage1, y_stage1, crashed = \
            ra.run_stage1(kick_angle)
    finally:
        ra._SEGMENTED_ALPHA_HOOK = None
    if crashed:
        return crashed_result(t_stage1=t_stage1, y_stage1=y_stage1)

    state2_init = pcs._strip_to_pmp_state(
        state2_init, np.deg2rad(sim_params.LAUNCH_LATITUDE))

    # ---- Stage-2 timing (identical to pso_coast) ----
    T_burn_total  = (delta_tr_pct   / 100.0) * pcs._T_MAX_2
    t_coast_start = (coast_start_pct / 100.0) * T_burn_total
    t_arc3_burn   = T_burn_total - t_coast_start
    t_ignition    = t2_start + pcs._T_IGNITION_DELAY

    s2_pieces = []

    # ---- Pre-ignition ballistic coast (stage sep -> ignition) ----
    sol_pre = _ballistic(t2_start, t_ignition, state2_init[:5], teval_fn)
    s2_pieces.append((sol_pre, 0.0))
    if len(sol_pre.t_events[0]) > 0:
        return crashed_result(t_stage2_start=t2_start, t_ignition=t_ignition,
                              t_stage1=t_stage1, y_stage1=y_stage1)
    state_at_ign = sol_pre.y[:5, -1].copy()

    # ---- Arc 1 (thrust) ----
    if t_coast_start > 0.01:
        t_a2, state_arc2, crashed, p = _thrust_phase(
            t_ignition, t_coast_start, state_at_ign, gs, segs, mgr, teval_fn)
        s2_pieces += [(s, r.F_THRUST_2) for s in p]
        if crashed:
            return crashed_result(t_stage2_start=t2_start, t_ignition=t_ignition,
                                  t_stage1=t_stage1, y_stage1=y_stage1)
        t_arc2_start = t_a2
    else:
        state_arc2 = state_at_ign.copy()
        t_arc2_start = t_ignition

    # ---- Arc 2 (coast) ----
    if delta_tc > 0.01:
        sol_c = _ballistic(t_arc2_start, t_arc2_start + delta_tc, state_arc2, teval_fn)
        s2_pieces.append((sol_c, 0.0))
        if len(sol_c.t_events[0]) > 0:
            return crashed_result(t_stage2_start=t2_start, t_ignition=t_ignition,
                                  t_arc2_start=t_arc2_start,
                                  t_stage1=t_stage1, y_stage1=y_stage1)
        state_arc3 = sol_c.y[:5, -1].copy()
        t_arc3_start = float(sol_c.t[-1])
    else:
        state_arc3 = state_arc2.copy()
        t_arc3_start = t_arc2_start

    # ---- Arc 3 (thrust to insertion) ----
    if t_arc3_burn > 0.01:
        t_end, state_final, crashed, p = _thrust_phase(
            t_arc3_start, t_arc3_burn, state_arc3, gs, segs, mgr, teval_fn)
        s2_pieces += [(s, r.F_THRUST_2) for s in p]
        if crashed:
            return crashed_result(t_stage2_start=t2_start, t_ignition=t_ignition,
                                  t_arc2_start=t_arc2_start,
                                  t_stage1=t_stage1, y_stage1=y_stage1)
        t_insertion = t_end
    else:
        state_final = state_arc3.copy()
        t_insertion = t_arc3_start

    result = {
        'crashed':        False,
        'state_final':    state_final,
        't_f':            T_burn_total + delta_tc,
        't_cf':           delta_tc,
        't_stage2_start': t2_start,
        't_ignition':     t_ignition,
        't_arc2_start':   t_arc2_start,
        't_arc3_end':     t_insertion,
        't_stage1':       t_stage1,
        'y_stage1':       y_stage1,
        'final_seg_idx':  mgr["idx"],
    }
    if collect:
        result['stage1'] = (t_stage1, y_stage1)
        result['s2_pieces'] = s2_pieces        # list of (sol, thrust_value)
        result['gs'] = gs
    if verbose and not result['crashed']:
        sf = state_final
        print(f"  insertion: h={(sf[1]-c.R_EARTH)/1e3:.1f} km  v={sf[2]:.1f} m/s  "
              f"gam={np.rad2deg(sf[3]):.3f} deg  (reached segment idx {mgr['idx']})")
    return result


# ---------------------------------------------------------------------------
# PyGMO problem + PSO runner
# ---------------------------------------------------------------------------

def _alts_from_fractions(fracs, lb, ub):
    """Map cumulative fractions in [0,1] to strictly-increasing altitudes in [lb, ub].

    a_1 = lb + f_1*(ub - lb);  a_k = a_{k-1} + f_k*(ub - a_{k-1}). Any fracs in
    [0,1] yield lb <= a_1 < a_2 < ... <= ub (a 1 m epsilon gap keeps them strictly
    increasing even when successive fractions are 0), so the ordering constraint is
    satisfied by construction — no wasted particles.
    """
    alts = []
    prev = float(lb)
    eps = 1.0   # 1 m minimum spacing
    for f in fracs:
        f = min(max(float(f), 0.0), 1.0)
        a = max(prev + f * (ub - prev), prev + eps)
        alts.append(a)
        prev = a
    return alts


class SegmentedPSOProblem:
    """UDP for PyGMO.

    Decision vector: the 4 base coast vars ``[delta_tc, delta_tr_pct,
    coast_start_pct, gamma_p]``, plus — when ``optimize_alts`` — ``(n-1)``
    activation-altitude fractions in [0,1] mapped cumulatively into
    ``[alt_lb, alt_ub]`` (segment 0 stays "after the kick", so its altitude is not
    a variable). The objective is unchanged (pso_coast's Stage-2 burn-time term +
    orbit-insertion penalties), so the altitudes are chosen to minimise Stage-2
    burn time subject to a clean insertion.
    """

    def __init__(self, segs, optimize_alts=False, alt_bounds=None):
        self._segs = segs
        self._optimize_alts = bool(optimize_alts)
        self._n_alt = (segs.n - 1) if self._optimize_alts else 0
        self._alt_lb, self._alt_ub = (alt_bounds if alt_bounds is not None else (0.0, 0.0))

    def fitness(self, x):
        try:
            dtc, dtr, cs, gp = float(x[0]), float(x[1]), float(x[2]), float(x[3])
            if self._n_alt > 0:
                alts = _alts_from_fractions(x[4:4 + self._n_alt],
                                            self._alt_lb, self._alt_ub)
                self._segs.set_activation_altitudes(alts)
            result = run_segmented_trajectory(dtc, dtr, cs, gp, self._segs)
            return [pcs.compute_coast_objective(result)]
        except Exception:
            return [pcs.CRASH_PENALTY]

    def get_bounds(self):
        lb = list(getattr(sim_params, "PSO_MG_LB", sim_params.PSO_COAST_LB))
        ub = list(getattr(sim_params, "PSO_MG_UB", sim_params.PSO_COAST_UB))
        if self._n_alt > 0:
            lb += [0.0] * self._n_alt
            ub += [1.0] * self._n_alt
        return (lb, ub)

    def get_nobj(self):
        return 1


def run_segmented_optimization(segs, optimize_alts=False, alt_bounds=None, verbose=True):
    """Run the PSO over the 4 base coast vars (+ the (n-1) activation-altitude
    fractions when ``optimize_alts``)."""
    n_particles = getattr(sim_params, "PSO_MG_N_PARTICLES", sim_params.PSO_COAST_N_PARTICLES)
    n_gen       = getattr(sim_params, "PSO_MG_MAX_GENERATIONS", sim_params.PSO_COAST_MAX_GENERATIONS)

    if verbose:
        print("\n" + "=" * 60)
        print("SEGMENTED GUIDANCE - PSO OPTIMISATION")
        print("=" * 60)
        sched = " -> ".join(f"{m}@{a/1e3:.0f}km" for m, a in segs.schedule)
        print(f"  Schedule : {sched}  -> orbit")
        if optimize_alts and alt_bounds is not None:
            print(f"  Optimising {segs.n - 1} activation altitude(s) in "
                  f"[{alt_bounds[0]/1e3:.0f}, {alt_bounds[1]/1e3:.0f}] km")
        print(f"  Particles: {n_particles}   Max gen.: {n_gen}")
        print("=" * 60 + "\n")

    t0 = time.time()
    try:
        import pygmo as pg
    except ImportError:
        raise ImportError("pygmo is required for the segmented guidance PSO. "
                          "Install it with: conda install -c conda-forge pygmo")

    _seed = getattr(sim_params, "PSO_MG_SEED", sim_params.PSO_COAST_SEED)
    prob = pg.problem(SegmentedPSOProblem(segs, optimize_alts=optimize_alts,
                                          alt_bounds=alt_bounds))
    algo = pg.algorithm(pg.pso(
        gen=n_gen,
        omega=getattr(sim_params, "PSO_MG_OMEGA", sim_params.PSO_COAST_OMEGA),
        eta1=getattr(sim_params, "PSO_MG_C1", sim_params.PSO_COAST_C1),
        eta2=getattr(sim_params, "PSO_MG_C2", sim_params.PSO_COAST_C2),
        max_vel=getattr(sim_params, "PSO_MG_VMAX", sim_params.PSO_COAST_VMAX),
        seed=_seed))
    if verbose:
        algo.set_verbosity(25)
    pop = pg.population(prob, size=n_particles, seed=_seed)
    pop = algo.evolve(pop)

    best_x = list(pop.champion_x)
    best_f = float(pop.champion_f[0])
    if verbose:
        print(f"\n[segmented PSO] finished in {time.time()-t0:.1f}s  best J' = {best_f:.4f}")
    return best_x, best_f


# ---------------------------------------------------------------------------
# Dense full re-run (for the report + a basic trajectory array)
# ---------------------------------------------------------------------------

def _teval_half_sec(t0, t1):
    pts = np.arange(t0, t1, 0.5)
    if len(pts) == 0 or pts[-1] < t1:
        pts = np.append(pts, t1)
    return pts


def run_segmented_full(optimal_params, segs, verbose=True):
    """Dense re-run of the optimum with full plot-suite assembly.

    Mirrors ``pso_coast_solver.run_pso_coast_full``: returns
    ``(time_full, data_full, thrust_full, alpha_full, t_ignition, result,
    coriolis_mag_data, centrifugal_mag_data)`` and writes the full-flight history
    channels (``ra.theta_*_history``, ``ra.tgo_*_history``, pseudo-force
    histories) + event markers so the shared plot suite in main.py renders the
    same plots as the single-law modes.
    """
    from Plots.plot_state_utils import interpolate_to_time

    dtc, dtr, cs, gp = (float(optimal_params[0]), float(optimal_params[1]),
                        float(optimal_params[2]), float(optimal_params[3]))

    result = run_segmented_trajectory(dtc, dtr, cs, gp, segs,
                                      teval_fn=_teval_half_sec, collect=True,
                                      verbose=verbose)

    # Crashed optimum: the 'stage1'/'s2_pieces'/'gs' collect keys are absent, so
    # return a degenerate payload (main.py detects the crash via result['crashed']).
    if result.get('crashed'):
        t1 = np.asarray(result.get('t_stage1', [0.0]), dtype=float)
        y1 = np.asarray(result.get('y_stage1', np.zeros((5, 1))), dtype=float)[:5]
        z = np.zeros(len(t1))
        return (t1, y1, z, z, result.get('t_ignition', 0.0), result, z, z)

    t_stage1, y_stage1 = result['stage1']
    t_stage1 = np.asarray(t_stage1, dtype=float)
    t_ignition = result['t_ignition']
    gs = result.get('gs')
    pieces = list(result.get('s2_pieces', []))           # [(sol, thrust_value), ...]

    # ---- Post-insertion orbit coast (thrust off) so plots show the full orbit ----
    # Orbital propagation needs the INERTIAL velocity, so convert the insertion
    # state (rotating-frame) here — diagnostic only (mirrors pso_coast). Expect a
    # small ~v_rot step in the velocity-vs-time plot at insertion.
    state_final = result.get('state_final')
    if state_final is not None:
        post_init = np.asarray(state_final[:5], dtype=float).copy()
        if sim_params.ENABLE_EARTH_ROTATION:
            v_in, g_in = ra.get_inertial_state_components(
                state_final[1], state_final[2], state_final[3],
                np.deg2rad(sim_params.LAUNCH_LATITUDE))
            post_init[2], post_init[3] = v_in, g_in
        t_post0 = result['t_arc3_end']
        sol_post = _ballistic(t_post0, t_post0 + sim_params.DURATION_AFTER_SIMULATION,
                              post_init, _teval_half_sec)
        if sol_post is not None and len(sol_post.t) > 0:
            pieces.append((sol_post, 0.0))

    # ---- Assemble Stage-2 arrays from the labeled pieces ----
    t_s2, y_s2, th_s2 = [], [], []
    for sol, F in pieces:
        if sol is None or len(sol.t) == 0:
            continue
        t_s2.append(np.asarray(sol.t, dtype=float))
        y_s2.append(np.asarray(sol.y, dtype=float)[:5, :])
        th_s2.append(np.full(len(sol.t), F))
    t_stage2_full = np.concatenate(t_s2)
    y_stage2_full = np.concatenate(y_s2, axis=1)
    thrust_stage2 = np.concatenate(th_s2)

    # Alpha: interpolate the gs log onto the Stage-2 output grid (samples the
    # accepted points, smoothing out RK trial-eval noise — same as pso_coast).
    if gs is not None and gs.time_log:
        alpha_stage2 = interpolate_to_time(gs.time_log, gs.alpha_log, t_stage2_full)
    else:
        alpha_stage2 = np.zeros(len(t_stage2_full))

    # ---- Stage 1 (from this run's ra.*_history; run_stage1 was called once) ----
    y1 = np.asarray(y_stage1, dtype=float)[:5, :]
    thrust_stage1 = interpolate_to_time(ra.time_history, ra.thrust_history, t_stage1)
    alpha_stage1  = interpolate_to_time(ra.alpha_time_history, ra.alpha_history, t_stage1)

    # ---- Combine Stage 1 + Stage 2 ----
    time_full   = np.concatenate([t_stage1, t_stage2_full])
    data_full   = np.concatenate([y1, y_stage2_full], axis=1)
    thrust_full = np.concatenate([thrust_stage1, thrust_stage2])
    alpha_full  = np.concatenate([alpha_stage1, alpha_stage2])
    n_stage1, n_stage2 = len(t_stage1), len(t_stage2_full)

    # ---- Latitude row (Earth rotation) so the latitude plot renders ----
    if sim_params.ENABLE_EARTH_ROTATION:
        lat_row = np.array([ra.get_latitude_from_downrange(s) for s in data_full[0]])
        data_full = np.vstack([data_full, lat_row])      # rows: s, r, v, gamma, m, lat

    # ---- Full-flight history channels for the plot suite (ra.*_history) ----
    theta_full = alpha_full + data_full[3]               # pitch theta = alpha + gamma
    ra.theta_history      = list(theta_full)
    ra.theta_time_history = list(time_full)
    if gs is not None and gs.tgo_time_log:
        ra.tgo_time_history = list(gs.tgo_time_log)       # apollo/lts/bilinear segments
        ra.tgo_history      = list(gs.tgo_log)
    else:
        ra.tgo_time_history = []
        ra.tgo_history      = []

    # Pseudo-forces: Stage-1 real (interpolated), Stage-2 inertial vacuum -> zeros.
    cor1 = np.asarray(ra.coriolis_mag_history, dtype=float)
    cen1 = np.asarray(ra.centrifugal_mag_history, dtype=float)
    cor_s1 = interpolate_to_time(ra.time_history, cor1, t_stage1) if len(cor1) else np.zeros(n_stage1)
    cen_s1 = interpolate_to_time(ra.time_history, cen1, t_stage1) if len(cen1) else np.zeros(n_stage1)
    coriolis_mag_data    = np.concatenate([cor_s1, np.zeros(n_stage2)])
    centrifugal_mag_data = np.concatenate([cen_s1, np.zeros(n_stage2)])

    if sim_params.COMPUTE_CROSS_HEADING_COUNTER_FORCE:
        chf = np.asarray(ra.cross_heading_counter_force_history, dtype=float)
        cha = np.asarray(ra.cross_heading_accel_history, dtype=float)
        chf = interpolate_to_time(ra.time_history, chf, t_stage1) if len(chf) else np.zeros(n_stage1)
        cha = interpolate_to_time(ra.time_history, cha, t_stage1) if len(cha) else np.zeros(n_stage1)
        ra.cross_heading_counter_force_history = list(np.concatenate([chf, np.zeros(n_stage2)]))
        ra.cross_heading_accel_history = list(np.concatenate([cha, np.zeros(n_stage2)]))

    # ---- Event markers used by plot_state_utils ----
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = result['t_arc3_end']
    ra.PSO_COAST_ARC2_START_TIME              = result['t_arc2_start']

    if verbose:
        sf = data_full[:, -1]
        print(f"\n[segmented full run] t_end={time_full[-1]:.1f}s, "
              f"h={(sf[1]-c.R_EARTH)/1e3:.1f}km, v={sf[2]:.0f}m/s, "
              f"gam={np.rad2deg(sf[3]):.2f}deg")

    return (time_full, data_full, thrust_full, alpha_full, t_ignition, result,
            coriolis_mag_data, centrifugal_mag_data)


# ---------------------------------------------------------------------------
# Top-level entry point (called by main.py when MULTI_GUIDANCE_ENABLED)
# ---------------------------------------------------------------------------

def validate_schedule():
    """Raise ValueError on a malformed GUIDANCE_SEGMENTS schedule."""
    segments = sim_params.GUIDANCE_SEGMENTS
    supported = {"apollo", "peg_new", "linear_tangent", "bilinear_tangent",
                 "gravity_turn", "indirect_pmp"}
    if not segments:
        raise ValueError("GUIDANCE_SEGMENTS is empty.")
    alts = [float(a) for _, a in segments]
    if any(alts[i] >= alts[i + 1] for i in range(len(alts) - 1)):
        raise ValueError("GUIDANCE_SEGMENTS activation altitudes must be strictly increasing.")
    for mode, _ in segments:
        if mode not in supported:
            raise ValueError(
                f"Unsupported segmented guidance law '{mode}'. "
                f"Supported: {sorted(supported)}.")


def run_segmented(verbose=True):
    """Build the PMP reference, optimise, and run the full dense trajectory.

    Returns a dict with everything main.py needs for the report and the shared
    plot suite: time, data, thrust, alpha, coriolis, centrifugal, t_ignition,
    result, best_x, best_f, segs.
    """
    validate_schedule()
    # A segment that reuses the stored indirect-PMP control needs the reference's
    # α history; otherwise the lighter state-only reference is enough.
    _uses_pmp = any(str(m) == "indirect_pmp" for m, _ in sim_params.GUIDANCE_SEGMENTS)
    if _uses_pmp:
        time_ref, data_ref, alpha_ref = segref.get_pmp_reference_full(verbose=verbose)
        segs = _Segments(time_ref, data_ref, alpha_full=alpha_ref)
    else:
        time_ref, data_ref = segref.get_pmp_reference(verbose=verbose)
        segs = _Segments(time_ref, data_ref)

    # Optional: let the PSO also choose the (n-1) activation altitudes (segment 0
    # stays right after the kick) to minimise Stage-2 burn time. Bounds come from
    # the config, clamped to the reference apogee so waypoint lookups stay on the
    # monotonic ascent prefix.
    optimize_alts = bool(getattr(sim_params, "MULTI_GUIDANCE_OPTIMIZE_ALTITUDES", False))
    alt_bounds = None
    if optimize_alts and segs.n > 1:
        apogee_alt = float(segs._alt_asc[-1])
        alt_ub = min(float(getattr(sim_params, "MULTI_GUIDANCE_ALT_UB", 200_000.0)),
                     0.98 * apogee_alt)
        alt_lb = float(getattr(sim_params, "MULTI_GUIDANCE_ALT_LB", 10_000.0))
        alt_lb = min(alt_lb, 0.5 * alt_ub)      # keep lb < ub even for a low apogee
        alt_bounds = (alt_lb, alt_ub)
    else:
        optimize_alts = False                    # nothing to optimise (single segment / flag off)

    if verbose and not optimize_alts:
        # Per-guidance objectives: the PMP-reference state (altitude, speed,
        # flight-path angle, time) at each law's hand-off altitude. The final
        # law aims at orbit insertion (target is None), so its objective is read
        # straight from the PMP reference at the orbit altitude.
        print("\nPer-guidance objectives (PMP reference at each hand-off altitude):")
        apogee_alt = float(segs._alt_asc[-1])   # clamp so the final (orbit) lookup
        for i in range(segs.n):                 # never trips the "exceeds apogee" warning
            mode, start = segs.schedule[i]
            look_alt = min(segs.target_alt[i], apogee_alt)
            wp = segref.waypoint_at_altitude(data_ref, time_ref, look_alt)
            tag = "  (orbit insertion)" if segs.target[i] is None else ""
            print(f"  {mode:16s} start {start/1e3:6.1f} km  ->  objective @ "
                  f"{wp['alt']/1e3:6.1f} km : v={wp['v']:7.1f} m/s, "
                  f"fpa={np.rad2deg(wp['gamma']):6.2f} deg, t={wp['t']:6.1f} s{tag}")
    elif verbose:
        print(f"\nActivation altitudes are PSO decision variables (bounds "
              f"[{alt_bounds[0]/1e3:.0f}, {alt_bounds[1]/1e3:.0f}] km); the "
              "per-guidance objectives follow the optimum.")

    best_x, best_f = run_segmented_optimization(
        segs, optimize_alts=optimize_alts, alt_bounds=alt_bounds, verbose=verbose)

    # Bake the optimal activation altitudes into segs before the dense re-run
    # (run_segmented_full uses only the 4 base params).
    opt_alts = None
    if optimize_alts:
        opt_alts = _alts_from_fractions(best_x[4:4 + (segs.n - 1)],
                                        alt_bounds[0], alt_bounds[1])
        segs.set_activation_altitudes(opt_alts)
        if verbose:
            print("  optimised activation altitudes: " +
                  ", ".join(f"{a/1e3:.1f} km" for a in opt_alts))

    (time_full, data_full, thrust_full, alpha_full, t_ignition, result,
     coriolis_mag_data, centrifugal_mag_data) = run_segmented_full(
        best_x[:4], segs, verbose=verbose)
    return {
        'time': time_full, 'data': data_full,
        'thrust': thrust_full, 'alpha': alpha_full,
        'coriolis': coriolis_mag_data, 'centrifugal': centrifugal_mag_data,
        't_ignition': t_ignition, 'result': result,
        'best_x': best_x, 'best_f': best_f, 'segs': segs,
        'optimized_altitudes': opt_alts,
    }
