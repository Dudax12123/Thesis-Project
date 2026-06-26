"""
Segmented Guidance Solver — multi-law, altitude-triggered ascent guidance.

Flies the ordered ``simulation_parameters.GUIDANCE_SEGMENTS`` schedule instead of a
single guidance law: a passive gravity turn from launch until the first activation
altitude, then each chosen law in turn. Each non-final segment aims at the
indirect-PMP optimal (alt, v, gamma) waypoint at the NEXT activation altitude; the
final segment aims at orbit insertion. Time-to-go is a planned-deadline countdown
(deadline - t) sourced from the PMP reference, so it never collapses across the
stage boundary — this is what lets the t_go-dependent laws fly DURING Stage 1.

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

    schedule[0] is the implicit ("gravity_turn", 0.0) prefix; schedule[1:] are the
    user's GUIDANCE_SEGMENTS. For segment i, target_alt[i] is the activation
    altitude of segment i+1 (or the orbit altitude for the final segment), and
    target[i] is the SegmentTarget aiming at that waypoint (or None ⇒ circular
    orbit for the final segment).
    """

    def __init__(self, time_full, data_full):
        self.schedule = [("gravity_turn", 0.0)] + [
            (str(mode), float(alt)) for (mode, alt) in sim_params.GUIDANCE_SEGMENTS
        ]
        self.n = len(self.schedule)

        # Ascent prefix for monotonic altitude->time / waypoint interpolation
        alt_col = np.asarray(data_full[1], dtype=float) - c.R_EARTH
        i_top = int(np.argmax(alt_col))
        if i_top < 1:
            i_top = len(alt_col) - 1
        self._alt_asc = alt_col[: i_top + 1]
        self._t_asc = np.asarray(time_full, dtype=float)[: i_top + 1]

        orbit_alt = float(sim_params.TARGET_ORBITAL_ALTITUDE)
        ft = float(getattr(sim_params, "SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD", 2.0))

        self.target_alt = []
        self.target = []
        for i in range(self.n):
            if i + 1 < self.n:
                t_alt = self.schedule[i + 1][1]
                wp = segref.waypoint_at_altitude(data_full, time_full, t_alt)
                self.target_alt.append(t_alt)
                self.target.append(pcs.SegmentTarget.from_waypoint(wp, freeze_threshold=ft))
            else:
                self.target_alt.append(orbit_alt)
                self.target.append(None)   # final segment -> circular orbit

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
    mgr = {"idx": 0}

    crashed_result = lambda **kw: {
        'crashed': True, 'state_final': None, 't_f': 0.0, 't_cf': 0.0,
        't_stage2_start': 0.0, 't_ignition': 0.0, 't_arc2_start': 0.0,
        't_arc3_end': 0.0, **kw}

    # ---- Stage 1 (gravity turn -> first chosen law via the steering hook) ----
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
    s2_pieces.append(sol_pre)
    if len(sol_pre.t_events[0]) > 0:
        return crashed_result(t_stage2_start=t2_start, t_ignition=t_ignition,
                              t_stage1=t_stage1, y_stage1=y_stage1)
    state_at_ign = sol_pre.y[:5, -1].copy()

    # ---- Arc 1 (thrust) ----
    if t_coast_start > 0.01:
        t_a2, state_arc2, crashed, p = _thrust_phase(
            t_ignition, t_coast_start, state_at_ign, gs, segs, mgr, teval_fn)
        s2_pieces += p
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
        s2_pieces.append(sol_c)
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
        s2_pieces += p
        if crashed:
            return crashed_result(t_stage2_start=t2_start, t_ignition=t_ignition,
                                  t_arc2_start=t_arc2_start,
                                  t_stage1=t_stage1, y_stage1=y_stage1)
    else:
        state_final = state_arc3.copy()

    result = {
        'crashed':        False,
        'state_final':    state_final,
        't_f':            T_burn_total + delta_tc,
        't_cf':           delta_tc,
        't_stage2_start': t2_start,
        't_ignition':     t_ignition,
        't_arc2_start':   t_arc2_start,
        't_arc3_end':     t_ignition + T_burn_total + delta_tc,
        't_stage1':       t_stage1,
        'y_stage1':       y_stage1,
        'final_seg_idx':  mgr["idx"],
    }
    if collect:
        result['stage1'] = (t_stage1, y_stage1)
        result['s2_pieces'] = s2_pieces
    if verbose and not result['crashed']:
        sf = state_final
        print(f"  insertion: h={(sf[1]-c.R_EARTH)/1e3:.1f} km  v={sf[2]:.1f} m/s  "
              f"gam={np.rad2deg(sf[3]):.3f} deg  (reached segment idx {mgr['idx']})")
    return result


# ---------------------------------------------------------------------------
# PyGMO problem + PSO runner
# ---------------------------------------------------------------------------

class SegmentedPSOProblem:
    """UDP for PyGMO: decision vector [delta_tc, delta_tr_pct, coast_start_pct, gamma_p]."""

    def __init__(self, segs):
        self._segs = segs

    def fitness(self, x):
        try:
            dtc, dtr, cs, gp = float(x[0]), float(x[1]), float(x[2]), float(x[3])
            result = run_segmented_trajectory(dtc, dtr, cs, gp, self._segs)
            return [pcs.compute_coast_objective(result)]
        except Exception:
            return [pcs.CRASH_PENALTY]

    def get_bounds(self):
        return (list(sim_params.PSO_COAST_LB), list(sim_params.PSO_COAST_UB))

    def get_nobj(self):
        return 1


def run_segmented_optimization(segs, verbose=True):
    """Run the PSO over [delta_tc, delta_tr_pct, coast_start_pct, gamma_p]."""
    n_particles = sim_params.PSO_COAST_N_PARTICLES
    n_gen       = sim_params.PSO_COAST_MAX_GENERATIONS

    if verbose:
        print("\n" + "=" * 60)
        print("SEGMENTED GUIDANCE - PSO OPTIMISATION")
        print("=" * 60)
        sched = " -> ".join(f"{m}@{a/1e3:.0f}km" for m, a in segs.schedule)
        print(f"  Schedule : gravity_turn -> {sched}")
        print(f"  Particles: {n_particles}   Max gen.: {n_gen}")
        print("=" * 60 + "\n")

    t0 = time.time()
    try:
        import pygmo as pg
    except ImportError:
        raise ImportError("pygmo is required for the segmented guidance PSO. "
                          "Install it with: conda install -c conda-forge pygmo")

    prob = pg.problem(SegmentedPSOProblem(segs))
    algo = pg.algorithm(pg.pso(
        gen=n_gen, omega=sim_params.PSO_COAST_OMEGA,
        eta1=sim_params.PSO_COAST_C1, eta2=sim_params.PSO_COAST_C2,
        max_vel=sim_params.PSO_COAST_VMAX, seed=sim_params.PSO_COAST_SEED))
    if verbose:
        algo.set_verbosity(25)
    pop = pg.population(prob, size=n_particles, seed=sim_params.PSO_COAST_SEED)
    pop = algo.evolve(pop)

    best_x = list(pop.champion_x)
    best_f = float(pop.champion_f[0])
    if verbose:
        print(f"\n[segmented PSO] finished in {time.time()-t0:.1f}s  best J' = {best_f:.4f}")
    return best_x, best_f


# ---------------------------------------------------------------------------
# Dense full re-run (for the report + a basic trajectory array)
# ---------------------------------------------------------------------------

def run_segmented_full(optimal_params, segs, verbose=True):
    """Dense re-run of the optimum. Returns (time_full, data_full, result)."""
    dtc, dtr, cs, gp = (float(optimal_params[0]), float(optimal_params[1]),
                        float(optimal_params[2]), float(optimal_params[3]))

    def _teval(t0, t1):
        pts = np.arange(t0, t1, 0.5)
        if len(pts) == 0 or pts[-1] < t1:
            pts = np.append(pts, t1)
        return pts

    result = run_segmented_trajectory(dtc, dtr, cs, gp, segs,
                                      teval_fn=_teval, collect=True, verbose=verbose)

    # Stitch Stage-1 + Stage-2 dense pieces into (time_full, data_full[5xN]).
    t_list, y_list = [], []
    if 'stage1' in result and result['stage1'][0] is not None:
        t1, y1 = result['stage1']
        t_list.append(np.asarray(t1, dtype=float))
        y_list.append(np.asarray(y1, dtype=float)[:5])
    for sol in result.get('s2_pieces', []):
        if sol is None or len(sol.t) == 0:
            continue
        t_list.append(np.asarray(sol.t, dtype=float))
        y_list.append(np.asarray(sol.y, dtype=float)[:5])

    if t_list:
        time_full = np.concatenate(t_list)
        data_full = np.concatenate(y_list, axis=1)
    else:
        time_full = np.array([0.0])
        data_full = np.zeros((5, 1))
    return time_full, data_full, result


# ---------------------------------------------------------------------------
# Top-level entry point (called by main.py when MULTI_GUIDANCE_ENABLED)
# ---------------------------------------------------------------------------

def validate_schedule():
    """Raise ValueError on a malformed GUIDANCE_SEGMENTS schedule."""
    segments = sim_params.GUIDANCE_SEGMENTS
    supported = {"apollo", "peg_new", "linear_tangent", "bilinear_tangent", "gravity_turn"}
    if not segments:
        raise ValueError("GUIDANCE_SEGMENTS is empty.")
    alts = [float(a) for _, a in segments]
    if any(alts[i] >= alts[i + 1] for i in range(len(alts) - 1)):
        raise ValueError("GUIDANCE_SEGMENTS activation altitudes must be strictly increasing.")
    for mode, _ in segments:
        if mode not in supported:
            raise ValueError(
                f"Unsupported segmented guidance law '{mode}'. "
                f"Supported: {sorted(supported - {'gravity_turn'})}.")


def run_segmented(verbose=True):
    """Build the PMP reference, optimise, and return (time_full, data_full, result, best_x, segs)."""
    validate_schedule()
    time_ref, data_ref = segref.get_pmp_reference(verbose=verbose)
    segs = _Segments(time_ref, data_ref)

    if verbose:
        print("\nResolved segment waypoints (from PMP reference):")
        for i in range(segs.n):
            mode, act = segs.schedule[i]
            tgt = segs.target[i]
            if tgt is None:
                print(f"  [{i}] {mode:16s} from {act/1e3:6.1f} km  ->  ORBIT "
                      f"({segs.target_alt[i]/1e3:.0f} km, circular)")
            else:
                print(f"  [{i}] {mode:16s} from {act/1e3:6.1f} km  ->  waypoint "
                      f"@ {tgt.alt/1e3:6.1f} km: v={tgt.v:.1f} m/s, "
                      f"gam={np.rad2deg(tgt.gamma):.2f} deg")

    best_x, best_f = run_segmented_optimization(segs, verbose=verbose)
    time_full, data_full, result = run_segmented_full(best_x, segs, verbose=verbose)
    return time_full, data_full, result, best_x, best_f, segs
