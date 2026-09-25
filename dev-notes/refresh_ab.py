#!/usr/bin/env python
"""Refresh A/B: guidance coefficients refreshed inside the ODE RHS vs once per guidance cycle.

peg_new and apollo refresh their steering coefficients inside the ODE right-hand side
(pso_coast_solver._compute_alpha_stage2), so the refresh -- and apollo's freeze latch --
fires on whatever point solve_ivp evaluates: RK stages and rejected steps up to _MAX_STEP
= 10 s ahead, on extrapolated states. This script measures what that costs, for the seven
results-matrix cases that fly it, WITHOUT touching production code:

  in_rhs  production, untouched; only records nfev and when each refresh fired
  cycle   every guided thrust arc is split at guidance-cycle boundaries. At each boundary the
          arc's own RHS is called once on the ACCEPTED state with the real rate -- so the
          production init/freeze/refresh code runs -- and the cycle is then integrated with
          the rate held at inf (and apollo's freeze threshold at -inf), so nothing refreshes
          mid-cycle. The law code is identical; only where the refresh is sampled changes.

It patches the module-level solve_ivp of pso_coast_solver, direct_pso_solver and
segmented_guidance_solver (each binds its own), recognising a guided arc by the
GuidanceState in the RHS lambda's closure, and wraps pcs._compute_alpha_stage2 to record
every coefficient update (and to refuse one inside a cycle, or in Stage 1 via the segmented
hook, which the wrapper does not cover).

Per case and mode:
  at x    J, final mass, insertion state, arc-1 end state
  cost    median wall time of the fitness body, nfev, solve_ivp calls
  defect  (in_rhs) share of coefficient updates made off an accepted step, max look-ahead
  noise   J and final mass along each decision coordinate: 9 points at h = 1e-6 and 1e-4 of
          the bound range, noise from k-th differences (Moré & Wild 2011), plus a 41-point
          line over +/-1e-3 of the range; compared with the archived swarm's late gbest gains
  flight  both full flights archived under --out/<case>/ for run_archive.py compare

The decision vectors are representative points, not current optima (see X_SOURCES).

Production has had the same thing since 2026-09-24: GUIDANCE_REFRESH_MODE = "cycle"
(pso_coast_solver.solve_guided_arc). This script pins the in_rhs production path for its
own two modes, and at each x also flies production with the switch on: that J must equal
this script's cycle J bit for bit (J_production_cycle in the per-case JSON).

--onoff flies each case's full flight through PRODUCTION with the switch off and on
(<case>__off / <case>__on, whose manifests differ in GUIDANCE_REFRESH_MODE alone), overlays
each pair with run_archive.py compare, tabulates them and redraws the noise figures
(Plots/results_figures/diag_refresh_noise.py) into Output_Plots/comparisons/refresh_onoff/.

Run from the repository root:
    C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/refresh_ab.py
    C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/refresh_ab.py --selftest
    C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/refresh_ab.py --onoff
"""

import argparse
import contextlib
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "Tese" / "src"
sys.path.insert(0, str(SRC))

from Input_File import simulation_parameters as sp  # noqa: E402
import run_results_matrix as rrm  # noqa: E402

OUT_DEFAULT = SRC / "Output" / "refresh_ab"
OUTPUT = SRC / "Output"

CASES = ("peg_baseline", "peg_vacuum", "peg_vacuum_norot", "peg_direct",
         "show_apollo", "show_seg_fixed_alt", "show_seg_opt_alt")

# Where each case's decision vector comes from. The peg_* vectors are the production
# 250x1000 swarms of 2026-09-18, flown before the peg_new realignment (28dd06d), so they no
# longer reproduce their rows. show_apollo has no production archive; the 09-22 50x100
# "arc1off" build is the only one with a vector on the current apollo code path. The two
# segmented cases have no archived vector at all and BORROW peg_baseline's 4-vector (same
# layout and bounds). show_seg_opt_alt adds a switch altitude of 150 km: its stale 100x500
# optimum (364.8 km) is never reached on this schedule, so peg_new would not fly at all.
# Measured 2026-09-24: at HEAD the three pso_coast peg vectors and the borrowed pair
# overshoot to 590-660 km at gamma -11..-17 deg, so their J is penalty-dominated.
X_SOURCES = {
    "peg_baseline": ("results_matrix", "peg_baseline", False),
    "peg_vacuum": ("results_matrix", "peg_vacuum", False),
    "peg_vacuum_norot": ("results_matrix", "peg_vacuum_norot", False),
    "peg_direct": ("results_matrix", "peg_direct", False),
    "show_apollo": ("results_matrix_arc1off_r50x100", "show_apollo", False),
    "show_seg_fixed_alt": ("results_matrix", "peg_baseline", True),
    "show_seg_opt_alt": ("results_matrix", "peg_baseline", True),
}
SEG_OPT_ALT_M = 150e3   # see X_SOURCES

REFRESH_RATE_PARAM = {"peg_new": "PEG_MAJOR_LOOP_RATE", "apollo": "GUIDANCE_UPDATE_RATE"}
PASS_THROUGH = {"gravity_turn", "cpr", "exp_shooting", "indirect_pmp"}
COAST_THRESHOLD = 0.01            # pso_coast: delta_tc <= this flies no coast (no restart)
SCALES = {"fine": 1e-6, "coarse": 1e-4}
N_NOISE = 9
N_LINE, LINE_HALF = 41, 1e-3

# Solver modules, imported only after the case's configuration is applied.
pcs = dps = sgs = segref = None
_REAL_SOLVE_IVP = None


def _import_solvers():
    global pcs, dps, sgs, segref, _REAL_SOLVE_IVP
    import Simulation.pso_coast_solver as _pcs
    import Simulation.direct_pso_solver as _dps
    import Simulation.segmented_guidance_solver as _sgs
    import Simulation.segment_reference as _segref
    pcs, dps, sgs, segref = _pcs, _dps, _sgs, _segref
    _REAL_SOLVE_IVP = pcs.solve_ivp
    assert dps.solve_ivp is _REAL_SOLVE_IVP and sgs.solve_ivp is _REAL_SOLVE_IVP


def configure(case, extra=None):
    rrm._apply(sp, rrm.BASELINE)
    rrm._apply(sp, {c["name"]: c for c in rrm.build_matrix()}[case]["overrides"])
    # Both modes here run over production's in_rhs path; "cycle" is emulated around it.
    # The flights measured are the swarm-timed ones (x layouts of 2026-09-24): the
    # matrix has since let peg_new end the peg_direct and segmented burns (2026-09-25),
    # and those law-terminated burns refresh outside the ODE in either mode.
    rrm._apply(sp, {"EVENTS_PRINT": False, "INTERRUPTS_PRINT": False,
                    "GUIDANCE_REFRESH_MODE": "in_rhs",
                    "DIRECT_LAW_TERMINATED_CUTOFF": False, "DIRECT_OPTIMIZER": "pso",
                    "SEGMENTED_LAW_TERMINATED_ARCS": False})
    if extra:
        rrm._apply(sp, extra)
    _import_solvers()


# ---------------------------------------------------------------------------
# The wrapper
# ---------------------------------------------------------------------------

class Recorder:
    """Per-evaluation bookkeeping; ``mode`` is 'in_rhs' or 'cycle'. ``force_split`` (s)
    splits pass-through arcs too -- the stitching self-test."""

    def __init__(self, mode, force_split=None):
        self.mode, self.force_split = mode, force_split
        self.reset()

    def reset(self):
        self.nfev = 0
        self.n_calls = 0
        self.n_boundary = 0
        self.events = []          # coefficient updates, see _log
        self.arc_ends = []        # (t, y) at the end of every guided call
        self._pending = None
        self.in_guided = False
        self.inner = False

    def _log(self, t, kind, law):
        e = {"t": float(t), "kind": kind, "law": law, "accepted": None, "lookahead": None,
             "where": "stage2" if self.in_guided else "outside"}
        if self.in_guided and self._pending is not None:
            self._pending.append(e)
        self.events.append(e)
        return e

    def stats(self):
        ev = [e for e in self.events if e["law"] in REFRESH_RATE_PARAM]
        known = [e for e in ev if e["accepted"] is not None]
        trial = [e for e in known if not e["accepted"]]
        looks = [e["lookahead"] for e in known if e["lookahead"] is not None]
        return {
            "n_updates": len(ev),
            "n_refresh": sum(e["kind"] == "refresh" for e in ev),
            "n_freeze": sum(e["kind"] == "freeze" for e in ev),
            "n_outside_stage2": sum(e["where"] != "stage2" for e in ev),
            "n_classified": len(known),
            "n_on_trial_points": len(trial),
            "share_on_trial_points": (len(trial) / len(known)) if known else float("nan"),
            "max_lookahead_s": max(looks) if looks else float("nan"),
            "nfev_stage2": self.nfev,
            "n_solve_ivp": self.n_calls,
            "n_boundary_calls": self.n_boundary,
        }


def _guidance_state_of(fun):
    for cell in (getattr(fun, "__closure__", None) or ()):
        try:
            v = cell.cell_contents
        except ValueError:
            continue
        if isinstance(v, pcs.GuidanceState):
            return v
    return None


def _law(gs):
    return gs.mode_override if gs.mode_override is not None else sp.GUIDANCE_MODE


def _classify(gs):
    """'refresh' for an in-RHS refreshing law, 'pass' for one with nothing to refresh."""
    law = _law(gs)
    if gs.peg_new_external or gs.apollo_external:
        return "pass"          # already refreshed outside the ODE by its caller
    if law in REFRESH_RATE_PARAM:
        if (law == "apollo" and gs.target is not None
                and gs.target.freeze_threshold is not None):
            raise NotImplementedError("apollo with a per-segment freeze threshold")
        return "refresh"
    if law in PASS_THROUGH:
        return "pass"
    if law in ("linear_tangent", "bilinear_tangent") and gs.pso_tan_theta0 is not None:
        return "pass"          # open-loop, swarm constants
    raise NotImplementedError(f"the refresh A/B does not cover {law!r} on this arc")


def _snap(gs):
    return (gs.guidance_phase_active, gs.last_guidance_update_time, gs.peg_new_t_epoch,
            gs.apollo_freeze_time, gs.peg_new_frozen, gs.apollo_coefficients_frozen,
            id(gs.guidance_coefficients))


def _make_alpha_wrapper(real_alpha, rec):
    def wrapped(t, state, F_T, Isp, gs):
        before = _snap(gs)
        alpha = real_alpha(t, state, F_T, Isp, gs)
        after = _snap(gs)
        if after != before:
            law = _law(gs)
            if not before[0] and after[0]:
                kind = "init"
            elif (after[4] and not before[4]) or (after[5] and not before[5]):
                kind = "freeze"
            else:
                kind = "refresh"
            if rec.inner and law in REFRESH_RATE_PARAM:
                raise AssertionError(f"cycle mode: {law} {kind} inside a cycle at t={t!r}")
            if (rec.mode == "cycle" and not rec.in_guided and law in REFRESH_RATE_PARAM):
                raise AssertionError(f"{law} {kind} outside a wrapped Stage-2 arc at t={t!r} "
                                     "(Stage-1 segmented hook?); the cycle mode cannot cover it")
            rec._log(t, kind, law)
        return alpha
    return wrapped


def _events_list(events):
    if events is None:
        return []
    return list(events) if isinstance(events, (list, tuple)) else [events]


def _make_solve_ivp(rec):
    from scipy.integrate._ivp.ivp import OdeResult

    def cycle_solve(fun, gs, kind, t_span, y0, kw):
        law = _law(gs)
        t0, tf = float(t_span[0]), float(t_span[1])
        t_eval = kw.pop("t_eval", None)
        kw.pop("dense_output", None)
        n_ev = len(_events_list(kw.get("events")))
        if kind == "refresh":
            rate_param = REFRESH_RATE_PARAM[law]
            rate_obj = getattr(sp, rate_param)
            rate = float(rate_obj)
        else:
            rate_param, rate_obj, rate = None, None, float(rec.force_split)
        freeze_obj = sp.APOLLO_FREEZE_THRESHOLD
        t, y = t0, np.asarray(y0, dtype=float).copy()
        ts, ys = [], []
        tev, yev = [[] for _ in range(n_ev)], [[] for _ in range(n_ev)]
        nfev, status, message, first = 0, 0, "cycle-split", True
        rec.in_guided = True
        try:
            while t < tf:
                if kind == "refresh":
                    # the guidance cycle, on the accepted state
                    prev = gs.last_guidance_update_time
                    due = gs.guidance_phase_active and (t - prev) >= rate - 1e-9
                    if due:
                        gs.last_guidance_update_time = -np.inf
                    n_before = len(rec.events)
                    fun(t, y)
                    rec.n_boundary += 1
                    if gs.last_guidance_update_time == -np.inf:   # frozen: nothing ran
                        gs.last_guidance_update_time = prev
                    for e in rec.events[n_before:]:              # made on the accepted state
                        e["accepted"], e["lookahead"] = True, 0.0
                        if e["kind"] == "refresh":
                            k = (t - gs.time_guidance_start) / rate
                            assert abs(k - round(k)) < 1e-6, \
                                f"refresh off the cycle grid at t={t!r} (k={k})"
                    frozen = gs.peg_new_frozen if law == "peg_new" else gs.apollo_coefficients_frozen
                    t_next = tf if frozen else min(gs.last_guidance_update_time + rate, tf)
                else:
                    t_next = min(t + rate, tf)
                if t_next <= t:
                    raise AssertionError(f"no progress at t={t!r}")
                snap = _snap(gs)
                if rate_param:
                    setattr(sp, rate_param, np.inf)
                if law == "apollo":
                    sp.APOLLO_FREEZE_THRESHOLD = -np.inf
                rec.inner = True
                try:
                    sol = _REAL_SOLVE_IVP(fun, (t, t_next), y, dense_output=t_eval is not None,
                                          **kw)
                finally:
                    rec.inner = False
                    if rate_param:
                        setattr(sp, rate_param, rate_obj)
                    sp.APOLLO_FREEZE_THRESHOLD = freeze_obj
                if kind == "refresh":
                    assert _snap(gs) == snap, f"guidance state changed inside the cycle ending {t_next!r}"
                nfev += sol.nfev
                t_hi = float(sol.t[-1])
                if t_eval is None:
                    ts.append(sol.t if first else sol.t[1:])
                    ys.append(sol.y if first else sol.y[:, 1:])
                else:
                    te = np.asarray(t_eval, dtype=float)
                    m = ((te >= t) if first else (te > t)) & (te <= t_hi)
                    ts.append(te[m])
                    ys.append(sol.sol(te[m]) if m.any() else np.empty((len(y), 0)))
                for i in range(n_ev):
                    tev[i].extend(np.atleast_1d(sol.t_events[i]).tolist())
                    yev[i].extend(list(sol.y_events[i]))
                first = False
                if sol.status != 0:            # terminal event (1) or failure (-1)
                    status, message = sol.status, sol.message
                    t, y = t_hi, sol.y[:, -1].copy()
                    break
                t, y = t_hi, sol.y[:, -1].copy()
        finally:
            rec.in_guided = False
        t_all = np.concatenate(ts) if ts else np.empty(0)
        y_all = np.concatenate(ys, axis=1) if ys else np.empty((len(y), 0))
        if n_ev:
            t_events = [np.asarray(a, dtype=float) for a in tev]
            y_events = [np.asarray(a, dtype=float).reshape(-1, len(y)) for a in yev]
        else:
            t_events = y_events = None
        rec.nfev += nfev
        rec.arc_ends.append((t, y.copy()))
        return OdeResult(t=t_all, y=y_all, sol=None, t_events=t_events, y_events=y_events,
                         nfev=nfev, njev=0, nlu=0, status=status, message=message,
                         success=status >= 0)

    def solve_ivp(fun, t_span, y0, **kw):
        rec.n_calls += 1
        gs = _guidance_state_of(fun)
        kind = _classify(gs) if gs is not None else None
        split = gs is not None and (
            (rec.mode == "cycle" and kind == "refresh")
            or (kind == "pass" and rec.force_split is not None))
        if split:
            return cycle_solve(fun, gs, kind, t_span, y0, dict(kw))
        if gs is None or rec.mode == "cycle":
            sol = _REAL_SOLVE_IVP(fun, t_span, y0, **kw)
            rec.nfev += sol.nfev
            if gs is not None:
                rec.arc_ends.append((float(sol.t[-1]), sol.y[:, -1].copy()))
            return sol
        # in_rhs: production, recorded
        rec.in_guided, rec._pending = True, []
        try:
            sol = _REAL_SOLVE_IVP(fun, t_span, y0, **kw)
        finally:
            rec.in_guided = False
        if kw.get("t_eval") is None:           # sol.t = every accepted step
            acc = np.asarray(sol.t, dtype=float)
            accset = set(acc.tolist())
            for e in rec._pending:
                e["accepted"] = e["t"] in accset
                prior = acc[acc <= e["t"]]
                e["lookahead"] = float(e["t"] - prior[-1]) if len(prior) else float("nan")
        rec._pending = None
        rec.nfev += sol.nfev
        rec.arc_ends.append((float(sol.t[-1]), sol.y[:, -1].copy()))
        return sol

    return solve_ivp


@contextlib.contextmanager
def refresh_mode(mode, force_split=None):
    """Patch the three solver modules for ``mode``; yields the Recorder."""
    rec = Recorder(mode, force_split)
    real_alpha = pcs._compute_alpha_stage2
    wrapper = _make_solve_ivp(rec)
    try:
        for m in (pcs, dps, sgs):
            m.solve_ivp = wrapper
        pcs._compute_alpha_stage2 = _make_alpha_wrapper(real_alpha, rec)
        yield rec
    finally:
        for m in (pcs, dps, sgs):
            m.solve_ivp = _REAL_SOLVE_IVP
        pcs._compute_alpha_stage2 = real_alpha


# ---------------------------------------------------------------------------
# Cases
# ---------------------------------------------------------------------------

def _load_npz(dirname, stem):
    p = OUTPUT / dirname / stem / f"{stem}.npz"
    z = np.load(p, allow_pickle=True)
    hist = None
    if "pso_gbest" in z.files and "pso_gen" in z.files:
        hist = (np.asarray(z["pso_gen"], dtype=float), np.asarray(z["pso_gbest"], dtype=float))
    return p, np.asarray(z["decision_vector"], dtype=float), hist


class Case:
    """Decision vector, bounds and the three ways of flying it for one matrix case."""

    def __init__(self, name):
        self.name = name
        dirname, stem, borrowed = X_SOURCES[name]
        path, x, hist = _load_npz(dirname, stem)
        self.x_source = str(path.relative_to(ROOT)).replace("\\", "/")
        self.borrowed = borrowed
        self.history = None if borrowed else hist
        if sp.MULTI_GUIDANCE_ENABLED:
            self.arch = "segmented"
            self._setup_segmented()
            if self.optimize_alts:
                lo, hi = self.alt_bounds
                frac = min(max((SEG_OPT_ALT_M - lo) / (hi - lo), 0.0), 1.0)
                x = np.append(x[:4], frac)
            self.lb, self.ub = map(np.asarray, sgs.SegmentedPSOProblem(
                self.segs, self.optimize_alts, self.alt_bounds).get_bounds())
        elif sp.COAST_METHOD == "direct":
            self.arch = "direct"
            self.lb, self.ub = map(np.asarray, dps._decision_bounds())
        else:
            self.arch = "pso_coast"
            self.lb, self.ub = map(np.asarray, pcs._coast_bounds())
        self.x = np.asarray(x, dtype=float)
        if len(self.x) != len(self.lb):
            raise SystemExit(f"{name}: vector of {len(self.x)} against {len(self.lb)} bounds")

    def _setup_segmented(self):
        def refuse(*a, **k):
            raise RuntimeError("refresh_ab refuses to rebuild the PMP reference")
        segref._run_pmp_reference = refuse
        sgs.validate_schedule()
        time_ref, data_ref = segref.get_pmp_reference(verbose=False)
        self.segs = sgs._Segments(time_ref, data_ref)
        self.optimize_alts = (bool(getattr(sp, "MULTI_GUIDANCE_OPTIMIZE_ALTITUDES", False))
                              and self.segs.n > 1)
        self.alt_bounds = None
        if self.optimize_alts:     # as run_segmented computes them
            apogee_alt = float(self.segs._alt_asc[-1])
            alt_ub = min(float(getattr(sp, "MULTI_GUIDANCE_ALT_UB", 200_000.0)), 0.98 * apogee_alt)
            alt_lb = float(getattr(sp, "MULTI_GUIDANCE_ALT_LB", 10_000.0))
            alt_lb = min(alt_lb, 0.5 * alt_ub)
            self.alt_bounds = (alt_lb, alt_ub)

    def _set_alts(self, x):
        if self.arch == "segmented" and self.optimize_alts:
            n = self.segs.n - 1
            self.segs.set_activation_altitudes(
                sgs._alts_from_fractions(x[4:4 + n], *self.alt_bounds))

    def production_J(self, x):
        """The solver's own fitness method, try/except included."""
        if self.arch == "pso_coast":
            return float(pcs.CoastPSOProblem().fitness(x)[0])
        if self.arch == "direct":
            return float(dps.DirectPSOProblem().fitness(x)[0])
        return float(sgs.SegmentedPSOProblem(self.segs, self.optimize_alts,
                                             self.alt_bounds).fitness(x)[0])

    def evaluate(self, x):
        """The fitness body without its try/except, so a wrapper assertion surfaces."""
        x = np.asarray(x, dtype=float)
        if self.arch == "pso_coast":
            dtc, dtr, cs, gp, extras = pcs._unpack_coast_x(x)
            result = pcs.run_pso_coast_trajectory(
                dtc, dtr, cs, gp, cpr_theta_dot=extras.get("cpr_theta_dot"),
                exp_a=extras.get("exp_a"), exp_b=extras.get("exp_b"),
                tan_theta0=extras.get("tan_theta0"), tan_thetaf=extras.get("tan_thetaf"),
                tan_mu=extras.get("tan_mu"))
            return float(pcs.compute_coast_objective(result)), result
        if self.arch == "direct":
            result = dps.run_pso_direct_trajectory(*x)
            return float(dps.compute_direct_objective(result)), result
        self._set_alts(x)
        result = sgs.run_segmented_trajectory(float(x[0]), float(x[1]), float(x[2]),
                                              float(x[3]), self.segs)
        return float(pcs.compute_coast_objective(result)), result

    def objective(self, result):
        if self.arch == "direct":
            return float(dps.compute_direct_objective(result))
        return float(pcs.compute_coast_objective(result))

    def full(self, x):
        if self.arch == "pso_coast":
            return pcs.run_pso_coast_full(x, verbose=False)
        if self.arch == "direct":
            return dps.run_pso_direct_full(x, verbose=False)
        self._set_alts(x)
        return sgs.run_segmented_full(x[:4], self.segs, verbose=False)


# ---------------------------------------------------------------------------
# Measurements
# ---------------------------------------------------------------------------

def _state_report(result):
    from Auxiliary import constants as c
    if result is None or result.get("crashed") or result.get("state_final") is None:
        return {"crashed": True}
    sf = np.asarray(result["state_final"], dtype=float)
    return {"crashed": False, "h_km": (sf[1] - c.R_EARTH) / 1e3, "v_ms": sf[2],
            "gamma_deg": float(np.rad2deg(sf[3])), "m_final_kg": sf[4]}


def _arc1_report(rec, result, arch):
    """State where the first burn ends: the coast start (pso_coast, segmented) or the
    single burn's cutoff (direct)."""
    from Auxiliary import constants as c
    if not rec.arc_ends:
        return None
    t_c = None if arch == "direct" or result is None else result.get("t_arc2_start")
    if t_c:
        t, y = min(rec.arc_ends, key=lambda e: abs(e[0] - t_c))
    else:
        t, y = rec.arc_ends[0]
    return {"t": float(t), "h_km": (y[1] - c.R_EARTH) / 1e3, "v_ms": float(y[2]),
            "gamma_deg": float(np.rad2deg(y[3])), "m_kg": float(y[4])}


def _safe_eval(case, x):
    """(J, m_final) with production's crash mapping, but wrapper faults re-raised."""
    try:
        J, result = case.evaluate(x)
    except (AssertionError, NotImplementedError):
        raise
    except Exception:
        return float(pcs.CRASH_PENALTY), float("nan")
    rep = _state_report(result)
    return J, (float("nan") if rep["crashed"] else float(rep["m_final_kg"]))


def noise_estimates(f):
    """sigma_k from k-th differences, k = 1..4: sigma_k^2 = mean((D^k f)^2) (k!)^2/(2k)!
    (the difference-table estimator behind Moré & Wild's ECnoise)."""
    f = np.asarray(f, dtype=float)
    if not np.all(np.isfinite(f)):
        return {k: float("nan") for k in range(1, 5)}
    out = {}
    for k in range(1, 5):
        d = np.diff(f, n=k)
        g = math.factorial(k) ** 2 / math.factorial(2 * k)
        out[k] = float(np.sqrt(g * np.mean(d ** 2)))
    return out


def _scan_points(case, i, half_span, n):
    """n points spanning 2*half_span around x_i, kept inside the bounds and -- for
    pso_coast's delta_tc -- on the same side of the no-coast threshold as x_i."""
    lo, hi, xi = float(case.lb[i]), float(case.ub[i]), float(case.x[i])
    coast_axis = case.arch in ("pso_coast", "segmented") and i == 0
    note = None
    if coast_axis and xi <= COAST_THRESHOLD:
        pts, note = np.linspace(lo, COAST_THRESHOLD, n), "held below the no-coast threshold"
    else:
        if coast_axis:
            lo = max(lo, COAST_THRESHOLD * 1.0001)
        span = 2.0 * half_span
        start = min(max(xi - half_span, lo), hi - span)
        pts = start + np.linspace(0.0, span, n)
        if start != xi - half_span:
            note = "shifted to stay in bounds"
    return pts, note


def scan(case):
    rng = case.ub - case.lb
    out = []
    for i in range(len(case.x)):
        row = {"coord": i, "x": float(case.x[i]), "range": float(rng[i]), "scales": {}}
        for label, frac in list(SCALES.items()) + [("line", None)]:
            if label == "line":
                pts, note = _scan_points(case, i, LINE_HALF * rng[i], N_LINE)
            else:
                h = frac * rng[i]
                pts, note = _scan_points(case, i, 0.5 * (N_NOISE - 1) * h, N_NOISE)
            Js, ms = [], []
            for p in pts:
                xx = case.x.copy()
                xx[i] = p
                J, m = _safe_eval(case, xx)
                Js.append(J)
                ms.append(m)
            entry = {"points": pts.tolist(), "J": Js, "m_final": ms, "note": note,
                     "h": float(pts[1] - pts[0])}
            if label != "line":
                entry["sigma_J"] = noise_estimates(Js)
                entry["sigma_m_kg"] = noise_estimates(ms)
            row["scales"][label] = entry
        out.append(row)
    return out


def late_gbest_gain(hist):
    if hist is None:
        return None
    gen, gb = hist
    n = len(gb)
    if n < 20:
        return None
    return {"n_generations": int(n),
            "gain_last_10pct": float(gb[int(0.9 * n)] - gb[-1]),
            "gain_last_100": float(gb[max(n - 101, 0)] - gb[-1]),
            "gbest_final": float(gb[-1])}


def measure(case_name, args):
    configure(case_name)
    case = Case(case_name)
    print(f"\n=== {case_name} ({case.arch}) ===   x from {case.x_source}"
          + ("  [BORROWED]" if case.borrowed else ""))
    print("    x =", np.array2string(case.x, precision=10))
    J_prod = case.production_J(case.x)
    walls = []
    case.evaluate(case.x)                                  # unpatched: the true production cost
    for _ in range(args.repeats):
        t0 = time.perf_counter()
        case.evaluate(case.x)
        walls.append(time.perf_counter() - t0)
    out = {"wall_s_production": float(np.median(walls)),"case": case_name, "arch": case.arch, "x": case.x.tolist(),
           "x_source": case.x_source, "borrowed": case.borrowed,
           "lb": case.lb.tolist(), "ub": case.ub.tolist(),
           "J_production": J_prod, "late_gbest_gain": late_gbest_gain(case.history),
           "modes": {}}
    for mode in ("in_rhs", "cycle"):
        with refresh_mode(mode) as rec:
            rec.reset()
            J, result = case.evaluate(case.x)
            at_x = {"J": J, "state": _state_report(result),
                    "arc1_end": _arc1_report(rec, result, case.arch), "stats": rec.stats()}
            if mode == "in_rhs":
                at_x["equals_production"] = (J == J_prod)
                if J != J_prod:
                    raise AssertionError(f"in_rhs J {J!r} != production {J_prod!r}")
            walls = []
            case.evaluate(case.x)                          # warm-up
            for _ in range(args.repeats):
                t0 = time.perf_counter()
                case.evaluate(case.x)
                walls.append(time.perf_counter() - t0)
            at_x["wall_s_median"] = float(np.median(walls))
            at_x["wall_s_all"] = walls
            entry = {"at_x": at_x}
            if not args.no_scan:
                t0 = time.perf_counter()
                entry["scan"] = scan(case)
                entry["scan_wall_s"] = time.perf_counter() - t0
            out["modes"][mode] = entry
        st, s = at_x["state"], at_x["stats"]
        print(f"  {mode:>6}: J {J:.10f}  "
              + ("crashed" if st["crashed"] else
                 f"h {st['h_km']:.3f} km  v {st['v_ms']:.3f} m/s  γ {st['gamma_deg']:+.4f}°  "
                 f"m_f {st['m_final_kg']:.2f} kg")
              + f"  | {1e3 * at_x['wall_s_median']:.1f} ms  nfev {s['nfev_stage2']}  "
                f"updates {s['n_updates']} (trial {s['n_on_trial_points']}, "
                f"look-ahead {s['max_lookahead_s']:.2f} s)")
    # The production switch must fly exactly this script's cycle mode.
    sp.GUIDANCE_REFRESH_MODE = "cycle"
    try:
        J_pc, _ = case.evaluate(case.x)
    finally:
        sp.GUIDANCE_REFRESH_MODE = "in_rhs"
    J_cy = out["modes"]["cycle"]["at_x"]["J"]
    out["J_production_cycle"] = J_pc
    out["production_cycle_equals_cycle"] = (J_pc == J_cy)
    print(f"  production GUIDANCE_REFRESH_MODE='cycle': J {J_pc:.10f}  "
          + ("== this script's cycle mode" if J_pc == J_cy else f"!= {J_cy!r}  MISMATCH"))
    path = Path(args.out) / case_name / f"{case_name}.refresh_ab.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")
    print(f"  -> {path}")


def archive(case_name, mode, args):
    configure(case_name)
    case = Case(case_name)
    from Archive import store
    t0 = time.time()
    with refresh_mode(mode) as rec:
        (time_a, data, thrust, alpha, _t_ign, result, _cor, _cen) = case.full(case.x)
        stats = rec.stats()
    J = case.objective(result)
    name = f"{case_name}__{mode}"
    store.save_run(sp, time_a, data, thrust, alpha, result, J=J, history=None,
                   extra={"decision_vector": case.x.tolist(), "refresh_mode": mode,
                          "x_source": case.x_source, "x_borrowed": case.borrowed,
                          "n_coefficient_updates": stats["n_updates"]},
                   wall_clock=time.time() - t0, name=name, root=Path(args.out) / case_name,
                   source="dev-notes/refresh_ab.py", label=f"{case_name}: {mode} refresh",
                   tags={"section": "dev", "factor": "refresh_ab"}, verbose=False)
    print(f"  archived {name}  J {J:.10f}")


def archive_production(case_name, state, args):
    """The production switch itself, no wrapper: GUIDANCE_REFRESH_MODE off / on."""
    configure(case_name)
    sp.GUIDANCE_REFRESH_MODE = {"off": "in_rhs", "on": "cycle"}[state]
    case = Case(case_name)
    from Archive import store
    t0 = time.time()
    (time_a, data, thrust, alpha, _t_ign, result, _cor, _cen) = case.full(case.x)
    J = case.objective(result)
    name = f"{case_name}__{state}"
    store.save_run(sp, time_a, data, thrust, alpha, result, J=J, history=None,
                   extra={"decision_vector": case.x.tolist(), "x_source": case.x_source,
                          "x_borrowed": case.borrowed},
                   wall_clock=time.time() - t0, name=name, root=Path(args.out) / case_name,
                   source="dev-notes/refresh_ab.py --onoff",
                   label=f"{case_name}: GUIDANCE_REFRESH_MODE={sp.GUIDANCE_REFRESH_MODE}",
                   tags={"section": "dev", "factor": "refresh_mode"}, verbose=False)
    print(f"  archived {name}  J {J:.10f}")


def onoff(names, args):
    me = str(Path(__file__).resolve())
    for name in names:
        for state in ("off", "on"):
            sys.stdout.flush()
            subprocess.run([sys.executable, me, "--case", name, "--archive-production", state,
                            "--out", args.out], check=True)
    fig_root = SRC / "Output_Plots" / "comparisons" / "refresh_onoff"
    rows = []
    for name in names:
        subprocess.run([sys.executable, str(SRC / "run_archive.py"), "compare",
                        f"{Path(args.out) / name}::{name}__off",
                        f"{Path(args.out) / name}::{name}__on",
                        "--labels", "refresh off (in-RHS),refresh on (cycle)",
                        "--out", str(fig_root / name)], check=True,
                       stdout=subprocess.DEVNULL)
        pair = [json.loads((Path(args.out) / name / f"{name}__{s}.json").read_text(
            encoding="utf-8")) for s in ("off", "on")]
        rows.append((name, pair))
    keys = ("J_prime", "insertion_alt_km", "insertion_v_ms", "insertion_fpa_deg",
            "periapsis_km", "apoapsis_km", "prop_remaining_kg")
    print()
    print("=" * 118)
    print(f"  {'case':<19}{'':>4}" + "".join(f"{k:>14}" for k in keys))
    for name, (off, on) in rows:
        for tag, row in (("off", off), ("on", on)):
            print(f"  {name if tag == 'off' else '':<19}{tag:>4}"
                  + "".join(f"{row.get(k, float('nan')):14.4f}" for k in keys))
    print("=" * 118)
    sys.path.insert(0, str(SRC))
    from Plots.results_figures import diag_refresh_noise
    diag_refresh_noise.make(OUT_DEFAULT if Path(args.out) == OUT_DEFAULT else args.out,
                            fig_root)
    print(f"  overlays and noise figures in {fig_root}")


# ---------------------------------------------------------------------------
# Self-test: the stitching, on laws with nothing to refresh
# ---------------------------------------------------------------------------

def selftest(args):
    peg = _load_npz("results_matrix", "peg_baseline")[1]
    ok = True
    for law, extra_x in (("gravity_turn", []), ("linear_tangent", None)):
        configure("peg_baseline", {"GUIDANCE_MODE": law})
        lb, ub = map(np.asarray, pcs._coast_bounds())
        x = np.concatenate([peg[:4], 0.5 * (lb[4:] + ub[4:])]) if extra_x is None else peg[:4]
        case = Case.__new__(Case)
        case.name, case.arch = f"selftest_{law}", "pso_coast"
        J0, _ = case.evaluate(x)
        with refresh_mode("cycle", force_split=2.0) as rec:
            J1, _ = case.evaluate(x)
            n_calls = rec.n_calls
        full0 = case.full(x)
        with refresh_mode("cycle", force_split=2.0):
            full1 = case.full(x)
        d0, d1 = np.asarray(full0[1], dtype=float), np.asarray(full1[1], dtype=float)
        same_grid = np.array_equal(np.asarray(full0[0]), np.asarray(full1[0]))
        rel_J = abs(J1 - J0) / max(abs(J0), 1e-300)
        rel_d = (float(np.max(np.abs(d1 - d0) / np.maximum(np.abs(d0), 1.0)))
                 if same_grid and d0.shape == d1.shape else float("inf"))
        passed = rel_J < 1e-6 and rel_d < 1e-6
        ok &= passed
        print(f"  {law:<15} J {J0:.12f} vs split {J1:.12f}  rel {rel_J:.2e}  "
              f"(solve_ivp calls {n_calls})  full-flight grid identical {same_grid}, "
              f"max rel state diff {rel_d:.2e}  -> {'OK' if passed else 'FAIL'}")
    if not ok:
        raise SystemExit("self-test FAILED")
    print("  self-test passed")


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

def _worst_sigma(scan_rows, scale, key, k=3):
    vals = []
    for r in scan_rows:
        d = r["scales"][scale][key]
        v = d.get(str(k), d.get(k))                      # JSON turns the int keys into str
        if v is not None and v == v:
            vals.append(v)
    return max(vals) if vals else float("nan")


def summary(names, out_root):
    rows = []
    for name in names:
        p = Path(out_root) / name / f"{name}.refresh_ab.json"
        if not p.exists():
            continue
        d = json.loads(p.read_text(encoding="utf-8"))
        a, b = d["modes"]["in_rhs"]["at_x"], d["modes"]["cycle"]["at_x"]
        sa, sb = a["state"], b["state"]
        row = {"case": name, "arch": d["arch"], "borrowed": d["borrowed"],
               "J_in_rhs": a["J"], "J_cycle": b["J"], "dJ": b["J"] - a["J"]}
        for key in ("h_km", "v_ms", "gamma_deg", "m_final_kg"):
            row[f"{key}_in_rhs"] = sa.get(key, float("nan"))
            row[f"{key}_cycle"] = sb.get(key, float("nan"))
            row[f"d_{key}"] = sb.get(key, float("nan")) - sa.get(key, float("nan"))
        row.update({
            "share_on_trial_points": a["stats"]["share_on_trial_points"],
            "max_lookahead_s": a["stats"]["max_lookahead_s"],
            "updates_in_rhs": a["stats"]["n_updates"], "updates_cycle": b["stats"]["n_updates"],
            "ms_production": 1e3 * d["wall_s_production"],
            "ms_in_rhs": 1e3 * a["wall_s_median"], "ms_cycle": 1e3 * b["wall_s_median"],
            "time_ratio": b["wall_s_median"] / a["wall_s_median"],
            "nfev_ratio": b["stats"]["nfev_stage2"] / max(a["stats"]["nfev_stage2"], 1),
            "nfev_in_rhs": a["stats"]["nfev_stage2"], "nfev_cycle": b["stats"]["nfev_stage2"],
            "solve_ivp_in_rhs": a["stats"]["n_solve_ivp"], "solve_ivp_cycle": b["stats"]["n_solve_ivp"],
        })
        for mode in ("in_rhs", "cycle"):
            sc = d["modes"][mode].get("scan")
            for scale in SCALES:
                row[f"sigmaJ_{scale}_{mode}"] = _worst_sigma(sc, scale, "sigma_J") if sc else float("nan")
                row[f"sigmaM_{scale}_{mode}"] = _worst_sigma(sc, scale, "sigma_m_kg") if sc else float("nan")
        g = d.get("late_gbest_gain") or {}
        row["gbest_gain_last_10pct"] = g.get("gain_last_10pct", float("nan"))
        row["gbest_gain_last_100"] = g.get("gain_last_100", float("nan"))
        rows.append(row)
    if not rows:
        print("no results")
        return
    out_root = Path(out_root)
    (out_root / "refresh_ab.json").write_text(json.dumps(rows, indent=1, default=float),
                                              encoding="utf-8")
    with open(out_root / "refresh_ab.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("\n" + "=" * 150)
    print(f"  {'case':<19}{'J in_rhs':>12}{'J cycle':>12}{'Δm_f kg':>10}{'Δh km':>9}{'Δv m/s':>9}"
          f"{'trial %':>9}{'look s':>8}{'ms prod':>8}{'ms cyc':>8}{'nfev×':>7}"
          f"{'σJ rhs':>10}{'σJ cyc':>10}{'σm rhs':>9}{'σm cyc':>9}{'gbest Δ10%':>12}")
    for r in rows:
        print(f"  {r['case'] + ('*' if r['borrowed'] else ''):<19}{r['J_in_rhs']:12.6f}"
              f"{r['J_cycle']:12.6f}{r['d_m_final_kg']:+10.2f}"
              f"{r['d_h_km']:+9.3f}{r['d_v_ms']:+9.3f}{100 * r['share_on_trial_points']:9.1f}"
              f"{r['max_lookahead_s']:8.2f}{r['ms_production']:8.1f}{r['ms_cycle']:8.1f}"
              f"{r['nfev_ratio']:7.2f}{r['sigmaJ_coarse_in_rhs']:10.2e}{r['sigmaJ_coarse_cycle']:10.2e}"
              f"{r['sigmaM_coarse_in_rhs']:9.3f}{r['sigmaM_coarse_cycle']:9.3f}"
              f"{r['gbest_gain_last_10pct']:12.2e}")
    print("=" * 150)
    print("  * borrowed decision vector.  σ: worst coordinate, 3rd-difference estimate, "
          "h = 1e-4 of the bound range (fine scale in the CSV).")


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cases", default=",".join(CASES), help="comma-separated subset")
    ap.add_argument("--case", help="one case, in-process (used by the driver)")
    ap.add_argument("--archive", choices=["in_rhs", "cycle"],
                    help="with --case: fly and archive that mode's full flight only")
    ap.add_argument("--no-scan", action="store_true", help="skip the objective scans")
    ap.add_argument("--repeats", type=int, default=5, help="timing repeats (default 5)")
    ap.add_argument("--out", default=str(OUT_DEFAULT), help="output root")
    ap.add_argument("--selftest", action="store_true", help="stitching self-test only")
    ap.add_argument("--summary-only", action="store_true", help="rebuild the summary table")
    ap.add_argument("--onoff", action="store_true",
                    help="production switch off vs on: archives, overlays, table, figures")
    ap.add_argument("--archive-production", choices=["off", "on"],
                    help="with --case: one production full flight with the switch off/on")
    args = ap.parse_args()

    if args.selftest:
        selftest(args)
        return
    if args.case and args.archive_production:
        archive_production(args.case, args.archive_production, args)
        return
    if args.case:
        if args.archive:
            archive(args.case, args.archive, args)
        else:
            measure(args.case, args)
        return
    names = [s.strip() for s in args.cases.split(",") if s.strip()]
    unknown = set(names) - set(CASES)
    if unknown:
        raise SystemExit("unknown case(s): " + ", ".join(sorted(unknown)))
    if args.onoff:
        onoff(names, args)
        return
    if not args.summary_only:
        failed = []
        me = str(Path(__file__).resolve())
        for name in names:
            base = [sys.executable, me, "--case", name, "--out", args.out]
            jobs = [base + ["--repeats", str(args.repeats)] + (["--no-scan"] if args.no_scan else []),
                    base + ["--archive", "in_rhs"], base + ["--archive", "cycle"]]
            for cmd in jobs:       # one process each: rocket_ascent keeps module globals
                sys.stdout.flush()
                if subprocess.run(cmd).returncode != 0:
                    failed.append(" ".join(cmd[2:5]))
        if failed:
            print("FAILED: " + "; ".join(failed))
    summary(names, args.out)


if __name__ == "__main__":
    main()
