"""
The recorded alpha of a segmented flight across its law hand-off (fixed 2026-09-30).

The Stage-2 alpha channel is interpolated from a log the ODE right-hand side appends
on every call, trial points included. At the segmented schedule's altitude hand-off
the ended integration had already probed past the crossing with the old law, and those
entries interleaved with the new law's once the log was sorted: show_seg_fixed_alt's
recorded alpha dipped from ~11 deg to the gravity turn's 0 for single samples over the
9 s after its 120 km hand-off, and its steering loss read 0.49 m/s low. The flight was
never affected -- the log is output-only -- and GuidanceState.discard_logs_after, called
at the crossing, is the fix. test_segmented_law_terminated.py pins that the flight, its
objective and its final mass are unchanged.
"""

import contextlib
import io
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Input_File import simulation_parameters as sim_params
import run_results_matrix as rm
import Simulation.pso_coast_solver as pcs
import Simulation.segment_reference as segref
import Simulation.segmented_guidance_solver as sgs

CASES = {case["name"]: case for case in rm.build_matrix()}

# The reference's own coast and kick, as in test_segmented_law_terminated.py
REFERENCE_X = [1446.8331219788931, 1.5371391567133106]


def _configure(monkeypatch, case_name):
    for key, value in {**rm.BASELINE, **CASES[case_name]["overrides"],
                       "EVENTS_PRINT": False, "INTERRUPTS_PRINT": False}.items():
        assert hasattr(sim_params, key), key
        monkeypatch.setattr(sim_params, key, value)
    monkeypatch.setattr(segref, "_run_pmp_reference",
                        lambda *a, **k: pytest.fail("the tracked PMP cache did not load"))


def _spikes(time, alpha_deg, thrust):
    """Samples more than 2 deg off both neighbours, the same way, with the engine on."""
    on = thrust > 0.01 * np.max(thrust)
    mid = np.arange(1, len(alpha_deg) - 1)
    d1 = alpha_deg[mid] - alpha_deg[mid - 1]
    d2 = alpha_deg[mid] - alpha_deg[mid + 1]
    spike = (on[mid] & on[mid - 1] & on[mid + 1]
             & (np.abs(d1) > 2.0) & (np.abs(d2) > 2.0) & (np.sign(d1) == np.sign(d2)))
    return time[mid[spike]]


def test_discard_logs_after_trims_both_logs_in_step():
    gs = pcs.GuidanceState()
    gs.time_log[:] = [1.0, 3.0, 2.0, 4.0, 2.5]
    gs.alpha_log[:] = [0.1, 0.3, 0.2, 0.4, 0.25]
    gs.tgo_time_log[:] = [1.0, 5.0, 2.0]
    gs.tgo_log[:] = [10.0, 50.0, 20.0]
    alpha_log = gs.alpha_log
    gs.discard_logs_after(2.5)
    assert gs.time_log == [1.0, 2.0, 2.5]
    assert gs.alpha_log == [0.1, 0.2, 0.25]
    assert gs.tgo_time_log == [1.0, 2.0]
    assert gs.tgo_log == [10.0, 20.0]
    assert gs.alpha_log is alpha_log             # trimmed in place


def test_the_recorded_alpha_has_no_old_law_samples_after_the_hand_off(monkeypatch):
    _configure(monkeypatch, "show_seg_fixed_alt")
    segs = sgs._Segments(*segref.get_pmp_reference(verbose=False))
    with contextlib.redirect_stdout(io.StringIO()):
        t, d, th, al, _, _res, _, _ = sgs.run_segmented_full(REFERENCE_X, segs,
                                                            verbose=False)
    t, d, th, al = (np.asarray(v, dtype=float) for v in (t, d, th, al))
    t_switch = float(t[np.argmax(d[1] - c.R_EARTH >= 120e3)])
    assert 150.0 < t_switch < 400.0              # the hand-off is in the first burn
    assert _spikes(t, np.rad2deg(al), th).size == 0
