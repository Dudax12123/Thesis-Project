"""
SEGMENTED_LAW_TERMINATED_ARCS (2026-09-25): the segmented final law, peg_new, ends both
Stage-2 burns on its own t_go -- arc 1 at the PMP reference's coast start, arc 3 at the
orbit -- the cutoff rule of COAST_METHOD="reference_track". The swarm keeps the kick,
the coast and (optionally) the hand-off altitudes: x = [delta_tc, gamma_p] (+ fractions).

Also pinned here, found on the way: the segmented solver never made the fairing
jettison checks at Stage-2 start and ignition that every other PSO architecture makes,
and --smoke now flies a copy of the tracked reference instead of a token one.
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
import Simulation.reference_track_solver as rts
import Simulation.rocket_ascent as ra
import Simulation.segment_reference as segref
import Simulation.segmented_guidance_solver as sgs

CASES = {c["name"]: c for c in rm.build_matrix()}

# The reference's own coast and kick (pmp_reference.npz, the 750x1500 seed-3 extremal)
REFERENCE_X = [1446.8331219788931, 1.5371391567133106]


def _configure(monkeypatch, case_name, **extra):
    for key, value in {**rm.BASELINE, **CASES[case_name]["overrides"],
                       "EVENTS_PRINT": False, "INTERRUPTS_PRINT": False, **extra}.items():
        assert hasattr(sim_params, key), key
        monkeypatch.setattr(sim_params, key, value)
    monkeypatch.setattr(segref, "_run_pmp_reference",
                        lambda *a, **k: pytest.fail("the tracked PMP cache did not load"))


def _segments():
    return sgs._Segments(*segref.get_pmp_reference(verbose=False))


def test_the_default_is_the_swarm_timed_form():
    assert sim_params.SEGMENTED_LAW_TERMINATED_ARCS is False


def test_both_segmented_matrix_cases_let_peg_new_end_its_burns():
    for name in ("show_seg_fixed_alt", "show_seg_opt_alt"):
        o = CASES[name]["overrides"]
        assert o["SEGMENTED_LAW_TERMINATED_ARCS"] is True
        assert o["GUIDANCE_SEGMENTS"][-1][0] == "peg_new"


def test_the_final_law_must_be_peg_new(monkeypatch):
    _configure(monkeypatch, "show_seg_fixed_alt",
               GUIDANCE_SEGMENTS=[("gravity_turn", 0.0), ("apollo", 120e3)])
    with pytest.raises(ValueError, match="peg_new as the final law"):
        sgs.validate_schedule()


def test_a_hand_off_above_the_coast_start_is_refused(monkeypatch):
    _configure(monkeypatch, "show_seg_fixed_alt",
               GUIDANCE_SEGMENTS=[("gravity_turn", 0.0), ("peg_new", 200e3)])
    with pytest.raises(ValueError, match="below the reference's coast start"):
        _segments()


def test_arc_1_aims_at_the_reference_coast_start(monkeypatch):
    """The same waypoint show_ref_track hands peg_new, at the same 10 s freeze."""
    _configure(monkeypatch, "show_seg_fixed_alt")
    segs = _segments()
    cs = segs.coast_start
    with contextlib.redirect_stdout(io.StringIO()):
        plan = rts.load_plan(verbose=False)
    assert [plan["delta_tc"], plan["gamma_p"]] == REFERENCE_X
    assert (cs.r, cs.v, cs.gamma) == (6542696.959761191, 7590.513949994785,
                                      0.04908455916045073)
    assert cs.freeze_threshold == sim_params.APOLLO_FREEZE_THRESHOLD == 10.0
    assert segs.target[-1] is cs                 # the final law's first burn
    assert segs.target_alt[-1] == cs.alt


def test_the_swarm_searches_the_kick_the_coast_and_the_hand_off(monkeypatch):
    _configure(monkeypatch, "show_seg_opt_alt")
    segs = _segments()
    lb, ub = sgs._altitude_bounds(segs)
    assert lb == sim_params.MULTI_GUIDANCE_ALT_LB
    assert ub == 0.98 * segs.coast_start.alt     # below the coast start, not 490 km
    prob = sgs.SegmentedPSOProblem(segs, optimize_alts=True, alt_bounds=(lb, ub))
    assert prob.get_bounds() == ([0.0, 1.50, 0.0], [2000.0, 1.57, 1.0])


def test_peg_new_ends_arc_1_at_the_coast_start(monkeypatch):
    """Flown on the reference's own kick and coast: the gravity turn to 120 km, then
    peg_new to the coast start, cut by its own t_go -- a miss the size of
    show_ref_track's (0.03 km, 0.1 m/s) -- and a last burn of a second or two."""
    _configure(monkeypatch, "show_seg_fixed_alt")
    segs = _segments()
    result = sgs._fly(REFERENCE_X, segs)
    assert not result["crashed"] and result["arc1_by_final_law"]
    y1, cs = result["y_arc1_end"], segs.coast_start
    assert abs(y1[1] - cs.r) < 100.0
    assert abs(y1[2] - cs.v) < 0.5
    assert abs(np.rad2deg(y1[3] - cs.gamma)) < 0.1
    assert result["t_cf"] == pytest.approx(REFERENCE_X[0], abs=1e-9)
    assert 0.0 < result["t_arc3_end"] - result["t_arc3_start"] < 5.0
    assert result["t_f"] - result["t_cf"] == pytest.approx(
        (result["t_arc2_start"] - result["t_ignition"])
        + (result["t_arc3_end"] - result["t_arc3_start"]), abs=1e-9)
    assert pcs.compute_coast_objective(result) == 1.8400310571360348
    assert float(result["state_final"][4]) == 25490.33297846938


def test_the_dense_rerun_flies_the_flight_the_swarm_scored(monkeypatch):
    _configure(monkeypatch, "show_seg_fixed_alt")
    segs = _segments()
    J_fit = pcs.compute_coast_objective(sgs._fly(REFERENCE_X, segs))
    with contextlib.redirect_stdout(io.StringIO()):
        out = sgs.run_segmented_full(REFERENCE_X, segs, verbose=False)
    assert pcs.compute_coast_objective(out[5]) == J_fit


def test_the_swarm_timed_runner_refuses_a_law_terminated_schedule(monkeypatch):
    _configure(monkeypatch, "show_seg_fixed_alt")
    with pytest.raises(ValueError, match="run_segmented_law_terminated"):
        sgs.run_segmented_trajectory(100.0, 78.0, 75.0, REFERENCE_X[1], _segments())


@pytest.mark.parametrize("law_terminated", [True, False])
def test_the_fairing_is_shed_when_staging_below_the_criterion(monkeypatch, law_terminated):
    """At the reference's kick Stage 2 starts below 65 km; the ignition check sheds
    the fairing, where the segmented solver used to carry it to orbit."""
    _configure(monkeypatch, "show_seg_fixed_alt",
               SEGMENTED_LAW_TERMINATED_ARCS=law_terminated)
    segs = _segments()
    x = REFERENCE_X if law_terminated else [100.0, 78.0, 75.0, REFERENCE_X[1]]
    result = sgs._fly(x, segs)
    assert ra.fairing_jettisoned
    assert ra.time_fairing_jettison == result["t_ignition"] > result["t_stage2_start"]


def test_smoke_flies_a_copy_of_the_tracked_reference(monkeypatch, tmp_path):
    assert not any(k.startswith("PMP_REFERENCE_PSO") for k in rm.SMOKE_BUDGET)
    _configure(monkeypatch, "show_seg_fixed_alt", **rm.SMOKE_BUDGET)
    tracked = rm.BASELINE.get("PMP_REFERENCE_CACHE", "Tese/src/Output/pmp_reference.npz")
    monkeypatch.setattr(sim_params, "PMP_REFERENCE_CACHE", str(tmp_path / "smoke.npz"))
    rm._prepare_smoke_reference(sim_params, tracked)
    src = segref._project_root() / tracked
    assert (tmp_path / "smoke.npz").read_bytes() == src.read_bytes()
    assert segref._load_cache(tmp_path / "smoke.npz", segref._reference_input_key()) is not None
