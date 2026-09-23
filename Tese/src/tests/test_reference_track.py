"""
COAST_METHOD = "reference_track": peg_new flies the PMP reference's plan, no optimiser
(Simulation/reference_track_solver.py, results-matrix case show_ref_track, 2026-09-23).

The plan is read from the reference cache, which stores the reference's decision
vector beside its trajectory since the same day; the flight from the tracked cache is
pinned here to the numbers dev-notes/arc1_reference_track.py measured.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Input_File import simulation_parameters as sim_params
from Archive import run_record
import run_results_matrix as rm
import Simulation.pso_coast_solver as pcs
import Simulation.reference_track_solver as rts
import Simulation.segment_reference as segref

# The tracked pmp_reference.npz: the 750x1500 seed-3 half-step extremal (bceab71),
# re-seeded with its decision vector on 2026-09-23.
REFERENCE_X = [-8.164825684552658e-06, -0.00595717532331387, -0.9999822558403237,
               1446.8331219788931, 75.9779861533329, 99.99030152462818,
               1.5371391567133106]
# J' of show_ref_track flown from it (pso_coast's objective).
SHOW_REF_TRACK_J = 1.8680088287750078


def _configure(monkeypatch, case_name, **extra):
    """Apply the harness's baseline and one case's overrides without leaking them."""
    case = next(c for c in rm.build_matrix() if c["name"] == case_name)
    for key, value in {**rm.BASELINE, **case["overrides"], **extra}.items():
        assert hasattr(sim_params, key), key
        monkeypatch.setattr(sim_params, key, value)


# --- the plan -----------------------------------------------------------------

def test_the_waypoint_instant_is_timed_as_the_reference_times_its_first_burn():
    plan = rts.plan_from_reference(np.zeros(1), np.zeros((6, 1)), REFERENCE_X)
    # indirect_pso_solver.run_indirect_full, term for term
    t_burn = (REFERENCE_X[4] / 100.0) * pcs._T_MAX_2
    assert plan["arc1"] == (REFERENCE_X[5] / 100.0) * t_burn
    assert plan["arc1"] + plan["arc3"] == pytest.approx(t_burn, rel=1e-15)
    assert plan["gamma_p"] == REFERENCE_X[6] and plan["delta_tc"] == REFERENCE_X[3]
    assert plan["data"].shape == (5, 1)       # latitude row dropped


def test_a_decision_vector_of_another_layout_is_refused():
    with pytest.raises(ValueError, match="7-element"):
        rts.plan_from_reference(np.zeros(1), np.zeros((5, 1)), REFERENCE_X[3:])


def test_a_duplicated_boundary_stamp_returns_the_later_sample():
    t = np.array([0.0, 1.0, 1.0, 2.0])
    d = np.vstack([np.arange(4.0)] * 5)
    plan = {"time": t, "data": d}
    assert rts.reference_state_at(plan, 1.0)[0] == 2.0
    assert rts.reference_state_at(plan, 1.0 + 1e-9)[0] == 2.0
    with pytest.raises(ValueError, match="no sample"):
        rts.reference_state_at(plan, 1.5)


def test_only_peg_new_is_accepted(monkeypatch):
    _configure(monkeypatch, "show_ref_track", GUIDANCE_MODE="apollo")
    with pytest.raises(ValueError, match="peg_new only"):
        rts.run_reference_track(verbose=False)


# --- the cache carries the plan ----------------------------------------------

@pytest.fixture
def cache_in_tmp(tmp_path, monkeypatch):
    monkeypatch.setattr(sim_params, "PMP_REFERENCE_CACHE", str(tmp_path / "ref.npz"))
    monkeypatch.setattr(sim_params, "PMP_REFERENCE_USE_CACHE", True)
    monkeypatch.setattr(sim_params, "PMP_REFERENCE_FORCE_RERUN", False)
    return tmp_path


def _archive(folder, with_x=True):
    t = np.linspace(0.0, 600.0, 50)
    data = np.vstack([t, 6.45e6 + t, 3000.0 + t, 0.3 - 1e-4 * t, 1e5 - t, 0.5 + 0 * t])
    arrays = dict(time=t, data=data, alpha=0.01 * t, thrust=0 * t)
    if with_x:
        arrays["decision_vector"] = np.asarray(REFERENCE_X)
    npz = folder / "pmp_baseline.npz"
    np.savez(npz, **arrays)
    return npz, t


def test_a_seeded_cache_returns_the_archives_decision_vector(cache_in_tmp, monkeypatch):
    monkeypatch.setattr(segref, "_run_pmp_reference",
                        lambda verbose: pytest.fail("the loader tried to REBUILD"))
    npz, t = _archive(cache_in_tmp)
    segref.cache_from_archive(npz, verbose=False, allow_other_search=True)
    t2, d2, x, source = segref.get_pmp_reference_plan(verbose=False)
    np.testing.assert_array_equal(t2, t)
    np.testing.assert_array_equal(x, REFERENCE_X)
    assert json.loads(source)["archive"].endswith("pmp_baseline.npz")


def test_a_seeded_cache_without_the_plan_is_not_silently_rebuilt(cache_in_tmp, monkeypatch):
    monkeypatch.setattr(segref, "_run_pmp_reference",
                        lambda verbose: pytest.fail("a rebuild would discard the archive"))
    npz, _t = _archive(cache_in_tmp, with_x=False)
    segref.cache_from_archive(npz, verbose=False, allow_other_search=True)
    with pytest.raises(RuntimeError, match="cache_from_archive"):
        segref.get_pmp_reference_plan(verbose=False)


def test_a_swarm_built_cache_without_the_plan_is_rebuilt_once(cache_in_tmp, monkeypatch):
    t = np.linspace(0.0, 10.0, 5)
    d, a = np.zeros((5, 5)), np.zeros(5)
    segref._save_cache(segref._abs_cache_path(), segref._reference_input_key(), t, d, a)
    calls = []

    def _rebuild(verbose):
        calls.append(1)
        return t, d, a, np.asarray(REFERENCE_X)
    monkeypatch.setattr(segref, "_run_pmp_reference", _rebuild)
    _t, _d, x, source = segref.get_pmp_reference_plan(verbose=False)
    np.testing.assert_array_equal(x, REFERENCE_X)
    assert source is None and len(calls) == 1
    segref.get_pmp_reference_plan(verbose=False)          # now cached
    assert len(calls) == 1


# --- the matrix case ------------------------------------------------------------

def test_the_case_is_its_own_architecture_in_section_6_7():
    case = next(c for c in rm.build_matrix() if c["name"] == "show_ref_track")
    assert case["section"] == "6.7"
    assert case["overrides"]["COAST_METHOD"] == "reference_track"


def test_the_case_name_leaves_the_6_2_and_6_3_filter_exact():
    """`--only gt_,peg_` is documented as exactly the ten cases of 6.2 and 6.3."""
    wanted = ("gt_", "peg_")
    picked = [c["name"] for c in rm.build_matrix() if any(s in c["name"] for s in wanted)]
    assert len(picked) == 10 and "show_ref_track" not in picked


def test_the_tracked_cache_holds_the_reference_plan(monkeypatch):
    _configure(monkeypatch, "show_ref_track")
    monkeypatch.setattr(segref, "_run_pmp_reference",
                        lambda verbose: pytest.fail("the tracked cache did not load"))
    _t, _d, x, _source = segref.get_pmp_reference_plan(verbose=False)
    np.testing.assert_array_equal(x, REFERENCE_X)


def test_the_flight_tracks_the_reference_and_reproduces_its_measurement(monkeypatch):
    _configure(monkeypatch, "show_ref_track")
    monkeypatch.setattr(segref, "_run_pmp_reference",
                        lambda verbose: pytest.fail("the tracked cache did not load"))
    assert run_record.architecture(sim_params) == "reference_track"
    time_a, data, _th, _al, _ti, result, _co, _ce = rts.run_reference_track(verbose=False)
    info = rts.LAST_REFERENCE_TRACK
    assert not result["crashed"]
    # Stage 1 is the reference's own, to the bit
    assert all(v == 0.0 for v in info["stage1_diffs"].values())
    # arc 1 reaches the reference's coast-start state (measured 2026-09-23:
    # +0.03 km, -0.11 m/s, +0.045 deg), ending within 0.1 s of the reference
    miss = np.asarray(info["y_arc1_end"]) - np.asarray(info["waypoint"])
    assert abs(miss[1]) < 100.0 and abs(miss[2]) < 0.5 and abs(np.rad2deg(miss[3])) < 0.1
    assert abs(info["t_arc1_end"] - info["t_reference_arc1_end"]) < 0.1
    # the coast is the reference's length, and arc 3 inserts near the target
    assert info["t_arc3_start"] - info["t_arc1_end"] == pytest.approx(REFERENCE_X[3], abs=1e-9)
    h_ins = (result["state_final"][1] - c.R_EARTH) / 1e3
    assert abs(h_ins - sim_params.TARGET_ORBITAL_ALTITUDE / 1e3) < 10.0
    assert pcs.compute_coast_objective(result) == SHOW_REF_TRACK_J
    extra = rts.archive_extra()
    assert extra["decision_vector"] == REFERENCE_X
    assert len(extra["realised_schedule"]) == 4
