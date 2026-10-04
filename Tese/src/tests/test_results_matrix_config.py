"""
The results-matrix configuration decided case by case on 2026-09-25, before the
production batch (run_results_matrix.build_matrix):

  peg_direct   peg_new ends its own burn; the kick is found by grid + Brent
  pmp_*        polished extremals re-flown from their decision vectors, not swarmed;
               pmp_baseline is the extremal the tracked reference cache holds

and on 2026-10-04 (dev-notes/pmp-references-750x1500-plan-2026-10-04.md):

  pmp_norot        the reference of gt_norot's environment, swarmed until its
                   refined extremal is stored
  --swarm-extremal re-swarms a stored extremal's case, to start a new search for it
"""

import contextlib
import io
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Input_File import simulation_parameters as sim_params
import run_results_matrix as rm
import Simulation.segment_reference as segref

CASES = {c["name"]: c for c in rm.build_matrix()}


def _configure(monkeypatch, case_name, **extra):
    for key, value in {**rm.BASELINE, **CASES[case_name]["overrides"],
                       "EVENTS_PRINT": False, "INTERRUPTS_PRINT": False, **extra}.items():
        assert hasattr(sim_params, key), key
        monkeypatch.setattr(sim_params, key, value)


def test_the_matrix_has_22_cases():
    assert len(CASES) == 22


def test_peg_direct_is_law_terminated_and_grid_searched():
    o = CASES["peg_direct"]["overrides"]
    assert o["DIRECT_LAW_TERMINATED_CUTOFF"] is True
    assert o["DIRECT_OPTIMIZER"] == "grid_brent"
    # gt_direct keeps the swarm-picked burn: a law with no terminal targeting
    assert "DIRECT_LAW_TERMINATED_CUTOFF" not in CASES["gt_direct"]["overrides"]


def test_the_pmp_rows_are_stored_extremals():
    for name in ("pmp_baseline", "pmp_vacuum"):
        ext = CASES[name]["extremal"]
        assert len(ext["x"]) == 7 and ext["seed"] == 3
    assert CASES["pmp_baseline"]["extremal"]["swarm_budget"] == [750, 1500]
    assert CASES["pmp_vacuum"]["extremal"]["swarm_budget"] == [250, 1000]


def test_pmp_norot_flies_gt_norots_environment(monkeypatch):
    """The reference differs from the case it serves in the law alone, so
    sec65_losses matches the two on every environment key; with the rotation off
    both stages are free of pseudo-forces and the target is the circular speed."""
    import Simulation.indirect_pso_solver as ips
    from Auxiliary import constants as c
    ref = dict(CASES["pmp_norot"]["overrides"])
    law = dict(CASES["gt_norot"]["overrides"])
    assert ref.pop("GUIDANCE_MODE") == "indirect_pmp"
    assert law.pop("GUIDANCE_MODE") == "gravity_turn"
    assert ref == law and ref["ENABLE_EARTH_ROTATION"] is False
    _configure(monkeypatch, "pmp_norot")
    r_t = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    assert ips.terminal_speed_target(r_t) == float(np.sqrt(c.MU_EARTH / r_t))
    assert ips._stage2_pseudo_forces() is False


def test_swarm_extremal_drops_the_stored_point_and_only_there():
    case = rm._select_case("pmp_vacuum", swarm_extremal=True)
    assert case["extremal"] is None
    assert rm._select_case("pmp_vacuum")["extremal"]["x"] == rm.PMP_VACUUM_EXTREMAL
    with pytest.raises(SystemExit):
        rm._select_case("pmp_norot", swarm_extremal=True)    # nothing to swarm instead of


def test_a_pmp_case_without_its_extremal_is_searched(monkeypatch):
    """--swarm-extremal reaches the swarm branch of _dispatch: the point flown is the
    search's (stubbed here), and no extremal provenance is written beside it."""
    import Simulation.indirect_pso_solver as ips
    _configure(monkeypatch, "pmp_vacuum")
    monkeypatch.setattr(ips, "run_pso_optimization",
                        lambda verbose=True: (list(rm.PMP_VACUUM_EXTREMAL), 0.0))
    with contextlib.redirect_stdout(io.StringIO()):
        out = rm._dispatch(sim_params, rm._select_case("pmp_vacuum", swarm_extremal=True))
    extra = out[7]
    assert extra["decision_vector"] == rm.PMP_VACUUM_EXTREMAL
    assert "extremal_source" not in extra


def test_pmp_baseline_is_the_extremal_the_reference_cache_holds(monkeypatch):
    """show_ref_track, show_ref_track_apollo and the segmented waypoints follow the
    tracked cache; Section 6.4 must present that same extremal."""
    _configure(monkeypatch, "pmp_baseline")
    cached = np.load(segref._abs_cache_path(), allow_pickle=True)["decision_vector"]
    assert np.array_equal(cached, np.asarray(rm.PMP_BASELINE_EXTREMAL))


def test_pmp_baseline_replays_the_polished_archive(monkeypatch):
    """Re-flown, not searched: J and the delivered mass of the 2026-09-21 polish
    archive (22 261.2 kg propellant remaining), to the last digit."""
    _configure(monkeypatch, "pmp_baseline")
    with contextlib.redirect_stdout(io.StringIO()):
        out = rm._dispatch(sim_params, CASES["pmp_baseline"])
    _t, _d, _thr, _a, result, J, history, extra = out
    assert history is None
    assert J == 0.7598833804841622
    assert float(result["state_final"][4]) == 26161.200231706312
    assert extra["decision_vector"] == rm.PMP_BASELINE_EXTREMAL
    assert extra["extremal_seed"] == 3


def test_the_segmented_rerun_flies_the_flight_the_swarm_scored(monkeypatch):
    """The dense re-run restarts from the altitude-switch root, not from the last t_eval
    point before it (up to 0.5 s early until 2026-09-25): its J is the fitness J.
    The swarm-timed form, which still takes the altitude switch inside a burn."""
    import Simulation.pso_coast_solver as pcs
    import Simulation.segmented_guidance_solver as sgs
    _configure(monkeypatch, "show_seg_fixed_alt", SEGMENTED_LAW_TERMINATED_ARCS=False)
    monkeypatch.setattr(segref, "_run_pmp_reference",
                        lambda *a, **k: (_ for _ in ()).throw(AssertionError("no rebuild")))
    segs = sgs._Segments(*segref.get_pmp_reference(verbose=False))
    x = [189.86662128991458, 78.73201854888482, 75.31523373706635, 1.5437469147727938]
    J_fit = pcs.compute_coast_objective(sgs.run_segmented_trajectory(*x, segs))
    with contextlib.redirect_stdout(io.StringIO()):
        out = sgs.run_segmented_full(x, segs, verbose=False)
    assert abs(pcs.compute_coast_objective(out[5]) - J_fit) <= 1e-9 * abs(J_fit)
