"""
Tests for the direct architecture's grid + Brent optimiser and its law-terminated burn
(DIRECT_OPTIMIZER = "grid_brent", DIRECT_LAW_TERMINATED_CUTOFF).

The defaults must leave the swarm path exactly as every archived direct row was flown:
the two results-matrix champions are re-flown here and reproduce their archived J to
the last digit.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Input_File import simulation_parameters as sim_params
import run_results_matrix as rm
import Guidance.peg_guidance_new as peg_new_mod
import Simulation.direct_pso_solver as dps

# Archived results-matrix champions (Output/results_matrix/*/<case>.npz decision_vector)
# and the J their rows report (<case>.json J_prime), flown at 250x1000. peg_direct's row
# (J 3.061334676353464) predates peg_new's 2026-09-23 realignment with Mahajan & Condon's
# Algorithm 1; until the case is re-flown, its J is the realigned law's at the same x.
# Both rows were flown with the coefficient refresh inside the ODE ("in_rhs"); the matrix
# baseline has flown "cycle" since 2026-09-25, so these tests ask for "in_rhs" by name.
PEG_DIRECT_X = [1.5661366962720822, 91.87527871677494]
PEG_DIRECT_J = 5.948042294131451
GT_DIRECT_X = [1.5480496369878718, 78.91330864548283]
GT_DIRECT_J = 41.13152999339198


def _configure(monkeypatch, case_name, **extra):
    """Apply the harness's baseline and one case's overrides without leaking them."""
    case = next(c for c in rm.build_matrix() if c["name"] == case_name)
    for key, value in {**rm.BASELINE, **case["overrides"], **extra}.items():
        assert hasattr(sim_params, key), key
        monkeypatch.setattr(sim_params, key, value)


# --- the optimiser on functions whose minimum is known -----------------------

def test_grid_then_brent_refines_an_interior_minimum():
    x, f, xs, fs, edge = dps._grid_then_brent(lambda x: (x - 0.3137) ** 2, 0.0, 1.0, 21, 1e-9)
    assert x == pytest.approx(0.3137, abs=1e-7)
    assert len(xs) == len(fs) == 21 and not edge


def test_grid_picks_the_global_basin_not_the_nearest():
    # two basins; the deeper one is narrow and lies away from the middle of the box
    f = lambda x: min((x - 0.2) ** 2 + 0.05, 40 * (x - 0.83) ** 2)
    x, fx, *_ = dps._grid_then_brent(f, 0.0, 1.0, 41, 1e-9)
    assert x == pytest.approx(0.83, abs=1e-6) and fx == pytest.approx(0.0, abs=1e-10)


def test_a_minimum_on_the_box_edge_is_flagged():
    x, _, _, _, edge = dps._grid_then_brent(lambda x: -x, 0.0, 1.0, 11, 1e-9)
    assert edge and x > 0.9


def test_unknown_optimizer_raises(monkeypatch):
    monkeypatch.setattr(sim_params, "DIRECT_OPTIMIZER", "annealing")
    with pytest.raises(ValueError):
        dps.run_direct_optimization(verbose=False)


# --- defaults leave the archived swarm path untouched -------------------------

@pytest.mark.parametrize("case, x, J", [("peg_direct", PEG_DIRECT_X, PEG_DIRECT_J),
                                        ("gt_direct", GT_DIRECT_X, GT_DIRECT_J)])
def test_default_path_reproduces_the_archived_rows(monkeypatch, case, x, J):
    _configure(monkeypatch, case, GUIDANCE_REFRESH_MODE="in_rhs")
    assert not dps.law_terminated()
    assert dps._decision_bounds() == (sim_params.PSO_DIRECT_LB, sim_params.PSO_DIRECT_UB)
    assert dps.compute_direct_objective(dps.run_pso_direct_trajectory(*x)) == J


def test_switch_is_inert_for_a_law_without_its_own_cutoff(monkeypatch):
    _configure(monkeypatch, "gt_direct", DIRECT_LAW_TERMINATED_CUTOFF=True)
    assert not dps.law_terminated()
    assert len(dps._decision_bounds()[0]) == 2
    assert dps.compute_direct_objective(dps.run_pso_direct_trajectory(*GT_DIRECT_X)) == GT_DIRECT_J


# --- the law-terminated burn --------------------------------------------------

def test_peg_new_ends_its_own_burn_outside_the_ode(monkeypatch):
    _configure(monkeypatch, "peg_direct", DIRECT_LAW_TERMINATED_CUTOFF=True)
    assert dps.law_terminated() and dps._decision_bounds() == ([1.50], [1.57])
    calls = []
    real = peg_new_mod.peg_new_major_loop
    monkeypatch.setattr(peg_new_mod, "peg_new_major_loop",
                        lambda *a, **k: calls.append(1) or real(*a, **k))

    res = dps.run_pso_direct_trajectory(PEG_DIRECT_X[0])
    assert not res["crashed"]
    # measured 2026-09-19: peg_new's own cutoff from this kick is ~310.7 s after ignition
    assert 300.0 < res["t_burn"] < 320.0
    # one major loop per 2 s guidance cycle plus the frozen last cycle -- not the
    # thousands an update from inside the ODE right-hand side would make
    cycle = sim_params.PEG_MAJOR_LOOP_RATE
    assert len(calls) <= res["t_burn"] / cycle + 2


def test_dense_rerun_flies_the_same_law_terminated_burn(monkeypatch):
    _configure(monkeypatch, "peg_direct", DIRECT_LAW_TERMINATED_CUTOFF=True,
               DURATION_AFTER_SIMULATION=10.0)
    res = dps.run_pso_direct_trajectory(PEG_DIRECT_X[0])
    out = dps.run_pso_direct_full([PEG_DIRECT_X[0]], verbose=False)
    full = out[5]
    assert full["t_burn"] == pytest.approx(res["t_burn"], abs=1e-9)
    np.testing.assert_allclose(full["state_final"], res["state_final"], rtol=0, atol=1e-6)
    assert np.all(np.diff(out[0]) >= 0)              # time never runs backwards
