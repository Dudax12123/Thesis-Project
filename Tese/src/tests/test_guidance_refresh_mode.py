"""
GUIDANCE_REFRESH_MODE (2026-09-24): where the closed-loop laws refresh their coefficients
under the PSO solvers (pso_coast_solver.solve_guided_arc).

"in_rhs" (default) is the historic behaviour and must stay plain solve_ivp. "cycle"
refreshes once per guidance cycle on the accepted state. Its values are pinned to the ones
dev-notes/refresh_ab.py measured: that script emulates the cycle refresh around the in_rhs
path, and the production switch reproduced it bit for bit on all seven affected cases.
The decision vectors are the ones it used (stale points, not optima).
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Input_File import simulation_parameters as sim_params
import run_results_matrix as rm
import Simulation.pso_coast_solver as pcs
import Simulation.direct_pso_solver as dps
import Simulation.segmented_guidance_solver as sgs
import Simulation.segment_reference as segref

PEG_BASELINE_X = [189.86662128991458, 78.73201854888482, 75.31523373706635, 1.5437469147727938]
PEG_DIRECT_X = [1.5661366962720822, 91.87527871677494]
SHOW_APOLLO_X = [0.0, 91.60758745728216, 94.00948186783256, 1.5668927372031136]

# (case, x, J with "in_rhs", J with "cycle") -- dev-notes/refresh_ab.py, 2026-09-24
MEASURED = [
    ("peg_baseline", PEG_BASELINE_X, 196.68278412119147, 193.06019613639359),
    ("peg_direct", PEG_DIRECT_X, 5.948042294131451, 4.753001936532169),
    ("show_apollo", SHOW_APOLLO_X, 0.9651918327810947, 1.2778648569777045),
    ("show_seg_fixed_alt", PEG_BASELINE_X, 194.1997792989455, 190.70919485959266),
]


def _configure(monkeypatch, case_name, **extra):
    case = next(c for c in rm.build_matrix() if c["name"] == case_name)
    for key, value in {**rm.BASELINE, **case["overrides"], "EVENTS_PRINT": False,
                       "INTERRUPTS_PRINT": False, **extra}.items():
        assert hasattr(sim_params, key), key
        monkeypatch.setattr(sim_params, key, value)


def _J(monkeypatch, x):
    """The solver's fitness body (without its try/except) for the configured case."""
    if sim_params.MULTI_GUIDANCE_ENABLED:
        monkeypatch.setattr(segref, "_run_pmp_reference",
                            lambda *a, **k: pytest.fail("the tracked PMP cache did not load"))
        segs = sgs._Segments(*segref.get_pmp_reference(verbose=False))
        result = sgs.run_segmented_trajectory(*x, segs)
        return pcs.compute_coast_objective(result)
    if sim_params.COAST_METHOD == "direct":
        return dps.compute_direct_objective(dps.run_pso_direct_trajectory(*x))
    return pcs.compute_coast_objective(pcs.run_pso_coast_trajectory(*x))


def test_the_default_is_the_historic_in_rhs_refresh():
    assert sim_params.GUIDANCE_REFRESH_MODE == "in_rhs"


def test_an_unknown_mode_is_refused(monkeypatch):
    monkeypatch.setattr(sim_params, "GUIDANCE_REFRESH_MODE", "sometimes")
    with pytest.raises(ValueError, match="GUIDANCE_REFRESH_MODE"):
        pcs.solve_guided_arc(lambda t, y: -y, pcs.GuidanceState(), (0.0, 1.0), [1.0])


@pytest.mark.parametrize("case, x, J_in_rhs, J_cycle", MEASURED,
                         ids=[m[0] for m in MEASURED])
def test_both_modes_reproduce_the_measurement(monkeypatch, case, x, J_in_rhs, J_cycle):
    _configure(monkeypatch, case)
    assert _J(monkeypatch, x) == J_in_rhs
    monkeypatch.setattr(sim_params, "GUIDANCE_REFRESH_MODE", "cycle")
    assert _J(monkeypatch, x) == J_cycle


@pytest.mark.parametrize("case, x", [("peg_direct", PEG_DIRECT_X),
                                     ("show_apollo", SHOW_APOLLO_X)])
def test_cycle_mode_updates_only_on_accepted_cycle_boundaries(monkeypatch, case, x):
    """Every coefficient update (refresh, freeze, init) happens outside the held cycle,
    and every refresh on the 2 s grid from the arc's guidance start."""
    _configure(monkeypatch, case, GUIDANCE_REFRESH_MODE="cycle")
    real = pcs._compute_alpha_stage2
    updates = []

    def recording(t, state, F_T, Isp, gs):
        before = (gs.guidance_phase_active, gs.last_guidance_update_time,
                  gs.peg_new_frozen, gs.apollo_coefficients_frozen,
                  id(gs.guidance_coefficients), gs.peg_new_t_epoch)
        hold = gs.refresh_hold
        alpha = real(t, state, F_T, Isp, gs)
        after = (gs.guidance_phase_active, gs.last_guidance_update_time,
                 gs.peg_new_frozen, gs.apollo_coefficients_frozen,
                 id(gs.guidance_coefficients), gs.peg_new_t_epoch)
        if after != before:
            updates.append((t, hold, before[0], after[1], gs.time_guidance_start))
        return alpha

    monkeypatch.setattr(pcs, "_compute_alpha_stage2", recording)
    _J(monkeypatch, x)
    assert len(updates) > 50
    assert not any(hold for _, hold, *_ in updates)
    period = (sim_params.PEG_MAJOR_LOOP_RATE if sim_params.GUIDANCE_MODE == "peg_new"
              else sim_params.GUIDANCE_UPDATE_RATE)
    for t, _, was_active, last, t_start in updates:
        if was_active and last == t:                      # a refresh, not init/freeze
            k = (t - t_start) / period
            assert abs(k - round(k)) < 1e-6, (t, k)


def test_laws_with_nothing_to_refresh_are_not_split(monkeypatch):
    """gravity_turn under pso_coast: the switch leaves the flight bit for bit alone."""
    _configure(monkeypatch, "peg_baseline", GUIDANCE_MODE="gravity_turn")
    J_in_rhs = _J(monkeypatch, PEG_BASELINE_X)
    monkeypatch.setattr(sim_params, "GUIDANCE_REFRESH_MODE", "cycle")
    assert _J(monkeypatch, PEG_BASELINE_X) == J_in_rhs


def test_a_split_arc_returns_what_its_callers_read(monkeypatch):
    """t_eval sampled on the requested grid, a terminal event ending the arc, and the
    event lists shaped as solve_ivp's."""
    _configure(monkeypatch, "peg_direct", GUIDANCE_REFRESH_MODE="cycle")
    gs = pcs.GuidanceState()
    gs.guidance_phase_active, gs.last_guidance_update_time = True, 0.0
    monkeypatch.setattr(pcs, "_compute_alpha_stage2", lambda t, y, F, I, g: 0.0)

    def fall(t, y):                                        # y = height, drops 1 m/s
        return np.array([-1.0])
    def ground(t, y):
        return y[0]
    ground.terminal, ground.direction = True, -1

    grid = np.arange(0.0, 20.0, 0.5)
    sol = pcs.solve_guided_arc(fall, gs, (0.0, 20.0), [7.3], t_eval=grid, events=ground,
                               rtol=1e-9, atol=1e-9)
    assert sol.status == 1
    assert sol.t_events[0] == pytest.approx([7.3]) and sol.y_events[0].shape == (1, 1)
    np.testing.assert_array_equal(sol.t, grid[grid <= 7.3])
    np.testing.assert_allclose(sol.y[0], 7.3 - sol.t, atol=1e-9)
