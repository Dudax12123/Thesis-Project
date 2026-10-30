"""cpr and exp_shooting as segment laws: their swarm-chosen constants ride in the
decision vector, one block per segment, in schedule order (segmented_guidance_solver).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Input_File import simulation_parameters as sim_params
import Simulation.segmented_guidance_solver as sgs
from Simulation.pso_coast_solver import GuidanceState


def _bare_segments(schedule):
    """A _Segments with a schedule and nothing else: the constants need no reference."""
    segs = object.__new__(sgs._Segments)
    segs.schedule = list(schedule)
    segs.n = len(schedule)
    segs.law_params = {}
    segs.target = [None] * segs.n
    return segs


def test_both_laws_are_accepted(monkeypatch):
    monkeypatch.setattr(sim_params, "GUIDANCE_SEGMENTS",
                        [("cpr", 0.0), ("exp_shooting", 50e3)])
    sgs.validate_schedule()


def test_bounds_follow_the_schedule_order():
    lb, ub = sgs._law_param_bounds([("cpr", 0.0), ("exp_shooting", 50e3)])
    assert lb == [np.deg2rad(sim_params.PSO_COAST_CPR_THETA_DOT_LB_DEG),
                  sim_params.PSO_COAST_EXP_A_LB, sim_params.PSO_COAST_EXP_B_LB]
    assert ub == [np.deg2rad(sim_params.PSO_COAST_CPR_THETA_DOT_UB_DEG),
                  sim_params.PSO_COAST_EXP_A_UB, sim_params.PSO_COAST_EXP_B_UB]


def test_other_laws_add_no_variables():
    assert sgs._law_param_bounds([("gravity_turn", 0.0), ("peg_new", 50e3)]) == ([], [])


def test_each_segment_receives_its_own_constants():
    segs = _bare_segments([("cpr", 0.0), ("exp_shooting", 50e3)])
    segs.set_law_params([0.01, 1.1, -0.02])
    assert segs.law_params == {0: {"cpr_theta_dot": 0.01},
                               1: {"exp_a": 1.1, "exp_b": -0.02}}

    gs = GuidanceState()
    segs.apply(gs, 0)
    assert (gs.mode_override, gs.pso_cpr_theta_dot, gs.pso_exp_a) == ("cpr", 0.01, None)
    segs.apply(gs, 1)
    assert (gs.mode_override, gs.pso_cpr_theta_dot, gs.pso_exp_a, gs.pso_exp_b) == \
        ("exp_shooting", None, 1.1, -0.02)


def test_a_schedule_without_them_leaves_the_state_untouched():
    segs = _bare_segments([("gravity_turn", 0.0), ("peg_new", 50e3)])
    segs.set_law_params([])
    gs = GuidanceState()
    segs.apply(gs, 1)
    assert (gs.pso_cpr_theta_dot, gs.pso_exp_a, gs.pso_exp_b) == (None, None, None)
