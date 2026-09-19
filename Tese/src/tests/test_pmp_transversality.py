"""
Tests for the transversality condition the indirect_pmp swarm penalises.

The swarm's Stage 2 is burn (D1) - coast (Dc) - burn (D3), with the three durations as
decision variables and only the costates (lambda_r, lambda_v, lambda_gamma). Stationarity of
the burn time in each duration can be written with that reduced Hamiltonian, no mass
costate needed:

    lambda(t_f) . dx_f/dDc = H_coast_end                                   -> = 0
    lambda(t_f) . dx_f/dD3 = H_burn_end                                    -> = -lambda0 < 0
    lambda(t_f) . dx_f/dD1 = H_burn1_end - H_last_burn_start + H_burn_end  -> H_burn1_end = H_last_burn_start

The form flown until 2026-09-13, |H_burn_end + H_coast_end - H_burn_start|, took H at Stage-2
ignition, which no condition involves, and could not be satisfied with the orbit constraints.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Input_File import simulation_parameters as sim_params
import Simulation.indirect_pso_solver as ips

# The logged pmp_prod_v2 swarm point (2026-09-11), used only as a trajectory that flies.
X_SWARM = [-0.002087, -0.938028, 0.995508, 280.11, 78.17, 93.21, 1.547791]


def _result(**overrides):
    r = dict(H_burn_start=-6.0, H_burn1_end=-20.0, H_coast_end=-0.5,
             H_last_burn_start=-26.0, H_burn_end=-32.0, t_cf=300.0)
    r.update(overrides)
    return r


@pytest.fixture
def stationarity(monkeypatch):
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_TRANSVERSALITY", "duration_stationarity")


def test_legacy_form_is_the_ignition_combination(monkeypatch):
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_TRANSVERSALITY", "pontani_eq38")
    assert ips.transversality_residual(_result()) == pytest.approx(abs(-32.0 - 0.5 + 6.0))


def test_interior_conditions_add_up(stationarity):
    assert ips.transversality_residual(_result()) == pytest.approx(0.5 + 6.0)


def test_zero_at_an_extremal(stationarity):
    assert ips.transversality_residual(
        _result(H_coast_end=0.0, H_burn1_end=-26.0)) == 0.0


def test_non_negative_final_hamiltonian_is_penalised(stationarity):
    assert ips.transversality_residual(
        _result(H_coast_end=0.0, H_burn1_end=-26.0, H_burn_end=2.0)) == pytest.approx(2.0)


def test_coast_condition_is_one_sided_at_the_upper_bound(stationarity):
    ub = sim_params.PSO_UB[3]
    base = dict(H_burn1_end=-26.0, t_cf=ub)
    assert ips.transversality_residual(_result(H_coast_end=-0.5, **base)) == 0.0
    assert ips.transversality_residual(_result(H_coast_end=+0.5, **base)) == pytest.approx(0.5)


def test_coast_condition_is_one_sided_at_the_lower_bound(stationarity):
    lb = sim_params.PSO_LB[3]
    base = dict(H_burn1_end=-26.0, t_cf=lb)
    assert ips.transversality_residual(_result(H_coast_end=+0.5, **base)) == 0.0
    assert ips.transversality_residual(_result(H_coast_end=-0.5, **base)) == pytest.approx(0.5)


def test_unweighted_residual_survives_a_zero_weight(stationarity, monkeypatch):
    """The archived diagnostic must not vanish with the penalty it measures."""
    from Auxiliary import constants as c
    r_t = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    res = _result(t_f=600.0, state_final=[0.0, r_t, ips.terminal_speed_target(r_t), 0.0, 2e4])
    nd = ips.transversality_residual_nd(res)
    assert nd == pytest.approx((0.5 + 6.0) / ips.terminal_speed_target(r_t))
    monkeypatch.setattr(sim_params, "PENALTY_W_TRANSVERS", 10.0)
    assert ips._objective_terms(res)['transv'] == pytest.approx(10.0 * nd)
    monkeypatch.setattr(sim_params, "PENALTY_W_TRANSVERS", 0.0)
    assert ips._objective_terms(res)['transv'] == 0.0
    assert ips.transversality_residual_nd(res) == nd


def test_unknown_mode_raises(monkeypatch):
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_TRANSVERSALITY", "eq38")
    with pytest.raises(ValueError):
        ips.transversality_residual(_result())


def _fly(lam, d1, dc, d3, gamma_p):
    T = d1 + d3
    return ips.run_indirect_trajectory(
        lam[0], lam[1], lam[2], dc, 100.0 * T / ips._T_MAX_2, 100.0 * d1 / T, gamma_p)


def test_closed_form_duration_sensitivities_match_finite_differences(monkeypatch):
    """Pins the derivation: lambda(t_f) . dx_f/dD against the reduced-H closed forms."""
    monkeypatch.setattr(sim_params, "ENABLE_EARTH_ROTATION", True)
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "inertial")
    lam = np.array(X_SWARM[:3]) / np.linalg.norm(X_SWARM[:3])
    T = X_SWARM[4] / 100.0 * ips._T_MAX_2
    d1, dc, d3, gp = X_SWARM[5] / 100.0 * T, X_SWARM[3], T - X_SWARM[5] / 100.0 * T, X_SWARM[6]

    nom = _fly(lam, d1, dc, d3, gp)
    assert not nom["crashed"]
    lf = nom["costates_final"]
    closed = {
        "D1": nom["H_burn1_end"] - nom["H_last_burn_start"] + nom["H_burn_end"],
        "Dc": nom["H_coast_end"],
        "D3": nom["H_burn_end"],
    }
    h = 0.01
    for name, dp in (("D1", (h, 0, 0)), ("Dc", (0, h, 0)), ("D3", (0, 0, h))):
        plus = _fly(lam, d1 + dp[0], dc + dp[1], d3 + dp[2], gp)["state_final_propagated"]
        minus = _fly(lam, d1 - dp[0], dc - dp[1], d3 - dp[2], gp)["state_final_propagated"]
        fd = float(lf @ ((np.asarray(plus)[1:4] - np.asarray(minus)[1:4]) / (2 * h)))
        assert fd == pytest.approx(closed[name], abs=1e-4), name
