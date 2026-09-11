"""Tests for the open-loop tangent laws flown under pso_coast (2026-09-11).

The bibliographic form: tan(theta) linear (or bilinear) in time, constants fixed
by an outer solve -- here the swarm -- over the span from the first Stage-2
ignition to the planned final cutoff, continuous through the coast, pitch from
the local horizontal (theta = alpha + gamma, Chapter 4's frame).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Auxiliary import constants as c                       # noqa: E402
from Auxiliary import rocket_specs as r                    # noqa: E402
from Input_File import simulation_parameters as sim_params  # noqa: E402
import Guidance.linear_tangent_steering as lts             # noqa: E402
import Guidance.bilinear_tangent_steering as bts           # noqa: E402
from Simulation import pso_coast_solver as pcs             # noqa: E402

TH0 = np.deg2rad(26.0)
THF = np.deg2rad(-12.0)
T0, TF = 150.0, 700.0
STATE = np.array([150e3, c.R_EARTH + 120e3, 3100.0, np.deg2rad(30.0), 90000.0])


# ---------------------------------------------------------------------------
# The laws
# ---------------------------------------------------------------------------

class TestLinearForm:

    def test_endpoints(self):
        assert lts.open_loop_tan_pitch(0.0, TH0, THF) == pytest.approx(TH0)
        assert lts.open_loop_tan_pitch(1.0, TH0, THF) == pytest.approx(THF)

    def test_tan_is_linear_in_time(self):
        s = np.linspace(0.0, 1.0, 11)
        y = np.tan([lts.open_loop_tan_pitch(x, TH0, THF) for x in s])
        assert np.allclose(np.diff(y, 2), 0.0, atol=1e-12)

    def test_held_outside_the_span(self):
        assert lts.open_loop_tan_pitch(-0.3, TH0, THF) == pytest.approx(TH0)
        assert lts.open_loop_tan_pitch(1.2, TH0, THF) == pytest.approx(THF)

    def test_equals_chapter_form_in_t_go(self):
        """tan(alpha + gamma) = a*t_go + b, t_go = tf - t (Chapter 4, Eq. lts)."""
        T = TF - T0
        a, b = lts.open_loop_coefficients(T, TH0, THF)
        for tau in (0.0, 100.0, 275.0, T):
            pitch = lts.open_loop_tan_pitch(1.0 - tau / T, TH0, THF)
            assert np.tan(pitch) == pytest.approx(a * tau + b)

    def test_alpha_is_pitch_minus_gamma(self):
        t = 0.5 * (T0 + TF)
        alpha = lts.open_loop_alpha(t, T0, TF, STATE[3], TH0, THF)
        assert alpha == pytest.approx(lts.open_loop_tan_pitch(0.5, TH0, THF) - STATE[3])


class TestBilinearForm:

    def test_mid_span_half_is_the_linear_law(self):
        for s in np.linspace(0.0, 1.0, 7):
            assert bts.open_loop_tan_pitch(s, TH0, THF, 0.5) == pytest.approx(
                lts.open_loop_tan_pitch(s, TH0, THF))

    @pytest.mark.parametrize("mu", [0.1, 0.3, 0.7, 0.9])
    def test_endpoints(self, mu):
        assert bts.open_loop_tan_pitch(0.0, TH0, THF, mu) == pytest.approx(TH0)
        assert bts.open_loop_tan_pitch(1.0, TH0, THF, mu) == pytest.approx(THF)

    @pytest.mark.parametrize("mu", [0.1, 0.25, 0.8, 0.9])
    def test_mu_is_the_mid_span_fraction(self, mu):
        y = np.tan(bts.open_loop_tan_pitch(0.5, TH0, THF, mu))
        frac = (y - np.tan(TH0)) / (np.tan(THF) - np.tan(TH0))
        assert frac == pytest.approx(mu)

    def test_denominator_never_vanishes_inside_bounds(self):
        for mu in np.linspace(sim_params.PSO_COAST_BTS_MID_LB,
                              sim_params.PSO_COAST_BTS_MID_UB, 33):
            k = bts.midpoint_to_kappa(mu)
            assert np.all(1.0 + k * np.linspace(0.0, 1.0, 101) > 0.0)

    @pytest.mark.parametrize("mu", [0.2, 0.5, 0.85])
    def test_equals_chapter_form_in_t_go(self, mu):
        """tan(alpha + gamma) = (c1 t_go + c2)/(c1' t_go + c2') (Chapter 4, Eq. bts)."""
        T = TF - T0
        c1, c2, c1p, c2p = bts.open_loop_coefficients(T, TH0, THF, mu)
        assert c2p == 1.0
        for tau in (0.0, 80.0, 300.0, T):
            pitch = bts.open_loop_tan_pitch(1.0 - tau / T, TH0, THF, mu)
            assert np.tan(pitch) == pytest.approx((c1 * tau + c2) / (c1p * tau + c2p))


# ---------------------------------------------------------------------------
# The pso_coast dispatcher
# ---------------------------------------------------------------------------

def _gs(mode, mu=None):
    gs = pcs.GuidanceState()
    gs.mode_override = mode
    gs.pso_tan_theta0, gs.pso_tan_thetaf, gs.pso_tan_mu = TH0, THF, mu
    gs.tan_t0, gs.tan_tf = T0, TF
    return gs


class TestDispatcher:

    def test_linear_flies_the_law_and_consults_no_t_go(self):
        gs = _gs("linear_tangent")
        t = 300.0
        alpha = pcs._compute_alpha_stage2(t, STATE, r.F_THRUST_2, r.ISP_2, gs)
        assert alpha == pytest.approx(lts.open_loop_alpha(t, T0, TF, STATE[3], TH0, THF))
        assert gs.tgo_log == []

    def test_bilinear_flies_the_law(self):
        gs = _gs("bilinear_tangent", mu=0.7)
        t = 420.0
        alpha = pcs._compute_alpha_stage2(t, STATE, r.F_THRUST_2, r.ISP_2, gs)
        assert alpha == pytest.approx(
            bts.open_loop_alpha(t, T0, TF, STATE[3], TH0, THF, 0.7))

    def test_continuous_through_the_coast(self):
        """restart_for_new_burn at the second ignition must NOT re-epoch the law:
        the command after the coast is the one at absolute normalised time."""
        gs = _gs("linear_tangent")
        pcs._compute_alpha_stage2(160.0, STATE, r.F_THRUST_2, r.ISP_2, gs)   # arc 1
        gs.restart_for_new_burn()                                            # coast ends
        t3 = 610.0
        alpha = pcs._compute_alpha_stage2(t3, STATE, r.F_THRUST_2, r.ISP_2, gs)
        sigma_abs = (t3 - T0) / (TF - T0)
        assert alpha == pytest.approx(
            lts.open_loop_tan_pitch(sigma_abs, TH0, THF) - STATE[3])
        assert gs.pso_tan_theta0 == TH0 and gs.tan_t0 == T0     # constants survive

    def test_fallback_without_constants_is_the_degenerate_legacy_law(self):
        """Architectures that supply no constants keep the closed-loop form. Its
        initial condition matches the current flight-path angle, so at
        engagement it commands alpha = 0 exactly -- the property the audit
        found, pinned here so it stays visible."""
        gs = pcs.GuidanceState()
        gs.mode_override = "linear_tangent"
        alpha = pcs._compute_alpha_stage2(200.0, STATE, r.F_THRUST_2, r.ISP_2, gs)
        assert alpha == pytest.approx(0.0, abs=1e-12)


class TestDecisionVector:

    def test_bounds_and_unpack_linear(self, monkeypatch):
        monkeypatch.setattr(sim_params, "GUIDANCE_MODE", "linear_tangent")
        lb, ub = pcs._coast_bounds()
        assert len(lb) == len(ub) == 6
        assert lb[4] == pytest.approx(np.deg2rad(sim_params.PSO_COAST_TAN_THETA0_LB_DEG))
        assert ub[5] == pytest.approx(np.deg2rad(sim_params.PSO_COAST_TAN_THETAF_UB_DEG))
        *_, extras = pcs._unpack_coast_x([300.0, 80.0, 50.0, 1.55, TH0, THF])
        assert extras == {"tan_theta0": TH0, "tan_thetaf": THF}

    def test_bounds_and_unpack_bilinear(self, monkeypatch):
        monkeypatch.setattr(sim_params, "GUIDANCE_MODE", "bilinear_tangent")
        lb, ub = pcs._coast_bounds()
        assert len(lb) == len(ub) == 7
        assert (lb[6], ub[6]) == (sim_params.PSO_COAST_BTS_MID_LB,
                                  sim_params.PSO_COAST_BTS_MID_UB)
        *_, extras = pcs._unpack_coast_x([300.0, 80.0, 50.0, 1.55, TH0, THF, 0.6])
        assert extras == {"tan_theta0": TH0, "tan_thetaf": THF, "tan_mu": 0.6}

    def test_other_modes_unchanged(self, monkeypatch):
        monkeypatch.setattr(sim_params, "GUIDANCE_MODE", "gravity_turn")
        lb, _ = pcs._coast_bounds()
        assert len(lb) == 4
