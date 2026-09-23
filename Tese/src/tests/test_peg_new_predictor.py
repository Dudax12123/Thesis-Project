"""peg_new against Algorithm 1 of Mahajan & Condon (AAS 25-844), realigned 2026-09-23.

- S1 is eq 69, S0·τ − c·t_go²/2, the closed form of ∫∫(T/m)·s (eq 55); it was S0·t_go.
- The gravity integrals come from quadrature along the predicted powered trajectory
  (step 16), and the corrector adds the predicted velocity miss to v_go until it
  converges (steps 18–20). Before, v_G was a two-point average of g_r that drifted
  to its burnout value and r_G was ½·ḡ·t_go², which from the state below commanded a
  20° pitch-down where the indirect-PMP optimum climbs.

The ignition state and target are the 750x1500 seed-3 PMP reference's second-stage
ignition and coast start, written out as numbers so the test does not depend on the
(gitignored) archive.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.integrate import quad, solve_ivp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Auxiliary import constants as c            # noqa: E402
from Auxiliary import rocket_specs as r         # noqa: E402
import Guidance.peg_guidance_new as pn          # noqa: E402

MU = c.MU_EARTH
VE = r.ISP_2 * c.G_0
F_T = r.F_THRUST_2

IGNITION = np.array([0.0, c.R_EARTH + 68836.674, 3381.710688, math.radians(13.253214), 96570.0])
WAYPOINT_R = c.R_EARTH + 164697.0
WAYPOINT_V, WAYPOINT_G = 7590.514, math.radians(2.8123)
WAYPOINT = dict(v_theta_T=WAYPOINT_V * math.cos(WAYPOINT_G), v_r_T=WAYPOINT_V * math.sin(WAYPOINT_G))
ORBIT_R = c.R_EARTH + 500e3


def _fly(state, out):
    """The planar model the law predicts with, under the returned constants, flown tightly."""
    vgo_r, vgo_th, L0, t_go, t_lam, lam = out
    m0, mdot = state[4], F_T / VE

    def rhs(t, y):
        ur, ut = vgo_r / L0 + lam * (t - t_lam), vgo_th / L0
        a = F_T / (m0 - mdot * t) / math.hypot(ur, ut)
        g_r = -MU / y[0] ** 2 + y[2] ** 2 / y[0]
        return [y[1], a * ur + g_r, a * ut - y[1] * y[2] / y[0], g_r, y[3]]

    y0 = [state[1], state[2] * math.sin(state[3]), state[2] * math.cos(state[3]), 0.0, 0.0]
    return solve_ivp(rhs, (0.0, t_go), y0, rtol=1e-11, atol=1e-9).y[:, -1]


def test_thrust_integrals_are_the_constant_thrust_quadratures():
    tau, t_go = 352.96, 257.33
    accel = lambda t: VE / (tau - t)
    L0 = VE * math.log(tau / (tau - t_go))
    S0, L1, S1, t_lam = pn.compute_thrust_integrals(L0, t_go, tau, VE)
    assert L1 == pytest.approx(quad(lambda t: accel(t) * t, 0, t_go)[0], rel=1e-10)
    assert S0 == pytest.approx(quad(lambda s: (t_go - s) * accel(s), 0, t_go)[0], rel=1e-10)
    assert S1 == pytest.approx(quad(lambda s: (t_go - s) * s * accel(s), 0, t_go)[0], rel=1e-10)
    assert t_lam == pytest.approx(L1 / L0)


def test_predictor_integrates_the_planar_model_and_its_gravity():
    out = pn.peg_new_major_loop(IGNITION, WAYPOINT_R, MU, VE, F_T, **WAYPOINT)
    vgo_r, vgo_th, L0, t_go, t_lam, lam = out
    v_r, v_th, rG = pn._predict_burnout(
        IGNITION[1], IGNITION[2] * math.sin(IGNITION[3]), IGNITION[2] * math.cos(IGNITION[3]),
        IGNITION[4], MU, VE, F_T, vgo_r / L0, vgo_th / L0, lam, t_lam, t_go)
    tight = _fly(IGNITION, out)
    assert v_r == pytest.approx(tight[1], abs=0.05)
    assert v_th == pytest.approx(tight[2], abs=0.05)
    assert rG == pytest.approx(tight[4], abs=50.0)          # ∫∫g_r ≈ −216 km here


@pytest.mark.parametrize("r_T, target", [(WAYPOINT_R, WAYPOINT),
                                         (ORBIT_R, dict(v_theta_T=math.sqrt(MU / ORBIT_R)))])
def test_corrector_converges_on_the_target_velocity(r_T, target):
    out = pn.peg_new_major_loop(IGNITION, r_T, MU, VE, F_T, **target)
    burnout = _fly(IGNITION, out)
    assert burnout[1] == pytest.approx(target.get("v_r_T", 0.0), abs=0.1)
    assert burnout[2] == pytest.approx(target["v_theta_T"], abs=0.1)
    tau = IGNITION[4] * VE / F_T
    assert out[3] == pytest.approx(tau * (1.0 - math.exp(-out[2] / VE)))


def test_ignition_steering_climbs_like_the_optimum():
    vgo_r, vgo_th, L0, t_go, t_lam, lam = pn.peg_new_major_loop(
        IGNITION, WAYPOINT_R, MU, VE, F_T, **WAYPOINT)
    pitch = lambda t: math.degrees(math.atan2(vgo_r / L0 + lam * (t - t_lam), vgo_th / L0))
    # measured 2026-09-23: 17.02° falling to 10.12°; the PMP optimum from this state flies
    # 16.10° falling to 9.40°, and the pre-realignment law commanded −6.97° rising to 3.98°
    assert 15.0 < pitch(0.0) < 19.0
    assert lam < 0.0 and pitch(t_go) < pitch(0.0)
    assert t_go == pytest.approx(257.33, abs=1.0)            # the PMP's own arc-1 burn


def test_an_unreachable_target_returns_a_flyable_answer():
    # falling at 84 km with ~400 kg of propellant, asked for a 500 km orbit: the corrector
    # grew v_go until the predicted burn consumed the whole vehicle and divided by zero
    falling = np.array([0.0, c.R_EARTH + 84.5e3, 7495.7, math.radians(-4.73), 4300.0])
    out = pn.peg_new_major_loop(falling, ORBIT_R, MU, VE, F_T, v_theta_T=7171.9)
    assert all(math.isfinite(v) for v in out)
    assert out[3] < falling[4] * VE / F_T


def test_a_nearly_finished_burn_stays_finite():
    at_target = np.array([0.0, ORBIT_R, math.sqrt(MU / ORBIT_R) - 0.5, 0.0, 27000.0])
    out = pn.peg_new_major_loop(at_target, ORBIT_R, MU, VE, F_T, v_theta_T=math.sqrt(MU / ORBIT_R))
    assert all(math.isfinite(v) for v in out)
    assert 0.0 < out[3] < 1.0
