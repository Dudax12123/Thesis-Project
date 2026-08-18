"""
Unit tests for the delta-v budget in Auxiliary/losses.py.

Every case below is constructed so that the integrals have a closed form: the
flight-path angle, the mass, the thrust and the angle of attack are all held
constant, and the trajectory sits at sea level, so each loss reduces to a
constant times the burn duration. That makes the expected values arithmetic
rather than a second implementation of the same integral.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import atmosphere as atm
from Auxiliary import constants as c
from Auxiliary import gravity as grav
from Auxiliary import losses as loss_mod
from Auxiliary import rocket_specs as r


DURATION = 100.0
MASS = 500e3
THRUST = 7607e3
SPEED = 200.0


def _flat_case(n=2001, gamma=np.pi / 2, alpha=0.0, alt=0.0):
    """Constant-everything trajectory: vertical, sea level, fixed mass and thrust."""
    t = np.linspace(0.0, DURATION, n)
    return dict(
        t=t,
        alt=np.full(n, alt),
        v=np.full(n, SPEED),
        gamma=np.full(n, gamma),
        m=np.full(n, MASS),
        thrust=np.full(n, THRUST),
        alpha=np.full(n, alpha),
    )


class TestLossHistories:
    """The four loss integrals against their closed forms."""

    def test_gravity_loss_is_g_sin_gamma_times_duration(self):
        case = _flat_case()
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="sea_level")
        expected = grav.gravitational_acceleration(c.R_EARTH) * DURATION
        assert hist["gravity"][-1] == pytest.approx(expected)

    def test_gravity_loss_vanishes_in_horizontal_flight(self):
        case = _flat_case(gamma=0.0)
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="sea_level")
        assert hist["gravity"][-1] == pytest.approx(0.0)

    def test_drag_loss_uses_rocket_specs_not_local_constants(self):
        case = _flat_case()
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="sea_level")
        q = atm.dynamic_pressure(SPEED, 0.0)
        expected = q * r.C_D * r.A / MASS * DURATION
        assert hist["drag"][-1] == pytest.approx(expected)

    def test_drag_loss_is_zero_without_atmosphere(self):
        case = _flat_case()
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=False,
                                       thrust_mode="sea_level")
        assert hist["drag"][-1] == pytest.approx(0.0)

    def test_steering_loss_vanishes_at_zero_alpha(self):
        case = _flat_case(alpha=0.0)
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="sea_level")
        assert hist["steering"][-1] == pytest.approx(0.0)

    def test_steering_loss_follows_the_one_minus_cosine_form(self):
        alpha = np.deg2rad(10.0)
        case = _flat_case(alpha=alpha)
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="sea_level")
        expected = THRUST / MASS * (1.0 - np.cos(alpha)) * DURATION
        assert hist["steering"][-1] == pytest.approx(expected)


class TestPressureLoss:
    """The pressure term exists only for the thrust model that produces it."""

    def test_not_applicable_under_a_constant_thrust_mode(self):
        case = _flat_case()
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="sea_level")
        assert hist["pressure_applicable"] is False
        assert hist["pressure"][-1] == pytest.approx(0.0)

    def test_applicable_under_pressure_mode(self):
        case = _flat_case()
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="pressure")
        expected = c.P_0 * r.A_E / MASS * DURATION
        assert hist["pressure_applicable"] is True
        assert hist["pressure"][-1] == pytest.approx(expected)

    def test_confined_to_the_stage_one_arc(self):
        """With MECO at half the burn, only half the deficit accrues."""
        case = _flat_case()
        hist = loss_mod.loss_histories(**case, t_meco=DURATION / 2.0, include_drag=True,
                                       thrust_mode="pressure")
        expected = c.P_0 * r.A_E / MASS * (DURATION / 2.0)
        assert hist["pressure"][-1] == pytest.approx(expected, rel=1e-3)

    def test_ideal_is_the_vacuum_integral(self):
        """dv_ideal must exceed the flown thrust integral by exactly the deficit."""
        case = _flat_case()
        hist = loss_mod.loss_histories(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="pressure")
        flown = THRUST / MASS * DURATION
        assert hist["ideal"][-1] == pytest.approx(flown + hist["pressure"][-1])


class TestBudgetClosure:
    """The budget identity, on a case where it must close exactly."""

    def test_closes_on_a_drag_free_vertical_climb(self):
        """No drag, no steering, no rotation: dv = dv_ideal - gravity, exactly.

        The speed history is made consistent with the forces rather than held
        constant, so the residual is a real test of the identity and not of the
        input arrays.
        """
        n = 4001
        t = np.linspace(0.0, DURATION, n)
        g = grav.gravitational_acceleration(c.R_EARTH)
        v = (THRUST / MASS - g) * t

        budget = loss_mod.delta_v_budget(
            t=t,
            alt=np.zeros(n),
            v=v,
            gamma=np.full(n, np.pi / 2),
            m=np.full(n, MASS),
            thrust=np.full(n, THRUST),
            alpha=np.zeros(n),
            t_meco=DURATION,
            include_drag=False,
            thrust_mode="sea_level",
            dv_gain=0.0,
        )
        assert budget["residual"] == pytest.approx(0.0, abs=1e-6)
        assert budget["dv_achieved"] == pytest.approx(
            budget["dv_ideal"] - budget["dv_losses"])

    def test_gain_enters_the_budget_with_the_right_sign(self):
        case = _flat_case()
        base = loss_mod.delta_v_budget(**case, t_meco=DURATION, include_drag=True,
                                       thrust_mode="sea_level", dv_gain=0.0)
        with_gain = loss_mod.delta_v_budget(**case, t_meco=DURATION, include_drag=True,
                                            thrust_mode="sea_level", dv_gain=400.0)
        assert with_gain["residual"] - base["residual"] == pytest.approx(400.0)
