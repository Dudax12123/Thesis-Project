"""
Tests for get_orbital_elements, and for the rounding case that used to give NaN.

Eccentricity is sqrt(1 - h^2/(mu*a)). The radicand is exactly zero for a circular orbit,
so for a near-circular one it is ~1e-16 of either sign and floating-point rounding decides
which. A negative one made e NaN, which propagated into the apoapsis and periapsis; the
Chapter 6 loader reads the periapsis to decide whether a case reached orbit, so the case
was hatched out as suborbital. It happened to gt_sea_level_engine on 2026-09-18, which had
inserted at 500.00 km. The radicand is clamped at zero, which changes no physics.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Simulation import rocket_ascent as ra

R_T = c.R_EARTH + 500e3
# The state the search found: its radicand is -2.22e-16, so before the clamp this returned
# NaN for e, the apoapsis and the periapsis.
V_NEGATIVE_RADICAND = 7612.683989022485


def _radicand(r_val, v, gamma):
    a = (c.MU_EARTH * r_val) / ((2 * c.MU_EARTH) - (r_val * v ** 2))
    return 1 - (r_val * v * np.cos(gamma)) ** 2 / (c.MU_EARTH * a)


def test_the_regression_state_really_does_round_negative():
    """Without this the test below would pass for the wrong reason."""
    assert _radicand(R_T, V_NEGATIVE_RADICAND, 0.0) < 0.0


def test_a_negative_radicand_gives_a_circular_orbit_not_nan():
    a, e, r_apo, r_peri, period = ra.get_orbital_elements(R_T, V_NEGATIVE_RADICAND, 0.0)
    assert np.isfinite([a, e, r_apo, r_peri, period]).all()
    assert e == 0.0
    assert r_peri == pytest.approx(R_T, abs=1e-6)
    assert r_apo == pytest.approx(R_T, abs=1e-6)


@pytest.mark.parametrize("dv", [1.0, 10.0, 100.0])
def test_a_real_ellipse_is_untouched(dv):
    """The clamp must not flatten orbits that are genuinely eccentric: raising the speed
    at a perigee-like point lifts the apoapsis and leaves the periapsis where it is."""
    v = V_NEGATIVE_RADICAND + dv
    a, e, r_apo, r_peri, _period = ra.get_orbital_elements(R_T, v, 0.0)
    assert e > 0.0
    assert r_peri == pytest.approx(R_T, rel=1e-9)
    assert r_apo > R_T + 1e3
    # Vis-viva: the same state must give the same semi-major axis.
    assert a == pytest.approx(1.0 / (2.0 / R_T - v * v / c.MU_EARTH), rel=1e-12)


def test_a_suborbital_state_still_reports_a_negative_periapsis():
    """gt_direct inserts ~1 115 km below the surface; the clamp must not hide that."""
    a, e, _r_apo, r_peri, _period = ra.get_orbital_elements(R_T, 7000.0, 0.0)
    assert e > 0.05
    assert (r_peri - c.R_EARTH) < -1e6
