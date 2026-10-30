"""
Tests for Auxiliary/orbit_trim.py, the two-burn apsidal correction.

An insertion in position but off circular (peg_direct: 492.8 x 507.0 km) is
priced as a burn at the first apsis reached, moving the opposite apsis to the
target, then a burn there that circularises. The tests apply the burns with
vis-viva and check that the result IS the target circle.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Auxiliary import orbit_trim
from Auxiliary import rocket_specs as r_specs

MU = c.MU_EARTH
R_T = c.R_EARTH + 500e3
ISP, F = r_specs.ISP_2, r_specs.F_THRUST_2
MASS = 20e3


def _state_on(r_peri, r_apo, r, climbing):
    """(v, gamma) at radius r on the ellipse with the given apsides."""
    a = 0.5 * (r_peri + r_apo)
    v = np.sqrt(MU * (2.0 / r - 1.0 / a))
    h = np.sqrt(MU * 2.0 * r_peri * r_apo / (r_peri + r_apo))
    gamma = np.arccos(min(1.0, h / (r * v)))
    return v, (gamma if climbing else -gamma)


def _peg_direct_like(climbing=True):
    rp, ra_ = c.R_EARTH + 492.8e3, c.R_EARTH + 507.0e3
    v, g = _state_on(rp, ra_, R_T + 40.0, climbing)
    return rp, ra_, v, g


def test_circular_orbit_needs_nothing():
    out = orbit_trim.apsidal_trim(R_T, np.sqrt(MU / R_T), 0.0, MASS, R_T, ISP, F)
    assert out['status'] == 'ok'
    assert out['dv_total_ms'] == pytest.approx(0.0, abs=1e-9)
    assert out['prop_total_kg'] == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("climbing, first", [(True, 'apoapsis'), (False, 'periapsis')])
def test_first_burn_is_at_the_next_apsis(climbing, first):
    rp, ra_, v, g = _peg_direct_like(climbing)
    out = orbit_trim.apsidal_trim(R_T + 40.0, v, g, MASS, R_T, ISP, F)
    assert out['burn1_at'] == first
    r1 = ra_ if climbing else rp
    assert out['burn1_alt_km'] == pytest.approx((r1 - c.R_EARTH) / 1e3, abs=1e-3)


@pytest.mark.parametrize("climbing", [True, False])
def test_burns_end_on_the_target_circle(climbing):
    rp, ra_, v, g = _peg_direct_like(climbing)
    out = orbit_trim.apsidal_trim(R_T + 40.0, v, g, MASS, R_T, ISP, F)
    r1 = ra_ if climbing else rp
    a0 = 0.5 * (rp + ra_)
    v1 = np.sqrt(MU * (2.0 / r1 - 1.0 / a0)) + out['dv1_ms']
    a1 = 1.0 / (2.0 / r1 - v1**2 / MU)            # vis-viva after burn 1
    assert 2.0 * a1 - r1 == pytest.approx(R_T, abs=1e-3)   # opposite apsis on target
    v2 = np.sqrt(MU * (2.0 / R_T - 1.0 / a1)) + out['dv2_ms']
    assert v2 == pytest.approx(np.sqrt(MU / R_T), abs=1e-9)


def test_peg_direct_signs():
    # Climbing to a 507 km apoapsis: raise the 492.8 km periapsis (prograde),
    # then lower the apoapsis at 500 km (retrograde).
    _, _, v, g = _peg_direct_like(True)
    out = orbit_trim.apsidal_trim(R_T + 40.0, v, g, MASS, R_T, ISP, F)
    assert out['dv1_ms'] > 0.0 > out['dv2_ms']
    assert out['moved_apsis_alt_km'] == pytest.approx(492.8, abs=1e-3)


def test_propellant_and_burn_time():
    _, _, v, g = _peg_direct_like(True)
    out = orbit_trim.apsidal_trim(R_T + 40.0, v, g, MASS, R_T, ISP, F)
    ve = c.G_0 * ISP
    m1 = MASS * np.exp(-abs(out['dv1_ms']) / ve)
    m2 = m1 * np.exp(-abs(out['dv2_ms']) / ve)
    assert out['prop_total_kg'] == pytest.approx(MASS - m2, rel=1e-12)
    assert out['burn_total_s'] == pytest.approx(out['prop_total_kg'] / (F / ve), rel=1e-12)


def test_coast_between_burns_is_half_the_transfer_period():
    _, ra_, v, g = _peg_direct_like(True)
    out = orbit_trim.apsidal_trim(R_T + 40.0, v, g, MASS, R_T, ISP, F)
    a_t = 0.5 * (ra_ + R_T)
    assert out['coast_between_burns_s'] == pytest.approx(np.pi * np.sqrt(a_t**3 / MU), rel=1e-9)


def test_hyperbolic_and_impact():
    assert orbit_trim.apsidal_trim(R_T, 1.5 * np.sqrt(2 * MU / R_T), 0.0,
                                   MASS, R_T, ISP, F)['status'] == 'hyperbolic'
    # Descending far below circular speed: the next apsis is inside the Earth.
    assert orbit_trim.apsidal_trim(R_T, 6000.0, np.deg2rad(-0.3),
                                   MASS, R_T, ISP, F)['status'] == 'impact'


def test_position_gate():
    tol_m, tol_g = 5e3, np.deg2rad(0.5)
    assert orbit_trim.in_position(R_T + 4e3, np.deg2rad(0.4), R_T, tol_m, tol_g)
    assert not orbit_trim.in_position(R_T + 6e3, 0.0, R_T, tol_m, tol_g)
    assert not orbit_trim.in_position(R_T, np.deg2rad(-0.6), R_T, tol_m, tol_g)


def test_archive_row_columns():
    from Archive.run_record import _trim_columns
    sp = SimpleNamespace(TARGET_ORBITAL_ALTITUDE=500e3,
                         DIRECT_INSERTION_ALTITUDE_TOL_KM=5.0,
                         DIRECT_INSERTION_FPA_TOL_DEG=0.5)
    _, _, v, g = _peg_direct_like(True)
    sf = np.array([0.0, R_T + 40.0, v, g, MASS])
    cols = _trim_columns(sp, sf, v, g, prop_remaining=10.0)
    assert cols['trim_in_position'] and cols['trim_status'] == 'ok'
    assert cols['trim_prop_remaining_after_kg'] == pytest.approx(10.0 - cols['trim_prop_total_kg'])
    assert not cols['trim_prop_sufficient']            # 10 kg cannot pay ~23 kg
    sf_far = sf.copy(); sf_far[1] = R_T + 20e3
    assert _trim_columns(sp, sf_far, v, g, 10.0) == {'trim_in_position': False}
