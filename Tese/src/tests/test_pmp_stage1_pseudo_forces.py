"""
Tests for the pseudo-forces under indirect_pmp (2026-09-16).

Stage 1 of the PMP is run_stage1's gravity turn, flown before any costate exists, so
it now carries the rotating-frame Coriolis/centrifugal terms like every other
architecture (INDIRECT_PMP_STAGE1_PSEUDO_FORCES). Until then the whole PMP ascent was
exempt, and its Stage 2 started 9.1 km lower, 44 m/s faster and 4.5 deg shallower
than the identical Stage 1 of every case it is compared with.

Stage 2 is propagated in the inertial frame, where no pseudo-force term exists. At
the equator, launching due east, that propagation is the exact counterpart of the
rotating frame with the terms (agreement to integration tolerance, tested below).
Away from it the two differ by convention, not by a bug: the frame transform
credits the full omega*r*cos(lat_launch) along-track -- the unprojected credit the
archive and the terminal targets use -- while the pseudo-force terms credit the
azimuth-projected part along the drifting latitude. Measured from the baseline
hand-off (28.5 deg, azimuth 45 deg): 0.6 km / 0.7 m/s / 0.23 deg after a 100 s
coast, 25.8 km / 57 m/s / 1.0 deg after 600 s, 125 km / 296 m/s after 1883 s.
That number is a property of the convention and is not frozen into a test.
"""

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.integrate import solve_ivp

sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r
from Input_File import simulation_parameters as sim_params
from Simulation import rocket_ascent as ra
import Simulation.indirect_pso_solver as ips
import Simulation.pso_coast_solver as pcs

# The logged pmp_prod_v2 swarm point (2026-09-11), used only as a trajectory that flies.
X_SWARM = [-0.002087, -0.938028, 0.995508, 280.11, 78.17, 93.21, 1.547791]
KICK = X_SWARM[6] - np.pi / 2.0


@pytest.fixture
def rotating_earth(monkeypatch):
    monkeypatch.setattr(sim_params, "ENABLE_EARTH_ROTATION", True)
    monkeypatch.setattr(sim_params, "INCLUDE_PSEUDO_FORCES", True)
    monkeypatch.setattr(sim_params, "LAUNCH_LATITUDE", 28.5)
    monkeypatch.setattr(sim_params, "TARGET_ORBITAL_ALTITUDE", 500e3)
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "inertial")
    monkeypatch.setattr(sim_params, "EVENTS_PRINT", False)
    monkeypatch.setattr(sim_params, "INTERRUPTS_PRINT", False)
    yield
    ra.set_pseudo_forces_for_run(True)


def _stage1_flown_by_the_pmp():
    """The Stage-1 part of run_indirect_full's dense output, and its last sample."""
    t, data, _thr, _alpha, _t_ign, _res = ips.run_indirect_full(X_SWARM, verbose=False)
    return t, data


def _stage1_reference(pseudo_forces):
    ra.set_pseudo_forces_for_run(pseudo_forces)
    _t2, _s2, _t_meco, t1, y1, crashed = ra.run_stage1(KICK)
    assert not crashed
    return t1, y1


def test_stage1_carries_pseudo_forces_like_every_other_architecture(rotating_earth, monkeypatch):
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE1_PSEUDO_FORCES", True)
    t1, y1 = _stage1_reference(pseudo_forces=True)
    t, data = _stage1_flown_by_the_pmp()
    n1 = len(t1)
    assert ra._PSEUDO_FORCES_THIS_RUN is True            # left where Stage 1 set it
    np.testing.assert_allclose(t[:n1], t1, rtol=0, atol=1e-12)
    np.testing.assert_allclose(data[:5, n1 - 1], y1[:5, -1], rtol=0, atol=1e-9)


def test_legacy_exemption_reproduces_the_pseudo_force_free_hand_off(rotating_earth, monkeypatch):
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE1_PSEUDO_FORCES", False)
    t1_on, y1_on = _stage1_reference(pseudo_forces=True)
    t1_off, y1_off = _stage1_reference(pseudo_forces=False)
    t, data = _stage1_flown_by_the_pmp()
    assert ra._PSEUDO_FORCES_THIS_RUN is False
    np.testing.assert_allclose(data[:5, len(t1_off) - 1], y1_off[:5, -1], rtol=0, atol=1e-9)
    # ... and that hand-off is the one the switch exists to leave behind.
    assert abs(y1_on[1, -1] - y1_off[1, -1]) > 5e3            # metres of altitude
    assert abs(np.rad2deg(y1_on[3, -1] - y1_off[3, -1])) > 3.0  # degrees of gamma


def test_stage1_pseudo_forces_refuse_the_legacy_rotating_stage2(rotating_earth, monkeypatch):
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE1_PSEUDO_FORCES", True)
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "rotating")
    with pytest.raises(ValueError, match="INDIRECT_PMP_STAGE1_PSEUDO_FORCES"):
        ips.run_indirect_trajectory(*X_SWARM)
    # The refusal is about mixing force models, so it lifts with the terms.
    monkeypatch.setattr(sim_params, "INCLUDE_PSEUDO_FORCES", False)
    assert ips._stage1_pseudo_forces() is True


# A Stage-2 hand-off state (the 2026-09-16 baseline, Stage 1 with pseudo-forces).
_HANDOFF = np.array([135.1e3, c.R_EARTH + 79759.0, 3314.123, np.deg2rad(20.8296), 96570.0])
_T2 = 149.015


def _coast_both_ways(duration):
    tol = dict(rtol=1e-11, atol=1e-9, max_step=1.0)
    rot = solve_ivp(lambda t, y: pcs._stage2_ode_guidance(t, y, 0.0, r.ISP_2, None),
                    (_T2, _T2 + duration), list(_HANDOFF), **tol)
    y0 = list(ips._to_stage2_frame(_HANDOFF)) + [0.0, 1.0, 0.0]
    inr = solve_ivp(lambda t, y: ips._stage2_ode(t, y, 0.0, r.ISP_2),
                    (_T2, _T2 + duration), y0, **tol)
    back = ips._from_stage2_frame(inr.y[:5, -1], inr.t[-1], _T2)
    return rot.y[:5, -1], back


def test_inertial_stage2_is_the_exact_counterpart_of_the_rotating_frame_at_the_equator(
        rotating_earth, monkeypatch):
    """Launching due east from the equator the orbit plane contains the rotation and
    the latitude stays put, so the frame transform and the pseudo-force terms must
    describe the same physics -- and they do, to integration tolerance."""
    monkeypatch.setattr(sim_params, "LAUNCH_LATITUDE", 0.0)
    monkeypatch.setattr(ra, "LAUNCH_LATITUDE_RAD", 0.0)
    monkeypatch.setattr(ra, "LAUNCH_AZIMUTH", np.pi / 2)
    monkeypatch.setattr(ra, "LAUNCH_AZIMUTH_INERTIAL", np.pi / 2)
    monkeypatch.setattr(ra, "_PSEUDO_FORCES_THIS_RUN", True)
    monkeypatch.setattr(ra, "PROPAGATING_IN_INERTIAL_FRAME", False)
    assert ra._pseudo_forces_active()
    rot, back = _coast_both_ways(600.0)
    assert back[1] == pytest.approx(rot[1], abs=1e-2)          # radius, m
    assert back[2] == pytest.approx(rot[2], abs=1e-5)          # speed, m/s
    assert back[3] == pytest.approx(rot[3], abs=1e-8)          # gamma, rad
    assert back[0] == pytest.approx(rot[0], abs=1e-2)          # downrange, m


def test_the_two_stage2_models_differ_off_the_equator_by_the_credit_convention(
        rotating_earth, monkeypatch):
    """Same check at the baseline site (28.5 deg, azimuth 45 deg): the difference is
    real and grows with the coast, which is what Chapter 6 has to qualify."""
    monkeypatch.setattr(ra, "_PSEUDO_FORCES_THIS_RUN", True)
    monkeypatch.setattr(ra, "PROPAGATING_IN_INERTIAL_FRAME", False)
    rot, back = _coast_both_ways(600.0)
    assert abs(back[1] - rot[1]) > 1e3                          # kilometres, not metres
    assert abs(back[2] - rot[2]) > 10.0
