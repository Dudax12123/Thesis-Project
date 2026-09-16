"""
Tests for the frame and force model the Stage-2 indirect-PMP arc is propagated and
targeted in (INDIRECT_PMP_STAGE2_FRAME).

The PMP costate equations are derived from the drag-free, rotation-free EOM. Until
2026-09-13 the arc propagated the ground-relative state with those equations against the
rotating-frame speed target sqrt(mu/r) - v_rot ("rotating"): in them that target is not a
circular orbit but the apoapsis of an ellipse whose periapsis is ~890 km below the surface,
and a local refinement reached it ballistically by deleting the circularisation burn.
"inertial" (2026-09-13 to 2026-09-16) converts the hand-off state and targets the inertial
circular speed. "rotating_pseudo_forces" (the default since 2026-09-16) keeps the
ground-relative state and the laws' target but carries the same Coriolis/centrifugal
terms every other architecture does in the STATE equations, with the costate equations
as published: with the terms present the laws' target is level flight -- exactly at the
equator due east, and at the baseline site it turns down only by the unprojected-credit
convention every rotation-on case shares. Everything reported outward stays
ground-relative in every form.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Auxiliary import earth_rotation as earth_rot
from Input_File import simulation_parameters as sim_params
import Simulation.indirect_pso_solver as ips

from Simulation import rocket_ascent as ra

LAT = np.deg2rad(28.5)

# The logged pmp_prod_v2 swarm point (2026-09-11), used only as a trajectory that flies.
X_SWARM = [-0.002087, -0.938028, 0.995508, 280.11, 78.17, 93.21, 1.547791]


@pytest.fixture
def rotating_earth(monkeypatch):
    monkeypatch.setattr(sim_params, "ENABLE_EARTH_ROTATION", True)
    monkeypatch.setattr(sim_params, "LAUNCH_LATITUDE", 28.5)
    monkeypatch.setattr(sim_params, "TARGET_ORBITAL_ALTITUDE", 500e3)


@pytest.mark.parametrize("v, gamma_deg, alt_km", [
    (3242.0, 26.68, 90.9),     # a Stage-2 hand-off
    (7171.9, 0.0, 500.0),      # insertion
    (1500.0, 60.0, 40.0),
    (7000.0, -3.0, 300.0),
])
def test_planar_frame_transform_round_trips(v, gamma_deg, alt_km):
    r_val = c.R_EARTH + alt_km * 1e3
    v_in, g_in = earth_rot.rotating_to_inertial_planar(v, np.deg2rad(gamma_deg), LAT, r_val)
    v_back, g_back = earth_rot.inertial_to_rotating_planar(v_in, g_in, LAT, r_val)
    assert v_back == pytest.approx(v, abs=1e-9)
    assert g_back == pytest.approx(np.deg2rad(gamma_deg), abs=1e-12)


def test_planar_transform_credits_only_the_horizontal_component():
    """Radial speed is frame-independent. ecef_to_eci_velocity keeps gamma instead,
    which invents ~170 m/s of radial speed at a 27 deg hand-off."""
    r_val = c.R_EARTH + 90.9e3
    v, g = 3242.0, np.deg2rad(26.68)
    v_in, g_in = earth_rot.rotating_to_inertial_planar(v, g, LAT, r_val)
    assert v_in * np.sin(g_in) == pytest.approx(v * np.sin(g), abs=1e-9)
    v_rot = c.OMEGA_EARTH * r_val * np.cos(LAT)
    assert v_in * np.cos(g_in) == pytest.approx(v * np.cos(g) + v_rot, abs=1e-9)


def test_planar_transform_matches_the_insertion_helper_at_zero_fpa():
    r_val = c.R_EARTH + 500e3
    v_ref, g_ref = earth_rot.ecef_to_eci_velocity(7171.9, 0.0, LAT, r_val)
    v_in, g_in = earth_rot.rotating_to_inertial_planar(7171.9, 0.0, LAT, r_val)
    assert v_in == pytest.approx(v_ref, abs=1e-9)
    assert g_in == pytest.approx(g_ref, abs=1e-15)


def _coast_rates_at_target(v_target, pseudo_forces=False):
    """(r_dot, v_dot, gamma_dot) of the PMP's unpowered Stage-2 equations at the target."""
    r_t = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    y = [0.0, r_t, v_target, 0.0, 10000.0, 0.0, 1.0, 0.0]
    d = ips._stage2_ode(0.0, y, 0.0, 348.0, pseudo_forces)
    return d[1], d[2], d[3]


def _launch_site(monkeypatch, lat_deg, azimuth_deg):
    """Point the config and the rocket_ascent globals the pseudo-force terms read at
    one site, with the run switch on, as a solver would have left them."""
    monkeypatch.setattr(sim_params, "LAUNCH_LATITUDE", lat_deg)
    monkeypatch.setattr(sim_params, "INCLUDE_PSEUDO_FORCES", True)
    monkeypatch.setattr(ra, "LAUNCH_LATITUDE_RAD", np.deg2rad(lat_deg))
    monkeypatch.setattr(ra, "LAUNCH_AZIMUTH", np.deg2rad(azimuth_deg))
    monkeypatch.setattr(ra, "LAUNCH_AZIMUTH_INERTIAL", np.deg2rad(azimuth_deg))
    monkeypatch.setattr(ra, "_PSEUDO_FORCES_THIS_RUN", True)
    monkeypatch.setattr(ra, "PROPAGATING_IN_INERTIAL_FRAME", False)


def test_inertial_target_is_a_circular_orbit_of_the_stage2_equations(rotating_earth, monkeypatch):
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "inertial")
    rdot, vdot, gdot = _coast_rates_at_target(ips.terminal_speed_target())
    assert abs(rdot) < 1e-12
    assert abs(vdot) < 1e-9
    assert abs(gdot) < 1e-12


def test_rotating_target_is_an_apoapsis_of_the_stage2_equations(rotating_earth, monkeypatch):
    """Pins the defect the inertial frame removes: in the rotation-free equations the
    legacy target turns down at ~-0.45 deg/min."""
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "rotating")
    _, _, gdot = _coast_rates_at_target(ips.terminal_speed_target())
    assert np.rad2deg(gdot) * 60.0 == pytest.approx(-0.454, abs=0.005)


def test_rotating_pseudo_forces_target_is_level_flight_at_the_equator(rotating_earth, monkeypatch):
    """With the Coriolis and centrifugal terms in the state equations, the laws' own
    target sqrt(mu/r) - omega*r is exactly level flight for a due-east launch at the
    equator: (v + omega*r)^2 / r = g. The ellipse defect of the legacy form is gone."""
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "rotating_pseudo_forces")
    _launch_site(monkeypatch, 0.0, 90.0)
    rdot, vdot, gdot = _coast_rates_at_target(ips.terminal_speed_target(), pseudo_forces=True)
    assert abs(rdot) < 1e-12
    assert abs(vdot) < 1e-9
    assert abs(gdot) < 1e-10


def test_rotating_pseudo_forces_target_turns_down_only_by_the_unprojected_credit(
        rotating_earth, monkeypatch):
    """At the baseline site (28.5 deg, azimuth 45 deg) the target credits the full
    omega*r*cos(lat) where the terms credit the azimuth-projected part -- the
    deliberately kept convention every rotation-on case shares -- so level flight is
    missed by ~129 m/s and the target turns down at -0.13 deg/min, against
    -0.45 deg/min for the legacy pseudo-force-free form (an ellipse with periapsis
    ~890 km below the surface)."""
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "rotating_pseudo_forces")
    _launch_site(monkeypatch, 28.5, 44.98)
    _, _, gdot_terms = _coast_rates_at_target(ips.terminal_speed_target(), pseudo_forces=True)
    _, _, gdot_legacy = _coast_rates_at_target(ips.terminal_speed_target(), pseudo_forces=False)
    assert np.rad2deg(gdot_terms) * 60.0 == pytest.approx(-0.129, abs=0.005)
    assert np.rad2deg(gdot_legacy) * 60.0 == pytest.approx(-0.454, abs=0.005)


def test_frame_setting_is_inert_with_the_rotation_off(monkeypatch):
    monkeypatch.setattr(sim_params, "ENABLE_EARTH_ROTATION", False)
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "inertial")
    v_inertial_mode = ips.terminal_speed_target()
    for frame in ("rotating", "rotating_pseudo_forces"):
        monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", frame)
        assert ips.terminal_speed_target() == v_inertial_mode
        assert ips._stage2_pseudo_forces() is False


def test_unknown_frame_raises(monkeypatch):
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "ecef")
    with pytest.raises(ValueError):
        ips.terminal_speed_target()


def test_reported_state_is_ground_relative_and_the_dense_run_agrees(rotating_earth, monkeypatch):
    """state_final is ground-relative, state_final_propagated is what was flown, and
    run_indirect_full's last dense sample is the same ground-relative state."""
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "inertial")
    res = ips.run_indirect_trajectory(*X_SWARM)
    assert not res["crashed"]
    assert res["stage2_frame"] == "inertial"
    rep, flown = np.asarray(res["state_final"]), np.asarray(res["state_final_propagated"])
    v_in, g_in = earth_rot.rotating_to_inertial_planar(rep[2], rep[3], LAT, rep[1])
    assert v_in == pytest.approx(flown[2], abs=1e-6)
    assert g_in == pytest.approx(flown[3], abs=1e-9)
    assert flown[2] - rep[2] > 400.0          # the credit, not a rounding difference

    _t, data, _thr, _alpha, _t_ign, _res = ips.run_indirect_full(X_SWARM, verbose=False)
    assert data[2, -1] == pytest.approx(rep[2], abs=1e-6)
    assert data[3, -1] == pytest.approx(rep[3], abs=1e-9)
    assert data[0, -1] == pytest.approx(rep[0], abs=1e-3)


def test_rotating_pseudo_forces_run_reports_the_flown_state(rotating_earth, monkeypatch):
    """Under the default form nothing is converted: the reported state IS the flown
    state, the dense run ends on it, and the run switch stays on through Stage 2."""
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "rotating_pseudo_forces")
    monkeypatch.setattr(sim_params, "INCLUDE_PSEUDO_FORCES", True)
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE1_PSEUDO_FORCES", True)
    res = ips.run_indirect_trajectory(*X_SWARM)
    assert not res["crashed"]
    assert res["stage2_frame"] == "rotating_pseudo_forces"
    np.testing.assert_array_equal(np.asarray(res["state_final"]),
                                  np.asarray(res["state_final_propagated"]))
    assert ra._PSEUDO_FORCES_THIS_RUN is True

    _t, data, _thr, _alpha, _t_ign, res2 = ips.run_indirect_full(X_SWARM, verbose=False)
    np.testing.assert_allclose(data[:5, -1], np.asarray(res["state_final"]), rtol=0, atol=1e-6)
    assert res2["stage2_frame"] == "rotating_pseudo_forces"


def test_dense_run_publishes_full_flight_pitch_and_arc_boundaries(rotating_earth, monkeypatch):
    """run_indirect_full leaves in the rocket_ascent globals what main.py's plot
    suite reads: a pitch history on the dense output grid covering the whole flight
    (not the ODE-RHS samples, which stop at separation and carry speculative
    evaluations past the kick), and the SECO / coast-start markers."""
    from Simulation import rocket_ascent as ra
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "inertial")
    t, data, thrust, alpha, t_ign, _res = ips.run_indirect_full(X_SWARM, verbose=False)

    theta_t = np.asarray(ra.theta_time_history)
    theta = np.asarray(ra.theta_history)
    assert np.array_equal(theta_t, t)                    # the output grid, every sample
    assert np.all(np.diff(theta_t) >= 0.0)               # no speculative RHS samples
    np.testing.assert_allclose(theta, alpha + data[3], rtol=0, atol=1e-12)
    assert theta_t[-1] > t_ign                           # reaches past Stage 1

    # SECO is the end of the planned sequence, the last dense sample; the coast
    # starts after ignition and before it, where the thrust actually drops.
    assert ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL == pytest.approx(t[-1])
    t_coast = ra.PSO_COAST_ARC2_START_TIME
    assert t_ign < t_coast < ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    i = int(np.searchsorted(t, t_coast, "right"))
    assert thrust[i - 2] > 0.0 and thrust[i] == 0.0
