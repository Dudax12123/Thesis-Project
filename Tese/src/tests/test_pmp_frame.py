"""
Tests for the frame the Stage-2 indirect-PMP arc is propagated and targeted in.

indirect_pmp flies without pseudo-forces, so its Stage-2 equations are correct only in a
non-rotating frame. Until 2026-09-13 they propagated the ground-relative state against the
rotating-frame speed target sqrt(mu/r) - v_rot. In those equations that target is not a
circular orbit but the apoapsis of an ellipse whose periapsis is ~890 km below the surface,
and a local refinement reached it ballistically by deleting the circularisation burn.
INDIRECT_PMP_STAGE2_FRAME = "inertial" converts the hand-off state and targets the inertial
circular speed; everything reported outward stays ground-relative.
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


def _coast_rates_at_target(v_target):
    """(r_dot, v_dot, gamma_dot) of the PMP's unpowered Stage-2 equations at the target."""
    r_t = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    y = [0.0, r_t, v_target, 0.0, 10000.0, 0.0, 1.0, 0.0]
    d = ips._stage2_ode(0.0, y, 0.0, 348.0)
    return d[1], d[2], d[3]


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


def test_frame_setting_is_inert_with_the_rotation_off(monkeypatch):
    monkeypatch.setattr(sim_params, "ENABLE_EARTH_ROTATION", False)
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "inertial")
    v_inertial_mode = ips.terminal_speed_target()
    monkeypatch.setattr(sim_params, "INDIRECT_PMP_STAGE2_FRAME", "rotating")
    assert ips.terminal_speed_target() == v_inertial_mode


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
