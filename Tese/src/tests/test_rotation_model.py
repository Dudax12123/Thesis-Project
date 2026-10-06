"""
EARTH_ROTATION_MODEL = "launch_site" (2026-10-05): the ascent is planar, the latitude is
held at its launch value, and the rotation is credited as the launch-site speed
omega*r*cos(lat) -- never resolved on the launch azimuth -- in the target, the frame
conversion, the delta-v budget gain and the pseudo-force terms alike.

The level-flight and frame-counterpart checks live with the PMP frame tests
(test_pmp_frame.py, test_pmp_stage1_pseudo_forces.py); these pin the rest.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Auxiliary import earth_rotation as earth_rot
from Auxiliary import losses as loss_mod
from Input_File import simulation_parameters as sim_params
from Simulation import rocket_ascent as ra


@pytest.fixture
def baseline_site(monkeypatch):
    monkeypatch.setattr(sim_params, "ENABLE_EARTH_ROTATION", True)
    monkeypatch.setattr(sim_params, "LAUNCH_LATITUDE", 28.5)
    monkeypatch.setattr(sim_params, "TARGET_ORBITAL_ALTITUDE", 500e3)
    monkeypatch.setattr(sim_params, "TARGET_ORBIT_INCLINATION", 51.6)
    monkeypatch.setattr(ra, "LAUNCH_LATITUDE_RAD", np.deg2rad(28.5))


def test_the_budget_gain_is_the_launch_site_speed_whatever_the_azimuth(baseline_site,
                                                                       monkeypatch):
    """Before 2026-10-05 the gain was resolved on the azimuth (311.5 m/s at the baseline
    site) while the target and the conversion credited 440.8 m/s, and the residual of
    every rotation-on case carried the -129 m/s difference."""
    r_t = c.R_EARTH + 500e3
    expected = c.OMEGA_EARTH * r_t * np.cos(np.deg2rad(28.5))
    assert loss_mod.launch_site_gain() == pytest.approx(expected, rel=1e-15)
    assert loss_mod.launch_site_gain() == pytest.approx(440.77, abs=0.01)
    monkeypatch.setattr(sim_params, "TARGET_ORBIT_INCLINATION", 80.0)   # another azimuth
    assert loss_mod.launch_site_gain() == pytest.approx(expected, rel=1e-15)
    monkeypatch.setattr(sim_params, "ENABLE_EARTH_ROTATION", False)
    assert loss_mod.launch_site_gain() == 0.0


def test_target_conversion_and_gain_share_one_credit(baseline_site):
    """At the target the inertial speed is circular, and the gain is the difference."""
    r_t = c.R_EARTH + 500e3
    v_target = earth_rot.v_circular_rotating(r_t, np.deg2rad(28.5))
    v_in, _ = earth_rot.ecef_to_eci_velocity(v_target, 0.0, np.deg2rad(28.5), r_t)
    assert v_in == pytest.approx(np.sqrt(c.MU_EARTH / r_t), abs=1e-9)
    assert v_in - v_target == pytest.approx(loss_mod.launch_site_gain(), abs=1e-9)


def test_the_pseudo_force_terms_ignore_the_azimuth(baseline_site, monkeypatch):
    rates = earth_rot.planar_pseudoforce_rates(6500.0, np.deg2rad(5.0),
                                               np.deg2rad(28.5), c.R_EARTH + 300e3)
    for azimuth in (0.0, 44.98, 90.0, 135.0):
        monkeypatch.setattr(ra, "LAUNCH_AZIMUTH", np.deg2rad(azimuth))
        assert earth_rot.planar_pseudoforce_rates(
            6500.0, np.deg2rad(5.0), np.deg2rad(28.5), c.R_EARTH + 300e3) == rates


def test_the_latitude_row_is_the_launch_latitude_throughout(baseline_site):
    data = np.vstack([np.linspace(0.0, 8e6, 7), np.full(7, c.R_EARTH + 1e5),
                      np.full(7, 5e3), np.zeros(7), np.full(7, 3e4)])
    out = ra.append_latitude_row(data)
    assert out.shape == (6, 7)
    assert np.all(out[5] == np.deg2rad(28.5))


def _case(name, rotation, model):
    from Plots.results_figures import _data
    t = np.linspace(0.0, 1.0, 3)
    config = {} if model is None else {"EARTH_ROTATION_MODEL": model}
    return _data.Case.from_arrays(name, t, np.zeros((5, 3)), np.zeros(3), np.zeros(3),
                                  row={"earth_rotation": rotation},
                                  manifest={"config": config})


def test_figures_and_tables_refuse_rows_of_two_rotation_models():
    """A partly re-flown matrix must not be drawn as one comparison. Rotation-off
    rows are untouched by the change, so an unlabelled one passes."""
    from Plots.results_figures import _data
    _data.check_one_rotation_model({"a": _case("a", True, "launch_site"),
                                    "b": _case("b", True, "launch_site"),
                                    "norot": _case("norot", False, None)})
    with pytest.raises(ValueError, match="gt_old"):
        _data.check_one_rotation_model({"a": _case("a", True, "launch_site"),
                                        "gt_old": _case("gt_old", True, None)})


def test_another_rotation_model_is_refused(monkeypatch):
    monkeypatch.setattr(sim_params, "EARTH_ROTATION_MODEL", "great_circle")
    with pytest.raises(ValueError, match="EARTH_ROTATION_MODEL"):
        ra._check_earth_rotation_model()
    monkeypatch.setattr(sim_params, "EARTH_ROTATION_MODEL", "launch_site")
    ra._check_earth_rotation_model()
