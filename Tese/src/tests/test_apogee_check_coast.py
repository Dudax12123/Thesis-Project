"""
Regression tests for the apogee-check architecture's coast (2026-09-30).

Two defects:

- The full-simulation branch of ``run()`` set ``rocket_specs.C_D = 0`` after
  SECO and never restored it. The flight never saw it -- ``atmosphere.drag_force``
  had bound C_D as a default at import -- but ``Auxiliary.losses``, imported
  lazily after the flight, bound the zero instead, and every archived
  apogee_check budget recorded no drag loss (gt_apogee: 34.9 m/s).
- The "inertial" coast converts the SECO state with the full w*r*cos(lat) and
  flies it without the pseudo-forces, which no other architecture does before
  insertion; it credited gt_apogee 0.36-0.70 t. ``APOGEE_CHECK_COAST_FRAME =
  "rotating"`` flies the other architectures' coast ODE and inserts at their
  target, and the results matrix flies it.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Archive import run_record
from Auxiliary import constants as c
from Auxiliary import earth_rotation as earth_rot
from Auxiliary import rocket_specs as r
from Input_File import simulation_parameters as sim_params
import Simulation.rocket_ascent as ra

# gt_apogee's kick in the 2026-09-26 results matrix [rad].
KICK = -0.030996526995096696


@pytest.fixture
def apogee_case(monkeypatch):
    """gt_apogee's settings that differ from the configuration defaults."""
    monkeypatch.setattr(sim_params, "GUIDANCE_MODE", "gravity_turn")
    monkeypatch.setattr(sim_params, "COAST_METHOD", "apogee_check")
    monkeypatch.setattr(sim_params, "KICK_PROFILE_MODE", "instantaneous")
    monkeypatch.setattr(ra, "SINGLE_BURN_FULL_SIMULATION", True)


def _fly(monkeypatch, frame):
    monkeypatch.setattr(sim_params, "APOGEE_CHECK_COAST_FRAME", frame)
    t, data = ra.run(KICK)[:2]
    return np.asarray(t), np.asarray(data)


class TestDragCoefficient:

    @pytest.mark.parametrize("frame", ["inertial", "rotating"])
    def test_a_full_flight_leaves_the_drag_coefficient_alone(self, apogee_case,
                                                            monkeypatch, frame):
        before = r.C_D
        _fly(monkeypatch, frame)
        assert r.C_D == before


class TestRotatingCoast:

    def test_the_final_state_is_reported_in_its_own_frame(self, apogee_case, monkeypatch):
        _fly(monkeypatch, "inertial")
        assert ra.FINAL_STATE_INERTIAL is True
        _fly(monkeypatch, "rotating")
        assert ra.FINAL_STATE_INERTIAL is False

    def test_the_coast_reaches_its_apoapsis_at_the_target_altitude(self, apogee_case,
                                                                    monkeypatch):
        t, data = _fly(monkeypatch, "rotating")
        i = int(np.searchsorted(t, ra.TIME_CIRCULARISATION))
        assert data[1, i] - c.R_EARTH == pytest.approx(sim_params.TARGET_ORBITAL_ALTITUDE,
                                                       abs=1.0)
        assert abs(data[3, i]) < 1e-9

    def test_the_burn_goes_to_the_coast_architectures_target(self, apogee_case, monkeypatch):
        t, data = _fly(monkeypatch, "rotating")
        i = int(np.searchsorted(t, ra.TIME_CIRCULARISATION))
        assert t[i + 1] == t[i]                       # the impulsive burn, as a doubled instant
        v_target = earth_rot.v_circular_rotating(
            data[1, i], np.deg2rad(sim_params.LAUNCH_LATITUDE), sim_params.ENABLE_EARTH_ROTATION)
        assert data[2, i + 1] == pytest.approx(v_target, abs=1e-9)

    def test_the_insertion_state_is_the_one_after_the_burn(self, apogee_case, monkeypatch):
        # Not the last sample: 1000 s on, far downrange, a rotating-frame state
        # does not convert to the orbit (the archive read 712 x 256 km off it).
        t, data = _fly(monkeypatch, "rotating")
        i = int(np.searchsorted(t, ra.TIME_CIRCULARISATION))
        assert np.array_equal(ra.STATE_INSERTION, data[:, i + 1])
        _fly(monkeypatch, "inertial")
        assert ra.STATE_INSERTION is None

    def test_nothing_steps_the_velocity_between_ignition_and_apoapsis(self, apogee_case,
                                                                      monkeypatch):
        # The inertial coast steps it by ~420 m/s at SECO, the frame conversion.
        t, data = _fly(monkeypatch, "rotating")
        i = int(np.searchsorted(t, ra.TIME_CIRCULARISATION))
        s = int(np.searchsorted(t, ra.time_main_engine_cutoff)) + 1
        assert np.max(np.abs(np.diff(data[2, s:i]))) < 1.0


class TestEvaluationCount:

    def test_the_apogee_check_reports_its_grid(self, monkeypatch):
        from Simulation import solver
        monkeypatch.setattr(sim_params, "MULTI_GUIDANCE_ENABLED", False)
        monkeypatch.setattr(sim_params, "GUIDANCE_MODE", "gravity_turn")
        monkeypatch.setattr(sim_params, "COAST_METHOD", "apogee_check")
        assert run_record.n_evaluations(sim_params) == solver.BRUTE_GRID_POINTS

    def test_a_dispatcher_count_wins_over_the_configured_budget(self, monkeypatch):
        monkeypatch.setattr(sim_params, "MULTI_GUIDANCE_ENABLED", False)
        monkeypatch.setattr(sim_params, "GUIDANCE_MODE", "indirect_pmp")
        assert run_record.n_evaluations(sim_params) == (
            sim_params.PSO_N_PARTICLES * sim_params.PSO_MAX_GENERATIONS)
        assert run_record.n_evaluations(sim_params, {'n_evaluations': 1}) == 1
