"""
Regression tests for the archived thrust record.

The thrust channel is not integrated: it is read back from a log that
``rocket_dynamics`` -- the ODE right-hand side -- appends on every call, and
solve_ivp calls it at trial times beyond the steps it accepts, past a terminal
event's root too. Until 2026-09-26 that left up to ~0.9 s of full Stage-1
thrust after MECO in every archive, and the coast's zero logged at the cutoff
instant pulled the last samples before it into a ramp: 5-22 m/s of dv_ideal
that no engine delivered. ``_close_logged_burn()`` now ends the log at each
root-found cutoff as a step, and ``thrust_on_grid()`` is the one reader.

The end-to-end property pinned here is the physical one: the recorded Stage-1
thrust, divided by Isp*g0, integrates to the propellant Stage 1 burned, on both
guidance dispatchers.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r
from Input_File import simulation_parameters as sim_params
import Simulation.rocket_ascent as ra


# The trapezoid rule spreads the cutoff step over the last grid interval, so the
# integral can fall short by up to half a TIME_STEP of thrust: 13.7 kg at the
# sea-level mass flow. The defect it guards against was ~1 t.
PROP_TOL = 20.0


@pytest.fixture
def sea_level_engine(monkeypatch):
    """Constant Stage-1 thrust and Isp, so F/(Isp*g0) is the mass flow exactly."""
    monkeypatch.setattr(sim_params, "THRUST_1_MODE", "sea_level")
    monkeypatch.setattr(sim_params, "ISP_1_MODE", "sea_level")


def _log(entries):
    """Append (t, thrust) pairs to the four right-hand-side logs, in step."""
    for t, f in entries:
        ra.time_history.append(t)
        ra.thrust_history.append(f)
        ra.coriolis_mag_history.append(10.0 * f)
        ra.centrifugal_mag_history.append(20.0 * f)


def _clear_logs():
    for log in (ra.time_history, ra.thrust_history,
                ra.coriolis_mag_history, ra.centrifugal_mag_history):
        log.clear()


class TestCloseLoggedBurn:
    """The log surgery itself, on a synthetic log."""

    def setup_method(self):
        _clear_logs()

    def test_trial_samples_past_the_cutoff_are_dropped(self):
        # A burn cut at 10.2 s whose integrator probed 10.3 and 10.7 s first,
        # out of time order, as solve_ivp does.
        _log([(9.0, 5.0), (9.6, 5.0), (10.3, 5.0), (10.0, 5.0), (10.7, 5.0)])
        ra._close_logged_burn(10.2)

        assert max(ra.time_history) < 10.2
        assert ra.time_history[-1] == np.nextafter(10.2, -np.inf)
        assert ra.thrust_history[-1] == 5.0
        # The four logs stay in step, closing sample included.
        n = len(ra.time_history)
        assert n == 4
        assert (len(ra.thrust_history) == len(ra.coriolis_mag_history)
                == len(ra.centrifugal_mag_history) == n)
        assert ra.coriolis_mag_history[-1] == 50.0

    def test_the_record_is_a_step_at_the_cutoff(self):
        _log([(9.0, 5.0), (9.6, 5.0), (10.3, 5.0), (10.0, 5.0), (10.7, 5.0)])
        ra._close_logged_burn(10.2)
        _log([(10.2, 0.0), (10.5, 0.0), (11.0, 0.0)])     # the coast that follows

        grid = np.array([9.8, 10.0, 10.1, 10.19, 10.2, 10.3, 10.6, 11.0])
        np.testing.assert_array_equal(
            ra.thrust_on_grid(grid), [5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0])

    def test_a_doubled_grid_instant_reads_the_burn_side_first(self):
        """One phase's last grid point and the next phase's first coincide."""
        _log([(9.0, 5.0), (10.0, 5.0)])
        ra._close_logged_burn(10.2)
        _log([(10.2, 0.0), (11.0, 0.0)])

        grid = np.array([10.1, 10.2, 10.2, 10.3])
        np.testing.assert_array_equal(ra.thrust_on_grid(grid), [5.0, 5.0, 0.0, 0.0])

    def test_logs_out_of_step_are_refused(self):
        _log([(9.0, 5.0)])
        ra.time_history.append(9.5)
        with pytest.raises(RuntimeError):
            ra._close_logged_burn(10.0)


def _stage1_window(t, t_meco):
    """Grid indices strictly before cutoff, and from it to separation."""
    t_sep = t_meco + r.TIME_First_STAGE_SEPARATION
    return t < t_meco, (t > t_meco) & (t <= t_sep)


@pytest.mark.parametrize("gamma_p", [1.54, 1.56])
class TestFlownStage1Record:
    """End-to-end: a flown Stage 1, read back through thrust_on_grid()."""

    def test_pso_dispatcher_step_and_propellant(self, gamma_p, sea_level_engine):
        ra.set_pseudo_forces_for_run(True)
        _t2, _s2, t_meco, t1, _y1, crashed = ra.run_stage1(gamma_p - np.pi / 2.0)
        assert not crashed

        thrust = ra.thrust_on_grid(t1)
        before, after = _stage1_window(t1, t_meco)
        np.testing.assert_allclose(thrust[before], r.F_THRUST_1_SL, rtol=1e-12)
        assert np.all(thrust[after] == 0.0)

        burned = np.trapezoid(thrust, t1) / (r.ISP_1_SL * c.G_0)
        assert burned == pytest.approx(r.M_PROP_1, abs=PROP_TOL)

    def test_legacy_dispatcher_step_and_propellant(self, gamma_p, sea_level_engine):
        ra.set_pseudo_forces_for_run(True)
        out = ra.run(gamma_p - np.pi / 2.0)
        t_all, thrust_log, time_log = out[0], out[5], out[6]
        t_meco = ra.time_main_engine_cutoff
        assert t_meco is not None

        thrust = ra.thrust_on_grid(t_all, time_log, thrust_log)
        before, after = _stage1_window(t_all, t_meco)
        np.testing.assert_allclose(thrust[before], r.F_THRUST_1_SL, rtol=1e-12)
        assert np.all(thrust[after] == 0.0)

        stage1 = t_all <= t_meco + r.TIME_First_STAGE_SEPARATION
        burned = np.trapezoid(thrust[stage1], t_all[stage1]) / (r.ISP_1_SL * c.G_0)
        assert burned == pytest.approx(r.M_PROP_1, abs=PROP_TOL)

    def test_default_engine_has_no_thrust_after_cutoff(self, gamma_p):
        """Under the configured (pressure) engine too: no sample of Stage-1
        thrust survives past MECO, and none before it drops below sea level."""
        ra.set_pseudo_forces_for_run(True)
        _t2, _s2, t_meco, t1, _y1, crashed = ra.run_stage1(gamma_p - np.pi / 2.0)
        assert not crashed

        thrust = ra.thrust_on_grid(t1)
        before, after = _stage1_window(t1, t_meco)
        assert thrust[before].min() >= r.F_THRUST_1_SL * (1.0 - 1e-12)
        assert np.all(thrust[after] == 0.0)
