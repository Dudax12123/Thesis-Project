"""Unit tests for the Chapter 6 table generator.

The tables are generated so that no number in the thesis is typed by hand. That
moves the risk to the places where a plausible wrong number can come out: the
formatter (a sign, a thousands group, a negative zero), the slot each
architecture keeps its pitch-over angle in, and the arc durations, whose
shortest coast is half a second. Everything is built in memory, not read from
the results matrix, so the tests do not depend on a batch having been flown.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Plots.results_figures import _data
from Plots.results_figures import tables


def _case(architecture, time=None, thrust=None, **channels):
    time = np.linspace(0.0, 100.0, 51) if time is None else np.asarray(time)
    n = len(time)
    data = np.vstack([time, 6.4e6 + time, 1e3 + time, np.full(n, 0.5), 1e5 - time])
    thrust = np.full(n, 1e6) if thrust is None else thrust
    return _data.Case.from_arrays("case", time, data, thrust, np.zeros(n),
                                  row={"architecture": architecture}, **channels)


class TestNumberFormat:

    def test_thousands_are_grouped_with_a_thin_space(self):
        assert tables.num(1125000.0, 0) == r"$1\,125\,000$"
        assert tables.num(1446.83) == r"$1\,446.8$"
        assert tables.num(518.0, 0) == "$518$"

    def test_a_sign_stays_outside_the_groups(self):
        assert tables.num(846.7, 0, signed=True) == "$+847$"
        assert tables.num(-3090.6, 0, signed=True) == r"$-3\,091$"
        assert tables.num(-0.11, 2, signed=True) == "$-0.11$"

    def test_a_value_that_rounds_to_zero_has_no_sign(self):
        assert tables.num(-0.028) == "$0.0$"
        assert tables.num(-1e-12, 3, signed=True) == "$0.000$"

    def test_a_missing_value_is_a_dash_not_a_zero(self):
        assert tables.num(None) == tables.DASH
        assert tables.num(float("nan")) == tables.DASH

    def test_durations_under_a_second_keep_the_milliseconds(self):
        assert tables.seconds(0.025) == "$0.025$"
        assert tables.seconds(257.327) == "$257.3$"

    def test_hours(self):
        assert tables.hours(1.8) == "$<0.001$"
        assert tables.hours(60.9) == "$0.017$"
        assert tables.hours(45406.9) == "$12.6$"


class TestGammaP:

    @pytest.mark.parametrize("architecture, x, expected", [
        ("pso_coast", [422.9, 77.7, 92.3, 1.5442], 1.5442),
        ("pso_coast", [724.5, 77.5, 98.7, 1.5441, 0.22, -0.45], 1.5441),
        ("indirect_pmp", [0.0, -0.006, -1.0, 1446.8, 76.0, 99.99, 1.5371], 1.5371),
        ("direct", [1.5529], 1.5529),
        ("segmented", [1272.6, 1.5377], 1.5377),
        ("segmented", [1269.2, 1.5365, 0.372], 1.5365),
        ("segmented", [400.0, 80.0, 90.0, 1.5450, 0.4], 1.5450),
    ])
    def test_each_architecture_reads_its_own_slot(self, architecture, x, expected):
        assert _case(architecture, decision_vector=x).gamma_p == pytest.approx(expected)

    def test_the_apogee_check_stores_the_kick_not_the_angle(self):
        case = _case("apogee_check", decision_vector=[-0.029])
        assert case.gamma_p == pytest.approx(np.pi / 2.0 - 0.029)

    def test_a_reference_track_case_reads_the_schedule_it_flew(self):
        case = _case("reference_track", decision_vector=[0.0] * 6 + [1.0],
                     realised_schedule=[1446.8, 76.4, 99.4, 1.5371])
        assert case.gamma_p == pytest.approx(1.5371)

    def test_no_decision_vector_is_none(self):
        assert _case("pso_coast").gamma_p is None


class TestArcs:

    def _flight(self, coast_s):
        """Stage 1 to 20 s, staging to 28 s, a first burn, a coast, a final burn."""
        time = np.round(np.arange(0.0, 120.0 + coast_s, 0.05), 6)
        on = ((time <= 20.0) | ((time >= 28.0) & (time <= 60.0))
              | ((time >= 60.0 + coast_s) & (time <= 70.0 + coast_s)))
        return _case("pso_coast", time=time, thrust=np.where(on, 1e6, 0.0),
                     t_meco=20.0, t_seco=70.0 + coast_s)

    def test_a_half_second_coast_is_kept(self):
        # Case.coast_intervals drops anything under 5 s; the swarm left PEG 0.5 s.
        first, coast, final = tables.arcs(self._flight(0.5))
        assert first == pytest.approx(32.0, abs=0.1)
        assert coast == pytest.approx(0.5, abs=0.1)
        assert final == pytest.approx(10.0, abs=0.1)

    def test_staging_is_not_a_coast(self):
        assert tables.arcs(self._flight(300.0))[1] == pytest.approx(300.0, abs=0.1)

    def test_one_burn_has_no_coast_and_no_final_burn(self):
        time = np.round(np.arange(0.0, 100.0, 0.05), 6)
        on = (time <= 20.0) | ((time >= 28.0) & (time <= 60.0))
        case = _case("direct", time=time, thrust=np.where(on, 1e6, 0.0),
                     t_meco=20.0, t_seco=60.0)
        first, coast, final = tables.arcs(case)
        assert first == pytest.approx(32.0, abs=0.1) and coast is None and final is None


def test_waypoint_miss_is_achieved_minus_target():
    case = _case("reference_track",
                 arc1_target=[6542696.96, 7590.51395, 0.0490845592],
                 arc1_achieved=[6542725.99, 7590.40542, 0.0498711005, 26151.1])
    dh, dv, dgamma = case.waypoint_miss()
    assert dh == pytest.approx(29.03, abs=0.01)
    assert dv == pytest.approx(-0.1085, abs=1e-4)
    assert dgamma == pytest.approx(0.04507, abs=1e-4)
    assert _case("pso_coast").waypoint_miss() is None
