"""
Unit tests for estimate_apollo_time_to_go in apollo_guidance.py.
"""

import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Guidance.apollo_guidance import estimate_apollo_time_to_go


class TestApolloTgo:
    """Tests for estimate_apollo_time_to_go."""

    # ------------------------------------------------------------------
    # Test 1: VG_vec = [0, 0]  →  t_go = min_tgo (0.0 by default)
    # ------------------------------------------------------------------
    def test_zero_vg_returns_zero(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([0.0, 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=100.0,
        )
        assert t_go == 0.0

    def test_zero_vg_respects_custom_min_tgo(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([0.0, 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=100.0,
            min_tgo=0.5,
        )
        assert t_go == pytest.approx(0.5)

    # ------------------------------------------------------------------
    # Test 2: VG=300, Ve=3000, T_BUP=100  →  x=0.1, t_go=9.5 s
    # ------------------------------------------------------------------
    def test_nominal_case(self):
        """Matches the reference calculation in the requirement."""
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=100.0,
        )
        # x = 0.1  →  t_go = 100 * 0.1 * (1 - 0.5*0.1) = 9.5
        assert t_go == pytest.approx(9.5, rel=1e-9)

    def test_nominal_case_2d_vg(self):
        """Same result regardless of VG_vec orientation."""
        # ||[VG/sqrt(2), VG/sqrt(2)]|| == VG = 300
        VG = 300.0
        vg = np.array([VG / np.sqrt(2), VG / np.sqrt(2)])
        t_go = estimate_apollo_time_to_go(vg, 3000.0, 100.0)
        assert t_go == pytest.approx(9.5, rel=1e-6)

    # ------------------------------------------------------------------
    # Test 3: Small VG/Ve  →  result ≈ T_BUP * VG / Ve (first order)
    # ------------------------------------------------------------------
    def test_small_ratio_first_order_approx(self):
        """For VG/Ve << 1 the second-order correction is negligible (<1%)."""
        VG = 10.0
        Ve = 3000.0
        T_BUP = 200.0
        first_order = T_BUP * VG / Ve   # = 0.6667 s
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([VG, 0.0]),
            effective_exhaust_velocity=Ve,
            time_to_burnout=T_BUP,
        )
        # Second-order term ~ T_BUP*(VG/Ve)^2/2 ~ 0.0011 s  →  < 0.2% error
        assert abs(t_go - first_order) / first_order < 0.01

    # ------------------------------------------------------------------
    # Test 4: Invalid inputs — must not crash and must return sensibly
    # ------------------------------------------------------------------
    def test_zero_ve_with_previous_returns_fallback(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=0.0,
            time_to_burnout=100.0,
            previous_tgo=50.0,
        )
        assert t_go == pytest.approx(50.0)

    def test_zero_ve_no_previous_returns_zero(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=0.0,
            time_to_burnout=100.0,
        )
        assert t_go == 0.0

    def test_negative_ve_returns_fallback(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=-100.0,
            time_to_burnout=100.0,
            previous_tgo=20.0,
        )
        assert t_go == pytest.approx(20.0)

    def test_zero_tbup_returns_fallback(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=0.0,
            previous_tgo=30.0,
        )
        assert t_go == pytest.approx(30.0)

    def test_negative_tbup_no_previous_returns_zero(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=-5.0,
        )
        assert t_go == 0.0

    def test_nan_ve_returns_fallback(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=float("nan"),
            time_to_burnout=100.0,
            previous_tgo=20.0,
        )
        assert t_go == pytest.approx(20.0)

    def test_inf_ve_returns_fallback(self):
        """inf fails np.isfinite so returns fallback."""
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=float("inf"),
            time_to_burnout=100.0,
            previous_tgo=15.0,
        )
        assert t_go == pytest.approx(15.0)

    def test_nan_in_vg_vec_returns_fallback(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([float("nan"), 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=100.0,
            previous_tgo=12.0,
        )
        assert t_go == pytest.approx(12.0)

    def test_inf_in_vg_vec_returns_fallback(self):
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([float("inf"), 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=100.0,
            previous_tgo=8.0,
        )
        assert t_go == pytest.approx(8.0)

    def test_vg_greater_than_ve_clamps_with_warning(self):
        """x > 1 emits RuntimeWarning and clamps x to 1 → t_go = T_BUP * 0.5."""
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            t_go = estimate_apollo_time_to_go(
                velocity_to_be_gained_vector=np.array([4000.0, 0.0]),
                effective_exhaust_velocity=3000.0,
                time_to_burnout=100.0,
            )
        assert any(issubclass(w.category, RuntimeWarning) for w in caught)
        # x clamped to 1  →  100 * 1 * (1 - 0.5) = 50
        assert t_go == pytest.approx(50.0)

    # ------------------------------------------------------------------
    # Clamping: max_tgo and min_tgo
    # ------------------------------------------------------------------
    def test_max_tgo_clamps_result(self):
        # Natural result = 9.5 s; cap at 5 s
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([300.0, 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=100.0,
            max_tgo=5.0,
        )
        assert t_go == pytest.approx(5.0)

    def test_min_tgo_raises_result(self):
        # VG = 1 m/s → tiny t_go; min_tgo = 5 s should win
        t_go = estimate_apollo_time_to_go(
            velocity_to_be_gained_vector=np.array([1.0, 0.0]),
            effective_exhaust_velocity=3000.0,
            time_to_burnout=100.0,
            min_tgo=5.0,
        )
        assert t_go == pytest.approx(5.0)
