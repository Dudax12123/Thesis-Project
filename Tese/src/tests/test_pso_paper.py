"""Unit tests for the PSO paper-mode trajectory machinery.

Covers:
* steering_from_costates degenerate / canonical cases
* costate_derivatives zero-costate sanity, finite-value correctness
* hamiltonian matches a direct hand-computed value

Run from the repository root or from `Tese/src`:
    pytest tests/test_pso_paper.py -v
"""

import sys
from pathlib import Path

# Repo layout: tests/ sits inside Tese/src so add the src dir to sys.path
_SRC = Path(__file__).resolve().parent.parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import numpy as np
import pytest

from Guidance import pso_paper_guidance as g
from Auxiliary import constants as c


# ──────────────────────────────────────────────────────────────────────────────
# steering_from_costates
# ──────────────────────────────────────────────────────────────────────────────

def test_steering_zero_costates_returns_zero():
    """All-zero costates: norm = 0 -> guard returns alpha = 0 (gravity turn)."""
    alpha = g.steering_from_costates(0.0, 0.0, 7000.0)
    assert alpha == 0.0


def test_steering_zero_velocity_returns_zero():
    """Pre-liftoff guard: V <= 1e-6 -> alpha = 0."""
    alpha = g.steering_from_costates(0.5, 0.5, 0.0)
    assert alpha == 0.0


def test_steering_lam_V_negative_lam_g_zero_yields_zero():
    """
    Paper eq. (34b): cos(alpha) = -lam_V / norm.  With (lam_V, lam_g) = (-1, 0)
    the denominator is |lam_V| = 1, so cos(alpha) = 1 -> alpha = 0.
    """
    alpha = g.steering_from_costates(-1.0, 0.0, 7000.0)
    assert pytest.approx(alpha, abs=1e-12) == 0.0


def test_steering_lam_V_positive_lam_g_zero_yields_pi():
    """Symmetric: (lam_V, lam_g) = (+1, 0) gives cos(alpha) = -1 -> alpha = ±π."""
    alpha = g.steering_from_costates(1.0, 0.0, 7000.0)
    assert pytest.approx(abs(alpha), abs=1e-12) == np.pi


def test_steering_lam_V_zero_lam_g_negative_yields_pi_over_2():
    """
    sin(alpha) = -(lam_g/V)/norm, cos(alpha) = 0.  With lam_g = -V (so lam_g/V = -1),
    the ratio is -1 and norm = 1, giving sin(alpha) = +1 -> alpha = +π/2.
    """
    V = 7000.0
    alpha = g.steering_from_costates(0.0, -V, V)
    assert pytest.approx(alpha, abs=1e-12) == np.pi / 2.0


# ──────────────────────────────────────────────────────────────────────────────
# costate_derivatives
# ──────────────────────────────────────────────────────────────────────────────

def test_costate_derivatives_zero_costates_are_zero():
    """All terms in eq. (30) are linear in (lam_h, lam_V, lam_g), so 0 -> 0."""
    r = c.R_EARTH + 200e3
    V = 7800.0
    gamma = 0.05
    alpha = 0.0
    F_T = 0.0
    m = 5000.0
    dlh, dlv, dlg = g.costate_derivatives(r, V, gamma, alpha,
                                          0.0, 0.0, 0.0, F_T, m, c.MU_EARTH)
    assert dlh == 0.0 and dlv == 0.0 and dlg == 0.0


def test_costate_derivatives_velocity_guard():
    """V near zero must short-circuit to (0, 0, 0) instead of dividing."""
    r = c.R_EARTH
    out = g.costate_derivatives(r, 0.0, 0.0, 0.0,
                                 1.0, 1.0, 1.0, 0.0, 1000.0, c.MU_EARTH)
    assert out == (0.0, 0.0, 0.0)


def test_costate_derivatives_match_hand_computation():
    """Compute eq. (30b–d) by direct substitution and compare."""
    r = c.R_EARTH + 100e3        # 100 km altitude
    V = 5000.0
    gamma = 0.1
    alpha = 0.01
    lam_h, lam_V, lam_g = 0.3, -0.5, 0.7
    F_T = 0.0   # post-atmosphere-exit, but check works for any thrust
    m = 8000.0
    mu = c.MU_EARTH

    c_g = np.cos(gamma); s_g = np.sin(gamma); s_a = np.sin(alpha)
    r2 = r * r; r3 = r2 * r; V2 = V * V

    dlh_ref = (V * lam_g * c_g) / r2 \
              - (2.0 * mu * lam_V * s_g + 2.0 * mu * lam_g * c_g / V) / r3
    bracket = c_g * (1.0 / r + mu / (r2 * V2)) - (F_T / m) * s_a / V2
    dlv_ref = -lam_h * s_g - lam_g * bracket
    dlg_ref = (-V * lam_h * c_g
               + mu * lam_V * c_g / r2
               + lam_g * s_g * (V / r - mu / (r2 * V)))

    dlh, dlv, dlg = g.costate_derivatives(r, V, gamma, alpha,
                                          lam_h, lam_V, lam_g, F_T, m, mu)
    assert pytest.approx(dlh, rel=1e-12) == dlh_ref
    assert pytest.approx(dlv, rel=1e-12) == dlv_ref
    assert pytest.approx(dlg, rel=1e-12) == dlg_ref


# ──────────────────────────────────────────────────────────────────────────────
# hamiltonian
# ──────────────────────────────────────────────────────────────────────────────

def test_hamiltonian_zero_velocity_returns_zero():
    """Guard against the V → 0 division before liftoff."""
    H = g.hamiltonian(0.0, 0.0, c.R_EARTH, 0.1, 0.2, 0.3,
                      0.0, 0.0, 1000.0, c.MU_EARTH, c.R_EARTH)
    assert H == 0.0


def test_hamiltonian_matches_direct_dot_product():
    """H = lam · f(state); compare to a manually evaluated EOM dot product."""
    r = c.R_EARTH + 50e3
    V = 4000.0
    gamma = 0.2
    alpha = 0.0
    lam_h, lam_V, lam_g = 0.1, -0.2, 0.3
    F_T = 1e5
    m = 1000.0
    mu = c.MU_EARTH

    g_acc = mu / (r * r)
    dh = V * np.sin(gamma)
    dV = (F_T / m) * np.cos(alpha) - g_acc * np.sin(gamma)
    dgamma = (1.0 / V) * ((F_T / m) * np.sin(alpha)
                          - (g_acc - V * V / r) * np.cos(gamma))
    H_ref = lam_h * dh + lam_V * dV + lam_g * dgamma

    H = g.hamiltonian(V, gamma, r, lam_h, lam_V, lam_g,
                      alpha, F_T, m, mu, c.R_EARTH)
    assert pytest.approx(H, rel=1e-12) == H_ref


# ──────────────────────────────────────────────────────────────────────────────
# Integration smoke test of the steering law inside the simulator
# (does not actually run an ascent — just confirms state-extension math.)
# ──────────────────────────────────────────────────────────────────────────────

def test_paper_costate_offset_helper():
    """rocket_ascent._paper_costate_offset locates costates regardless of
    Earth-rotation/heading flags."""
    from Simulation import rocket_ascent as ra
    from Input_File import simulation_parameters as sp

    saved_er = sp.ENABLE_EARTH_ROTATION
    saved_th = sp.TRACK_HEADING_STATE
    try:
        sp.ENABLE_EARTH_ROTATION = False
        sp.TRACK_HEADING_STATE   = False
        # state = [s, r, v, gamma, m, lam_h, lam_V, lam_g] -> offset 5
        assert ra._paper_costate_offset(8) == 5
        # too short
        assert ra._paper_costate_offset(5) is None

        sp.ENABLE_EARTH_ROTATION = True
        sp.TRACK_HEADING_STATE   = False
        # state = [s, r, v, gamma, m, lat, lam_h, lam_V, lam_g] -> offset 6
        assert ra._paper_costate_offset(9) == 6

        sp.ENABLE_EARTH_ROTATION = True
        sp.TRACK_HEADING_STATE   = True
        # state = [s, r, v, gamma, m, lat, heading, lam_h, lam_V, lam_g] -> offset 7
        assert ra._paper_costate_offset(10) == 7
    finally:
        sp.ENABLE_EARTH_ROTATION = saved_er
        sp.TRACK_HEADING_STATE   = saved_th
