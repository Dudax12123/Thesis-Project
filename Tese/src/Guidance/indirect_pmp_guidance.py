"""
Indirect Trajectory Optimization — PMP Guidance Module

Implements Pontryagin's Minimum Principle (PMP) guidance equations for the
free-flight phase after fairing jettison, as described in the thesis paper
(Sect. 4.2.1).

The paper's notation uses altitude h, this code uses radius r = R_E + h.
Wherever the paper writes (R_E + h), we substitute r directly.

Drag-free dynamics are assumed for the costate equations — valid for the
high-altitude free-flight phase where aerodynamic forces are negligible.

References:
  Eq. 28  : Hamiltonian definition
  Eq. 30a : dλ_s/dt = 0  →  λ_s = 0 everywhere (also from transversality)
  Eq. 30b : dλ_r/dt
  Eq. 30c : dλ_v/dt
  Eq. 30d : dλ_γ/dt
  Eq. 34  : Optimal angle of attack from PMP
  Eq. 38  : Transversality condition
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from Auxiliary import constants as c


# ---------------------------------------------------------------------------
# PMP optimal control law
# ---------------------------------------------------------------------------

def pmp_control_law(lambda_v, lambda_gamma, v):
    """
    PMP optimal angle of attack α (Eq. 34 of the paper).

    Derived by minimising H w.r.t. α  (∂H/∂α = 0):

        sin α = -(λ_γ/V) / √[(λ_γ/V)² + λ_V²]
        cos α = -λ_V     / √[(λ_γ/V)² + λ_V²]

    Equivalently:  α = atan2(-λ_γ/V, -λ_V)

    Parameters
    ----------
    lambda_v     : float  Velocity costate λ_V
    lambda_gamma : float  FPA costate λ_γ
    v            : float  Velocity magnitude [m/s]

    Returns
    -------
    alpha : float  Optimal angle of attack [rad]
    """
    _EPS = 1e-10

    if abs(v) < _EPS:
        return 0.0

    lg_over_v = lambda_gamma / v
    denom = np.sqrt(lg_over_v ** 2 + lambda_v ** 2)

    if denom < _EPS:
        # Near-singular arc — costates both ≈ 0, set α = 0 (coast-like)
        return 0.0

    sin_alpha = -lg_over_v * denom
    cos_alpha = -lambda_v * denom
    return float(np.arctan2(sin_alpha, cos_alpha))


# ---------------------------------------------------------------------------
# Costate (adjoint) ODEs
# ---------------------------------------------------------------------------

def costate_derivatives(r_val, v, gamma, F_T, m, lam_r, lam_v, lam_g, alpha):
    """
    Adjoint (costate) ODEs from Eqs. 30b–30d of the paper.

    Derived from −(∂H/∂x)^T using drag-free EOM (free-flight phase).
    λ_s = 0 everywhere (Eq. 30a + transversality), so it is not propagated.

    With r = R_E + h, every occurrence of (R_E + h) in the paper becomes r_val.

    Parameters
    ----------
    r_val : float  Radius from Earth's centre [m]  (= R_E + h)
    v     : float  Velocity magnitude [m/s]
    gamma : float  Flight path angle [rad]
    F_T   : float  Thrust force [N]   (0 during coast arcs)
    m     : float  Current vehicle mass [kg]
    lam_r : float  Costate for r  (λ_h in paper notation)
    lam_v : float  Costate for V  (λ_V)
    lam_g : float  Costate for γ  (λ_γ)
    alpha : float  Current angle of attack [rad]

    Returns
    -------
    [dlam_r, dlam_v, dlam_g] : list of float  Costate time-derivatives [1/s]
    """
    mu = c.MU_EARTH
    _EPS = 1e-10

    cg = np.cos(gamma)
    sg = np.sin(gamma)
    sa = np.sin(alpha)

    r2 = r_val ** 2
    r3 = r_val ** 3
    v_safe = max(abs(v), _EPS)
    v2 = v_safe ** 2

    T_over_m = (F_T / m) if m > _EPS else 0.0

    # Eq. 30b  — λ̇_r
    dlam_r = ((v * cg / r2) * lam_g
              - (2.0 * mu / r3) * (lam_v * sg + (lam_g * cg / v_safe)))

    # Eq. 30c  — λ̇_v
    dlam_v = (-lam_r * sg
              - lam_g * (cg * (1.0 / r_val + mu / (r2 * v2))
                         - T_over_m * sa / v2))

    # Eq. 30d  — λ̇_γ
    dlam_g = (-v * lam_r * cg
              + mu * lam_v * cg / r2
              + lam_g * sg * (v / r_val - mu / (r2 * v_safe)))

    return [dlam_r, dlam_v, dlam_g]


# ---------------------------------------------------------------------------
# Hamiltonian
# ---------------------------------------------------------------------------

def compute_hamiltonian(r_val, v, gamma, F_T, m, alpha, lam_r, lam_v, lam_g):
    """
    Hamiltonian H at a given state/costate point (Eq. 28 of the paper).

        H = λ_s·ṡ + λ_r·ṙ + λ_v·V̇ + λ_γ·γ̇

    λ_s = 0, so the ṡ term vanishes.
    Drag-free EOM used for ṙ, V̇, γ̇ (consistent with costate_derivatives).

    Parameters
    ----------
    r_val : float  Radius from Earth's centre [m]
    v     : float  Velocity magnitude [m/s]
    gamma : float  Flight path angle [rad]
    F_T   : float  Thrust force [N]   (0 during coast arcs)
    m     : float  Current vehicle mass [kg]
    alpha : float  Angle of attack [rad]
    lam_r : float  Costate λ_r
    lam_v : float  Costate λ_v
    lam_g : float  Costate λ_γ

    Returns
    -------
    H : float  Hamiltonian value
    """
    mu = c.MU_EARTH
    _EPS = 1e-10

    g_local = mu / r_val ** 2
    T_over_m = (F_T / m) if m > _EPS else 0.0

    cg = np.cos(gamma)
    sg = np.sin(gamma)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    # Drag-free EOM  (F_D = F_L = 0)
    drdt    = v * sg
    dvdt    = T_over_m * ca - g_local * sg
    if abs(v) < _EPS:
        dgammadt = 0.0
    else:
        dgammadt = (1.0 / v) * (T_over_m * sa - (g_local - v ** 2 / r_val) * cg)

    # H = λ_r·ṙ + λ_v·V̇ + λ_γ·γ̇
    return float(lam_r * drdt + lam_v * dvdt + lam_g * dgammadt)
