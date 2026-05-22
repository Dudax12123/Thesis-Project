"""Indirect optimal-control machinery for the PSO paper-mode trajectory.

Reference
---------
Morgado, Marta, Gil (2022) — "Multistage rocket preliminary design and
trajectory optimization using a multidisciplinary approach", Structural and
Multidisciplinary Optimization 65:192, https://doi.org/10.1007/s00158-022-03285-y

This module provides:

* costate_derivatives(...) — paper eq. (30), re-indexed onto the simulator's
  (s, r, v, gamma, m) state where the paper's altitude h ≡ r - R_E.
* steering_from_costates(...) — paper eq. (34): closed-form alpha from
  Pontryagin's Minimum Principle, given (lam_V, lam_gamma, V).
* hamiltonian(...) — paper eq. (28), used by the PSO objective to evaluate
  the transversality residual (paper eq. 38).

Scope of validity
-----------------
The paper's costate equations drop the aerodynamic-force partials
(∂F_D/∂h, ∂F_D/∂V, ∂F_L/∂h, ∂F_L/∂V) because the indirect method is applied
only AFTER fairing jettison (post-atmosphere-exit), where rho → 0 implies
F_D, F_L → 0 and the omission is exact.  The simulator's in-atmosphere
phase (Stage 1 + early Stage 2) keeps its full drag/lift physics; costates
are simply frozen (dlam/dt = 0) until guidance activates.

The paper also neglects Coriolis and centrifugal pseudo-forces (sec 3.2.2),
so paper-mode internally forces INCLUDE_PSEUDO_FORCES = False during the PSO
run.  Earth rotation may stay enabled — only the pseudo-forces are skipped.
"""

import numpy as np


# Numerical safety guard for the steering formula when |λ| → 0 (no information
# in the costates; commanded angle of attack is undefined).  Falling back to
# alpha = 0 (gravity turn) is the safe default and is consistent with the
# paper's stated initial guess strategy.
_LAMBDA_NORM_EPSILON = 1.0e-12


def costate_derivatives(r_val, V, gamma, alpha, lam_h, lam_V, lam_gamma,
                        F_T, m, mu_earth):
    """Compute (dlam_h/dt, dlam_V/dt, dlam_gamma/dt) per paper eq. (30).

    Paper eq. (30a) (lam_x_dot = 0) is omitted: lam_s drops out of the
    steering law and is not needed.

    The substitution (R_E + h) -> r_val is exact since the simulator's
    state uses geocentric radius directly.

    Parameters
    ----------
    r_val   : float, geocentric radius [m]
    V       : float, velocity magnitude [m/s]
    gamma   : float, flight path angle [rad]
    alpha   : float, current angle of attack [rad]
    lam_h, lam_V, lam_gamma : floats, current costates
    F_T     : float, current thrust [N]
    m       : float, current vehicle mass [kg]
    mu_earth: float, gravitational parameter [m^3/s^2]

    Returns
    -------
    (dlam_h, dlam_V, dlam_gamma) : tuple of floats
        Time derivatives of the three integrated costates.
    """
    c_g = np.cos(gamma)
    s_g = np.sin(gamma)
    s_a = np.sin(alpha)

    # Guard against V == 0 (only happens at the moment of liftoff;
    # paper-mode never integrates costates that early but keep it safe).
    if V <= 1.0e-6:
        return 0.0, 0.0, 0.0

    r2 = r_val * r_val
    r3 = r2 * r_val
    V2 = V * V

    # Eq. (30b)
    dlam_h = ((V * lam_gamma * c_g) / r2
              - (2.0 * mu_earth * lam_V * s_g
                 + 2.0 * mu_earth * lam_gamma * c_g / V) / r3)

    # Eq. (30c) — note that the (T/m) * sin(alpha) / V^2 contribution is
    # NEGATIVE in the bracket per the paper sign convention.
    bracket = c_g * (1.0 / r_val + mu_earth / (r2 * V2)) - (F_T / m) * s_a / V2
    dlam_V = -lam_h * s_g - lam_gamma * bracket

    # Eq. (30d)
    dlam_gamma = (-V * lam_h * c_g
                  + mu_earth * lam_V * c_g / r2
                  + lam_gamma * s_g * (V / r_val - mu_earth / (r2 * V)))

    return dlam_h, dlam_V, dlam_gamma


def steering_from_costates(lam_V, lam_gamma, V):
    """Closed-form steering law from Pontryagin's Minimum Principle.

    Paper eq. (34a) + (34b) jointly define alpha via
        sin(alpha) = -(lam_gamma/V) / sqrt((lam_gamma/V)^2 + lam_V^2)
        cos(alpha) = - lam_V        / sqrt((lam_gamma/V)^2 + lam_V^2)
    Since the shared denominator is strictly positive, this is exactly
        alpha = atan2(-lam_gamma/V, -lam_V).

    The epsilon guard (|lam| → 0) is OUR addition — the paper does not
    discuss the degenerate case — and returns alpha = 0 (gravity turn).

    Parameters
    ----------
    lam_V, lam_gamma : floats, current costates
    V                : float, velocity magnitude [m/s]

    Returns
    -------
    alpha : float, commanded angle of attack [rad]
    """
    if V <= 1.0e-6:
        return 0.0
    ratio = lam_gamma / V
    norm = np.sqrt(ratio * ratio + lam_V * lam_V)
    if norm < _LAMBDA_NORM_EPSILON:
        return 0.0
    return float(np.arctan2(-ratio, -lam_V))


def hamiltonian(V, gamma, r_val, lam_h, lam_V, lam_gamma,
                alpha, F_T, m, mu_earth, R_earth):
    """Hamiltonian H from paper eq. (28).

    H = lam_h * (dh/dt) + lam_V * (dV/dt) + lam_gamma * (d gamma/dt)

    Drag and lift are intentionally omitted to match eq. (28) — the
    transversality residual is only meaningful post-atmosphere-exit where
    aerodynamic forces are negligible.  The simulator's pseudo-forces are
    also omitted (forced off during PSO).

    Returns
    -------
    H : float
    """
    if V <= 1.0e-6:
        return 0.0
    g = mu_earth / (r_val * r_val)
    dh_dt = V * np.sin(gamma)
    dV_dt = (F_T / m) * np.cos(alpha) - g * np.sin(gamma)
    dgamma_dt = (1.0 / V) * ((F_T / m) * np.sin(alpha)
                             - (g - V * V / r_val) * np.cos(gamma))
    return lam_h * dh_dt + lam_V * dV_dt + lam_gamma * dgamma_dt
