"""
CFPAR — Constant Flight-Path-Angle Rate Guidance

Commands the flight-path angle to change at a constant rate:
    dgamma/dt = gamma_dot_cmd

Uses the inverse-dynamics formula: solves the flight-path-angle EOM directly for
the angle of attack that produces exactly dgamma/dt = gamma_dot_cmd.

    sin(alpha) = [gamma_dot_cmd * v + (g - v^2/r) * cos(gamma)] / (F_T/m)
    alpha = arcsin(...)

At gamma = 90° the gravity/centripetal term vanishes, leaving:
    sin(alpha) = gamma_dot_cmd * v / (F_T/m)   (non-zero immediately)

gamma0 = 90 deg (vertical), gamma_final = 0 deg (horizontal).
gamma_dot is derived from the guidance duration: gamma_dot = -pi/2 / duration.
"""

import numpy as np


def alpha_cfpar(gamma_dot_cmd, v, F_T, m, a_grav, r_val, gamma, eps=1e-9):
    """
    Inverse-dynamics CFPAR: angle of attack that makes dgamma/dt = gamma_dot_cmd.

    Parameters
    ----------
    gamma_dot_cmd : float
        Commanded flight-path-angle rate [rad/s]
    v : float
        Current velocity [m/s]
    F_T : float
        Current thrust [N]
    m : float
        Current mass [kg]
    a_grav : float
        Local gravitational acceleration magnitude [m/s²]
    r_val : float
        Current radius from Earth centre [m]
    gamma : float
        Current flight-path angle [rad]

    Returns
    -------
    float
        Commanded angle of attack [rad]; 0 when thrust is unavailable.
    """
    if F_T < eps or m < eps or v < eps:
        return 0.0
    a_star = F_T / m
    sin_alpha = (gamma_dot_cmd * v + (a_grav - v**2 / r_val) * np.cos(gamma)) / a_star
    sin_alpha = np.clip(sin_alpha, -1.0, 1.0)
    return np.arcsin(sin_alpha)
