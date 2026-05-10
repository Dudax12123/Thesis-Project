"""
CPR — Constant Pitch Rate Guidance

Commands the vehicle pitch angle to change at a constant rate:
    dtheta/dt = theta_dot_cmd

Uses the inverse-dynamics formula: solves the flight-path-angle EOM for the
angle of attack that produces exactly dgamma/dt = theta_dot_cmd.

    sin(alpha) = [theta_dot_cmd * v + (g - v^2/r) * cos(gamma)] / (F_T/m)
    alpha = arcsin(...)

At gamma = 90° the gravity/centripetal term vanishes, leaving:
    sin(alpha) = theta_dot_cmd * v / (F_T/m)   (non-zero immediately)

theta0 = 90 deg (vertical), theta_final = 0 deg (horizontal).
theta_dot is derived from the guidance duration: theta_dot = -pi/2 / duration.
"""

import numpy as np


def alpha_cpr(theta_dot_cmd, v, F_T, m, a_grav, r_val, gamma, eps=1e-9):
    """
    Inverse-dynamics CPR: angle of attack that makes dgamma/dt = theta_dot_cmd.

    Parameters
    ----------
    theta_dot_cmd : float
        Commanded pitch rate [rad/s]
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
    sin_alpha = (theta_dot_cmd * v + (a_grav - v**2 / r_val) * np.cos(gamma)) / a_star
    sin_alpha = np.clip(sin_alpha, -1.0, 1.0)
    return np.arcsin(sin_alpha)
