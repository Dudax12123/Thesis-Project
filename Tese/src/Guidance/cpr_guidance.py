"""
Constant Pitch Rate (CPR) Guidance

The pitch angle θ is ramped linearly from θ_initial (vertical, 90°) to
θ_end = 0° (horizontal) over the estimated time-to-go.  The constant pitch
rate θ_dot = (θ_initial − θ_end) / t_go is computed once at guidance start.

At each step: α = θ_cmd − γ
"""

import numpy as np


def cpr_alpha(t, t_start, theta_initial, theta_dot, gamma):
    """
    Return angle of attack for CPR guidance.

    Parameters
    ----------
    t : float
        Current time [s]
    t_start : float
        Time when CPR guidance began [s]
    theta_initial : float
        Pitch angle at guidance start [rad] (typically π/2)
    theta_dot : float
        Constant pitch rate [rad/s] (positive → decreasing θ)
    gamma : float
        Current flight-path angle from EOM [rad]

    Returns
    -------
    float
        Commanded angle of attack α [rad]
    """
    theta_cmd = theta_initial - theta_dot * (t - t_start)
    theta_cmd = max(theta_cmd, 0.0)   # clamp: never pitch below horizontal
    return theta_cmd - gamma
