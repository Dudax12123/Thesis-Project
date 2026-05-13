"""
Powered Explicit Guidance (PEG)

Closed-loop orbital-insertion guidance originally developed for the Saturn V.
Maintains a linear pitch program  sin(pitch[t]) = A + B*t  and updates the
steering constants A, B and burn-time estimate T every major-loop cycle to
drive the predicted burnout state to the target orbit.

Reference: https://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance
"""

import numpy as np


def compute_peg_integrals(T, v_e, tau):
    """Compute PEG rocket-equation integrals b_0, b_1, c_0, c_1 (eqs 7a-7d).

    Parameters
    ----------
    T : float
        Burn time remaining [s]
    v_e : float
        Effective exhaust velocity = Isp * g_0 [m/s]
    tau : float
        v_e / a_0 — time to burn the vehicle completely [s]

    Returns
    -------
    (b0, b1, c0, c1) : tuple of float
    """
    T = min(T, tau * 0.9999)
    b0 = -v_e * np.log(1.0 - T / tau)
    b1 = b0 * tau - v_e * T
    c0 = b0 * T - b1
    c1 = c0 * tau - v_e * T ** 2 / 2.0
    return b0, b1, c0, c1


def compute_peg_AB(state, T, v_e, F_T, r_T, r_dot_T=0.0):
    """Guide step: solve M_A·[A,B]ᵀ = M_B for steering constants.

    Parameters
    ----------
    state : array-like [s, r, v, gamma, m, ...]
    T : float
        Current burn-time estimate [s]
    v_e : float
        Effective exhaust velocity [m/s]
    F_T : float
        Current thrust [N]
    r_T : float
        Target radius (R_Earth + target altitude) [m]
    r_dot_T : float, optional
        Target radial velocity at burnout [m/s] — 0 for circular orbit

    Returns
    -------
    (A, B) : tuple of float
        Steering constants for sin(pitch[t]) = A + B*t
    """
    r     = float(state[1])
    v     = float(state[2])
    gamma = float(state[3])
    m     = float(state[4])

    a0  = F_T / m
    tau = v_e / a0
    T   = min(max(T, 0.1), tau * 0.9999)

    r_dot = v * np.sin(gamma)
    b0, b1, c0, c1 = compute_peg_integrals(T, v_e, tau)

    MB = np.array([r_dot_T - r_dot,
                   r_T - r - r_dot * T])
    # det = b0*c1 - b1*c0
    det = b0 * c1 - b1 * c0
    if abs(det) < 1e-10:
        return 0.0, 0.0

    A = (c1 * MB[0] - b1 * MB[1]) / det
    B = (b0 * MB[1] - c0 * MB[0]) / det
    return float(A), float(B)


def estimate_peg_T(A, B, T, state, v_e, F_T, r_T, mu):
    """Estimate step: update burn-time T from the angular-momentum gap.

    Parameters
    ----------
    A, B : float
        Current steering constants
    T : float
        Current burn-time estimate [s]
    state : array-like [s, r, v, gamma, m, ...]
    v_e : float
        Effective exhaust velocity [m/s]
    F_T : float
        Current thrust [N]
    r_T : float
        Target radius [m]
    mu : float
        Gravitational parameter [m³/s²]

    Returns
    -------
    T_new : float
        Updated burn-time estimate [s]
    """
    r     = float(state[1])
    v     = float(state[2])
    gamma = float(state[3])
    m     = float(state[4])

    a0  = F_T / m
    tau = v_e / a0
    T   = min(max(T, 0.1), tau * 0.9999)

    v_theta = v * np.cos(gamma)
    h       = r * v_theta
    h_T     = np.sqrt(mu * r_T)
    delta_h = h_T - h
    r_bar   = (r_T + r) / 2.0

    # Pitch-program derivatives at t=0
    f_r      = A
    f_r_dot  = B                        # ḟ_r = (A+B*T - A)/T = B

    f_theta      = 1.0 - f_r ** 2 / 2.0
    f_theta_dot  = -(f_r * f_r_dot)
    f_theta_ddot = -(f_r_dot ** 2) / 2.0

    num = delta_h / r_bar + v_e * T * (f_theta_dot + f_theta_ddot * T)
    den = f_theta + f_theta_dot * tau + f_theta_ddot * tau ** 2

    if abs(den) < 1e-6 or num <= 0.0:
        return T

    delta_v = num / den
    if delta_v <= 0.0:
        return T

    T_new = tau * (1.0 - np.exp(-delta_v / v_e))
    return float(min(max(T_new, 0.1), tau * 0.9999))


def converge_peg(state, T_init, v_e, F_T, r_T, mu,
                 max_iter=30, tol=0.5, damping=0.5):
    """Damped Guide+Estimate iteration until T converges.

    Damping prevents the 2-point cycle that arises with undamped iteration
    when the rocket is far from the target orbit (early Stage-2 conditions).
    Each step: T_next = damping*T_est + (1-damping)*T_current
    """
    m   = float(state[4])
    a0  = F_T / m
    tau = v_e / a0
    T   = float(np.clip(T_init, 0.1, tau * 0.9999))
    A, B = 0.0, 0.0

    for _ in range(max_iter):
        A_new, B_new = compute_peg_AB(state, T, v_e, F_T, r_T)
        T_est = estimate_peg_T(A_new, B_new, T, state, v_e, F_T, r_T, mu)
        T_next = float(np.clip(damping * T_est + (1.0 - damping) * T,
                               0.1, tau * 0.9999))
        A, B = A_new, B_new
        if abs(T_next - T) < tol:
            T = T_next
            break
        T = T_next

    return A, B, T


def peg_alpha(t_since_epoch, A, B, gamma):
    """Minor loop: compute steering angle α from the PEG pitch program.

    Parameters
    ----------
    t_since_epoch : float
        Time elapsed since last major-loop update [s]
    A, B : float
        Current steering constants
    gamma : float
        Current flight-path angle [rad]

    Returns
    -------
    alpha : float
        Angle of attack (thrust vs velocity) [rad]
    """
    sin_pitch = float(np.clip(A + B * t_since_epoch, -1.0, 1.0))
    pitch = np.arcsin(sin_pitch)
    return pitch - gamma
