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


def estimate_peg_T(A, B, T, state, v_e, F_T, r_T, mu, v_theta_T=None):
    """Estimate step: update burn-time T from the angular-momentum gap.

    ``v_theta_T`` is the target tangential (horizontal) velocity at burnout;
    defaults to the inertial circular value ``√(μ/r_T)``.  pso_coast passes the
    rotating-frame value ``√(μ/r_T) − v_rot`` so the angular-momentum target
    matches the ground-relative trajectory.

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

    if v_theta_T is None:
        v_theta_T = np.sqrt(mu / r_T)
    v_theta = v * np.cos(gamma)
    h       = r * v_theta
    h_T     = r_T * v_theta_T          # = √(μ·r_T) for the inertial default
    delta_h = h_T - h
    r_bar   = (r_T + r) / 2.0

    # Gravity/centrifugal correction (wiki: C = (μ/r̄²−ω²r̄)/a₀)
    # At the circular target orbit C_T = 0 exactly (μ/r_T² = ω_T²·r_T),
    # so f_{r,T} = A+B·T requires no correction.
    omega = v_theta / r
    C     = (mu / r_bar ** 2 - omega ** 2 * r_bar) / a0

    # Gravity-corrected radial thrust fraction at t=0:
    #   sin(pitch[t=0]) = A  →  f_r = A + C·A = A·(1+C)
    f_r     = A * (1.0 + C)
    f_r_dot = (A + B * T - f_r) / T if T > 1e-3 else B  # (f_{r,T} - f_r) / T

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
                 max_iter=30, tol=0.5, damping=0.5, v_theta_T=None):
    """Damped Guide+Estimate iteration until T converges.

    The undamped Guide→Estimate fixed-point iteration can exhibit a 2-point
    cycle for early Stage-2 conditions (rocket far from target orbit).  The
    fix is **Successive Under-Relaxation (SUR)**:

        T_{n+1} = damping · T_est(T_n)  +  (1−damping) · T_n,   damping ∈ (0,1]

    With damping = 0.5 the 2-point cycle is broken and the sequence converges
    to the fixed point in ≈ 4 steps from a propellant-based seed.

    References
    ----------
    Burden, R. L., & Faires, J. D. (2016). *Numerical Analysis* (10th ed.).
    Cengage Learning. §2.2 (fixed-point iteration convergence) and §7.4
    (successive over/under-relaxation). Establishes that the relaxed iterate
    x_{n+1} = ω·g(x_n) + (1−ω)·x_n converges when the spectral radius of
    the linearised iteration is < 1, even when the undamped iteration diverges.

    McHenry, R. L., Brand, T. J., Long, A. D., Cockrell, B. F., &
    Thibodeau, J. R. (1979). Space Shuttle Ascent Guidance, Navigation and
    Control. *Journal of the Astronautical Sciences*, 27(1), 1–38.
    Original PEG description; §4 iterates Guide+Estimate to convergence
    (not a fixed count) at each major cycle.

    Brand, T. J., Gans, N. R., & Laue, G. H. (1993). *Powered Explicit
    Guidance Improvements and Comparison with PEG4 on the Space Shuttle*.
    NASA JSC. Convergence analysis of the T-update loop.
    """
    m   = float(state[4])
    a0  = F_T / m
    tau = v_e / a0
    T   = float(np.clip(T_init, 0.1, tau * 0.9999))
    A, B = 0.0, 0.0

    for _ in range(max_iter):
        A_new, B_new = compute_peg_AB(state, T, v_e, F_T, r_T)
        T_est = estimate_peg_T(A_new, B_new, T, state, v_e, F_T, r_T, mu,
                               v_theta_T=v_theta_T)
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
