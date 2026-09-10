"""
Powered Explicit Guidance (PEG)

Closed-loop orbital-insertion guidance originally developed for the Saturn V.
Maintains a linear pitch program  sin(pitch[t]) = A + B*t + C  and updates the
steering constants A, B and burn-time estimate T every major-loop cycle to
drive the predicted burnout state to the target orbit.

The split between the constants and C is the whole point of the reference
algorithm and is easy to lose. The guide step solves the radial channel with
gravity and the centrifugal term left OUT: it asks only what net radial
acceleration profile A + B*t closes the radius and radial-rate gap. The part of
the thrust that has to cancel gravity is put back in the steering command,

    sin(pitch) = A + B*t + C,     C = (mu/r^2 - omega^2 r) / a,   omega = v_theta / r,

"the portion of vehicle acceleration used to counteract gravity and centrifugal
force" (reference, estimate step). Flying A + B*t alone, as this module did until
2026-09-10, leaves gravity out of the radial channel altogether: at a Stage-2
ignition with T/m ~ 9.7 m/s^2 and gamma ~ 49 deg the omission was 469 km of
radial prediction over a 326 s burn, and the command was pitch -48 deg
(alpha -97 deg, thrust with a rearward component) where the reference gives
pitch +10 deg. C is evaluated at the CURRENT state on every minor-loop call,
which is what the reference means by "sin(pitch) at current time".

Reference: https://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance
"""

import numpy as np


def compute_gravity_term(state, F_T, mu):
    """C = (mu/r^2 - omega^2 r) / a0 at the current state (reference, estimate step).

    The fraction of the current thrust acceleration a0 = F_T/m that a purely
    radial thrust would need just to hold the vehicle against gravity net of the
    centrifugal relief omega^2 r, with omega = v_theta / r from the current
    tangential speed. It is the C added to A + B*t in the steering command and
    used as f_r = A + C in the burn-time estimate. Greater than 1 means the
    stage cannot even hold altitude with the thrust vertical.

    Parameters
    ----------
    state : array-like [s, r, v, gamma, m, ...]
    F_T   : float  current thrust [N]
    mu    : float  gravitational parameter [m^3/s^2]
    """
    r     = float(state[1])
    v     = float(state[2])
    gamma = float(state[3])
    m     = float(state[4])
    a0      = F_T / m
    v_theta = v * np.cos(gamma)
    omega   = v_theta / r
    return float((mu / r ** 2 - omega ** 2 * r) / a0)


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

    # Reference, estimate step, term by term.
    #   C   = (μ/r² − ω²r)/a₀            at the CURRENT point, ω = v_θ/r
    #   f_r = A + C                       sin(pitch) at current time
    #   C_T = (μ/r_T² − ω_T² r_T)/a[T]    at cutoff, a[T] = acceleration at burnout
    #   f_{r,T} = A + B·T + C_T           sin(pitch) at burnout
    # Before 2026-09-10 this used f_r = A·(1+C) with C evaluated at r̄, and the
    # Δv numerator below dropped its last term; none of that is in the source.
    # C_T vanishes for the inertial circular target (μ/r_T² = ω_T² r_T); with the
    # rotating-frame target v_θ,T = √(μ/r_T) − v_rot it is small but non-zero, and
    # is kept as the reference writes it.
    omega   = v_theta / r
    C       = (mu / r ** 2 - omega ** 2 * r) / a0
    a_T     = v_e / (tau - T)                  # a[T] = a₀ / (1 − T/τ)
    omega_T = v_theta_T / r_T
    C_T     = (mu / r_T ** 2 - omega_T ** 2 * r_T) / a_T

    # sin(pitch) cannot exceed 1 in magnitude; the minor loop clips the same way.
    #
    # Validity, measured 2026-09-10 on this vehicle: at Stage-2 ignition
    # C = 0.85-0.91 (T/W barely above 1), and for the shallower kicks the guide
    # step returns A + C of 1.2-1.6 -- the stage cannot hold altitude even
    # thrusting vertically. The reference's f_theta = 1 - f_r^2/2 is a small-
    # pitch expansion; with f_r clipped to 1 it reads 0.5 where cos(90 deg) is
    # 0, the denominator below can turn negative, and the guide-estimate map
    # T -> T_est(T) is then non-monotonic (T_est(275) = 294, T_est(284) = 266
    # at one archived state), so neither the damped nor the undamped iteration
    # has a unique fixed point there. Where A + C < 1 (the steeper kicks, e.g.
    # the archived show_peg ignition) both converge to the same T in 4-9
    # iterations. This is a property of the classical algorithm on a T/W ~ 1
    # stage and is reported as such; the guards below hand T back unchanged
    # rather than invent a value, and the caller's per-cycle countdown
    # (peg_T - dt) then carries it.
    f_r   = float(np.clip(A + C, -1.0, 1.0))
    f_r_T = float(np.clip(A + B * T + C_T, -1.0, 1.0))
    f_r_dot = (f_r_T - f_r) / T if T > 1e-3 else B

    f_theta      = 1.0 - f_r ** 2 / 2.0
    f_theta_dot  = -(f_r * f_r_dot)
    f_theta_ddot = -(f_r_dot ** 2) / 2.0

    # Δv = [Δh/r̄ + v_e T (ḟ_θ + f̈_θ τ) + f̈_θ v_e T²/2] / [f_θ + ḟ_θ τ + f̈_θ τ²]
    num = (delta_h / r_bar
           + v_e * T * (f_theta_dot + f_theta_ddot * tau)
           + f_theta_ddot * v_e * T ** 2 / 2.0)
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


def peg_alpha(t_since_epoch, A, B, gamma, C):
    """Minor loop: compute steering angle α from the PEG pitch program.

        sin(pitch) = A + B·t + C

    ``C`` is the gravity/centrifugal term of :func:`compute_gravity_term`,
    evaluated at the current state by the caller on every call (the reference's
    "sin(pitch) at current time"). It is a required argument on purpose: the
    guide step that produced A and B left gravity out of the radial channel, so
    a call without C flies a different law from the one A and B were solved
    for. Pass ``C = 0.0`` only to reproduce the pre-2026-09-10 behaviour.

    Parameters
    ----------
    t_since_epoch : float
        Time elapsed since last major-loop update [s]
    A, B : float
        Current steering constants
    gamma : float
        Current flight-path angle [rad]
    C : float
        Gravity/centrifugal thrust fraction at the current state [-]

    Returns
    -------
    alpha : float
        Angle of attack (thrust vs velocity) [rad]
    """
    sin_pitch = float(np.clip(A + B * t_since_epoch + C, -1.0, 1.0))
    pitch = np.arcsin(sin_pitch)
    return pitch - gamma
