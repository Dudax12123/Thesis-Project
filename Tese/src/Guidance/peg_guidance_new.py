"""
PEG_new — Analytical Predictor-Corrector from First Principles

Derives the Powered Explicit Guidance steering law from Pontryagin's minimum
principle and Jaggers' "Coke Machine" orthogonality assumption (Jagger 1977).
The primary variable is v_go (2D velocity-to-be-gained vector).  Time-to-go
is obtained directly from v_go via the rocket equation, avoiding the T-estimation
2-point cycle of the Orbiter Wiki formulation.

The resulting steering law (first-order, free-downrange 2D case) is:

    û(t) = v_go/L₀  +  λ'_r·(t − t_λ)·r̂     [paper eq 72, linearised]

which is identical in form to sin(pitch) = A + B·t but derived physically.

References
----------
Sagliano, M., Mooij, E., & Theil, S. PEG Derivation from First Principles.
    Eqs (61)–(72) and Algorithm 1 (analytical predictor-corrector variant).

Jaggers, R. F. (1977). An explicit solution to the exoatmospheric powered
    flight guidance and trajectory optimisation problem for rocket propelled
    vehicles. AIAA Paper 77-1051.

McHenry, R. L., Brand, T. J., Long, A. D., Cockrell, B. F., & Thibodeau, J. R.
    (1979). Space Shuttle Ascent Guidance, Navigation and Control.
    Journal of the Astronautical Sciences, 27(1), 1–38.
"""

import numpy as np


def compute_vgo(v_r, v_theta, r, m, r_T, mu, ve, F_T, max_iter=15, tol=0.01):
    """Converge v_go and t_go with constant-gravity radial correction.

    v_go_θ = v_θ_T − v_θ  (tangential — no tangential gravity, constant)
    v_go_r is iterated until the gravity velocity integral converges.

    Parameters
    ----------
    v_r, v_theta : float
        Current radial and tangential velocity [m/s]
    r : float
        Current radius from Earth centre [m]
    m : float
        Current wet mass [kg]
    r_T : float
        Target radius [m]
    mu : float
        Gravitational parameter [m³/s²]
    ve : float
        Exhaust speed = Isp·g₀ [m/s]
    F_T : float
        Thrust magnitude [N]

    Returns
    -------
    vgo_r, vgo_theta, L0, t_go : float
    """
    tau = m * ve / F_T                          # vehicle time constant τ
    v_T = np.sqrt(mu / r_T)                     # target circular orbit speed

    # Net radial gravity (gravity − centrifugal), negative for sub-orbital
    g_r = -mu / r**2 + v_theta**2 / r

    vgo_theta = v_T - v_theta                   # tangential gap (constant)
    vgo_r = -v_r                                # initial guess: no gravity correction

    for _ in range(max_iter):
        L0 = np.sqrt(vgo_r**2 + vgo_theta**2)
        L0 = max(L0, 1.0)                       # guard against zero
        t_go = tau * (1.0 - np.exp(-L0 / ve))

        vG_r = g_r * t_go                       # gravity velocity integral (constant-g)
        vgo_r_new = -v_r - vG_r                 # = (v_r_T=0) − v_r − vG_r

        if abs(vgo_r_new - vgo_r) < tol:
            vgo_r = vgo_r_new
            break
        vgo_r = vgo_r_new

    L0 = np.sqrt(vgo_r**2 + vgo_theta**2)
    L0 = max(L0, 1.0)
    t_go = tau * (1.0 - np.exp(-L0 / ve))
    return float(vgo_r), float(vgo_theta), float(L0), float(t_go)


def compute_thrust_integrals(L0, t_go, tau, ve):
    """Constant-thrust integrals S₀, L₁, S₁ and reference time t_λ.

    Paper eqs 66–69 (constant thrust magnitude):
        L₀ = ‖v_go‖  (passed in)
        S₀ = −L₀(τ − t_go) + c·t_go
        L₁ = L₀·t_go − S₀
        S₁ = S₀·t_go − c·t_go²/2
        t_λ = L₁/L₀   (reference time, paper eq 64)

    Parameters
    ----------
    L0, t_go, tau, ve : float

    Returns
    -------
    S0, L1, S1, t_lambda : float
    """
    S0 = -L0 * (tau - t_go) + ve * t_go
    L1 = L0 * t_go - S0
    S1 = S0 * t_go - ve * t_go**2 / 2.0
    t_lambda = L1 / L0 if L0 > 1e-6 else 0.0
    return float(S0), float(L1), float(S1), float(t_lambda)


def compute_lambda_r(r, v_r, r_T, vgo_r, L0, t_go, t_lambda, S0, S1, g_r):
    """Radial position costate from paper eq 71 (free-downrange projection).

    For the free-downrange case the tangential component of λ'_r is zero, so
    only the radial component remains:

        λ'_r = (L₀·rgo_r − S₀·vgo_r) / (L₀·(S₁ − t_λ·S₀))

    where:
        rgo_r = (r_T − r) − v_r·t_go − ½·g_r·t_go²

    Parameters
    ----------
    r, v_r, r_T : float   current radius, radial velocity, target radius [m, m/s, m]
    vgo_r, L0   : float   radial v_go component and its magnitude [m/s]
    t_go, t_lambda, S0, S1 : float   thrust-integral quantities
    g_r         : float   net radial gravity acceleration [m/s²]

    Returns
    -------
    lambda_r_prime : float  [1/s]
    """
    rG_r  = 0.5 * g_r * t_go**2            # gravity position integral (constant-g)
    rgo_r = (r_T - r) - v_r * t_go - rG_r  # radial position to be gained

    denom = L0 * (S1 - t_lambda * S0)
    if abs(denom) < 1e-6:
        return 0.0
    return float((L0 * rgo_r - S0 * vgo_r) / denom)


def peg_new_major_loop(state, r_T, mu, ve, F_T):
    """Full major-loop step: compute v_go, thrust integrals, and λ'_r.

    Parameters
    ----------
    state : array-like [s, r, v, gamma, m, ...]
    r_T   : float   target radius [m]
    mu    : float   gravitational parameter [m³/s²]
    ve    : float   exhaust speed [m/s]
    F_T   : float   thrust [N]

    Returns
    -------
    vgo_r, vgo_theta, L0, t_go, t_lambda, lambda_r_prime : float
    """
    r     = float(state[1])
    v     = float(state[2])
    gamma = float(state[3])
    m     = float(state[4])

    v_r     = v * np.sin(gamma)
    v_theta = v * np.cos(gamma)
    tau     = m * ve / F_T

    # Net radial gravity (gravity − centrifugal)
    g_r = -mu / r**2 + v_theta**2 / r

    vgo_r, vgo_theta, L0, t_go = compute_vgo(
        v_r, v_theta, r, m, r_T, mu, ve, F_T)

    S0, L1, S1, t_lambda = compute_thrust_integrals(L0, t_go, tau, ve)

    lambda_r_prime = compute_lambda_r(
        r, v_r, r_T, vgo_r, L0, t_go, t_lambda, S0, S1, g_r)

    return vgo_r, vgo_theta, L0, t_go, t_lambda, lambda_r_prime


def peg_new_alpha(t_since_epoch, vgo_r, vgo_theta, L0, lambda_r_prime, t_lambda, gamma):
    """Minor loop: steering angle α from the analytical PEG thrust direction.

    From paper eq 72 (exact, normalised):

        û(t) = [v_go/L₀  +  λ'_r·(t − t_λ)·r̂] / ‖…‖

    with the convention that:
        u_r = sin β  (radial/vertical component of thrust unit vector)
        u_θ = cos β  (tangential/horizontal component)
        α   = β − γ  (angle of attack = pitch minus flight-path angle)

    Parameters
    ----------
    t_since_epoch : float   seconds since last major-loop update
    vgo_r, vgo_theta, L0 : float   v_go components and magnitude [m/s]
    lambda_r_prime : float   radial position costate [1/s]
    t_lambda : float   reference time t_λ [s]
    gamma : float   current flight-path angle [rad]

    Returns
    -------
    alpha : float   angle of attack [rad]
    """
    t_rel = t_since_epoch - t_lambda       # time relative to reference epoch t_λ

    u_r     = vgo_r / L0 + lambda_r_prime * t_rel
    u_theta = vgo_theta / L0

    # Normalise (exact formula, paper eq 72)
    mag = np.sqrt(u_r**2 + u_theta**2)
    if mag > 1e-10:
        u_r     /= mag
        u_theta /= mag

    # Pitch angle β (from local horizontal); α = β − γ
    beta  = np.arctan2(u_r, abs(u_theta))
    return float(beta - gamma)
