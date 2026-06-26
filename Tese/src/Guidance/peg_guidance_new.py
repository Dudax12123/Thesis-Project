"""
PEG_new — Analytical Predictor-Corrector from First Principles

Derives the Powered Explicit Guidance steering law from Pontryagin's minimum
principle and Jaggers' "Coke Machine" orthogonality assumption (Jaggers 1977).
The primary variable is v_go (2D velocity-to-be-gained vector).  Time-to-go
is obtained directly from v_go via the rocket equation.

The major-loop implements Algorithm 1 of the reference paper, including the
predictor-corrector steps (steps 15–19).  At first order the velocity miss is
zero by construction (v_miss = v_r + g_r·t_go + v_go_r = 0), so the practical
benefit of the predictor step is refining the gravity integral: instead of using
g_r at the *current* position only, it averages g_r over the predicted trajectory
via a trapezoidal rule (Algorithm 1 step 16).  From 100 km to 500 km the net
radial gravity changes from −6.6 to ≈ 0 m/s²; the averaged value (−3.3 m/s²)
changes the v_go_r estimate by ≈ 720 m/s compared to the uncorrected version,
giving a physically correct pitch-down direction for orbit insertion.

The resulting steering law (first-order, free-downrange 2D) is:

    û(t) = v_go/L₀  +  λ'_r·(t − t_λ)·r̂     [paper eq 72, normalised]

References
----------
Sagliano, M., Mooij, E., & Theil, S. PEG Derivation from First Principles.
    Algorithm 1 and eqs (61)–(72) (analytical predictor-corrector variant).

Jaggers, R. F. (1977). An explicit solution to the exoatmospheric powered
    flight guidance and trajectory optimisation problem for rocket propelled
    vehicles. AIAA Paper 77-1051.

McHenry, R. L., Brand, T. J., Long, A. D., Cockrell, B. F., & Thibodeau, J. R.
    (1979). Space Shuttle Ascent Guidance, Navigation and Control.
    Journal of the Astronautical Sciences, 27(1), 1–38.
"""

import numpy as np


def compute_vgo_with_gr(v_r, v_theta, r, m, r_T, mu, ve, F_T, g_r,
                        max_iter=15, tol=0.01, v_theta_T=None, v_r_T=0.0):
    """Converge v_go and t_go for a given net radial gravity g_r.

    Inner loop of the PEG major cycle (Algorithm 1 steps 4–14).
    v_go_θ = v_θ_T − v_θ (tangential, constant — no tangential gravity term).
    v_go_r is iterated until the gravity velocity integral self-consistently
    determines t_go.

    ``v_r_T`` is the target radial velocity at burnout (default 0.0 ⇒ horizontal
    insertion, γ_T = 0, identical to the original behaviour). Pass
    ``v_r_T = v_wp·sin(γ_wp)`` to aim at a waypoint with non-zero flight-path angle.

    Parameters
    ----------
    v_r, v_theta : float   current radial and tangential velocity [m/s]
    r            : float   current radius from Earth centre [m]
    m            : float   current wet mass [kg]
    r_T          : float   target radius [m]
    mu           : float   gravitational parameter [m³/s²]
    ve           : float   exhaust speed = Isp·g₀ [m/s]
    F_T          : float   thrust magnitude [N]
    g_r          : float   net radial gravity (provided by caller) [m/s²]

    Returns
    -------
    vgo_r, vgo_theta, L0, t_go : float
    """
    tau       = m * ve / F_T
    # Target tangential velocity: inertial √(μ/r_T) by default; pso_coast passes
    # the rotating-frame value √(μ/r_T) − v_rot to match the ground-relative frame.
    v_T       = np.sqrt(mu / r_T) if v_theta_T is None else v_theta_T
    vgo_theta = v_T - v_theta       # tangential deficit (constant)
    vgo_r     = v_r_T - v_r         # initial guess (radial deficit; no gravity correction)

    for _ in range(max_iter):
        L0    = max(np.sqrt(vgo_r**2 + vgo_theta**2), 1.0)
        t_go  = tau * (1.0 - np.exp(-L0 / ve))
        vgo_r_new = (v_r_T - v_r) - g_r * t_go
        if abs(vgo_r_new - vgo_r) < tol:
            vgo_r = vgo_r_new
            break
        vgo_r = vgo_r_new

    L0   = max(np.sqrt(vgo_r**2 + vgo_theta**2), 1.0)
    t_go = tau * (1.0 - np.exp(-L0 / ve))
    return float(vgo_r), float(vgo_theta), float(L0), float(t_go)


def compute_vgo(v_r, v_theta, r, m, r_T, mu, ve, F_T, max_iter=15, tol=0.01,
                v_theta_T=None, v_r_T=0.0):
    """Backward-compatible wrapper: converge v_go using g_r at current position.

    Delegates to compute_vgo_with_gr after computing g_r internally.
    """
    g_r = -mu / r**2 + v_theta**2 / r
    return compute_vgo_with_gr(v_r, v_theta, r, m, r_T, mu, ve, F_T, g_r,
                               max_iter, tol, v_theta_T=v_theta_T, v_r_T=v_r_T)


def compute_thrust_integrals(L0, t_go, tau, ve):
    """Constant-thrust integrals S₀, L₁, S₁ and reference time t_λ.

    Paper eqs 66–69 (constant thrust magnitude):
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
    S0       = -L0 * (tau - t_go) + ve * t_go
    L1       = L0 * t_go - S0
    S1       = S0 * t_go - ve * t_go**2 / 2.0
    t_lambda = L1 / L0 if L0 > 1e-6 else 0.0
    return float(S0), float(L1), float(S1), float(t_lambda)


def compute_lambda_r(r, v_r, r_T, vgo_r, L0, t_go, t_lambda, S0, S1, g_r):
    """Radial position costate from paper eq 71 (free-downrange projection).

    For free downrange the tangential component of λ'_r is zero, leaving only:

        λ'_r = (L₀·rgo_r − S₀·vgo_r) / (L₀·(S₁ − t_λ·S₀))

    where rgo_r = (r_T − r) − v_r·t_go − ½·g_r·t_go²

    Parameters
    ----------
    r, v_r, r_T          : float   current radius, radial velocity, target radius
    vgo_r, L0            : float   radial v_go and its magnitude [m/s]
    t_go, t_lambda, S0, S1 : float thrust-integral quantities
    g_r                  : float   net radial gravity [m/s²]

    Returns
    -------
    lambda_r_prime : float  [1/s]
    """
    rG_r  = 0.5 * g_r * t_go**2
    rgo_r = (r_T - r) - v_r * t_go - rG_r

    denom = L0 * (S1 - t_lambda * S0)
    if abs(denom) < 1e-6:
        return 0.0
    return float((L0 * rgo_r - S0 * vgo_r) / denom)


def peg_new_major_loop(state, r_T, mu, ve, F_T, n_pred_iter=3, v_theta_T=None,
                       v_r_T=0.0):
    """Full major-loop step with predictor-corrector (Algorithm 1 steps 4–20).

    The outer loop refines the gravity integral v_G = g_r·t_go by evaluating
    g_r at the predicted burnout position and averaging with the initial value
    (trapezoidal quadrature, Algorithm 1 step 16).

    Parameters
    ----------
    state       : array-like [s, r, v, gamma, m, ...]
    r_T         : float   target radius [m]
    mu          : float   gravitational parameter [m³/s²]
    ve          : float   exhaust speed [m/s]
    F_T         : float   thrust [N]
    n_pred_iter : int     outer predictor-corrector iterations (default 3)

    Returns
    -------
    vgo_r, vgo_theta, L0, t_go, t_lambda, lambda_r_prime : float
    """
    r       = float(state[1])
    v       = float(state[2])
    gamma   = float(state[3])
    m       = float(state[4])

    v_r     = v * np.sin(gamma)
    v_theta = v * np.cos(gamma)
    tau     = m * ve / F_T

    # Initial net radial gravity at current position
    g_r = -mu / r**2 + v_theta**2 / r

    # --- Outer predictor-corrector (Algorithm 1 steps 15–20) ---
    for _ in range(n_pred_iter):
        # Inner: converge v_go with current g_r (steps 4–14)
        vgo_r, vgo_theta, L0, t_go = compute_vgo_with_gr(
            v_r, v_theta, r, m, r_T, mu, ve, F_T, g_r,
            v_theta_T=v_theta_T, v_r_T=v_r_T)

        # Predictor: estimate burnout position and velocity (step 15)
        r_end       = r + v_r * t_go + 0.5 * g_r * t_go**2
        r_end       = max(r_end, r * 0.5)      # safety clamp
        v_theta_end = v_theta + vgo_theta       # ≈ v_θ_T

        # Gravity at predicted burnout
        g_r_end = -mu / r_end**2 + v_theta_end**2 / r_end

        # Trapezoidal average of g_r over trajectory (step 16 — quadrature)
        g_r_avg = 0.5 * (g_r + g_r_end)

        # v_miss_r = (g_r − g_r_avg)·t_go  [zero at first order by construction]
        # Corrector: v_go_r_new = −v_r − g_r_avg·t_go  (steps 18–19)
        if abs(g_r_avg - g_r) * t_go < 1.0:    # |v_miss_r| < 1 m/s → converged
            g_r = g_r_avg
            break
        g_r = g_r_avg

    # Final inner convergence with settled g_r
    vgo_r, vgo_theta, L0, t_go = compute_vgo_with_gr(
        v_r, v_theta, r, m, r_T, mu, ve, F_T, g_r,
        v_theta_T=v_theta_T, v_r_T=v_r_T)

    S0, L1, S1, t_lambda = compute_thrust_integrals(L0, t_go, tau, ve)

    lambda_r_prime = compute_lambda_r(
        r, v_r, r_T, vgo_r, L0, t_go, t_lambda, S0, S1, g_r)

    return vgo_r, vgo_theta, L0, t_go, t_lambda, lambda_r_prime


def peg_new_tgo(state, r_T, mu, ve, F_T, v_theta_T=None, v_r_T=0.0):
    """Gravity-aware time-to-go only (for reuse by other guidance modes).

    Runs the same v_go convergence + predictor-corrector gravity averaging as the
    full major loop and returns just ``t_go`` (the steering costates λ'_r, t_λ and
    the thrust integrals are computed but discarded — negligible extra cost). This
    is the physically-correct PEG burn-time estimate ``τ·(1−exp(−‖v_go‖/c))`` with
    the radial gravity loss folded into ‖v_go‖, unlike the gravity-blind
    rocket-equation estimate the other modes use by default.
    """
    return peg_new_major_loop(state, r_T, mu, ve, F_T,
                              v_theta_T=v_theta_T, v_r_T=v_r_T)[3]


def peg_new_alpha(t_since_epoch, vgo_r, vgo_theta, L0, lambda_r_prime, t_lambda, gamma):
    """Minor loop: steering angle α from the analytical PEG thrust direction.

    From paper eq 72 (exact, normalised):

        û(t) = [v_go/L₀  +  λ'_r·(t − t_λ)·r̂] / ‖…‖

    Convention:
        u_r = sin β  (radial component of thrust unit vector)
        u_θ = cos β  (tangential component)
        α   = β − γ  (angle of attack = pitch minus flight-path angle)

    Parameters
    ----------
    t_since_epoch  : float   seconds since last major-loop update
    vgo_r, vgo_theta, L0 : float   v_go components and magnitude [m/s]
    lambda_r_prime : float   radial position costate [1/s]
    t_lambda       : float   reference time t_λ [s]
    gamma          : float   current flight-path angle [rad]

    Returns
    -------
    alpha : float   angle of attack [rad]
    """
    t_rel   = t_since_epoch - t_lambda

    u_r     = vgo_r / L0 + lambda_r_prime * t_rel
    u_theta = vgo_theta / L0

    # Normalise (exact formula, paper eq 72)
    mag = np.sqrt(u_r**2 + u_theta**2)
    if mag > 1e-10:
        u_r     /= mag
        u_theta /= mag

    # Pitch angle β (from local horizontal); α = β − γ
    beta = np.arctan2(u_r, u_theta)
    return float(beta - gamma)
