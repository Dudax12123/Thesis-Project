"""
PEG_new — Analytical Predictor-Corrector from First Principles

Derives the Powered Explicit Guidance steering law from Pontryagin's minimum
principle and Jaggers' "Coke Machine" orthogonality assumption (Jaggers 1977).
The primary variable is v_go (2D velocity-to-be-gained vector).  Time-to-go
is obtained directly from v_go via the rocket equation.

The major loop is Algorithm 1 of Mahajan & Condon in its analytical form: the
steering constants come from the constant-thrust integrals (eqs 66–71), and the
gravity integrals v_G and r_G come from numerical quadrature along the predicted
powered trajectory (steps 15–16). The prediction flies the planar polar model the
law works in — net radial gravity −μ/r² + v_θ²/r and the tangential transport term
−v_r·v_θ/r; Earth-rotation pseudo-forces are left to the closed loop — and the
corrector adds the predicted velocity miss to v_go until it falls below
CORRECTOR_TOL (steps 18–20).

The resulting steering law (first-order, free-downrange 2D) is:

    û(t) = v_go/L₀  +  λ'_r·(t − t_λ)·r̂     [paper eq 72, normalised]

References
----------
Mahajan, B., & Condon, G. L. (2025). Enhancements to Space Shuttle Powered
    Explicit Guidance for Planetary Ascent and Descent. AAS 25-844. Section "PEG
    Derivation from First Principles": Algorithm 1 and eqs (61)–(72).

Jaggers, R. F. (1977). An explicit solution to the exoatmospheric powered
    flight guidance and trajectory optimisation problem for rocket propelled
    vehicles. AIAA Paper 77-1051.

McHenry, R. L., Brand, T. J., Long, A. D., Cockrell, B. F., & Thibodeau, J. R.
    (1979). Space Shuttle Ascent Guidance, Navigation and Control.
    Journal of the Astronautical Sciences, 27(1), 1–38.
"""

import math

import numpy as np

PREDICTOR_STEP = 20.0      # RK4 step of the predicted trajectory [s]
CORRECTOR_TOL = 0.05       # |v_miss| at which the corrector stops [m/s] (Algorithm 1 step 20)
CORRECTOR_MAX_ITER = 20


def compute_vgo_with_gr(v_r, v_theta, r, m, r_T, mu, ve, F_T, g_r,
                        max_iter=15, tol=0.01, v_theta_T=None, v_r_T=0.0):
    """Converge v_go and t_go for a given net radial gravity g_r.

    The major loop's initial v_go (Algorithm 1 step 3), with g_r held constant.
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
        S₁ = S₀·τ − c·t_go²/2      (∫∫(T/m)·s, eq 55)
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
    S1       = S0 * tau - ve * t_go**2 / 2.0
    t_lambda = L1 / L0 if L0 > 1e-6 else 0.0
    return float(S0), float(L1), float(S1), float(t_lambda)


def compute_lambda_r(r, v_r, r_T, vgo_r, L0, t_go, t_lambda, S0, S1, rG_r):
    """Radial position costate from paper eq 71 (free-downrange projection).

    For free downrange the tangential component of λ'_r is zero, leaving only:

        λ'_r = (L₀·rgo_r − S₀·vgo_r) / (L₀·(S₁ − t_λ·S₀))

    where rgo_r = (r_T − r) − v_r·t_go − r_G,r   (Algorithm 1 step 10)

    Parameters
    ----------
    r, v_r, r_T          : float   current radius, radial velocity, target radius
    vgo_r, L0            : float   radial v_go and its magnitude [m/s]
    t_go, t_lambda, S0, S1 : float thrust-integral quantities
    rG_r                 : float   radial position gravity integral ∫∫g_r over the burn [m]

    Returns
    -------
    lambda_r_prime : float  [1/s]
    """
    rgo_r = (r_T - r) - v_r * t_go - rG_r

    denom = L0 * (S1 - t_lambda * S0)
    if abs(denom) < 1e-6:
        return 0.0
    return float((L0 * rgo_r - S0 * vgo_r) / denom)


def _steering_constants(vgo_r, vgo_theta, r, v_r, r_T, rG_r, tau, ve):
    """Algorithm 1 steps 4, 6, 10 and 13 for a given v_go and r_G: (L0, t_go, t_λ, λ'_r)."""
    L0 = max(math.hypot(vgo_r, vgo_theta), 1.0)
    t_go = tau * (1.0 - math.exp(-L0 / ve))
    S0, L1, S1, t_lambda = compute_thrust_integrals(L0, t_go, tau, ve)
    lambda_r_prime = compute_lambda_r(r, v_r, r_T, vgo_r, L0, t_go, t_lambda, S0, S1, rG_r)
    return L0, t_go, t_lambda, lambda_r_prime


def _predict_burnout(r, v_r, v_theta, m, mu, ve, F_T, u_r0, u_theta, lambda_r_prime,
                     t_lambda, t_go):
    """Algorithm 1 steps 15–16: fly the planar model under the current steering for t_go.

    RK4 on (r, v_r, v_θ, v_G,r, r_G,r) with the thrust along the normalised eq 72
    direction. v_G,r = ∫g_r and r_G,r = ∫v_G,r = ∫∫g_r are integrated along the same
    steps, so they are quadratures over the predicted powered trajectory, not over a
    guess of it.

    Returns the predicted burnout v_r, v_θ and r_G,r; NaN when t_go reaches τ, i.e. the
    burn would consume the vehicle's whole mass.
    """
    mdot = F_T / ve
    uth2 = u_theta * u_theta

    def rates(t, r, v_r, v_theta):
        mass = m - mdot * t
        if mass <= 0.0:
            return math.nan, math.nan, math.nan
        u_r = u_r0 + lambda_r_prime * (t - t_lambda)
        norm = math.sqrt(u_r * u_r + uth2)
        a = F_T / mass / norm if norm > 1e-10 else 0.0
        g_r = -mu / (r * r) + v_theta * v_theta / r
        return a * u_r + g_r, a * u_theta - v_r * v_theta / r, g_r

    n = max(2, math.ceil(t_go / PREDICTOR_STEP))
    h = t_go / n
    t = vG_r = rG_r = 0.0
    for _ in range(n):
        a1, b1, g1 = rates(t, r, v_r, v_theta)
        a2, b2, g2 = rates(t + 0.5 * h, r + 0.5 * h * v_r,
                           v_r + 0.5 * h * a1, v_theta + 0.5 * h * b1)
        a3, b3, g3 = rates(t + 0.5 * h, r + 0.5 * h * (v_r + 0.5 * h * a1),
                           v_r + 0.5 * h * a2, v_theta + 0.5 * h * b2)
        a4, b4, g4 = rates(t + h, r + h * (v_r + 0.5 * h * a2),
                           v_r + h * a3, v_theta + h * b3)
        r += h * v_r + h * h / 6.0 * (a1 + a2 + a3)
        rG_r += h * vG_r + h * h / 6.0 * (g1 + g2 + g3)
        v_r += h / 6.0 * (a1 + 2.0 * a2 + 2.0 * a3 + a4)
        v_theta += h / 6.0 * (b1 + 2.0 * b2 + 2.0 * b3 + b4)
        vG_r += h / 6.0 * (g1 + 2.0 * g2 + 2.0 * g3 + g4)
        t += h
    return v_r, v_theta, rG_r


def peg_new_major_loop(state, r_T, mu, ve, F_T, v_theta_T=None, v_r_T=0.0,
                       max_iter=CORRECTOR_MAX_ITER, tol=CORRECTOR_TOL):
    """Full major-loop step with predictor-corrector (Algorithm 1 steps 3–20).

    v_go starts from the gravity at the current position (step 3). Each pass sets the
    steering constants from v_go and r_G (steps 4–13), flies the predicted trajectory
    to get the burnout velocity and a new r_G (steps 15–16), and adds the velocity
    miss to v_go (steps 18–19), until |v_miss| < tol (step 20).

    The miss is added with a secant step length rather than the paper's unit step,
    which overshoots and oscillates when the steering turns through a large angle
    (arc 1 aimed at the final orbit from a low ignition); the fixed point, v_miss = 0,
    is the same. A target the remaining burn cannot reach (a large radius error with
    little v_go left) has no fixed point: the corrector stops after max_iter passes, or
    earlier at the last v_go whose burn could still be predicted.

    Parameters
    ----------
    state       : array-like [s, r, v, gamma, m, ...]
    r_T         : float   target radius [m]
    mu          : float   gravitational parameter [m³/s²]
    ve          : float   exhaust speed [m/s]
    F_T         : float   thrust [N]
    v_theta_T   : float   target tangential velocity (None ⇒ inertial √(μ/r_T))
    v_r_T       : float   target radial velocity
    max_iter    : int     corrector passes at most
    tol         : float   velocity miss at which the corrector stops [m/s]

    Returns
    -------
    vgo_r, vgo_theta, L0, t_go, t_lambda, lambda_r_prime : float
    """
    r       = float(state[1])
    v       = float(state[2])
    gamma   = float(state[3])
    m       = float(state[4])

    v_r     = v * math.sin(gamma)
    v_theta = v * math.cos(gamma)
    tau     = m * ve / F_T
    v_theta_T = math.sqrt(mu / r_T) if v_theta_T is None else float(v_theta_T)

    g_r = -mu / r**2 + v_theta**2 / r
    vgo_r, vgo_theta, L0, t_go = compute_vgo_with_gr(
        v_r, v_theta, r, m, r_T, mu, ve, F_T, g_r, v_theta_T=v_theta_T, v_r_T=v_r_T)
    rG_r = 0.5 * g_r * t_go**2

    step_len, last_miss, last_step = 1.0, None, None
    for _ in range(max_iter):
        L0, t_go, t_lambda, lambda_r_prime = _steering_constants(
            vgo_r, vgo_theta, r, v_r, r_T, rG_r, tau, ve)
        v_rP, v_thetaP, rG_pred = _predict_burnout(
            r, v_r, v_theta, m, mu, ve, F_T, vgo_r / L0, vgo_theta / L0,
            lambda_r_prime, t_lambda, t_go)
        if not (math.isfinite(v_rP) and math.isfinite(v_thetaP) and math.isfinite(rG_pred)):
            if last_step is not None:           # back to the last v_go that could be flown
                vgo_r -= last_step[0]
                vgo_theta -= last_step[1]
            break
        rG_r = rG_pred
        miss_r, miss_theta = v_r_T - v_rP, v_theta_T - v_thetaP
        if last_step is not None:
            # how much of the last step the predicted burnout velocity took up
            d_r, d_theta = last_step
            taken = -((miss_r - last_miss[0]) * d_r + (miss_theta - last_miss[1]) * d_theta) \
                / (d_r * d_r + d_theta * d_theta)
            if taken > 0.0:
                step_len = min(max(1.0 / taken, 0.25), 2.0)
        last_miss = (miss_r, miss_theta)
        last_step = (step_len * miss_r, step_len * miss_theta)
        vgo_r += last_step[0]
        vgo_theta += last_step[1]
        if math.hypot(miss_r, miss_theta) < tol:
            break

    L0, t_go, t_lambda, lambda_r_prime = _steering_constants(
        vgo_r, vgo_theta, r, v_r, r_T, rG_r, tau, ve)
    return (float(vgo_r), float(vgo_theta), float(L0), float(t_go), float(t_lambda),
            float(lambda_r_prime))


def peg_new_tgo(state, r_T, mu, ve, F_T, v_theta_T=None, v_r_T=0.0):
    """Gravity-aware time-to-go only (for reuse by other guidance modes).

    Runs the full major loop and returns just ``t_go``, the PEG burn-time estimate
    ``τ·(1−exp(−‖v_go‖/c))`` with the gravity integrals of the predicted trajectory
    folded into v_go, unlike the gravity-blind rocket-equation estimate the other
    modes use by default. It costs the whole predictor-corrector (0.02–0.45 ms), and
    apollo asks for it on every right-hand-side evaluation.
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
