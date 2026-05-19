"""Indirect optimal control guidance via Pontryagin's Maximum Principle (PMP).

Solves the minimum-fuel ascent TPBVP once at guidance start.

Problem formulation
-------------------
  States  : x = [r, v, γ, m]
  Control : u = α  (angle of attack)
  Cost    : Φ = −m(tf),  L = 0  (Mayer / minimum fuel)
  Terminal: r(tf) = r_T,  v(tf) = √(μ/r_T),  γ(tf) = 0

Hamiltonian (eq 60)
-------------------
  H = λ_r·v·sin γ
    + λ_v·[(F_T/m)cos α − (μ/r²)sin γ]
    + λ_γ·[(F_T/m)sin α / v − (μ/r² − v²/r)cos γ / v]
    + λ_m·[−F_T/(Isp·g0)]

Optimality condition H_u = 0  (eq 63)
--------------------------------------
  ∂H/∂α = 0  →  α*(t) = arctan2(λ_γ,  v·λ_v)

λ_m does NOT appear in α* or in the other adjoint equations, so it is
omitted from the integrated state vector (only the 7-state system
[r, v, γ, m, λ_r, λ_v, λ_γ] is integrated).

Adjoint equations  λ̇ = −H_x  (eq 62)
--------------------------------------
  λ̇_r = −λ_v·(2μ/r³)·sin γ  −  λ_γ·(2μ/r³ − v²/r²)·cos γ / v
  λ̇_v = −λ_r·sin γ  +  λ_γ·(γ̇/v − 2cos γ / r)
  λ̇_γ = −λ_r·v·cos γ + λ_v·(μ/r²)·cos γ − λ_γ·(μ/r²−v²/r)·sin γ / v

Shooting problem
----------------
  Unknowns : p = [λ_r(t0), λ_v(t0), λ_γ(t0)]
  Residuals: F(p) = [r(tf)−r_T,  v(tf)−v_circ,  γ(tf)]
  Solver   : scipy.optimize.fsolve
  tf fixed : propellant-based  tf = (m0 − m_dry)·Isp·g0 / F_T

After convergence the optimal α*(t) is stored as a dense array and
interpolated at each live ODE step (no costates in the real ODE).
"""

import warnings
import numpy as np
from scipy.optimize import fsolve


# ─── 7-state combined ODE ─────────────────────────────────────────────────────

def _combined_odes(y, F_T, Isp, g0, mu):
    """7-state ODE: [r, v, γ, m, λ_r, λ_v, λ_γ] (no drag, no lift)."""
    r, v, gamma, m, lam_r, lam_v, lam_gamma = y

    g   = mu / r**2
    T_m = F_T / m

    # Degenerate-costate guard
    if abs(lam_v) < 1e-12 and abs(lam_gamma) < 1e-12:
        alpha = 0.0
    else:
        alpha = np.arctan2(lam_gamma, v * lam_v)

    # ── state equations ──
    drdt     = v * np.sin(gamma)
    dvdt     = T_m * np.cos(alpha) - g * np.sin(gamma)
    dgammadt = (T_m * np.sin(alpha) - (g - v**2 / r) * np.cos(gamma)) / v
    dmdt     = -F_T / (Isp * g0)

    # ── adjoint equations ──
    dlam_r = (-lam_v * (2.0 * mu / r**3) * np.sin(gamma)
               - lam_gamma * (2.0 * mu / r**3 - v**2 / r**2) * np.cos(gamma) / v)

    dlam_v = (-lam_r * np.sin(gamma)
               + lam_gamma * (dgammadt / v - 2.0 * np.cos(gamma) / r))

    dlam_g = (-lam_r * v * np.cos(gamma)
               + lam_v * g * np.cos(gamma)
               - lam_gamma * (g - v**2 / r) * np.sin(gamma) / v)

    return np.array([drdt, dvdt, dgammadt, dmdt, dlam_r, dlam_v, dlam_g],
                    dtype=float)


# ─── RK4 forward integration ──────────────────────────────────────────────────

def _forward_integrate(lam0, r0, v0, gamma0, m0, m_dry, tf, F_T, Isp, g0, mu,
                       dt=0.5, store_alpha=False):
    """Integrate the 7-state system with RK4 from t=0 to t=tf.

    Returns
    -------
    r_f, v_f, gamma_f : float  — terminal state components
    t_arr, alpha_arr  : ndarray or (None, None) — only populated when
                        store_alpha=True
    """
    from Auxiliary import constants as _c
    R_EARTH = _c.R_EARTH

    y = np.array([r0, v0, gamma0, m0, lam0[0], lam0[1], lam0[2]], dtype=float)

    t_list     = [] if store_alpha else None
    alpha_list = [] if store_alpha else None

    t_rel = 0.0
    mdot  = F_T / (Isp * g0)

    while t_rel < tf:
        dt_step = min(dt, tf - t_rel)

        if store_alpha:
            alpha_now = (np.arctan2(y[6], y[1] * y[5])
                         if (abs(y[5]) > 1e-12 or abs(y[6]) > 1e-12)
                         else 0.0)
            t_list.append(t_rel)
            alpha_list.append(alpha_now)

        k1 = _combined_odes(y,                  F_T, Isp, g0, mu)
        k2 = _combined_odes(y + 0.5*dt_step*k1, F_T, Isp, g0, mu)
        k3 = _combined_odes(y + 0.5*dt_step*k2, F_T, Isp, g0, mu)
        k4 = _combined_odes(y +     dt_step*k3, F_T, Isp, g0, mu)

        y     = y + (dt_step / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        t_rel += dt_step

        # Stop if mass drops below dry mass
        if y[3] <= m_dry:
            break
        # Crash guard
        if y[0] < R_EARTH:
            if store_alpha:
                return (y[0], y[1], y[2],
                        np.array(t_list), np.array(alpha_list))
            return y[0], y[1], y[2], None, None

    # Store the final point
    if store_alpha:
        alpha_final = (np.arctan2(y[6], y[1] * y[5])
                       if (abs(y[5]) > 1e-12 or abs(y[6]) > 1e-12)
                       else 0.0)
        t_list.append(t_rel)
        alpha_list.append(alpha_final)
        return y[0], y[1], y[2], np.array(t_list), np.array(alpha_list)

    return y[0], y[1], y[2], None, None


# ─── Residual for fsolve ──────────────────────────────────────────────────────

def _residual(lam0, r0, v0, gamma0, m0, m_dry, tf, F_T, Isp, g0, mu, r_T):
    r_f, v_f, gamma_f, _, _ = _forward_integrate(
        lam0, r0, v0, gamma0, m0, m_dry, tf, F_T, Isp, g0, mu,
        store_alpha=False
    )
    v_circ = np.sqrt(mu / r_T)
    # Scale residuals to similar magnitudes for better fsolve conditioning
    return [
        (r_f - r_T) / 1e5,          # position residual [×100 km]
        (v_f - v_circ) / 1e3,       # velocity residual [×km/s]
        gamma_f,                     # FPA residual [rad]
    ]


# ─── Public API ───────────────────────────────────────────────────────────────

def solve_tpbvp(state, r_T, mu, F_T, Isp, m_dry, g0):
    """Solve the minimum-fuel TPBVP via single shooting.

    Parameters
    ----------
    state : array-like, length ≥ 5
        ODE state [s, r, v, gamma, m] at guidance start.
    r_T   : float  Target orbital radius [m].
    mu    : float  Gravitational parameter [m³/s²].
    F_T   : float  Current thrust [N].
    Isp   : float  Specific impulse [s].
    m_dry : float  Dry mass of active stage [kg].
    g0    : float  Standard gravity [m/s²].

    Returns
    -------
    t_arr    : ndarray  Time array (t_rel) for α*(t) [s]
    alpha_arr: ndarray  Optimal angle-of-attack history [rad]
    """
    _, r0, v0, gamma0, m0 = (state[0], state[1], state[2],
                              state[3], state[4])

    mdot = F_T / (Isp * g0)
    tf   = (m0 - m_dry) / mdot          # propellant-based burnout time

    # Initial guess: prograde thrust, no radial or pitch-rate correction
    lam0_guess = np.array([0.0, 1.0, 0.0])

    args = (r0, v0, gamma0, m0, m_dry, tf, F_T, Isp, g0, mu, r_T)

    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sol, info, ier, msg = fsolve(_residual, lam0_guess,
                                          args=args, full_output=True)
        if ier == 1:
            lam0 = sol
        else:
            print(f"[indirect] fsolve did not converge: {msg}. Using initial guess.")
            lam0 = lam0_guess
    except Exception as exc:
        print(f"[indirect] optimizer error: {exc}. Using initial guess.")
        lam0 = lam0_guess

    # Re-integrate with the found costates to build the α*(t) trajectory
    _, _, _, t_arr, alpha_arr = _forward_integrate(
        lam0, r0, v0, gamma0, m0, m_dry, tf, F_T, Isp, g0, mu,
        store_alpha=True
    )

    if t_arr is None or len(t_arr) == 0:
        # Fallback: zero angle of attack
        t_arr = np.array([0.0, tf])
        alpha_arr = np.array([0.0, 0.0])

    return t_arr, alpha_arr


def tpbvp_alpha(t_since_epoch, t_arr, alpha_arr):
    """Interpolate the precomputed optimal α*(t) at the current time.

    Clamps to the first/last value outside the stored range.
    """
    return float(np.interp(t_since_epoch, t_arr, alpha_arr,
                            left=float(alpha_arr[0]),
                            right=float(alpha_arr[-1])))
