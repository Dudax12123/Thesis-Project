"""Indirect optimal control guidance via Pontryagin's Maximum Principle (PMP).

Solves the minimum-fuel ascent TPBVP once at guidance start via single shooting.

Problem formulation
-------------------
  States  : x = [r, v, γ, m]
  Control : u = α  (angle of attack)
  Cost    : Φ = −m(tf),  L = 0  (Mayer / minimum fuel)
  Terminal: r(tf) = r_T,  v(tf) = √(μ/r_T),  γ(tf) = 0

Hamiltonian with drag (eq 60)
------------------------------
  H = λ_r · v·sinγ
    + λ_v · [(F_T/m)cosα − g·sinγ − F_D/m]
    + λ_γ · [(F_T/m)sinα/v + F_L/(mv) − (g − v²/r)cosγ/v]
    + λ_m · [−F_T/(Isp·g0)]

  ρ = ρ0·exp(−(r−R_E)/H_atm),  q = ½ρv²,  F_D = C_D·A·q,  F_L = C_L·A·q

Optimality condition H_u = 0  (eq 63)
--------------------------------------
  α*(t) = arctan2(λ_γ,  v·λ_v)     [λ_m does NOT appear]

Full adjoint equations λ̇ = −H_x  (eq 62)
------------------------------------------
  λ̇_r = −λ_v·(2μ/r³)·sinγ − λ_γ·(2μ/r³ − v²/r²)·cosγ/v
         − λ_v·F_D/(m·H_atm) + λ_γ·F_L/(m·v·H_atm)   [drag terms]

  λ̇_v = −λ_r·sinγ + λ_γ·(γ̇/v − 2cosγ/r)
         + 2λ_v·F_D/(m·v) − 2λ_γ·F_L/(m·v²)           [drag terms]

  λ̇_γ = −λ_r·v·cosγ + λ_v·(μ/r²)·cosγ − λ_γ·(μ/r²−v²/r)·sinγ/v
         [unchanged — drag/lift have no γ dependence]

  λ̇_m = (F_T/m²)·(λ_v·cosα + λ_γ·sinα/v)

Complete 4D shooting (satisfies all FONCs)
------------------------------------------
  H is constant for autonomous systems with L=0.
  Transversality (eq 64b) → H(tf) = 0, so H(t0) = 0 as well.
  λ_m0 is computed analytically from H(t0) = 0, eliminating one unknown.

  Unknowns : p = [λ_r0, λ_v0, λ_γ0, tf]
  Residuals: F(p) = [r(tf)−r_T, v(tf)−v_circ, γ(tf), λ_m(tf)+1]
  H(tf) = 0 satisfied automatically via the H = const = 0 first integral.

  Combined ODE: 8 states [r, v, γ, m, λ_r, λ_v, λ_γ, λ_m]

Drag inclusion
--------------
  Drag is included when include_drag=True (used when GUIDANCE_START_MODE =
  "after_kick"; when starting after atmosphere exit drag ≈ 0 and can be ignored).
"""

import warnings
import numpy as np
from scipy.optimize import fsolve


# ─── Drag / lift helper ───────────────────────────────────────────────────────

def _compute_drag_lift(r, v, include_drag):
    """Return (F_D, F_L) [N].  Both zero when include_drag=False."""
    if not include_drag:
        return 0.0, 0.0
    from Auxiliary import constants as _c
    from Auxiliary import rocket_specs as _r
    rho = _c.RHO_0 * np.exp(-(r - _c.R_EARTH) / _c.H)
    q   = 0.5 * rho * v**2
    return _r.C_D * _r.A * q, _r.C_L * _r.A * q


# ─── λ_m0 from H(t0) = 0 ─────────────────────────────────────────────────────

def _compute_lam_m0(lam_r0, lam_v0, lam_gamma0,
                    r0, v0, gamma0, m0, F_T, Isp, g0, mu, include_drag):
    """Compute λ_m0 analytically from H(t0) = 0 (autonomous-system first integral).

    H = H_rest + λ_m·(−F_T/(Isp·g0)) = 0
    → λ_m0 = H_rest · Isp · g0 / F_T
    """
    F_D, F_L = _compute_drag_lift(r0, v0, include_drag)
    g   = mu / r0**2
    T_m = F_T / m0

    if abs(lam_v0) < 1e-12 and abs(lam_gamma0) < 1e-12:
        alpha0 = 0.0
    else:
        alpha0 = np.arctan2(lam_gamma0, v0 * lam_v0)

    dgammadt0 = (T_m * np.sin(alpha0) + F_L / m0
                 - (g - v0**2 / r0) * np.cos(gamma0)) / v0

    H_rest = (lam_r0 * v0 * np.sin(gamma0)
              + lam_v0 * (T_m * np.cos(alpha0) - g * np.sin(gamma0) - F_D / m0)
              + lam_gamma0 * dgammadt0)

    return H_rest * Isp * g0 / F_T


# ─── 8-state combined ODE ─────────────────────────────────────────────────────

def _combined_odes(y, F_T, Isp, g0, mu, include_drag):
    """8-state ODE: [r, v, γ, m, λ_r, λ_v, λ_γ, λ_m]."""
    r, v, gamma, m, lam_r, lam_v, lam_gamma, lam_m = y

    F_D, F_L = _compute_drag_lift(r, v, include_drag)
    g   = mu / r**2
    T_m = F_T / m

    if abs(lam_v) < 1e-12 and abs(lam_gamma) < 1e-12:
        alpha = 0.0
    else:
        alpha = np.arctan2(lam_gamma, v * lam_v)

    # ── state equations ──
    drdt     = v * np.sin(gamma)
    dvdt     = T_m * np.cos(alpha) - g * np.sin(gamma) - F_D / m
    dgammadt = (T_m * np.sin(alpha) + F_L / m
                - (g - v**2 / r) * np.cos(gamma)) / v
    dmdt     = -F_T / (Isp * g0)

    # ── adjoint equations ──
    H_atm = 8500.0   # atmospheric scale height [m] — _c.H

    dlam_r = (-lam_v * (2.0 * mu / r**3) * np.sin(gamma)
               - lam_gamma * (2.0 * mu / r**3 - v**2 / r**2) * np.cos(gamma) / v
               - lam_v * F_D / (m * H_atm)          # drag ∂F_D/∂r = −F_D/H_atm
               + lam_gamma * F_L / (m * v * H_atm)) # drag ∂F_L/∂r

    dlam_v = (-lam_r * np.sin(gamma)
               + lam_gamma * (dgammadt / v - 2.0 * np.cos(gamma) / r)
               + 2.0 * lam_v * F_D / (m * v)        # drag ∂F_D/∂v = 2F_D/v
               - 2.0 * lam_gamma * F_L / (m * v**2))# drag ∂F_L/∂v

    dlam_g = (-lam_r * v * np.cos(gamma)
               + lam_v * g * np.cos(gamma)
               - lam_gamma * (g - v**2 / r) * np.sin(gamma) / v)
    # drag/lift have no γ dependence → dlam_g unchanged

    dlam_m = (F_T / m**2) * (lam_v * np.cos(alpha) + lam_gamma * np.sin(alpha) / v)

    return np.array([drdt, dvdt, dgammadt, dmdt,
                     dlam_r, dlam_v, dlam_g, dlam_m], dtype=float)


# ─── RK4 forward integration ──────────────────────────────────────────────────

def _forward_integrate(lam0_3, lam_m0, r0, v0, gamma0, m0, m_dry, tf,
                       F_T, Isp, g0, mu, include_drag, dt=0.5, store_alpha=False):
    """Integrate the 8-state system with RK4 from t=0 to t=tf.

    Parameters
    ----------
    lam0_3 : array-like [λ_r0, λ_v0, λ_γ0]
    lam_m0 : float       λ_m0 (computed from H(t0) = 0)

    Returns
    -------
    r_f, v_f, gamma_f, lam_m_f : float — terminal values
    t_arr, alpha_arr            : ndarray or None — populated when store_alpha=True
    """
    from Auxiliary import constants as _c
    R_EARTH = _c.R_EARTH

    y = np.array([r0, v0, gamma0, m0,
                  lam0_3[0], lam0_3[1], lam0_3[2], lam_m0], dtype=float)

    t_list     = [] if store_alpha else None
    alpha_list = [] if store_alpha else None

    t_rel = 0.0

    while t_rel < tf:
        dt_step = min(dt, tf - t_rel)

        if store_alpha:
            alpha_now = (np.arctan2(y[6], y[1] * y[5])
                         if (abs(y[5]) > 1e-12 or abs(y[6]) > 1e-12)
                         else 0.0)
            t_list.append(t_rel)
            alpha_list.append(alpha_now)

        k1 = _combined_odes(y,                  F_T, Isp, g0, mu, include_drag)
        k2 = _combined_odes(y + 0.5*dt_step*k1, F_T, Isp, g0, mu, include_drag)
        k3 = _combined_odes(y + 0.5*dt_step*k2, F_T, Isp, g0, mu, include_drag)
        k4 = _combined_odes(y +     dt_step*k3, F_T, Isp, g0, mu, include_drag)

        y     = y + (dt_step / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        t_rel += dt_step

        if y[3] <= m_dry:
            break
        if y[0] < R_EARTH:   # crash guard
            if store_alpha:
                return y[0], y[1], y[2], y[7], np.array(t_list), np.array(alpha_list)
            return y[0], y[1], y[2], y[7], None, None

    if store_alpha:
        alpha_final = (np.arctan2(y[6], y[1] * y[5])
                       if (abs(y[5]) > 1e-12 or abs(y[6]) > 1e-12)
                       else 0.0)
        t_list.append(t_rel)
        alpha_list.append(alpha_final)
        return y[0], y[1], y[2], y[7], np.array(t_list), np.array(alpha_list)

    return y[0], y[1], y[2], y[7], None, None


# ─── Residual for fsolve (4D) ─────────────────────────────────────────────────

def _residual(params, r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, r_T, include_drag):
    lam_r0, lam_v0, lam_gamma0, tf = params

    if tf <= 0.0:
        return [1e6, 1e6, 1e6, 1e6]

    lam_m0 = _compute_lam_m0(lam_r0, lam_v0, lam_gamma0,
                               r0, v0, gamma0, m0, F_T, Isp, g0, mu, include_drag)

    r_f, v_f, gamma_f, lam_m_f, _, _ = _forward_integrate(
        [lam_r0, lam_v0, lam_gamma0], lam_m0,
        r0, v0, gamma0, m0, m_dry, tf,
        F_T, Isp, g0, mu, include_drag, store_alpha=False
    )

    v_circ = np.sqrt(mu / r_T)
    return [
        (r_f - r_T)      / 1e5,   # position   [scaled to ~O(1)]
        (v_f - v_circ)   / 1e3,   # velocity   [scaled to ~O(1)]
        gamma_f,                   # FPA        [rad, already O(0.1)]
        lam_m_f + 1.0,             # λ_m(tf)=−1 transversality
    ]


# ─── Public API ───────────────────────────────────────────────────────────────

def solve_tpbvp(state, r_T, mu, F_T, Isp, m_dry, g0, include_drag=False):
    """Solve the minimum-fuel TPBVP via single shooting (complete FONCs).

    Satisfies:
      • Terminal constraints r(tf) = r_T, v(tf) = v_circ, γ(tf) = 0
      • Transversality λ_m(tf) = −1  (eq 64a)
      • H(tf) = 0  (eq 64b) — enforced analytically via λ_m0 = H_rest·Isp·g0/F_T
      • Adjoint equations for all four costates including λ_m  (eq 62)

    Parameters
    ----------
    state       : array-like [s, r, v, gamma, m] at guidance start
    r_T         : float  Target orbital radius [m]
    mu          : float  Gravitational parameter [m³/s²]
    F_T         : float  Current thrust [N]
    Isp         : float  Specific impulse [s]
    m_dry       : float  Dry mass of active stage [kg]
    g0          : float  Standard gravity [m/s²]
    include_drag: bool   Include aerodynamic drag and lift in dynamics and
                         adjoint equations (use when GUIDANCE_START_MODE =
                         "after_kick"; negligible after atmosphere exit)

    Returns
    -------
    t_arr    : ndarray  t_rel values for α*(t) [s]
    alpha_arr: ndarray  Optimal angle-of-attack history [rad]
    """
    _, r0, v0, gamma0, m0 = (state[0], state[1], state[2],
                              state[3], state[4])

    mdot  = F_T / (Isp * g0)
    tf0   = (m0 - m_dry) / mdot   # propellant-based burnout time (initial guess)

    # Initial guess: [λ_r0=0, λ_v0=1 (prograde), λ_γ0=0, tf=tf0]
    p0 = np.array([0.0, 1.0, 0.0, tf0])

    args = (r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, r_T, include_drag)

    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sol, info, ier, msg = fsolve(_residual, p0, args=args,
                                          full_output=True, xtol=1e-8)
        if ier == 1:
            lam_r0, lam_v0, lam_gamma0, tf = sol
        else:
            print(f"[indirect] fsolve did not converge: {msg}. Using initial guess.")
            lam_r0, lam_v0, lam_gamma0, tf = p0
    except Exception as exc:
        print(f"[indirect] optimizer error: {exc}. Using initial guess.")
        lam_r0, lam_v0, lam_gamma0, tf = p0

    tf = max(tf, 1.0)   # guard against non-positive tf

    lam_m0 = _compute_lam_m0(lam_r0, lam_v0, lam_gamma0,
                               r0, v0, gamma0, m0, F_T, Isp, g0, mu, include_drag)

    _, _, _, _, t_arr, alpha_arr = _forward_integrate(
        [lam_r0, lam_v0, lam_gamma0], lam_m0,
        r0, v0, gamma0, m0, m_dry, tf,
        F_T, Isp, g0, mu, include_drag, store_alpha=True
    )

    if t_arr is None or len(t_arr) == 0:
        t_arr     = np.array([0.0, tf])
        alpha_arr = np.array([0.0, 0.0])

    return t_arr, alpha_arr


def tpbvp_alpha(t_since_epoch, t_arr, alpha_arr):
    """Interpolate the precomputed optimal α*(t) at the current time."""
    return float(np.interp(t_since_epoch, t_arr, alpha_arr,
                            left=float(alpha_arr[0]),
                            right=float(alpha_arr[-1])))
