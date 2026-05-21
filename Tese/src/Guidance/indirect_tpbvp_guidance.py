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
                    r0, v0, gamma0, m0, F_T, Isp, g0, mu, include_drag,
                    cost_mode="min_fuel", earth_rot_params=None):
    """Compute λ_m0 analytically from H(t0) = H_target (first integral).

    H is constant for autonomous systems with L=0.
    Transversality (eq 64b):
      min_fuel (Φ=−m(tf)): H(tf) = 0  → H_target = 0
      min_time (Φ=tf):     H(tf) = −1 → H_target = −1

    H = H_rest + λ_m·(−F_T/(Isp·g0)) = H_target
    → λ_m0 = (H_rest − H_target) · Isp · g0 / F_T
    """
    H_target = 0.0 if cost_mode == "min_fuel" else 1.0
    F_D, F_L = _compute_drag_lift(r0, v0, include_drag)
    g   = mu / r0**2
    T_m = F_T / m0

    if abs(lam_v0) < 1e-12 and abs(lam_gamma0) < 1e-12:
        alpha0 = 0.0
    else:
        alpha0 = np.arctan2(lam_gamma0, v0 * lam_v0)

    dvdt0     = T_m * np.cos(alpha0) - g * np.sin(gamma0) - F_D / m0
    dgammadt0 = (T_m * np.sin(alpha0) + F_L / m0
                 - (g - v0**2 / r0) * np.cos(gamma0)) / v0

    if earth_rot_params is not None:
        A1, A2, B1 = earth_rot_params
        dvdt0     += A1 * r0 * np.sin(gamma0) - A2 * r0 * np.cos(gamma0)
        dgammadt0 += B1 + (A2 * np.sin(gamma0) + A1 * np.cos(gamma0)) * r0 / v0

    H_rest = (lam_r0 * v0 * np.sin(gamma0)
              + lam_v0 * dvdt0
              + lam_gamma0 * dgammadt0)

    return (H_rest - H_target) * Isp * g0 / F_T


# ─── Switching function / bang-bang thrust ────────────────────────────────────

def _compute_thrust(lam_v, lam_gamma, lam_m, alpha, v, m, F_T_max, Isp, g0,
                    allow_throttle):
    """Return effective thrust [N] based on switching function σ.

    σ = ∂H/∂F_T = (λ_v·cosα + λ_γ·sinα/v) / m  −  λ_m / (Isp·g0)
    Bang-bang: F_T = F_T_max if σ > 0 else 0 (coast).
    When allow_throttle=False, always returns F_T_max.
    """
    if not allow_throttle:
        return F_T_max
    sigma = ((lam_v * np.cos(alpha) + lam_gamma * np.sin(alpha) / v) / m
             - lam_m / (Isp * g0))
    return F_T_max if sigma > 0.0 else 0.0


# ─── 8-state combined ODE ─────────────────────────────────────────────────────

def _combined_odes(y, F_T, Isp, g0, mu, include_drag, allow_throttle,
                   earth_rot_params=None):
    """8-state ODE: [r, v, γ, m, λ_r, λ_v, λ_γ, λ_m]."""
    r, v, gamma, m, lam_r, lam_v, lam_gamma, lam_m = y

    F_D, F_L = _compute_drag_lift(r, v, include_drag)
    g   = mu / r**2

    if abs(lam_v) < 1e-12 and abs(lam_gamma) < 1e-12:
        alpha = 0.0
    else:
        alpha = np.arctan2(lam_gamma, v * lam_v)

    F_T_eff = _compute_thrust(lam_v, lam_gamma, lam_m, alpha, v, m,
                               F_T, Isp, g0, allow_throttle)
    T_m = F_T_eff / m

    # ── state equations ──
    drdt     = v * np.sin(gamma)
    dvdt     = T_m * np.cos(alpha) - g * np.sin(gamma) - F_D / m
    dgammadt = (T_m * np.sin(alpha) + F_L / m
                - (g - v**2 / r) * np.cos(gamma)) / v
    dmdt     = -F_T_eff / (Isp * g0)   # zero during coast

    # ── adjoint equations ──
    H_atm = 8500.0   # atmospheric scale height [m] — _c.H

    dlam_r = (-lam_v * (2.0 * mu / r**3) * np.sin(gamma)
               - lam_gamma * (2.0 * mu / r**3 - v**2 / r**2) * np.cos(gamma) / v
               - lam_v * F_D / (m * H_atm)
               + lam_gamma * F_L / (m * v * H_atm))

    dlam_v = (-lam_r * np.sin(gamma)
               + lam_gamma * (dgammadt / v - 2.0 * np.cos(gamma) / r)
               + 2.0 * lam_v * F_D / (m * v)
               - 2.0 * lam_gamma * F_L / (m * v**2))

    dlam_g = (-lam_r * v * np.cos(gamma)
               + lam_v * g * np.cos(gamma)
               - lam_gamma * (g - v**2 / r) * np.sin(gamma) / v)

    # dlam_m = 0 automatically during coast (F_T_eff = 0)
    dlam_m = (F_T_eff / m**2) * (lam_v * np.cos(alpha) + lam_gamma * np.sin(alpha) / v)

    # ── Earth-rotation pseudo-force additions (fixed lat, heading) ──
    if earth_rot_params is not None:
        A1, A2, B1 = earth_rot_params
        # State additions: Δv̇ = r(A1·sinγ − A2·cosγ),  Δγ̇ = B1 + r(A2·sinγ + A1·cosγ)/v
        dvdt     += A1 * r * np.sin(gamma) - A2 * r * np.cos(gamma)
        dgammadt += B1 + (A2 * np.sin(gamma) + A1 * np.cos(gamma)) * r / v
        # Adjoint additions: Δλ̇ = −∂(λ_v·Δv̇ + λ_γ·Δγ̇)/∂x
        dlam_r += (-lam_v * (A1 * np.sin(gamma) - A2 * np.cos(gamma))
                    - lam_gamma * (A2 * np.sin(gamma) + A1 * np.cos(gamma)) / v)
        dlam_v += lam_gamma * (A2 * np.sin(gamma) + A1 * np.cos(gamma)) * r / v**2
        dlam_g += (-lam_v * r * (A1 * np.cos(gamma) + A2 * np.sin(gamma))
                    - lam_gamma * r * (A2 * np.cos(gamma) - A1 * np.sin(gamma)) / v)

    return np.array([drdt, dvdt, dgammadt, dmdt,
                     dlam_r, dlam_v, dlam_g, dlam_m], dtype=float)


# ─── Hamiltonian evaluation ──────────────────────────────────────────────────

def _compute_hamiltonian(y, F_T_eff, Isp, g0, mu, F_D, F_L):
    """Evaluate H at a state/costate vector with known effective thrust."""
    r, v, gamma, m, lam_r, lam_v, lam_gamma, lam_m = y
    g   = mu / r**2
    T_m = F_T_eff / m
    if abs(lam_v) > 1e-12 or abs(lam_gamma) > 1e-12:
        alpha = np.arctan2(lam_gamma, v * lam_v)
    else:
        alpha = 0.0
    dgammadt = (T_m * np.sin(alpha) + F_L / m
                - (g - v**2 / r) * np.cos(gamma)) / v
    return (lam_r * v * np.sin(gamma)
            + lam_v * (T_m * np.cos(alpha) - g * np.sin(gamma) - F_D / m)
            + lam_gamma * dgammadt
            + lam_m * (-F_T_eff / (Isp * g0)))


# ─── RK4 forward integration ──────────────────────────────────────────────────

def _forward_integrate(lam0_3, lam_m0, r0, v0, gamma0, m0, m_dry, tf,
                       F_T, Isp, g0, mu, include_drag, allow_throttle,
                       earth_rot_params=None, dt=0.5, store_alpha=False):
    """Integrate the 8-state system with RK4 from t=0 to t=tf.

    Parameters
    ----------
    lam0_3 : array-like [λ_r0, λ_v0, λ_γ0]
    lam_m0 : float       λ_m0 (computed from H(t0) = 0)

    Returns
    -------
    r_f, v_f, gamma_f, lam_m_f, lam_r_f, lam_v_f, lam_gamma_f : float
    t_arr, alpha_arr : ndarray or None — populated when store_alpha=True
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

        k1 = _combined_odes(y,                  F_T, Isp, g0, mu, include_drag, allow_throttle, earth_rot_params)
        k2 = _combined_odes(y + 0.5*dt_step*k1, F_T, Isp, g0, mu, include_drag, allow_throttle, earth_rot_params)
        k3 = _combined_odes(y + 0.5*dt_step*k2, F_T, Isp, g0, mu, include_drag, allow_throttle, earth_rot_params)
        k4 = _combined_odes(y +     dt_step*k3, F_T, Isp, g0, mu, include_drag, allow_throttle, earth_rot_params)

        y     = y + (dt_step / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        t_rel += dt_step

        if y[3] <= m_dry:
            break
        if y[0] < R_EARTH:   # crash guard
            if store_alpha:
                return (y[0], y[1], y[2], y[7], y[4], y[5], y[6],
                        np.array(t_list), np.array(alpha_list))
            return y[0], y[1], y[2], y[7], y[4], y[5], y[6], None, None

    if store_alpha:
        alpha_final = (np.arctan2(y[6], y[1] * y[5])
                       if (abs(y[5]) > 1e-12 or abs(y[6]) > 1e-12)
                       else 0.0)
        t_list.append(t_rel)
        alpha_list.append(alpha_final)
        return (y[0], y[1], y[2], y[7], y[4], y[5], y[6],
                np.array(t_list), np.array(alpha_list))

    return y[0], y[1], y[2], y[7], y[4], y[5], y[6], None, None


# ─── Residual for fsolve (4D no-throttle / 5D bang-bang) ─────────────────────

def _residual(params, r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, r_T,
              include_drag, cost_mode, allow_throttle, earth_rot_params):
    """4D residual when allow_throttle=False; 5D when allow_throttle=True.

    The 5D case treats lam_m0 as a free variable (the H=const analytical
    trick breaks down for bang-bang because the actual initial thrust depends
    on lam_m0 itself — circular dependency). H(tf)=H_target is then enforced
    as the explicit 5th residual.
    """
    if allow_throttle:
        if len(params) != 5:
            return [1e6] * 5
        lam_r0, lam_v0, lam_gamma0, lam_m0, tf = params
    else:
        if len(params) != 4:
            return [1e6] * 4
        lam_r0, lam_v0, lam_gamma0, tf = params
        lam_m0 = _compute_lam_m0(lam_r0, lam_v0, lam_gamma0,
                                   r0, v0, gamma0, m0, F_T, Isp, g0, mu,
                                   include_drag, cost_mode, earth_rot_params)

    penalty = [1e6] * (5 if allow_throttle else 4)
    if tf <= 0.0:
        return penalty

    (r_f, v_f, gamma_f, lam_m_f,
     lam_r_f, lam_v_f, lam_gamma_f, _, _) = _forward_integrate(
        [lam_r0, lam_v0, lam_gamma0], lam_m0,
        r0, v0, gamma0, m0, m_dry, tf,
        F_T, Isp, g0, mu, include_drag, allow_throttle, earth_rot_params,
        store_alpha=False
    )

    v_circ = np.sqrt(mu / r_T)
    lam_m_terminal = 1.0 if cost_mode == "min_fuel" else 0.0
    res = [
        (r_f - r_T)           / 1e5,   # position  [scaled ~O(1)]
        (v_f - v_circ)        / 1e3,   # velocity  [scaled ~O(1)]
        gamma_f,                        # FPA       [rad]
        lam_m_f - lam_m_terminal,       # λ_m(tf) transversality
    ]

    if allow_throttle:
        # Enforce H(tf) = H_target explicitly (free-time transversality eq 64b)
        y_f = np.array([r_f, v_f, gamma_f, m_dry,   # m_f ≈ m_dry at burnout
                        lam_r_f, lam_v_f, lam_gamma_f, lam_m_f])
        F_D_f, F_L_f = _compute_drag_lift(r_f, v_f, include_drag)
        if abs(lam_v_f) > 1e-12 or abs(lam_gamma_f) > 1e-12:
            alpha_f = np.arctan2(lam_gamma_f, v_f * lam_v_f)
        else:
            alpha_f = 0.0
        F_T_eff_f = _compute_thrust(lam_v_f, lam_gamma_f, lam_m_f, alpha_f,
                                     v_f, m_dry, F_T, Isp, g0, True)
        H_f = _compute_hamiltonian(y_f, F_T_eff_f, Isp, g0, mu, F_D_f, F_L_f)
        H_target = 0.0 if cost_mode == "min_fuel" else 1.0
        res.append((H_f - H_target) * 0.1)   # scale: H ~ 10 m/s² → residual ~O(1)

    return res


# ─── Public API ───────────────────────────────────────────────────────────────

def solve_tpbvp(state, r_T, mu, F_T, Isp, m_dry, g0,
                include_drag=False, cost_mode="min_fuel", allow_throttle=False,
                lat=0.0, heading=0.0, include_earth_rotation=False):
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
    include_drag    : bool   Include aerodynamic drag and lift in dynamics and
                             adjoint equations (use when GUIDANCE_START_MODE =
                             "after_kick"; negligible after atmosphere exit)
    cost_mode       : str    "min_fuel" or "min_time" — sets H_target and λ_m(tf)
    allow_throttle  : bool   Enable bang-bang thrust via switching function σ.
                             When True, F_T ∈ {0, F_T_max} at each RK4 step;
                             min_fuel may then produce coast arcs (σ ≤ 0).

    Returns
    -------
    t_arr    : ndarray  t_rel values for α*(t) [s]
    alpha_arr: ndarray  Optimal angle-of-attack history [rad]
    """
    _, r0, v0, gamma0, m0 = (state[0], state[1], state[2],
                              state[3], state[4])

    mdot  = F_T / (Isp * g0)
    tf0   = (m0 - m_dry) / mdot   # propellant-based burnout time (initial guess)

    # Earth-rotation constants (fixed lat, heading)
    if include_earth_rotation:
        from Auxiliary import constants as _c
        _om = _c.OMEGA_EARTH
        A1 = _om**2 * np.cos(lat)**2
        A2 = _om**2 * np.sin(lat) * np.cos(lat) * np.cos(heading)
        B1 = 2.0 * _om * np.cos(lat) * np.sin(heading)
        earth_rot_params = (A1, A2, B1)
    else:
        earth_rot_params = None

    args = (r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, r_T,
            include_drag, cost_mode, allow_throttle, earth_rot_params)

    # Multi-start: try physically motivated λ_γ0 values to escape the trivial solution.
    # Physical reasoning: α*(t=0) = arctan2(λ_γ0, v0·λ_v0) → λ_γ0 = v0·tan(α_init).
    # Ascent typically requires slightly negative α (pitch toward horizontal), so we
    # sweep from −30° to +5° and keep the converged solution with the smallest residual.
    _alpha_inits = np.deg2rad([-30., -20., -10., -5., 0., 5.])

    best_sol   = None
    best_rnorm = np.inf

    for _a0 in _alpha_inits:
        _lg0 = v0 * np.tan(_a0)
        _lm0 = _compute_lam_m0(0.0, 1.0, _lg0, r0, v0, gamma0, m0, F_T, Isp, g0, mu,
                                include_drag, cost_mode, earth_rot_params)
        _p = (np.array([0.0, 1.0, _lg0, _lm0, tf0]) if allow_throttle
              else np.array([0.0, 1.0, _lg0, tf0]))
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                _sol, _info, _ier, _ = fsolve(_residual, _p, args=args,
                                               full_output=True, xtol=1e-8)
            if _ier == 1:
                _rn = np.linalg.norm(_info['fvec'])
                if _rn < best_rnorm:
                    best_rnorm = _rn
                    best_sol   = _sol
        except Exception:
            pass

    if best_sol is not None:
        sol, ier = best_sol, 1
    else:
        # Fallback: use α₀ = −10° — more physical than the gravity-turn (α₀=0°) guess
        _lg0_fb = v0 * np.tan(np.deg2rad(-10.))
        _lm0_fb = _compute_lam_m0(0.0, 1.0, _lg0_fb, r0, v0, gamma0, m0, F_T, Isp, g0, mu,
                                   include_drag, cost_mode, earth_rot_params)
        sol = (np.array([0.0, 1.0, _lg0_fb, _lm0_fb, tf0]) if allow_throttle
               else np.array([0.0, 1.0, _lg0_fb, tf0]))
        ier = 0
        print("[indirect] fsolve did not converge with any initial guess. Using fallback.")

    if allow_throttle:
        lam_r0, lam_v0, lam_gamma0, lam_m0, tf = sol
    else:
        lam_r0, lam_v0, lam_gamma0, tf = sol
        lam_m0 = _compute_lam_m0(lam_r0, lam_v0, lam_gamma0,
                                  r0, v0, gamma0, m0, F_T, Isp, g0, mu,
                                  include_drag, cost_mode, earth_rot_params)

    tf = max(tf, 1.0)   # guard against non-positive tf

    _, _, _, _, _, _, _, t_arr, alpha_arr = _forward_integrate(
        [lam_r0, lam_v0, lam_gamma0], lam_m0,
        r0, v0, gamma0, m0, m_dry, tf,
        F_T, Isp, g0, mu, include_drag, allow_throttle, earth_rot_params,
        store_alpha=True
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
