"""Indirect optimal control guidance via Pontryagin's Maximum Principle (PMP).

Solves the minimum-time (≡ minimum-fuel for constant thrust) ascent TPBVP
once at Stage-2 guidance start, then interpolates α*(t) during flight.

Problem formulation
-------------------
  States  : x = [r, v, γ, m]
  Control : u = α  (angle of attack)
  Cost    : Φ = −m(tf),  L = 0  (Mayer / minimum fuel)
            With constant thrust, min-fuel ⟺ min-tf (free final time).
  Terminal: r(tf) = r_T,  v(tf) = √(μ/r_T),  γ(tf) = 0

Hamiltonian
-----------
  H = λ_r·v·sin γ
    + λ_v·[(F_T/m)cos α − (μ/r²)sin γ]
    + λ_γ·[(F_T/m)sin α / v − (μ/r² − v²/r)cos γ / v]
    + λ_m·[−F_T/(Isp·g0)]

Optimality condition H_u = 0
-----------------------------
  ∂H/∂α = 0  →  α*(t) = arctan2(λ_γ,  v·λ_v)

λ_m does NOT appear in α* or in the other adjoint equations, so it is
omitted from the integrated state vector (only the 7-state system
[r, v, γ, m, λ_r, λ_v, λ_γ] is integrated).

Adjoint equations  λ̇ = −H_x
-----------------------------
  λ̇_r = −λ_v·(2μ/r³)·sin γ  −  λ_γ·(2μ/r³ − v²/r²)·cos γ / v
  λ̇_v = −λ_r·sin γ  +  λ_γ·(γ̇/v − 2cos γ / r)
  λ̇_γ = −λ_r·v·cos γ + λ_v·(μ/r²)·cos γ − λ_γ·(μ/r²−v²/r)·sin γ / v

Shooting problem (4D — free final time)
----------------------------------------
  Unknowns : p = [λ_r(t0), λ_v(t0), λ_γ(t0), tf]
  Residuals: F(p) = [r(tf)−r_T,  v(tf)−v_circ,  γ(tf),  H(tf)]
             where H(tf)=0 is the free-tf transversality condition,
             evaluated with λ_m(tf)=−1  (from ∂Φ/∂m = ∂(−m)/∂m = −1).
  Solver   : scipy.optimize.fsolve with up to 4 multi-start attempts.

Initial guess strategy
----------------------
  PEG_new major-loop is called at guidance start to obtain:
    • t_go  → initial tf guess
    • PEG steering direction → initial (λ_v, λ_γ) via α* = arctan2(λ_γ, v·λ_v)
  If the TPBVP still fails to converge, the PEG-new steering trajectory is
  used as a non-trivial fallback (never pure gravity turn).
"""

import warnings
import numpy as np
from scipy.optimize import fsolve


# ─── Lazy import of PEG_new (avoids circular deps at module load) ─────────────

def _get_peg_mod():
    import Guidance.peg_guidance_new as _m
    return _m


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

    # ── adjoint equations  λ̇ = −∂H/∂x ──
    dlam_r     = (-lam_v * (2.0 * mu / r**3) * np.sin(gamma)
                  - lam_gamma * (2.0 * mu / r**3 - v**2 / r**2) * np.cos(gamma) / v)

    dlam_v     = (-lam_r * np.sin(gamma)
                  + lam_gamma * (dgammadt / v - 2.0 * np.cos(gamma) / r))

    dlam_gamma = (-lam_r * v * np.cos(gamma)
                  + lam_v * g * np.cos(gamma)
                  - lam_gamma * (g - v**2 / r) * np.sin(gamma) / v)

    return np.array([drdt, dvdt, dgammadt, dmdt,
                     dlam_r, dlam_v, dlam_gamma], dtype=float)


# ─── Hamiltonian evaluator ────────────────────────────────────────────────────

def _eval_hamiltonian(y, F_T, Isp, g0, mu):
    """Evaluate H at state y using λ_m(tf) = −1 (free-tf transversality).

    H = λ_r·v·sinγ + λ_v·[(F_T/m)cosα − g·sinγ]
      + λ_γ·[(F_T/m)sinα/v − (g−v²/r)cosγ/v]
      + (−1)·[−F_T/(Isp·g0)]
    """
    r, v, gamma, m, lam_r, lam_v, lam_gamma = y
    g   = mu / r**2
    T_m = F_T / m

    if abs(lam_v) < 1e-12 and abs(lam_gamma) < 1e-12:
        alpha = 0.0
    else:
        alpha = np.arctan2(lam_gamma, v * lam_v)

    lam_m = -1.0  # transversality: ∂Φ/∂m = ∂(−m)/∂m = −1

    H = (lam_r * v * np.sin(gamma)
         + lam_v * (T_m * np.cos(alpha) - g * np.sin(gamma))
         + lam_gamma * (T_m * np.sin(alpha) / v
                        - (g - v**2 / r) * np.cos(gamma) / v)
         + lam_m * (-F_T / (Isp * g0)))
    return float(H)


# ─── RK4 forward integration ──────────────────────────────────────────────────

def _forward_integrate(lam0, r0, v0, gamma0, m0, tf,
                       F_T, Isp, g0, mu,
                       m_dry=None, dt=0.5, store_alpha=False):
    """Integrate the 7-state system with RK4 from t=0 to t=tf.

    Parameters
    ----------
    lam0      : array-like  [λ_r(0), λ_v(0), λ_γ(0)]
    r0 … m0   : float       initial state
    tf        : float       integration horizon [s]  ← now a free parameter
    m_dry     : float | None  optional mass floor safety guard
    store_alpha: bool        whether to accumulate (t, α) history

    Returns
    -------
    r_f, v_f, gamma_f : float  — terminal state components
    y_final           : ndarray shape (7,)  — full 7-state at tf
    t_arr, alpha_arr  : ndarray or (None, None)
    """
    from Auxiliary import constants as _c
    R_EARTH = _c.R_EARTH

    y = np.array([r0, v0, gamma0, m0, lam0[0], lam0[1], lam0[2]], dtype=float)

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

        k1 = _combined_odes(y,                  F_T, Isp, g0, mu)
        k2 = _combined_odes(y + 0.5*dt_step*k1, F_T, Isp, g0, mu)
        k3 = _combined_odes(y + 0.5*dt_step*k2, F_T, Isp, g0, mu)
        k4 = _combined_odes(y +     dt_step*k3, F_T, Isp, g0, mu)

        y     = y + (dt_step / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        t_rel += dt_step

        # Mass floor guard (safety only — tf is the primary stop criterion)
        if m_dry is not None and y[3] <= m_dry:
            break
        # Crash guard
        if y[0] < R_EARTH:
            y_out = y.copy()
            if store_alpha:
                return (y[0], y[1], y[2], y_out,
                        np.array(t_list), np.array(alpha_list))
            return y[0], y[1], y[2], y_out, None, None

    # Store the final point
    if store_alpha:
        alpha_final = (np.arctan2(y[6], y[1] * y[5])
                       if (abs(y[5]) > 1e-12 or abs(y[6]) > 1e-12)
                       else 0.0)
        t_list.append(t_rel)
        alpha_list.append(alpha_final)
        return y[0], y[1], y[2], y.copy(), np.array(t_list), np.array(alpha_list)

    return y[0], y[1], y[2], y.copy(), None, None


# ─── 4-state integration for PEG fallback ────────────────────────────────────

def _peg_fallback_traj(state5, peg_outputs, F_T, Isp, g0, mu, m_dry, dt=0.5):
    """Integrate 4-state dynamics with PEG_new steering as fallback α(t).

    Uses the PEG major-loop outputs (frozen at guidance start) and
    peg_new_alpha() to compute a meaningful, non-zero steering profile
    for the full time-to-go.

    Returns
    -------
    t_arr, alpha_arr : ndarray  — PEG-steered trajectory
    """
    peg_mod = _get_peg_mod()
    vgo_r, vgo_theta, L0, t_go, t_lambda, lam_r_prime = peg_outputs

    y = np.array([float(state5[1]), float(state5[2]),
                  float(state5[3]), float(state5[4])], dtype=float)
    t_rel = 0.0

    def _odes4(yy, alpha):
        gr = mu / yy[0]**2
        Tm = F_T / yy[3]
        return np.array([
            yy[1] * np.sin(yy[2]),
            Tm * np.cos(alpha) - gr * np.sin(yy[2]),
            (Tm * np.sin(alpha) - (gr - yy[1]**2 / yy[0]) * np.cos(yy[2])) / yy[1],
            -F_T / (Isp * g0),
        ])

    t_list = [0.0]
    alpha_list = []

    # Compute alpha at t=0 before the first step
    a0 = peg_mod.peg_new_alpha(0.0, vgo_r, vgo_theta, L0,
                                lam_r_prime, t_lambda, float(y[2]))
    alpha_list.append(a0)

    while t_rel < t_go:
        dt_step = min(dt, t_go - t_rel)

        # Current alpha from PEG (uses current γ from integrated state)
        alpha_cur = peg_mod.peg_new_alpha(t_rel, vgo_r, vgo_theta, L0,
                                           lam_r_prime, t_lambda, float(y[2]))

        k1 = _odes4(y, alpha_cur)
        a2 = peg_mod.peg_new_alpha(t_rel + 0.5*dt_step, vgo_r, vgo_theta,
                                    L0, lam_r_prime, t_lambda,
                                    float(y[2] + 0.5*dt_step*k1[2]))
        k2 = _odes4(y + 0.5*dt_step*k1, a2)
        k3 = _odes4(y + 0.5*dt_step*k2, a2)
        a4 = peg_mod.peg_new_alpha(t_rel + dt_step, vgo_r, vgo_theta,
                                    L0, lam_r_prime, t_lambda,
                                    float(y[2] + dt_step*k3[2]))
        k4 = _odes4(y + dt_step*k3, a4)

        y     = y + (dt_step / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        t_rel += dt_step

        t_list.append(t_rel)
        alpha_next = peg_mod.peg_new_alpha(t_rel, vgo_r, vgo_theta, L0,
                                            lam_r_prime, t_lambda, float(y[2]))
        alpha_list.append(alpha_next)

        if m_dry is not None and y[3] <= m_dry:
            break

    return np.array(t_list), np.array(alpha_list)


# ─── PEG-based initial guess for costates ─────────────────────────────────────

def _peg_initial_guess(state5, r_T, mu, F_T, Isp, g0):
    """Use PEG_new to compute a good initial guess for the TPBVP costates.

    Returns
    -------
    lam0_guess   : ndarray [λ_r(0), λ_v(0), λ_γ(0)]
    tf_guess     : float   initial tf estimate [s]
    peg_outputs  : tuple   raw PEG major-loop outputs (for fallback use)
    or None if PEG major-loop raises.
    """
    peg_mod = _get_peg_mod()
    ve = Isp * g0
    try:
        vgo_r, vgo_theta, L0, t_go, t_lambda, lam_r_prime = \
            peg_mod.peg_new_major_loop(state5, r_T, mu, ve, F_T)
    except Exception as exc:
        print(f"[indirect] PEG major-loop failed for initial guess: {exc}")
        return None

    gamma0 = float(state5[3])
    v0     = float(state5[2])

    # PEG steering direction at t=0 (paper eq 72, evaluated at t_rel=0)
    u_r     = vgo_r / L0 + lam_r_prime * (0.0 - t_lambda)
    u_theta = vgo_theta / L0
    mag     = np.sqrt(u_r**2 + u_theta**2)
    if mag > 1e-10:
        u_r     /= mag
        u_theta /= mag
    beta0  = np.arctan2(u_r, u_theta)   # pitch angle (from local horizontal)
    alpha0 = beta0 - gamma0             # angle of attack at t=0

    # Map to TPBVP costates via α* = arctan2(λ_γ, v·λ_v)
    # Choose normalisation λ_v = cos(α0), λ_γ = v·sin(α0)
    lam_v0     = np.cos(alpha0)
    lam_gamma0 = v0 * np.sin(alpha0)
    lam_r0     = 0.0   # radial costate starts small

    peg_outputs = (vgo_r, vgo_theta, L0, t_go, t_lambda, lam_r_prime)
    return (np.array([lam_r0, lam_v0, lam_gamma0]),
            float(t_go),
            peg_outputs)


# ─── 4D residual for fsolve ───────────────────────────────────────────────────

def _residual_4d(p, r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, r_T, tf_ref):
    """4D shooting residual with free final time.

    Unknowns  : p = [λ_r(0), λ_v(0), λ_γ(0), tf]
    Residuals :
      0  (r(tf) − r_T)    / 1e5        [× 100 km]
      1  (v(tf) − v_circ) / 1e3        [× km/s]
      2  γ(tf)                          [rad]
      3  H(tf) / (F_T/(Isp·g0))        [dimensionless, transversality]
    """
    lam_r0, lam_v0, lam_g0, tf = p

    # Soft barrier: keep tf in a physically meaningful range
    if tf < 0.05 * tf_ref or tf > 3.0 * tf_ref:
        return [1e6, 1e6, 1e6, 1e6]

    lam0 = [lam_r0, lam_v0, lam_g0]
    r_f, v_f, gamma_f, y_f, _, _ = _forward_integrate(
        lam0, r0, v0, gamma0, m0, tf, F_T, Isp, g0, mu,
        m_dry=m_dry, store_alpha=False)

    v_circ  = np.sqrt(mu / r_T)
    H_f     = _eval_hamiltonian(y_f, F_T, Isp, g0, mu)
    H_scale = F_T / (Isp * g0)   # normalises the λ_m·(−ṁ) term to O(1)

    return [
        (r_f - r_T)     / 1e5,
        (v_f - v_circ)  / 1e3,
        gamma_f,
        H_f / H_scale,
    ]


# ─── Public API ───────────────────────────────────────────────────────────────

def solve_tpbvp(state, r_T, mu, F_T, Isp, m_dry, g0):
    """Solve the minimum-time/fuel TPBVP via single shooting with free tf.

    Parameters
    ----------
    state : array-like, length ≥ 5
        ODE state [s, r, v, gamma, m] at guidance start (Stage 2 ignition).
    r_T   : float  Target orbital radius [m].
    mu    : float  Gravitational parameter [m³/s²].
    F_T   : float  Current thrust [N].
    Isp   : float  Specific impulse [s].
    m_dry : float  Dry mass of Stage 2 [kg].
    g0    : float  Standard gravity [m/s²].

    Returns
    -------
    t_arr    : ndarray  Time array (t_rel) for α(t) [s]
    alpha_arr: ndarray  Angle-of-attack history [rad]
    """
    _, r0, v0, gamma0, m0 = (state[0], state[1], state[2],
                              state[3], state[4])

    mdot       = F_T / (Isp * g0)
    tf_burnout = (m0 - m_dry) / mdot   # reference / upper bound on tf

    # ── 1. PEG-based initial guess ────────────────────────────────────────────
    peg_result = _peg_initial_guess(state[:5], r_T, mu, F_T, Isp, g0)

    if peg_result is not None:
        lam0_peg, tf_peg, peg_outputs = peg_result
        print(f"[indirect] PEG initial guess: α₀={np.rad2deg(np.arctan2(lam0_peg[2], state[2]*lam0_peg[1])):.2f}°, "
              f"tf_guess={tf_peg:.1f}s (burnout={tf_burnout:.1f}s)")
    else:
        lam0_peg   = np.array([0.0, 1.0, 0.0])
        tf_peg     = tf_burnout
        peg_outputs = None
        print(f"[indirect] PEG guess unavailable — using [0,1,0], tf={tf_peg:.1f}s")

    # ── 2. Single shooting attempt from PEG initial guess ────────────────────
    p_init = np.array([lam0_peg[0], lam0_peg[1], lam0_peg[2], tf_peg])
    args   = (r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, r_T, tf_burnout)

    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sol, info, ier, msg = fsolve(_residual_4d, p_init,
                                          args=args, full_output=True)

        if ier == 1:
            lam0_sol = sol[:3]
            tf_sol   = float(sol[3])

            if tf_sol < 10.0 or tf_sol > 2.0 * tf_burnout:
                print(f"[indirect] fsolve converged but tf={tf_sol:.1f}s is unrealistic — falling back.")
            else:
                _, _, _, _, t_arr, alpha_arr = _forward_integrate(
                    lam0_sol, r0, v0, gamma0, m0, tf_sol, F_T, Isp, g0, mu,
                    m_dry=m_dry, store_alpha=True)

                if t_arr is not None and len(t_arr) > 0:
                    print(f"[indirect] TPBVP converged: "
                          f"tf_opt={tf_sol:.1f}s  (saves {tf_burnout - tf_sol:.1f}s vs full burn)")
                    return t_arr, alpha_arr

                print("[indirect] fsolve converged but produced empty trajectory — falling back.")
        else:
            print(f"[indirect] fsolve did not converge: {msg.strip()}")

    except Exception as exc:
        print(f"[indirect] fsolve error: {exc}")

    # ── 3. Fallback: PEG steering trajectory ─────────────────────────────────
    print("[indirect] All TPBVP attempts failed — using PEG steering fallback (non-zero α).")
    if peg_outputs is not None:
        t_arr, alpha_arr = _peg_fallback_traj(
            state[:5], peg_outputs, F_T, Isp, g0, mu, m_dry)
        if t_arr is not None and len(t_arr) > 0:
            return t_arr, alpha_arr

    # Last-resort: tiny constant AOA based on current flight-path angle
    print("[indirect] PEG fallback also failed — using minimal AOA constant.")
    aoa_fallback = float(np.clip(-0.05 * gamma0, -np.deg2rad(5), np.deg2rad(5)))
    t_arr     = np.array([0.0, tf_burnout])
    alpha_arr = np.array([aoa_fallback, 0.0])
    return t_arr, alpha_arr


def tpbvp_alpha(t_since_epoch, t_arr, alpha_arr):
    """Interpolate the precomputed optimal α*(t) at the current time.

    Clamps to the first/last value outside the stored range.
    """
    return float(np.interp(t_since_epoch, t_arr, alpha_arr,
                            left=float(alpha_arr[0]),
                            right=float(alpha_arr[-1])))
