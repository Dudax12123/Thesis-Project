"""Exponential pitch-law guidance with single-shot shooting-method optimization.

Pitch angle law:  θ(t_rel) = a · exp(b · t_rel)
Angle of attack:  α = θ(t_rel) − γ

(a, b) are found once at guidance start via scipy.optimize.fsolve so that the
trajectory satisfies two terminal constraints:
    r(T_burnout)  = r_T        (target orbital radius)
    γ(T_burnout)  = 0          (horizontal at burnout)

The forward simulation inside the optimizer uses simplified 2-D dynamics
(no drag, no Earth rotation) with a fixed RK4 integrator.
"""

import warnings
import numpy as np
from scipy.optimize import fsolve


# ─── RK4 helper ──────────────────────────────────────────────────────────────

def _derivatives(r, v, gamma, m, alpha, F_T, Isp, g0, mu):
    """Simplified 2-D rocket dynamics (no drag, no lift, point-mass gravity)."""
    g = mu / r**2
    T_over_m = F_T / m
    drdt     = v * np.sin(gamma)
    dvdt     = T_over_m * np.cos(alpha) - g * np.sin(gamma)
    dgammadt = (1.0 / v) * (T_over_m * np.sin(alpha) - (g - v**2 / r) * np.cos(gamma))
    dmdt     = -F_T / (Isp * g0)
    return drdt, dvdt, dgammadt, dmdt


def _rk4_step(r, v, gamma, m, a, b, t_rel, F_T, Isp, g0, mu, dt):
    """One RK4 step; returns updated (r, v, gamma, m, t_rel)."""
    alpha = a * np.exp(b * t_rel) - gamma

    def f(r_, v_, g_, m_, tr_):
        al = a * np.exp(b * tr_) - g_
        return _derivatives(r_, v_, g_, m_, al, F_T, Isp, g0, mu)

    dr1, dv1, dg1, dm1 = f(r,            v,            gamma,            m,            t_rel)
    dr2, dv2, dg2, dm2 = f(r+0.5*dt*dr1, v+0.5*dt*dv1, gamma+0.5*dt*dg1, m+0.5*dt*dm1, t_rel+0.5*dt)
    dr3, dv3, dg3, dm3 = f(r+0.5*dt*dr2, v+0.5*dt*dv2, gamma+0.5*dt*dg2, m+0.5*dt*dm2, t_rel+0.5*dt)
    dr4, dv4, dg4, dm4 = f(r+    dt*dr3, v+    dt*dv3, gamma+    dt*dg3, m+    dt*dm3, t_rel+    dt)

    r_new     = r     + (dt/6.0)*(dr1 + 2*dr2 + 2*dr3 + dr4)
    v_new     = v     + (dt/6.0)*(dv1 + 2*dv2 + 2*dv3 + dv4)
    gamma_new = gamma + (dt/6.0)*(dg1 + 2*dg2 + 2*dg3 + dg4)
    m_new     = m     + (dt/6.0)*(dm1 + 2*dm2 + 2*dm3 + dm4)
    return r_new, v_new, gamma_new, m_new, t_rel + dt


# ─── Forward simulation ───────────────────────────────────────────────────────

def _simulate_burn(a, b, r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, dt=0.5):
    """Integrate from t_rel=0 until m ≤ m_dry or trajectory crashes.

    Returns (r_f, gamma_f).  On crash returns a large-penalty tuple.
    """
    from Auxiliary import constants as _c   # local import to avoid circular
    R_EARTH = _c.R_EARTH

    r, v, gamma, m, t_rel = r0, v0, gamma0, m0, 0.0
    mdot = F_T / (Isp * g0)

    while m > m_dry:
        remaining = m - m_dry
        dt_step   = min(dt, remaining / mdot)
        r, v, gamma, m, t_rel = _rk4_step(r, v, gamma, m, a, b, t_rel,
                                            F_T, Isp, g0, mu, dt_step)
        if r < R_EARTH:
            return r0, np.pi / 2.0  # crashed — large penalty

    return r, gamma


# ─── Residual for fsolve ──────────────────────────────────────────────────────

def _residual(params, r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, r_T):
    a, b = params
    r_f, gamma_f = _simulate_burn(a, b, r0, v0, gamma0, m0, m_dry,
                                   F_T, Isp, g0, mu)
    return [r_f - r_T, gamma_f]


# ─── Public API ───────────────────────────────────────────────────────────────

def optimize_exp_pitch(state, r_T, mu, F_T, Isp, m_dry, g0):
    """Find (a, b) once via shooting so that the trajectory hits the target orbit.

    Parameters
    ----------
    state : array-like, length ≥ 5
        ODE state [s, r, v, gamma, m] at the moment guidance starts.
    r_T   : float  Target orbital radius [m].
    mu    : float  Gravitational parameter [m³/s²].
    F_T   : float  Current thrust [N].
    Isp   : float  Specific impulse of active stage [s].
    m_dry : float  Dry mass of active stage [kg].
    g0    : float  Standard gravity [m/s²].

    Returns
    -------
    (a, b) : floats  Optimized coefficients; (a0, b0) fallback if not converged.
    """
    _, r0, v0, gamma0, m0 = state[0], state[1], state[2], state[3], state[4]

    # Initial guess: a = current pitch angle (≈γ since α≈0 after kick); b = slow decay
    a0 = float(gamma0)
    b0 = -0.005

    args = (r0, v0, gamma0, m0, m_dry, F_T, Isp, g0, mu, r_T)

    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sol, info, ier, msg = fsolve(_residual, [a0, b0], args=args, full_output=True)
        if ier == 1:
            return float(sol[0]), float(sol[1])
        else:
            print(f"[exp_shooting] fsolve did not converge: {msg}. Using initial guess.")
            return a0, b0
    except Exception as exc:
        print(f"[exp_shooting] optimizer error: {exc}. Using initial guess.")
        return a0, b0


def exp_pitch_alpha(t_rel, a, b, gamma):
    """Angle of attack commanded by the exponential pitch law.

    alpha = a·exp(b·t_rel) − gamma
    """
    return a * np.exp(b * t_rel) - gamma
