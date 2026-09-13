#!/usr/bin/env python
"""Which transversality conditions does the indirect_pmp parameterisation actually need?

The swarm's Stage-2 structure is burn (D1) - coast (Dc) - burn (D3) with the three
durations as decision variables, fixed thrust, and the costates (λ_r, λ_v, λ_γ) only --
mass is a known function of time, so the reduced Hamiltonian H = λ·f is not conserved on
the burns. Stationarity of J' = λ0·(D1 + D3) + ν·Ψ(x_f) in each duration, with ν = λ(t_f),
gives, WITHOUT a mass costate (∂H/∂α = 0 under the PMP steering, so control variations drop
out; the mass shift in the last burn integrates by dH/dt = -ṁ λ·∂f/∂m):

    λ(t_f)·∂x_f/∂Dc = H_coast_end                                  -> H_coast_end = 0
    λ(t_f)·∂x_f/∂D3 = H_burn_end                                   -> H_burn_end = -λ0 < 0
    λ(t_f)·∂x_f/∂D1 = H_burn1_end - H_last_burn_start + H_burn_end -> H_burn1_end = H_last_burn_start

(H_burn1_end and H_last_burn_start with the engine on; H_coast_end with it off.)
The solver's penalised residual H_burn_end + H_coast_end - H_burn_start is none of these:
H_burn_start is taken at ignition, a fixed instant no condition involves.

This script checks the three sensitivities by finite differences at the converged
direct-refinement point (variant B, inertial frame), evaluates the conditions there, and
solves orbit constraints + the two equalities at fixed gamma_p.

Run from the repository root:
    PYTHONIOENCODING=utf-8 C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/pmp_duration_conditions.py
"""

import contextlib
import io
import json
import sys
import warnings
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "Tese" / "src"
sys.path.insert(0, str(SRC))

from Input_File import simulation_parameters as sp  # noqa: E402
import run_results_matrix as rrm  # noqa: E402

rrm._apply(sp, rrm.BASELINE)
rrm._apply(sp, {"GUIDANCE_MODE": "indirect_pmp", "INDIRECT_PMP_STAGE2_FRAME": "inertial"})

from Auxiliary import constants as c  # noqa: E402
from Auxiliary import rocket_specs as rs  # noqa: E402
import Simulation.indirect_pso_solver as ips  # noqa: E402

REFINE_JSON = SRC / "Output" / "pmp_refine" / "pmp_refine_20260913_170035.json"
T_MAX = ips._T_MAX_2
M_PROP_2 = float(rs.M_PROP_2)
R_T = c.R_EARTH + sp.TARGET_ORBITAL_ALTITUDE
V_T = ips.terminal_speed_target(R_T)


def fly(lam, d1, dc, d3, gamma_p):
    T = d1 + d3
    x = [lam[0], lam[1], lam[2], dc, 100.0 * T / T_MAX, 100.0 * d1 / T, gamma_p]
    with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
        warnings.simplefilter("ignore")
        res = ips.run_indirect_trajectory(*x)
    if res["crashed"]:
        raise RuntimeError("trajectory crashed")
    return res


def costates(a, b):
    return np.array([np.sin(b), np.cos(b) * np.cos(a), np.cos(b) * np.sin(a)])


def main():
    B = json.loads(REFINE_JSON.read_text(encoding="utf-8"))["variants"]["B"]
    a, b, tc, tr, cs, gp = B["u"]
    T = tr / 100.0 * T_MAX
    d1, dc, d3 = cs / 100.0 * T, tc, T - cs / 100.0 * T
    lam = costates(a, b)
    print(f"variant B (inertial): D1 {d1:.4f} s, Dc {dc:.4f} s, D3 {d3:.4f} s, "
          f"gamma_p {gp:.9f} rad, prop left {M_PROP_2 * (1 - T / T_MAX):.3f} kg")

    nom = fly(lam, d1, dc, d3, gp)
    lf = nom["costates_final"]
    H = {k: nom[k] for k in ("H_burn_start", "H_burn1_end", "H_coast_end",
                             "H_last_burn_start", "H_burn_end")}
    print("H values:  " + "  ".join(f"{k} {v:+.6f}" for k, v in H.items()))

    # 1. finite-difference sensitivities against the closed forms
    closed = {
        "Dc": H["H_coast_end"],
        "D3": H["H_burn_end"],
        "D1": H["H_burn1_end"] - H["H_last_burn_start"] + H["H_burn_end"],
    }
    print("\n1. lambda(t_f) . dx_f/dD  (central differences)  vs closed form")
    for step in (0.01, 0.03):
        for name in ("D1", "Dc", "D3"):
            dp = {"D1": (step, 0, 0), "Dc": (0, step, 0), "D3": (0, 0, step)}[name]
            sp_ = fly(lam, d1 + dp[0], dc + dp[1], d3 + dp[2], gp)["state_final_propagated"]
            sm_ = fly(lam, d1 - dp[0], dc - dp[1], d3 - dp[2], gp)["state_final_propagated"]
            dxf = (np.asarray(sp_)[1:4] - np.asarray(sm_)[1:4]) / (2 * step)
            fd = float(lf @ dxf)
            print(f"   h={step:<5} {name}:  FD {fd:+.6f}   closed {closed[name]:+.6f}   "
                  f"diff {fd - closed[name]:+.2e}")

    # 2. the conditions at B
    print("\n2. stationarity conditions at B")
    print(f"   H_coast_end                    = {H['H_coast_end']:+.6f}   (should be 0)")
    print(f"   H_burn1_end - H_last_burn_start = {H['H_burn1_end'] - H['H_last_burn_start']:+.6f}"
          f"   (should be 0)")
    print(f"   H_burn_end                     = {H['H_burn_end']:+.6f}   (should be < 0)")
    print(f"   solver's residual H_be+H_ce-H_bs = "
          f"{H['H_burn_end'] + H['H_coast_end'] - H['H_burn_start']:+.6f}")
    print(f"   scale: |H_burn_end| = {abs(H['H_burn_end']):.3f}, "
          f"|lambda_r v sin(gamma)| at the coast end would be O(1)")

    # 3. orbit + the two equalities, gamma_p fixed, from B
    u0 = np.array([a, b, d1, dc, d3])
    S = np.array([1e-3, 1e-3, 1.0, 1.0, 1.0])

    def resid(z):
        u = u0 + z * S
        try:
            res = fly(costates(u[0], u[1]), u[2], u[3], u[4], gp)
        except RuntimeError:
            return np.full(5, 1e3)
        s = res["state_final_propagated"]
        return np.array([(s[1] - R_T) / 1000.0, (s[2] - V_T) / 10.0,
                         np.rad2deg(s[3]) / 0.1,
                         res["H_coast_end"] / 1.0,
                         (res["H_burn1_end"] - res["H_last_burn_start"]) / 1.0])

    def jac(z):
        h = 1e-2
        J = np.zeros((5, 5))
        for j in range(5):
            e = np.zeros(5)
            e[j] = h
            J[:, j] = (resid(z + e) - resid(z - e)) / (2 * h)
        return J

    print("\n3. solve orbit (3) + H_coast_end = 0 + H_burn1_end = H_last_burn_start, gamma_p fixed")
    sol = least_squares(resid, np.zeros(5), jac=jac, method="lm", xtol=1e-12, ftol=1e-12,
                        max_nfev=200)
    u = u0 + sol.x * S
    r = resid(sol.x)
    fin = fly(costates(u[0], u[1]), u[2], u[3], u[4], gp)
    Tn = u[2] + u[4]
    print(f"   status {sol.status} ({sol.message}), nfev {sol.nfev}")
    print(f"   residuals: alt {r[0] * 1000:+.3e} m | vel {r[1] * 10:+.3e} m/s | "
          f"fpa {r[2] * 0.1:+.3e} deg | H_coast_end {r[3]:+.3e} | H1-H3 {r[4]:+.3e}")
    print(f"   D1 {u[2]:.4f} s, Dc {u[3]:.4f} s, D3 {u[4]:.4f} s  (B: {d1:.4f}, {dc:.4f}, {d3:.4f})")
    print(f"   H_burn_end {fin['H_burn_end']:+.6f} (must be < 0)")
    print(f"   prop left {M_PROP_2 * (1 - Tn / T_MAX):.3f} kg   (B: {M_PROP_2 * (1 - T / T_MAX):.3f} kg)")


if __name__ == "__main__":
    main()
