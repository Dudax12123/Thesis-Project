"""Does the exact planar ECEF->ECI conversion make the apogee-check event land on the cut-off?

apogee_check detects the cut-off with the Kepler apogee of the SECO state converted to
the inertial frame by ecef_to_eci_velocity, which keeps gamma_eci = gamma. The cut-off is
then placed by a root search on the rotating-frame coast (ra._finish_single_burn_rotating).
This script asks what the event would do with the exact conversion
(earth_rot.rotating_to_inertial_planar), without touching the simulator:

1. identity: rotating_to_inertial_planar then inertial_to_rotating_planar returns the input,
   and at gamma = 0 it agrees with ecef_to_eci_velocity;
2. along the burn continued past the event, the Kepler apogee predicted with the old and
   with the exact conversion, against the apoapsis the rotating-frame coast reaches;
3. the instant each prediction crosses the target against the root-found cut-off.

Run from the repository root:
  PYTHONPATH=Tese/src PYTHONIOENCODING=utf-8 \
    C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/exact_conversion_check.py
"""
import sys
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Tese" / "src"))

from Auxiliary import constants as c
from Auxiliary import earth_rotation as earth_rot
from Input_File import simulation_parameters as sim_params
import Simulation.rocket_ascent as ra

# gt_apogee's kick in the 2026-09-26 results matrix [rad], and neighbours
KICKS = [-0.030996526995096696 + d for d in (0.0, 0.0015, 0.003)]
SAMPLES_S = (0.0, 1.0, 2.0, 4.0, 8.0)       # seconds past the old event


def identity_check():
    rng = np.random.default_rng(1)
    r_v = c.R_EARTH + rng.uniform(70e3, 400e3, 200)
    v = rng.uniform(2000.0, 7500.0, 200)
    g = rng.uniform(-0.1, 0.6, 200)
    lat = ra.LAUNCH_LATITUDE_RAD
    vi, gi = earth_rot.rotating_to_inertial_planar(v, g, lat, r_v)
    vb, gb = earth_rot.inertial_to_rotating_planar(vi, gi, lat, r_v)
    print("1. round trip: max |dv| = %.2e m/s, max |dgamma| = %.2e rad"
          % (np.max(np.abs(vb - v)), np.max(np.abs(gb - g))))
    v0, g0 = earth_rot.ecef_to_eci_velocity(3000.0, 0.0, lat, c.R_EARTH + 100e3)
    v1, g1 = earth_rot.rotating_to_inertial_planar(3000.0, 0.0, lat, c.R_EARTH + 100e3)
    print("   at gamma = 0: old (%.6f, %.6f)  exact (%.6f, %.6f)" % (v0, g0, v1, g1))


def kepler_apogee(r_val, v, gamma, exact):
    lat = ra.LAUNCH_LATITUDE_RAD
    conv = earth_rot.rotating_to_inertial_planar if exact else earth_rot.ecef_to_eci_velocity
    vi, gi = conv(v, gamma, lat, r_val)
    return ra.get_orbital_elements(r_val, vi, gi)[2]


def analyse(sol_1, sol_2, fairing_in_mass):
    """The burn past the old event, as _finish_single_burn_rotating continues it."""
    r_target = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    t_a = float(sol_2.t[-1])
    y_a = np.asarray(sol_2.y[:, -1], dtype=float)
    for ev in (ra.interrupt_radius_check, ra.interrupt_stage_2_burnt, ra.interrupt_ground_collision):
        ev.terminal, ev.direction = True, 0
    t_cap = t_a + ra.r.M_PROP_2 / (ra.r.F_THRUST_2 / (c.G_0 * ra.r.ISP_2)) + 1.0
    burn = solve_ivp(ra.rocket_dynamics, (t_a, t_cap), y_a, dense_output=True, max_step=1,
                     events=[ra.interrupt_radius_check, ra.interrupt_stage_2_burnt,
                             ra.interrupt_ground_collision], atol=1e-8)
    t_end = float(burn.t[-1])

    def coast_apo(t):
        return ra._apoapsis_radius_miss(t, burn.sol(t), r_target) + r_target

    def pred(t, exact):
        y = burn.sol(t)
        return kepler_apogee(y[1], y[2], y[3], exact)

    print("   %6s %12s %12s %12s %12s %12s" % ("t-t_a", "coast apo", "old pred", "old err",
                                              "exact pred", "exact err"))
    for dt in SAMPLES_S:
        t = min(t_a + dt, t_end)
        ca = coast_apo(t)
        po, pe = pred(t, False), pred(t, True)
        print("   %6.1f %10.3f km %10.3f km %9.1f m %10.3f km %9.1f m"
              % (dt, (ca - c.R_EARTH) / 1e3, (po - c.R_EARTH) / 1e3, po - ca,
                 (pe - c.R_EARTH) / 1e3, pe - ca))

    def first_crossing(f):
        """First 1 s step at which f turns non-negative (the burn goes unbound later,
        where the Kepler apogee is meaningless), then Brent inside it."""
        lo, probe = t_a, t_a
        if f(lo) >= 0.0:
            return lo
        while probe < t_end:
            probe = min(probe + 1.0, t_end)
            if f(probe) >= 0.0:
                return brentq(f, lo, probe, xtol=1e-6)
            lo = probe
        return float("nan")

    roots = {name: first_crossing(lambda t, e=exact: pred(t, e) - r_target)
             for name, exact in (("old", False), ("exact", True))}
    lo, hi, probe = t_a, None, t_a
    while probe < t_end:
        probe = min(probe + 1.0, t_end)
        if coast_apo(probe) - r_target >= 0.0:
            hi = probe
            break
        lo = probe
    if hi is None:
        print("   burnt out below the target apoapsis")
        return
    t_cut = brentq(lambda t: coast_apo(t) - r_target, lo, hi, xtol=1e-6)
    print("   3. crossing of the target, T+ s:  old event %.3f   exact event %.3f   "
          "root-found cut-off %.3f" % (roots["old"], roots["exact"], t_cut))
    print("      exact event - cut-off = %.3f s;  old event - cut-off = %.3f s"
          % (roots["exact"] - t_cut, roots["old"] - t_cut))


def main():
    identity_check()
    orig = ra._finish_single_burn_rotating

    def wrapper(sol_1, sol_2, fairing_in_mass):
        analyse(sol_1, sol_2, fairing_in_mass)
        return orig(sol_1, sol_2, fairing_in_mass)

    ra._finish_single_burn_rotating = wrapper
    sim_params.GUIDANCE_MODE = "gravity_turn"
    sim_params.COAST_METHOD = "apogee_check"
    sim_params.KICK_PROFILE_MODE = "instantaneous"
    sim_params.APOGEE_CHECK_COAST_FRAME = "rotating"
    ra.SINGLE_BURN_FULL_SIMULATION = False
    for k in KICKS:
        print("\n2. kick %.6f rad" % k)
        try:
            ra.run(k)
        except RuntimeError as exc:
            print("   run refused: %s" % str(exc)[:120])


if __name__ == "__main__":
    main()
