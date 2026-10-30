"""
ORBIT TRIM MODULE

What it would cost to correct an insertion that is in position but not circular.

A law that cuts off at the target altitude with a flight-path angle inside the
tolerance can still leave an ellipse: peg_direct inserts at 500.04 km with a
speed 0.09 m/s off circular, yet its 0.06 deg flight-path angle puts it on a
492.8 x 507.0 km orbit. This module prices the correction as two impulsive
burns at the apsides, the textbook trim:

1. At the first apsis reached after insertion (apoapsis if the vehicle is
   climbing, periapsis if it is descending), change the speed so that the
   OPPOSITE apsis moves to the target radius.
2. Half a transfer period later, at that opposite apsis, change the speed again
   so that the orbit becomes circular at the target radius.

Each burn is priced with the rocket equation on the Stage-2 engine, and its
duration is the propellant over the engine's mass flow at constant thrust.
The burns are impulsive: at ~45 m/s^2 a trim of a few m/s lasts a fraction of a
second, so the finite-burn error is negligible against the miss it corrects.

Everything here is in the inertial frame, where the target orbit is circular
(the launch-site rotation model credits omega*r*cos(phi0) along-track, so the
rotating-frame target converts to exactly sqrt(mu/r_target)).

The module is pure, like Auxiliary/losses.py: numbers in, numbers out, no
module state, no knowledge of which solver produced the state. It is a report
only; nothing in a solve reads it.

Functions:
- in_position: the altitude / flight-path-angle gate
- apsidal_trim: the two-burn correction and its propellant and burn times
"""

import numpy as np

from Auxiliary import constants as c


def in_position(r, gamma_inertial, r_target, alt_tol_m, fpa_tol_rad):
    """True when the insertion altitude and flight-path angle are within tolerance.

    Only position is checked: a speed error is exactly what the trim corrects.
    """
    return bool(abs(r - r_target) <= alt_tol_m
                and abs(gamma_inertial) <= fpa_tol_rad)


def _burn(mass, dv, isp, thrust, g0=c.G_0):
    """Propellant [kg] and duration [s] of an impulsive burn of |dv| from `mass`."""
    prop = mass * (1.0 - np.exp(-abs(dv) / (g0 * isp)))
    mdot = thrust / (g0 * isp)
    return prop, prop / mdot


def _time_to_next_apsis(r, v, gamma, a, e, mu):
    """Coast time [s] from the state to the next apsis along the orbit."""
    if e < 1e-12:
        return 0.0
    p = a * (1.0 - e * e)
    e_cos = p / r - 1.0
    e_sin = v * np.sin(gamma) * np.sqrt(p / mu)
    nu = np.arctan2(e_sin, e_cos) % (2.0 * np.pi)
    E = 2.0 * np.arctan2(np.sqrt(1.0 - e) * np.sin(nu / 2.0),
                         np.sqrt(1.0 + e) * np.cos(nu / 2.0))
    M = (E - e * np.sin(E)) % (2.0 * np.pi)
    n = np.sqrt(mu / a**3)
    target_M = np.pi if M < np.pi else 2.0 * np.pi
    return float((target_M - M) / n)


def apsidal_trim(r, v_inertial, gamma_inertial, mass, r_target, isp, thrust,
                 mu=c.MU_EARTH, g0=c.G_0):
    """Two impulsive apsidal burns that turn the osculating orbit into a circle.

    Parameters
    ----------
    r, v_inertial, gamma_inertial : float
        Insertion radius [m], inertial speed [m/s], inertial flight-path angle [rad].
    mass : float
        Vehicle mass at insertion [kg].
    r_target : float
        Radius of the target circular orbit [m].
    isp, thrust : float
        Specific impulse [s] and thrust [N] of the engine that flies the trim.

    Returns
    -------
    dict
        ``status`` is ``"ok"``, ``"hyperbolic"`` (no apsis to burn at) or
        ``"impact"`` (the first apsis is a periapsis below the surface). Under
        ``"ok"``: the apsis of each burn (``burn1_at``, ``burn2_at``) with its
        altitude, the signed speed change of each (positive prograde), the
        propellant and burn time of each and their totals, and the coast from
        insertion to burn 1 and from burn 1 to burn 2.
    """
    energy = 0.5 * v_inertial**2 - mu / r
    if energy >= 0.0:
        return {'status': 'hyperbolic'}
    a = -mu / (2.0 * energy)
    h = r * v_inertial * np.cos(gamma_inertial)
    e = float(np.sqrt(max(0.0, 1.0 - h * h / (mu * a))))
    r_apo, r_peri = a * (1.0 + e), a * (1.0 - e)

    # Climbing means the apoapsis comes first. At gamma = 0 the vehicle sits on
    # an apsis: whichever one its radius is nearer to.
    if gamma_inertial > 0.0 or (gamma_inertial == 0.0 and r >= a):
        first, r1, r_other = 'apoapsis', r_apo, r_peri
    else:
        first, r1, r_other = 'periapsis', r_peri, r_apo
    if r1 <= c.R_EARTH:
        return {'status': 'impact'}
    second = 'periapsis' if first == 'apoapsis' else 'apoapsis'

    # Burn 1, at r1: move the opposite apsis from r_other to r_target.
    v1_before = np.sqrt(mu * (2.0 / r1 - 1.0 / a))
    a_transfer = 0.5 * (r1 + r_target)
    v1_after = np.sqrt(mu * (2.0 / r1 - 1.0 / a_transfer))
    dv1 = float(v1_after - v1_before)

    # Burn 2, at r_target: circularise.
    v2_before = np.sqrt(mu * (2.0 / r_target - 1.0 / a_transfer))
    dv2 = float(np.sqrt(mu / r_target) - v2_before)

    prop1, t1 = _burn(mass, dv1, isp, thrust, g0)
    prop2, t2 = _burn(mass - prop1, dv2, isp, thrust, g0)

    return {
        'status': 'ok',
        'burn1_at': first,
        'burn1_alt_km': float((r1 - c.R_EARTH) / 1e3),
        'burn2_at': second,
        'burn2_alt_km': float((r_target - c.R_EARTH) / 1e3),
        'moved_apsis_alt_km': float((r_other - c.R_EARTH) / 1e3),
        'dv1_ms': dv1,
        'dv2_ms': dv2,
        'dv_total_ms': abs(dv1) + abs(dv2),
        'prop1_kg': float(prop1),
        'prop2_kg': float(prop2),
        'prop_total_kg': float(prop1 + prop2),
        'burn1_s': float(t1),
        'burn2_s': float(t2),
        'burn_total_s': float(t1 + t2),
        'coast_to_burn1_s': _time_to_next_apsis(r, v_inertial, gamma_inertial,
                                                a, e, mu),
        'coast_between_burns_s': float(np.pi * np.sqrt(a_transfer**3 / mu)),
    }
