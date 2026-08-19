"""
DELTA-V BUDGET MODULE

Evaluates the ascent delta-v budget of Chapter 2 as *numbers*, so that it can be
tabulated across many runs rather than only drawn:

    dv_achieved = dv_ideal - (gravity + drag + steering + pressure) + gain + residual

The four loss integrals are those of Eq. (dv_loss_terms):

    gravity   = int g(r) sin(gamma) dt
    drag      = int (D/m) dt
    steering  = int (T/m) (1 - cos alpha) dt      [the 1-cos form of the identity]
    pressure  = int (p_a(h) A_e / m) dt           [stage-1 thrusting arc only]

and the ideal term is the VACUUM thrust integral,

    dv_ideal  = int (T_vac/m) dt = int ((T + p_a A_e)/m) dt ,

which reduces to the staged rocket equation for a constant exhaust velocity and
handles staging and coast arcs without being told where they are.

This module is deliberately pure: it takes arrays and returns numbers, holds no
module state, and knows nothing about the solver that produced the trajectory.
Both the plot layer and the results harness call it, so there is one definition
of a loss in the project rather than one per consumer.

Functions:
- loss_histories: cumulative delta-v loss curves along the trajectory
- delta_v_budget: the closed budget as scalars
- launch_site_gain: the rotational gain credited by Eq. (dv_gain)
"""
import sys
from pathlib import Path

# Add parent directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from Auxiliary import atmosphere as atm
from Auxiliary import constants as c
from Auxiliary import gravity as grav
from Auxiliary import rocket_specs as r


def _cumulative_trapezoid(f, t):
    """Cumulative integral of *f* over *t*, starting at zero, same length as *t*."""
    f = np.asarray(f, dtype=float)
    t = np.asarray(t, dtype=float)
    if len(t) < 2:
        return np.zeros_like(t)
    increments = 0.5 * (f[1:] + f[:-1]) * np.diff(t)
    return np.concatenate(([0.0], np.cumsum(increments)))


def _pressure_deficit_accel(t, alt, m, thrust, t_meco, thrust_mode,
                            include_drag=True):
    """Ambient-pressure thrust deficit per unit mass [m/s^2], and whether it applies.

    Two conditions have to hold for the term to be a real quantity.

    The trajectory must have been *flown* with the pressure-dependent thrust
    model: under the constant and time-ramped modes the thrust history does not
    depend on ambient pressure, so there is no deficit to integrate and reporting
    one would describe a vehicle that was not simulated.

    And there must have been an atmosphere to push back. ``include_drag`` mirrors
    the master no-atmosphere switch, under which the simulator flies vacuum
    thrust throughout (rocket_ascent._ambient_pressure_for_run); a pressure loss
    reported for such a run would be a deficit against air that was not there.

    When either fails this returns zeros and ``False``, and the caller must not
    present the result as the four-term budget of Chapter 2.
    """
    if thrust_mode != "pressure" or not include_drag:
        return np.zeros_like(np.asarray(t, dtype=float)), False

    t = np.asarray(t, dtype=float)
    thrust = np.asarray(thrust, dtype=float)
    stage1_thrusting = thrust > 0.0
    if t_meco is not None:
        stage1_thrusting = stage1_thrusting & (t <= t_meco)

    p_a = np.array([atm.ambient_pressure(float(max(0.0, h))) for h in alt])
    deficit = np.where(stage1_thrusting, p_a * r.A_E / np.asarray(m, dtype=float), 0.0)
    return deficit, True


def loss_histories(t, alt, v, gamma, m, thrust, alpha,
                   t_meco=None, include_drag=True, thrust_mode=None,
                   C_D=r.C_D, A=r.A):
    """Cumulative delta-v losses along the trajectory.

    Parameters
    ----------
    t, alt, v, gamma, m : array_like
        Time [s], altitude above the surface [m], speed [m/s], flight-path angle
        [rad] and total mass [kg], all on the same grid. Truncate them at the
        cut-off of interest before calling: this function integrates whatever it
        is given, from the first sample to the last.
    thrust, alpha : array_like
        Thrust [N] and angle of attack [rad] interpolated onto the same grid.
    t_meco : float, optional
        Main-engine cut-off time [s], used to confine the pressure deficit to the
        stage-1 arc. Without it, any thrusting sample is treated as stage 1.
    include_drag : bool
        Mirrors ``INCLUDE_DRAG``, the master no-atmosphere switch. False zeroes
        the drag integral, matching the no-atmosphere runs where the force is not
        in the equations of motion, and suppresses the pressure term with it: no
        air means no back-pressure on the nozzle either.
    thrust_mode : str, optional
        The ``THRUST_1_MODE`` the trajectory was flown under. Only "pressure"
        produces a pressure loss; see ``_pressure_deficit_accel``.

    Returns
    -------
    dict with cumulative arrays 'gravity', 'drag', 'steering', 'pressure',
    'total' and 'ideal', plus the bool 'pressure_applicable'.
    """
    t = np.asarray(t, dtype=float)
    alt = np.asarray(alt, dtype=float)
    gamma = np.asarray(gamma, dtype=float)
    m = np.asarray(m, dtype=float)
    thrust = np.asarray(thrust, dtype=float)
    alpha = np.asarray(alpha, dtype=float)

    r_val = alt + c.R_EARTH
    g = np.array([grav.gravitational_acceleration(float(rv)) for rv in r_val])

    q = np.array([atm.dynamic_pressure(float(v_i), float(max(0.0, h)))
                  for v_i, h in zip(np.asarray(v, dtype=float), alt)])
    drag_accel = (q * C_D * A) / m if include_drag else np.zeros_like(t)

    thrust_accel = thrust / m
    steering_accel = thrust_accel * (1.0 - np.cos(alpha))

    deficit_accel, pressure_applicable = _pressure_deficit_accel(
        t, alt, m, thrust, t_meco, thrust_mode, include_drag=include_drag)

    gravity = _cumulative_trapezoid(g * np.sin(gamma), t)
    drag = _cumulative_trapezoid(drag_accel, t)
    steering = _cumulative_trapezoid(steering_accel, t)
    pressure = _cumulative_trapezoid(deficit_accel, t)
    # Vacuum thrust integral: the deficit is added back on, so this is what the
    # engine would have delivered with no ambient pressure to work against.
    ideal = _cumulative_trapezoid(thrust_accel + deficit_accel, t)

    return {
        'gravity': gravity,
        'drag': drag,
        'steering': steering,
        'pressure': pressure,
        'total': gravity + drag + steering + pressure,
        'ideal': ideal,
        'pressure_applicable': pressure_applicable,
    }


def launch_site_gain():
    """Rotational gain credited by Eq. (dv_gain), read from the active configuration.

    Returns 0.0 when Earth rotation is disabled, which is the physically correct
    credit for a non-rotating Earth rather than a missing value.
    """
    from Input_File import simulation_parameters as sim_params
    from Auxiliary import earth_rotation as earth_rot

    if not sim_params.ENABLE_EARTH_ROTATION:
        return 0.0

    _, beta_formula, _ = earth_rot.select_launch_azimuth(
        sim_params.TARGET_ORBIT_INCLINATION,
        sim_params.LAUNCH_LATITUDE,
        sim_params.TARGET_ORBITAL_ALTITUDE,
    )
    return float(earth_rot.delta_v_gain(
        sim_params.LAUNCH_LATITUDE,
        beta_formula,
        c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE,
    ))


def delta_v_budget(t, alt, v, gamma, m, thrust, alpha,
                   t_meco=None, include_drag=True, thrust_mode=None,
                   dv_gain=None, C_D=r.C_D, A=r.A):
    """The closed delta-v budget as scalars.

    Same arguments as ``loss_histories``, plus:

    dv_gain : float, optional
        The rotational gain. Defaults to ``launch_site_gain()``, i.e. read from
        the active configuration.

    Returns
    -------
    dict of floats: 'dv_ideal', 'dv_gravity', 'dv_drag', 'dv_steering',
    'dv_pressure', 'dv_losses', 'dv_gain', 'dv_achieved', 'residual', plus the
    bool 'pressure_applicable'.

    The residual is ``dv_ideal - dv_losses + dv_gain - dv_achieved``. It is not
    expected to vanish: the identity is derived from the tangential equation of
    motion alone, so the residual collects the work done by the rotating-frame
    pseudo-forces, the frame offset between the speed the simulator integrates
    and the inertial speed the gain refers to, and the integration error of the
    output grid. It is reported so that a budget which fails to close for some
    *other* reason is visible rather than absorbed.
    """
    hist = loss_histories(t, alt, v, gamma, m, thrust, alpha,
                          t_meco=t_meco, include_drag=include_drag,
                          thrust_mode=thrust_mode, C_D=C_D, A=A)

    if dv_gain is None:
        dv_gain = launch_site_gain()

    v = np.asarray(v, dtype=float)
    dv_achieved = float(v[-1] - v[0]) if len(v) else 0.0
    dv_losses = float(hist['total'][-1]) if len(hist['total']) else 0.0
    dv_ideal = float(hist['ideal'][-1]) if len(hist['ideal']) else 0.0

    return {
        'dv_ideal': dv_ideal,
        'dv_gravity': float(hist['gravity'][-1]) if len(hist['gravity']) else 0.0,
        'dv_drag': float(hist['drag'][-1]) if len(hist['drag']) else 0.0,
        'dv_steering': float(hist['steering'][-1]) if len(hist['steering']) else 0.0,
        'dv_pressure': float(hist['pressure'][-1]) if len(hist['pressure']) else 0.0,
        'dv_losses': dv_losses,
        'dv_gain': float(dv_gain),
        'dv_achieved': dv_achieved,
        'residual': dv_ideal - dv_losses + float(dv_gain) - dv_achieved,
        'pressure_applicable': hist['pressure_applicable'],
    }
