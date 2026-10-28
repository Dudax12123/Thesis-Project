"""A generic two-burn ascent profile for the schematic figures of Section 4.5.3.

Not simulator output: a smooth altitude-against-time shape in normalized units
(time 0 to 1.8, altitude 0 to 1 = the target orbit), with a burn that ends low
and flat at T_COAST0, a coast that climbs to apoapsis at the orbit altitude,
and a short final burn there. fig_segmented_waypoints.py and
fig_segmented_schedule.py draw on it, so the two figures show one ascent.
"""
import numpy as np
from scipy.interpolate import PchipInterpolator

T_KICK = 0.03
T_MECO = 0.36
T_COAST0 = 1.00          # end of the first second-stage burn (coast start)
T_COAST1 = 1.74          # end of the coast, start of the final burn
T_END = 1.80             # insertion
H_ORB = 1.0

_T = [0.00, 0.15, 0.36, 0.55, 0.75, 1.00, 1.15, 1.40, 1.62, 1.74, 1.80]
_H = [0.00, 0.05, 0.20, 0.32, 0.40, 0.45, 0.52, 0.75, 0.95, 0.99, 1.00]
_curve = PchipInterpolator(_T, _H)


def h(t):
    return _curve(np.asarray(t, dtype=float))


def t_at(alt):
    """First time the profile climbs through ``alt``."""
    tt = np.linspace(0.0, T_END, 4001)
    return float(np.interp(alt, h(tt), tt))


def grid(a, b, n=400):
    return np.linspace(a, b, n)
