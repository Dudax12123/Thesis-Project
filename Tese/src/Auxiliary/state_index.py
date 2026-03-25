"""
State index helpers for ascent simulation modes.

Legacy layouts are preserved, while pseudo-3DOF extends the state vector with
longitude and heading to support passive crossrange drift modeling.
"""


S = 0
R = 1
V = 2
GAMMA = 3
M = 4
LAT = 5
LON = 6
CHI = 7


def is_pseudo_3dof_state(state):
    """Return True when state includes lon/chi extension."""
    return len(state) >= 8
