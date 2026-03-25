import numpy as np

from Auxiliary import constants as c
from Auxiliary import derived_quantities


def test_drag_direction_uses_inertial_velocity_for_non_rotating_atmosphere():
    r_i = np.array([c.R_EARTH + 100e3, 0.0, 0.0], dtype=float)

    v_i = np.array([0.0, 200.0, 50.0], dtype=float)

    u_d = derived_quantities.drag_direction_eci(r_i, v_i)
    u_rel = v_i / np.linalg.norm(v_i)

    assert np.allclose(u_d, u_rel, atol=1e-12)


def test_drag_direction_returns_zero_for_zero_air_relative_speed():
    r_i = np.array([c.R_EARTH, 0.0, 0.0], dtype=float)
    v_i = np.zeros(3, dtype=float)

    u_d = derived_quantities.drag_direction_eci(r_i, v_i)
    assert np.allclose(u_d, np.zeros(3), atol=1e-12)
