import numpy as np

from Auxiliary import constants as c
from Auxiliary import derived_quantities


def test_drag_direction_uses_air_relative_velocity_with_rotation():
    r_i = np.array([c.R_EARTH + 100e3, 0.0, 0.0], dtype=float)

    omega = np.array([0.0, 0.0, c.OMEGA_EARTH], dtype=float)
    v_atm = np.cross(omega, r_i)

    # Add inertial surplus eastward component.
    v_i = v_atm + np.array([0.0, 200.0, 0.0], dtype=float)

    u_d = derived_quantities.drag_direction_eci(r_i, v_i)
    v_rel = derived_quantities.air_relative_velocity_eci(r_i, v_i)
    u_rel = v_rel / np.linalg.norm(v_rel)

    assert np.allclose(u_d, u_rel, atol=1e-12)


def test_drag_direction_returns_zero_for_zero_air_relative_speed():
    r_i = np.array([c.R_EARTH, 0.0, 0.0], dtype=float)
    omega = np.array([0.0, 0.0, c.OMEGA_EARTH], dtype=float)
    v_i = np.cross(omega, r_i)

    u_d = derived_quantities.drag_direction_eci(r_i, v_i)
    assert np.allclose(u_d, np.zeros(3), atol=1e-12)
