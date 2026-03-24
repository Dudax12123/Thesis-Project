import numpy as np

from Auxiliary import constants as c
from Simulation.eci_point_mass import ECIPropagationConfig, point_mass_3dof_eci


def _thrust_model(_t, _state):
    return 100_000.0, 300.0


def _pitch_model(_t, _state):
    return np.deg2rad(5.0)


def test_eci_point_mass_smoke_derivative_shape_and_signs():
    cfg = ECIPropagationConfig(
        drag_coefficient=0.3,
        reference_area_m2=10.52,
    )

    r_i = np.array([c.R_EARTH + 1_000.0, 0.0, 0.0], dtype=float)
    v_i = np.array([0.0, 50.0, 0.0], dtype=float)
    m = 500_000.0
    state = np.concatenate((r_i, v_i, [m]))

    deriv = point_mass_3dof_eci(
        t_s=0.0,
        state=state,
        thrust_model=_thrust_model,
        pitch_model=_pitch_model,
        fallback_azimuth_rad=np.deg2rad(90.0),
        cfg=cfg,
    )

    assert deriv.shape == (7,)
    assert np.all(np.isfinite(deriv))

    # dr/dt equals v
    assert np.allclose(deriv[0:3], v_i, atol=1e-12)

    # Mass must decrease with positive thrust.
    assert deriv[6] < 0.0

    # On +X axis, gravity must contribute negative X acceleration.
    assert deriv[3] < 0.0
