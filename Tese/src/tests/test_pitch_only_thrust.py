import numpy as np

from Auxiliary import constants as c
from Auxiliary import frames


def test_thrust_direction_stays_in_local_vertical_plane():
    time_s = 321.0
    r_i = np.array([c.R_EARTH + 5_000.0, 100.0, 50.0], dtype=float)
    v_i = np.array([10.0, 7600.0, 120.0], dtype=float)
    pitch = np.deg2rad(12.0)
    fallback_az = np.deg2rad(90.0)

    u_t = frames.thrust_direction_pitch_only(
        r_I=r_i,
        vel_ref_I=v_i,
        time_s=time_s,
        pitch_from_horizontal_rad=pitch,
        fallback_azimuth_rad=fallback_az,
    )

    up_i, east_i, north_i = frames.local_basis_from_eci(r_i, time_s)
    u_h = frames.local_horizontal_direction(r_i, v_i, time_s, fallback_az)

    # No component outside span{u_h, up}.
    plane_normal = np.cross(u_h, up_i)
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    out_of_plane = float(np.dot(u_t, plane_normal))

    assert np.isclose(np.linalg.norm(u_t), 1.0, atol=1e-12)
    assert np.isclose(out_of_plane, 0.0, atol=1e-12)

    # Additional sanity: no unintended basis-axis contamination check is implicit in out_of_plane.
    assert np.isclose(np.dot(u_t, east_i) ** 2 + np.dot(u_t, north_i) ** 2 + np.dot(u_t, up_i) ** 2, 1.0, atol=1e-12)


def test_horizontal_fallback_uses_initial_azimuth_when_vertical_velocity():
    time_s = 0.0
    r_i = np.array([c.R_EARTH, 0.0, 0.0], dtype=float)
    v_vertical = np.array([100.0, 0.0, 0.0], dtype=float)
    fallback_az = np.deg2rad(45.0)

    u_h = frames.local_horizontal_direction(r_i, v_vertical, time_s, fallback_az)
    up_i, east_i, north_i = frames.local_basis_from_eci(r_i, time_s)

    expected = np.cos(fallback_az) * north_i + np.sin(fallback_az) * east_i
    expected = expected / np.linalg.norm(expected)

    assert np.isclose(np.dot(u_h, up_i), 0.0, atol=1e-12)
    assert np.allclose(u_h, expected, atol=1e-12)
