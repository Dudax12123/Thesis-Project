import numpy as np

from Auxiliary import constants as c
from Auxiliary import frames


def test_eci_ecef_round_trip_consistency():
    vec_i = np.array([7_000e3, -2_000e3, 1_100e3], dtype=float)
    time_s = 1234.5

    vec_e = frames.eci_to_ecef(vec_i, time_s)
    vec_i_back = frames.ecef_to_eci(vec_e, time_s)

    assert np.allclose(vec_i_back, vec_i, rtol=0.0, atol=1e-9)
    assert np.isclose(np.linalg.norm(vec_e), np.linalg.norm(vec_i), rtol=0.0, atol=1e-9)


def test_local_basis_is_orthonormal_and_right_handed():
    lat = np.deg2rad(28.5)
    lon = np.deg2rad(-80.5)
    r_e = c.R_EARTH * np.array([
        np.cos(lat) * np.cos(lon),
        np.cos(lat) * np.sin(lon),
        np.sin(lat),
    ])

    basis = frames.local_basis_from_ecef(r_e)

    assert np.isclose(np.linalg.norm(basis.up_E), 1.0, atol=1e-12)
    assert np.isclose(np.linalg.norm(basis.east_E), 1.0, atol=1e-12)
    assert np.isclose(np.linalg.norm(basis.north_E), 1.0, atol=1e-12)

    assert np.isclose(np.dot(basis.up_E, basis.east_E), 0.0, atol=1e-12)
    assert np.isclose(np.dot(basis.up_E, basis.north_E), 0.0, atol=1e-12)
    assert np.isclose(np.dot(basis.east_E, basis.north_E), 0.0, atol=1e-12)

    assert np.allclose(np.cross(basis.east_E, basis.north_E), basis.up_E, atol=1e-12)


def test_geocentric_lat_lon_recovery_from_ecef():
    r_eq = np.array([c.R_EARTH, 0.0, 0.0], dtype=float)
    _, h_eq, lat_eq, lon_eq = frames.geocentric_lat_lon_from_ecef(r_eq)
    assert np.isclose(h_eq, 0.0, atol=1e-9)
    assert np.isclose(lat_eq, 0.0, atol=1e-12)
    assert np.isclose(lon_eq, 0.0, atol=1e-12)

    r_np = np.array([0.0, 0.0, c.R_EARTH], dtype=float)
    _, _, lat_np, lon_np = frames.geocentric_lat_lon_from_ecef(r_np)
    assert np.isclose(lat_np, np.pi / 2.0, atol=1e-12)
    assert np.isclose(lon_np, 0.0, atol=1e-12)

    r_q2 = np.array([0.0, c.R_EARTH, 0.0], dtype=float)
    _, _, lat_q2, lon_q2 = frames.geocentric_lat_lon_from_ecef(r_q2)
    assert np.isclose(lat_q2, 0.0, atol=1e-12)
    assert np.isclose(lon_q2, np.pi / 2.0, atol=1e-12)
