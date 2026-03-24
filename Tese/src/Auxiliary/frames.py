"""Frame and vector utilities for ECI/ECEF ascent modeling.

Conventions:
- Spherical Earth rotating about +Z with angular rate c.OMEGA_EARTH.
- ECI is inertial.
- ECEF rotates with Earth.
- Azimuth/heading are clockwise from north toward east.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from Auxiliary import constants as c


_EPS = 1e-12


@dataclass(frozen=True)
class LocalBasis:
    """Local ENU-like basis at a position on spherical Earth in ECEF."""

    up_E: np.ndarray
    east_E: np.ndarray
    north_E: np.ndarray


def _rotation_z(theta: float) -> np.ndarray:
    cth = np.cos(theta)
    sth = np.sin(theta)
    return np.array(
        [
            [cth, -sth, 0.0],
            [sth, cth, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def safe_normalize(vec: np.ndarray, eps: float = _EPS) -> Tuple[np.ndarray, float]:
    """Return normalized vector and its norm with a small-norm safeguard."""
    norm = float(np.linalg.norm(vec))
    if norm < eps:
        return np.zeros(3, dtype=float), norm
    return vec / norm, norm


def earth_rotation_vector() -> np.ndarray:
    """Earth angular velocity vector in ECI [rad/s]."""
    return np.array([0.0, 0.0, c.OMEGA_EARTH], dtype=float)


def eci_to_ecef(vec_I: np.ndarray, time_s: float) -> np.ndarray:
    """Rotate a 3-vector from ECI to ECEF at epoch time_s."""
    return _rotation_z(-c.OMEGA_EARTH * time_s) @ vec_I


def ecef_to_eci(vec_E: np.ndarray, time_s: float) -> np.ndarray:
    """Rotate a 3-vector from ECEF to ECI at epoch time_s."""
    return _rotation_z(c.OMEGA_EARTH * time_s) @ vec_E


def geocentric_lat_lon_from_ecef(r_E: np.ndarray) -> Tuple[float, float, float, float]:
    """Return (radius, altitude, geocentric_lat, longitude) from ECEF position."""
    x_e, y_e, z_e = r_E
    radius = float(np.linalg.norm(r_E))
    if radius < _EPS:
        raise ValueError("ECEF position norm is too small to compute geocentric coordinates.")

    altitude = radius - c.R_EARTH
    lat_gc = float(np.arcsin(np.clip(z_e / radius, -1.0, 1.0)))
    lon = float(np.arctan2(y_e, x_e))
    return radius, altitude, lat_gc, lon


def local_basis_from_ecef(r_E: np.ndarray) -> LocalBasis:
    """Compute local up/east/north basis vectors in ECEF."""
    up_E, r_norm = safe_normalize(r_E)
    if r_norm < _EPS:
        raise ValueError("Cannot compute local basis at near-zero position.")

    z_hat = np.array([0.0, 0.0, 1.0], dtype=float)
    east_raw = np.cross(z_hat, up_E)
    east_E, east_norm = safe_normalize(east_raw)

    # At/near poles, define east from longitude in x-y plane for continuity.
    if east_norm < _EPS:
        lon = float(np.arctan2(r_E[1], r_E[0]))
        east_E = np.array([-np.sin(lon), np.cos(lon), 0.0], dtype=float)
        east_E, _ = safe_normalize(east_E)

    north_E = np.cross(up_E, east_E)
    north_E, _ = safe_normalize(north_E)

    return LocalBasis(up_E=up_E, east_E=east_E, north_E=north_E)


def local_basis_from_eci(r_I: np.ndarray, time_s: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute local up/east/north basis vectors in ECI at time_s."""
    r_E = eci_to_ecef(r_I, time_s)
    basis_E = local_basis_from_ecef(r_E)
    up_I = ecef_to_eci(basis_E.up_E, time_s)
    east_I = ecef_to_eci(basis_E.east_E, time_s)
    north_I = ecef_to_eci(basis_E.north_E, time_s)
    return up_I, east_I, north_I


def local_horizontal_direction(
    r_I: np.ndarray,
    vel_ref_I: np.ndarray,
    time_s: float,
    fallback_azimuth_rad: float,
) -> np.ndarray:
    """Return horizontal unit direction from velocity projection, with azimuth fallback."""
    up_I, east_I, north_I = local_basis_from_eci(r_I, time_s)

    vel_horizontal = vel_ref_I - np.dot(vel_ref_I, up_I) * up_I
    u_h, h_norm = safe_normalize(vel_horizontal)
    if h_norm >= _EPS:
        return u_h

    # Fallback direction from launch azimuth (clockwise from north).
    fallback = np.cos(fallback_azimuth_rad) * north_I + np.sin(fallback_azimuth_rad) * east_I
    u_h_fb, _ = safe_normalize(fallback)
    return u_h_fb


def thrust_direction_pitch_only(
    r_I: np.ndarray,
    vel_ref_I: np.ndarray,
    time_s: float,
    pitch_from_horizontal_rad: float,
    fallback_azimuth_rad: float,
) -> np.ndarray:
    """Build pitch-only thrust unit vector in local vertical plane.

    The command has no yaw/bank degree of freedom and lies in span{u_h, u_up}.
    """
    up_I, _, _ = local_basis_from_eci(r_I, time_s)
    u_h = local_horizontal_direction(r_I, vel_ref_I, time_s, fallback_azimuth_rad)
    u_t = np.cos(pitch_from_horizontal_rad) * u_h + np.sin(pitch_from_horizontal_rad) * up_I
    u_t, _ = safe_normalize(u_t)
    return u_t


def heading_from_velocity(
    r_I: np.ndarray,
    vel_ref_I: np.ndarray,
    time_s: float,
) -> float:
    """Heading angle clockwise from north in local horizontal plane [rad]."""
    up_I, east_I, north_I = local_basis_from_eci(r_I, time_s)
    vel_horizontal = vel_ref_I - np.dot(vel_ref_I, up_I) * up_I
    u_h, h_norm = safe_normalize(vel_horizontal)
    if h_norm < _EPS:
        return 0.0

    east_comp = float(np.dot(u_h, east_I))
    north_comp = float(np.dot(u_h, north_I))
    return float(np.arctan2(east_comp, north_comp))


def flight_path_angle(r_I: np.ndarray, vel_ref_I: np.ndarray, time_s: float) -> float:
    """Flight-path angle relative to local horizontal [rad]."""
    up_I, _, _ = local_basis_from_eci(r_I, time_s)
    speed = float(np.linalg.norm(vel_ref_I))
    if speed < _EPS:
        return 0.0

    v_up = float(np.dot(vel_ref_I, up_I))
    v_h = np.sqrt(max(speed * speed - v_up * v_up, 0.0))
    return float(np.arctan2(v_up, v_h))
