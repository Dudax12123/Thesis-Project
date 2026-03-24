"""Launch azimuth helpers for spherical-Earth ascent initialization.

Azimuth convention: clockwise from north.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from Auxiliary import constants as c
from Auxiliary import frames


def inertial_azimuth_for_inclination(target_inclination_deg: float, launch_latitude_deg: float) -> float:
    """Classical inertial azimuth from spherical-Earth geometry.

    Relation for prograde ascending solution:
        sin(beta_I) = cos(i) / cos(phi)
    """
    i_rad = np.deg2rad(target_inclination_deg)
    phi_rad = np.deg2rad(launch_latitude_deg)

    cos_phi = float(np.cos(phi_rad))
    if np.isclose(cos_phi, 0.0):
        raise ValueError("Launch latitude too close to the poles for azimuth computation.")

    sin_beta_i = float(np.cos(i_rad) / cos_phi)
    if sin_beta_i < -1.0 - 1e-12 or sin_beta_i > 1.0 + 1e-12:
        raise ValueError("Target inclination is unreachable from this latitude in prograde ascent.")

    sin_beta_i = float(np.clip(sin_beta_i, -1.0, 1.0))
    return float(np.arcsin(sin_beta_i))


def launch_site_position_ecef(lat_deg: float, lon_deg: float, radius_m: float = c.R_EARTH) -> np.ndarray:
    """Launch-site position in ECEF for spherical Earth."""
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    clat = np.cos(lat)
    return np.array(
        [
            radius_m * clat * np.cos(lon),
            radius_m * clat * np.sin(lon),
            radius_m * np.sin(lat),
        ],
        dtype=float,
    )


def local_north_east_eci(lat_deg: float, lon_deg: float, time_s: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """Return local north/east unit vectors in ECI at launch site."""
    r_e = launch_site_position_ecef(lat_deg, lon_deg)
    basis_e = frames.local_basis_from_ecef(r_e)
    north_i = frames.ecef_to_eci(basis_e.north_E, time_s)
    east_i = frames.ecef_to_eci(basis_e.east_E, time_s)
    return north_i, east_i


def corrected_launch_azimuth_with_rotation(
    target_inclination_deg: float,
    launch_latitude_deg: float,
    launch_longitude_deg: float,
    target_altitude_m: float,
    time_s: float = 0.0,
) -> Tuple[float, float, float]:
    """Return (beta_launch, beta_inertial, v_surface_east).

    beta_inertial is the geometric inertial azimuth.
    beta_launch is the Earth-fixed azimuth that yields beta_inertial after adding
    launch-site Earth-rotation velocity to the initial inertial velocity vector.
    """
    beta_inertial = inertial_azimuth_for_inclination(target_inclination_deg, launch_latitude_deg)

    r_target = c.R_EARTH + target_altitude_m
    v_orb = np.sqrt(c.MU_EARTH / r_target)

    north_i, east_i = local_north_east_eci(launch_latitude_deg, launch_longitude_deg, time_s=time_s)
    u_i = np.cos(beta_inertial) * north_i + np.sin(beta_inertial) * east_i
    v_desired_i = v_orb * u_i

    r_launch_i = frames.ecef_to_eci(launch_site_position_ecef(launch_latitude_deg, launch_longitude_deg), time_s)
    v_atm_i = np.cross(frames.earth_rotation_vector(), r_launch_i)

    # Required air-relative launch velocity in local horizontal plane.
    v_rel_i = v_desired_i - v_atm_i

    # Project onto local N/E to get Earth-fixed azimuth.
    v_n = float(np.dot(v_rel_i, north_i))
    v_e = float(np.dot(v_rel_i, east_i))
    beta_launch = float(np.arctan2(v_e, v_n))

    v_surface_east = c.OMEGA_EARTH * c.R_EARTH * np.cos(np.deg2rad(launch_latitude_deg))
    return beta_launch, beta_inertial, float(v_surface_east)
