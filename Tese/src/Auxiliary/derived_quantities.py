"""Derived kinematic quantities for 3-DOF ECI ascent states."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from Auxiliary import constants as c
from Auxiliary import frames


@dataclass(frozen=True)
class DerivedFlightState:
    """Derived local/inertial quantities for outputs and guidance."""

    radius_m: float
    altitude_m: float
    latitude_rad: float
    longitude_rad: float
    speed_inertial_mps: float
    speed_air_relative_mps: float
    gamma_rad: float
    heading_rad: float


def atmosphere_velocity_eci(r_I: np.ndarray) -> np.ndarray:
    """Atmosphere velocity in ECI under rigid co-rotation model."""
    return np.cross(frames.earth_rotation_vector(), r_I)


def air_relative_velocity_eci(r_I: np.ndarray, v_I: np.ndarray) -> np.ndarray:
    """Air-relative velocity in ECI."""
    return v_I - atmosphere_velocity_eci(r_I)


def drag_direction_eci(r_I: np.ndarray, v_I: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """Drag unit vector in ECI, opposite air-relative velocity."""
    v_rel_I = air_relative_velocity_eci(r_I, v_I)
    u_rel, rel_norm = frames.safe_normalize(v_rel_I, eps=eps)
    if rel_norm < eps:
        return np.zeros(3, dtype=float)
    return u_rel


def drag_magnitude(altitude_m: float, v_rel_norm_mps: float, c_d: float, area_m2: float) -> float:
    """Drag magnitude using existing exponential atmosphere model style."""
    rho = c.RHO_0 * np.exp(-altitude_m / c.H)
    q = 0.5 * rho * (v_rel_norm_mps ** 2)
    return float(q * c_d * area_m2)


def compute_derived_flight_state(time_s: float, r_I: np.ndarray, v_I: np.ndarray) -> DerivedFlightState:
    """Compute derived quantities from ECI position/velocity."""
    r_E = frames.eci_to_ecef(r_I, time_s)
    radius, altitude, lat_gc, lon = frames.geocentric_lat_lon_from_ecef(r_E)

    v_rel_I = air_relative_velocity_eci(r_I, v_I)

    speed_I = float(np.linalg.norm(v_I))
    speed_rel = float(np.linalg.norm(v_rel_I))
    gamma = frames.flight_path_angle(r_I, v_I, time_s)
    heading = frames.heading_from_velocity(r_I, v_I, time_s)

    return DerivedFlightState(
        radius_m=radius,
        altitude_m=altitude,
        latitude_rad=lat_gc,
        longitude_rad=lon,
        speed_inertial_mps=speed_I,
        speed_air_relative_mps=speed_rel,
        gamma_rad=gamma,
        heading_rad=heading,
    )
