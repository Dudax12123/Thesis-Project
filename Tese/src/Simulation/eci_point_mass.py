"""Conventional 3-DOF point-mass ascent dynamics in ECI.

State vector:
    x = [r_Ix, r_Iy, r_Iz, v_Ix, v_Iy, v_Iz, m]

Dynamics:
    dr_I/dt = v_I
    dv_I/dt = (T/m) u_T - (D/m) u_D - mu * r_I / |r_I|^3
    dm/dt   = -T / (Isp * g0)

Atmosphere model used here is non-rotating in inertial space:
    v_atm_I = 0
    v_rel_I = v_I
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Tuple

import numpy as np

from Auxiliary import constants as c
from Auxiliary import frames


StateVector = np.ndarray
ThrustModel = Callable[[float, StateVector], Tuple[float, float]]
PitchModel = Callable[[float, StateVector], float]
DensityModel = Callable[[float], float]


@dataclass(frozen=True)
class ECIPropagationConfig:
    """Configuration for 3-DOF ECI point-mass dynamics."""

    mu_m3_s2: float = c.MU_EARTH
    g0_m_s2: float = c.G_0
    r_earth_m: float = c.R_EARTH
    omega_earth_rad_s: float = c.OMEGA_EARTH
    drag_coefficient: float = 0.3
    reference_area_m2: float = 10.52
    eps_speed_mps: float = 1e-9
    min_mass_kg: float = 1e-6


def exponential_density_model(altitude_m: float) -> float:
    """Exponential atmosphere density model."""
    return float(c.RHO_0 * np.exp(-altitude_m / c.H))


def atmosphere_velocity_eci(r_I: np.ndarray, cfg: ECIPropagationConfig) -> np.ndarray:
    """Atmospheric velocity in ECI.

    This project currently uses a non-rotating atmosphere for drag calculations.
    """
    return np.zeros(3, dtype=float)


def air_relative_velocity_eci(r_I: np.ndarray, v_I: np.ndarray, cfg: ECIPropagationConfig) -> np.ndarray:
    """Air-relative velocity in ECI."""
    return v_I - atmosphere_velocity_eci(r_I, cfg)


def drag_magnitude(
    altitude_m: float,
    speed_rel_mps: float,
    cfg: ECIPropagationConfig,
    density_model: DensityModel = exponential_density_model,
) -> float:
    """Aerodynamic drag magnitude [N]."""
    rho = density_model(altitude_m)
    q = 0.5 * rho * speed_rel_mps * speed_rel_mps
    return float(q * cfg.drag_coefficient * cfg.reference_area_m2)


def gravity_acceleration_eci(r_I: np.ndarray, cfg: ECIPropagationConfig) -> np.ndarray:
    """Point-mass central gravity acceleration in ECI."""
    r_norm = float(np.linalg.norm(r_I))
    if r_norm <= 0.0:
        raise ValueError("Position norm must be positive for gravity computation.")
    return -(cfg.mu_m3_s2 / (r_norm ** 3)) * r_I


def point_mass_3dof_eci(
    t_s: float,
    state: StateVector,
    thrust_model: ThrustModel,
    pitch_model: PitchModel,
    fallback_azimuth_rad: float,
    cfg: ECIPropagationConfig,
    use_air_relative_for_pitch: bool = False,
    density_model: DensityModel = exponential_density_model,
) -> np.ndarray:
    """Evaluate 3-DOF ECI point-mass dynamics derivative."""
    r_I = np.asarray(state[0:3], dtype=float)
    v_I = np.asarray(state[3:6], dtype=float)
    m = max(float(state[6]), cfg.min_mass_kg)

    radius_m = float(np.linalg.norm(r_I))
    altitude_m = radius_m - cfg.r_earth_m

    v_rel_I = air_relative_velocity_eci(r_I, v_I, cfg)
    speed_rel = float(np.linalg.norm(v_rel_I))

    u_drag, drag_norm = frames.safe_normalize(v_rel_I, eps=cfg.eps_speed_mps)
    if drag_norm < cfg.eps_speed_mps:
        u_drag = np.zeros(3, dtype=float)

    d_mag = drag_magnitude(altitude_m, speed_rel, cfg, density_model=density_model)

    theta_pitch = float(pitch_model(t_s, state))
    vel_ref = v_rel_I if use_air_relative_for_pitch else v_I
    u_thrust = frames.thrust_direction_pitch_only(
        r_I=r_I,
        vel_ref_I=vel_ref,
        time_s=t_s,
        pitch_from_horizontal_rad=theta_pitch,
        fallback_azimuth_rad=fallback_azimuth_rad,
    )

    thrust_n, isp_s = thrust_model(t_s, state)
    thrust_n = max(float(thrust_n), 0.0)
    isp_s = max(float(isp_s), 1e-9)

    accel_thrust = (thrust_n / m) * u_thrust
    accel_drag = (d_mag / m) * u_drag
    accel_gravity = gravity_acceleration_eci(r_I, cfg)

    drdt = v_I
    dvdt = accel_thrust - accel_drag + accel_gravity
    dmdt = -thrust_n / (isp_s * cfg.g0_m_s2)

    return np.array([drdt[0], drdt[1], drdt[2], dvdt[0], dvdt[1], dvdt[2], dmdt], dtype=float)
