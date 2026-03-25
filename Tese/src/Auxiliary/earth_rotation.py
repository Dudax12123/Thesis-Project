import numpy as np

from Auxiliary import constants as c


def surface_rotation_velocity(lat_deg, radius=c.R_EARTH):
    """
    Surface eastward velocity due to Earth rotation at latitude lat_deg.

    Parameters:
    -----------
    lat_deg : float
        Latitude [deg]
    radius : float
        Radius from Earth's center [m]

    Returns:
    --------
    float
        Eastward rotation speed [m/s]
    """
    lat_rad = np.deg2rad(lat_deg)
    return c.OMEGA_EARTH * radius * np.cos(lat_rad)


def geometric_azimuth(inc_deg, lat_deg):
    """
    Compute launch azimuth from inclination and latitude (ascending node).

    Formula:
        sin(beta) = cos(i) / cos(phi)

    Parameters:
    -----------
    inc_deg : float
        Target orbital inclination [deg]
    lat_deg : float
        Launch latitude [deg]

    Returns:
    --------
    beta : float
        Geometric inertial azimuth [rad], NE solution
    """
    i_rad = np.deg2rad(inc_deg)
    phi_rad = np.deg2rad(lat_deg)

    cos_phi = np.cos(phi_rad)
    if np.isclose(cos_phi, 0.0):
        raise ValueError("Launch latitude too close to poles for azimuth computation.")

    sin_beta = np.cos(i_rad) / cos_phi

    # Numerical guard around +/-1
    if sin_beta > 1.0 + 1e-10 or sin_beta < -1.0 - 1e-10:
        raise ValueError(
            "Requested inclination is not reachable from this launch latitude in prograde ascent."
        )

    sin_beta = np.clip(sin_beta, -1.0, 1.0)
    return np.arcsin(sin_beta)


def corrected_azimuth(inc_deg, lat_deg, target_altitude):
    """
    Compute corrected ECEF launch azimuth accounting for Earth rotation.

    Parameters:
    -----------
    inc_deg : float
        Target orbital inclination [deg]
    lat_deg : float
        Launch latitude [deg]
    target_altitude : float
        Target circular orbit altitude [m]

    Returns:
    --------
    beta_corrected : float
        Corrected launch azimuth in rotating frame [rad]
    beta_inertial : float
        Geometric/inertial azimuth [rad]
    v_rot_surface : float
        Surface rotation speed at launch latitude [m/s]
    """
    beta_inertial = geometric_azimuth(inc_deg, lat_deg)

    r_target = c.R_EARTH + target_altitude
    v_orb = np.sqrt(c.MU_EARTH / r_target)
    v_rot_surface = surface_rotation_velocity(lat_deg, radius=c.R_EARTH)

    v_north = v_orb * np.cos(beta_inertial)
    v_east_ecef = v_orb * np.sin(beta_inertial) - v_rot_surface

    beta_corrected = np.arctan2(v_east_ecef, v_north)

    return beta_corrected, beta_inertial, v_rot_surface


def ecef_to_eci_velocity(v_ecef, gamma_ecef, azimuth, lat_rad, r_val):
    """
    Convert local velocity magnitude/FPA from ECEF-like frame to ECI.

    Parameters:
    -----------
    v_ecef : float
        Velocity magnitude in rotating frame [m/s]
    gamma_ecef : float
        Flight path angle in rotating frame [rad]
    azimuth : float
        Launch azimuth in rotating frame [rad]
    lat_rad : float
        Current latitude [rad]
    r_val : float
        Current geocentric radius [m]

    Returns:
    --------
    v_eci : float
        Inertial velocity magnitude [m/s]
    gamma_eci : float
        Inertial flight path angle [rad]
    """
    v_horizontal = v_ecef * np.cos(gamma_ecef)
    v_radial = v_ecef * np.sin(gamma_ecef)

    v_north = v_horizontal * np.cos(azimuth)
    v_east = v_horizontal * np.sin(azimuth)

    # Add Earth rotation contribution to eastward inertial component.
    v_rot = c.OMEGA_EARTH * r_val * np.cos(lat_rad)
    v_east_eci = v_east + v_rot

    v_horizontal_eci = np.sqrt(v_east_eci**2 + v_north**2)
    v_eci = np.sqrt(v_horizontal_eci**2 + v_radial**2)
    gamma_eci = np.arctan2(v_radial, v_horizontal_eci)

    return v_eci, gamma_eci


def rotation_pseudo_accel_along_track(v, gamma, azimuth, lat_rad, r_val,
                                      omega_earth=c.OMEGA_EARTH):
    """
    Compute Coriolis + centrifugal pseudo-accelerations projected in the
    local flight plane.

    Parameters:
    -----------
    v : float
        Velocity magnitude in rotating frame [m/s]
    gamma : float
        Flight-path angle relative to local horizontal [rad]
    azimuth : float
        Heading/launch azimuth measured from north to east [rad]
    lat_rad : float
        Geocentric latitude [rad]
    r_val : float
        Geocentric radius [m]
    omega_earth : float
        Earth rotation rate [rad/s]

    Returns:
    --------
    a_t : float
        Pseudo-acceleration component along velocity direction [m/s^2]
    a_n : float
        Pseudo-acceleration component normal to velocity in flight plane,
        positive for increasing flight-path angle [m/s^2]
    """
    c_lat = np.cos(lat_rad)
    s_lat = np.sin(lat_rad)
    c_gamma = np.cos(gamma)
    s_gamma = np.sin(gamma)
    s_az = np.sin(azimuth)
    c_az = np.cos(azimuth)

    # Velocity resolved in local ENU basis.
    v_east = v * c_gamma * s_az
    v_north = v * c_gamma * c_az
    v_up = v * s_gamma

    # Coriolis pseudo-acceleration in rotating frame: a_cor = -2 * Omega x v.
    a_cor_east = -2.0 * omega_earth * (c_lat * v_up - s_lat * v_north)
    a_cor_north = -2.0 * omega_earth * s_lat * v_east
    a_cor_up = 2.0 * omega_earth * c_lat * v_east

    # Centrifugal pseudo-acceleration in local ENU basis:
    # a_cen = -Omega x (Omega x r).
    a_cen_east = 0.0
    a_cen_north = -omega_earth**2 * r_val * s_lat * c_lat
    a_cen_up = omega_earth**2 * r_val * c_lat**2

    a_east = a_cor_east + a_cen_east
    a_north = a_cor_north + a_cen_north
    a_up = a_cor_up + a_cen_up

    # Local horizontal heading unit vector and in-plane basis vectors.
    h_east = s_az
    h_north = c_az

    t_east = c_gamma * h_east
    t_north = c_gamma * h_north
    t_up = s_gamma

    n_east = -s_gamma * h_east
    n_north = -s_gamma * h_north
    n_up = c_gamma

    a_t = a_east * t_east + a_north * t_north + a_up * t_up
    a_n = a_east * n_east + a_north * n_north + a_up * n_up

    return a_t, a_n


def delta_v_gain(lat_deg, azimuth, radius):
    """
    Estimate inertial speed gain from Earth rotation projected onto launch azimuth.

    Parameters:
    -----------
    lat_deg : float
        Latitude [deg]
    azimuth : float
        Launch azimuth [rad]
    radius : float
        Radius where gain is evaluated [m]

    Returns:
    --------
    float
        Effective eastward inertial speed gain [m/s]
    """
    return surface_rotation_velocity(lat_deg, radius=radius) * np.sin(azimuth)


def orbit_inclination(lat_deg, beta_inertial):
    """
    Compute inclination from launch latitude and inertial azimuth.

    Parameters:
    -----------
    lat_deg : float
        Latitude [deg]
    beta_inertial : float
        Inertial launch azimuth [rad]

    Returns:
    --------
    float
        Inclination [deg]
    """
    lat_rad = np.deg2rad(lat_deg)
    cos_i = np.cos(lat_rad) * np.sin(beta_inertial)
    cos_i = np.clip(cos_i, -1.0, 1.0)
    return np.rad2deg(np.arccos(cos_i))
