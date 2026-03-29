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


def select_launch_azimuth(inc_deg, lat_deg, target_altitude, mode="corrected"):
    """
    Select active launch azimuth mode for rotating-frame propagation.

    Parameters:
    -----------
    inc_deg : float
        Target orbital inclination [deg]
    lat_deg : float
        Launch latitude [deg]
    target_altitude : float
        Target circular orbit altitude [m]
    mode : str
        Azimuth mode: "corrected" or "geometric"

    Returns:
    --------
    beta_active : float
        Active azimuth used in rotating-frame decomposition [rad]
    beta_inertial : float
        Geometric/inertial azimuth [rad]
    v_rot_surface : float
        Surface rotation speed at launch latitude [m/s]
    """
    beta_corrected, beta_inertial, v_rot_surface = corrected_azimuth(
        inc_deg,
        lat_deg,
        target_altitude,
    )

    mode_key = mode.lower().strip()
    if mode_key == "corrected":
        beta_active = beta_corrected
    elif mode_key == "geometric":
        beta_active = beta_inertial
    else:
        raise ValueError("EARTH_ROTATION_AZIMUTH_MODE must be 'corrected' or 'geometric'.")

    return beta_active, beta_inertial, v_rot_surface


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


def achieved_inclination_from_local_state(v_ecef, gamma_ecef, heading_ecef, lat_rad, r_val):
    """
    Compute achieved orbital inclination from local state including Earth rotation.

    Parameters:
    -----------
    v_ecef : float
        Velocity magnitude in rotating frame [m/s]
    gamma_ecef : float
        Flight path angle in rotating frame [rad]
    heading_ecef : float
        Heading/azimuth in rotating frame [rad]
    lat_rad : float
        Current geocentric latitude [rad]
    r_val : float
        Current geocentric radius [m]

    Returns:
    --------
    float
        Achieved inclination [deg]
    """
    v_horizontal = v_ecef * np.cos(gamma_ecef)
    v_north = v_horizontal * np.cos(heading_ecef)
    v_east = v_horizontal * np.sin(heading_ecef)

    v_rot = c.OMEGA_EARTH * r_val * np.cos(lat_rad)
    v_east_eci = v_east + v_rot

    beta_inertial = np.arctan2(v_east_eci, v_north)
    cos_i = np.cos(lat_rad) * np.sin(beta_inertial)
    cos_i = np.clip(cos_i, -1.0, 1.0)

    return np.rad2deg(np.arccos(cos_i))


def rotating_frame_pseudoforce_rates(v_ecef, gamma_ecef, heading_ecef, lat_rad, r_val):
    """
    Compute pseudo-force contributions for rotating-frame 2D EOM.

    The returned rates are projections of Coriolis + centrifugal acceleration
    onto (i) the along-velocity direction and (ii) the vertical-plane normal
    used in the flight-path-angle equation.

    Parameters:
    -----------
    v_ecef : float
        Velocity magnitude in rotating frame [m/s]
    gamma_ecef : float
        Flight path angle in rotating frame [rad]
    heading_ecef : float
        Heading/azimuth in rotating frame [rad]
    lat_rad : float
        Current geocentric latitude [rad]
    r_val : float
        Current geocentric radius [m]

    Returns:
    --------
    delta_dvdt : float
        Additive contribution to dv/dt [m/s^2]
    delta_dgammadt : float
        Additive contribution to dgamma/dt [rad/s]
    delta_dheadingdt : float
        Additive contribution to dheading/dt from cross-heading pseudo-forces [rad/s]
    """
    v_horizontal = v_ecef * np.cos(gamma_ecef)
    v_east = v_horizontal * np.sin(heading_ecef)
    v_north = v_horizontal * np.cos(heading_ecef)
    v_up = v_ecef * np.sin(gamma_ecef)

    omega_cos = c.OMEGA_EARTH * np.cos(lat_rad)
    omega_sin = c.OMEGA_EARTH * np.sin(lat_rad)

    # Coriolis acceleration in local ENU coordinates.
    a_cor_east = -2.0 * omega_cos * v_up + 2.0 * omega_sin * v_north
    a_cor_north = -2.0 * omega_sin * v_east
    a_cor_up = 2.0 * omega_cos * v_east

    # Centrifugal acceleration in local ENU coordinates.
    a_cent_east = 0.0
    a_cent_north = -(c.OMEGA_EARTH**2) * r_val * np.sin(lat_rad) * np.cos(lat_rad)
    a_cent_up = (c.OMEGA_EARTH**2) * r_val * (np.cos(lat_rad)**2)

    a_east = a_cor_east + a_cent_east
    a_north = a_cor_north + a_cent_north
    a_up = a_cor_up + a_cent_up

    # Horizontal along-track acceleration in heading direction.
    a_horizontal_along_heading = (
        a_east * np.sin(heading_ecef) + a_north * np.cos(heading_ecef)
    )

    # Cross-heading horizontal acceleration (perpendicular to heading in horizontal plane).
    a_cross_heading = (
        a_east * np.cos(heading_ecef) - a_north * np.sin(heading_ecef)
    )

    # Projection onto velocity direction (affects dv/dt).
    delta_dvdt = a_horizontal_along_heading * np.cos(gamma_ecef) + a_up * np.sin(gamma_ecef)

    # Projection onto in-plane normal (affects dgamma/dt).
    a_normal_in_plane = -a_horizontal_along_heading * np.sin(gamma_ecef) + a_up * np.cos(gamma_ecef)

    epsilon = 1e-9
    if abs(v_ecef) < epsilon:
        delta_dgammadt = 0.0
        delta_dheadingdt = 0.0
    else:
        delta_dgammadt = a_normal_in_plane / v_ecef
        # Cross-heading acceleration causes heading drift (divided by horizontal speed).
        # Use a physically meaningful threshold: heading is ill-defined when
        # horizontal speed is very small (near-vertical flight), so suppress
        # the cross-heading contribution below 10 m/s to avoid the coordinate
        # singularity at gamma ≈ 90°.
        v_horizontal = v_ecef * np.cos(gamma_ecef)
        V_HORIZ_THRESHOLD = 10.0  # [m/s]
        if abs(v_horizontal) < V_HORIZ_THRESHOLD:
            delta_dheadingdt = 0.0
        else:
            delta_dheadingdt = a_cross_heading / v_horizontal

    return delta_dvdt, delta_dgammadt, delta_dheadingdt
