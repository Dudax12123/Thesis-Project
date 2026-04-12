"""
Launch Azimuth Computation
==========================
Computes the inertial launch azimuth and the Earth-rotation-corrected
ground-relative heading for a spherical, non-rotating-Earth planar 3DoF
ascent simulator.

The azimuth selects which orbital plane the 2D trajectory lives in.
No Coriolis or centrifugal terms are injected into the equations of motion.

Conventions
-----------
* Input angles in **degrees**, internal arithmetic in **radians**.
* Azimuths measured **clockwise from true north** (0° = north, 90° = east).
* Positive latitude = north.

Key formulae
------------
Feasibility:  |cos(i) / cos(φ)| <= 1

Inertial azimuth (ascending node):
    A_I = arcsin(cos(i) / cos(φ))

Inertial azimuth (descending node):
    A_I = π − arcsin(cos(i) / cos(φ))

Launch-site eastward rotational speed:
    v_E = Ω_E · R_E · cos(φ)

Earth-fixed (ground-relative) heading:
    A_G = atan2(V_ref · sin(A_I) − v_E,  V_ref · cos(A_I))
"""

import numpy as np
from Auxiliary import constants as c


# ── Named launch-site database ──────────────────────────────────────────────
LAUNCH_SITES = {
    "KSC":        {"lat": 28.573,  "lon": -80.649},   # Kennedy Space Center, FL
    "Vandenberg": {"lat": 34.632,  "lon": -120.611},   # Vandenberg SFB, CA
    "Kourou":     {"lat":  5.236,  "lon": -52.775},    # Guiana Space Centre
    "Baikonur":   {"lat": 45.965,  "lon":  63.305},    # Baikonur Cosmodrome
}


def get_launch_latitude(site_name, custom_lat_deg=None):
    """
    Resolve a launch-site name (or "custom") to a geodetic latitude.

    Parameters
    ----------
    site_name : str
        One of the keys in ``LAUNCH_SITES`` or ``"custom"``.
    custom_lat_deg : float or None
        Geodetic latitude in degrees.  Required when *site_name* is ``"custom"``.

    Returns
    -------
    lat_rad : float
        Latitude in radians.
    lat_deg : float
        Latitude in degrees (for display).
    """
    if site_name.lower() == "custom":
        if custom_lat_deg is None:
            raise ValueError(
                "LAUNCH_SITE is 'custom' but CUSTOM_LATITUDE_DEG is not set. "
                "Please provide a latitude value in degrees."
            )
        lat_deg = float(custom_lat_deg)
    else:
        key = _resolve_site_key(site_name)
        lat_deg = LAUNCH_SITES[key]["lat"]

    lat_rad = np.deg2rad(lat_deg)
    return lat_rad, lat_deg


def compute_inertial_azimuth(lat_rad, inclination_rad, branch="ascending"):
    """
    Compute the inertial launch azimuth for a spherical Earth.

    Parameters
    ----------
    lat_rad : float
        Launch-site geodetic latitude [rad].
    inclination_rad : float
        Target orbit inclination [rad].
    branch : str
        ``"ascending"`` or ``"descending"`` node launch.

    Returns
    -------
    A_I : float
        Inertial launch azimuth [rad], measured clockwise from true north.

    Raises
    ------
    ValueError
        If the requested inclination is unreachable from the given latitude
        (i.e. ``|cos(i)/cos(φ)| > 1``).
    """
    cos_ratio = np.cos(inclination_rad) / np.cos(lat_rad)

    if np.abs(cos_ratio) > 1.0:
        inc_deg = np.rad2deg(inclination_rad)
        lat_deg = np.rad2deg(lat_rad)
        raise ValueError(
            f"Inclination {inc_deg:.2f}° is not reachable by direct launch "
            f"from latitude {lat_deg:.2f}° without a plane-change manoeuvre. "
            f"The minimum achievable inclination equals the launch-site "
            f"latitude ({abs(lat_deg):.2f}°)."
        )

    A_I_asc = np.arcsin(cos_ratio)

    if branch.lower() == "ascending":
        return A_I_asc
    elif branch.lower() == "descending":
        return np.pi - A_I_asc
    else:
        raise ValueError(
            f"AZIMUTH_BRANCH must be 'ascending' or 'descending', "
            f"got '{branch}'."
        )


def compute_ground_relative_heading(A_I, lat_rad, v_ref):
    """
    Earth-fixed (ground-relative) launch heading corrected for Earth rotation.

    Parameters
    ----------
    A_I : float
        Inertial launch azimuth [rad].
    lat_rad : float
        Launch-site latitude [rad].
    v_ref : float
        Reference inertial speed used for the correction [m/s].
        Typically the circular orbital speed at the target altitude.

    Returns
    -------
    A_G : float
        Ground-relative heading [rad], clockwise from true north.
    v_E : float
        Eastward rotational speed at the launch site [m/s].
    """
    v_E = c.OMEGA_EARTH * c.R_EARTH * np.cos(lat_rad)

    A_G = np.arctan2(
        v_ref * np.sin(A_I) - v_E,
        v_ref * np.cos(A_I),
    )
    return A_G, v_E


def compute_launch_azimuth(site, custom_lat_deg, inclination_deg,
                           branch="ascending", v_ref_mps=None,
                           target_altitude=None):
    """
    Top-level convenience function: resolve site, compute both azimuths.

    Parameters
    ----------
    site : str
        Named launch site or ``"custom"``.
    custom_lat_deg : float or None
        Latitude when *site* is ``"custom"`` [deg].
    inclination_deg : float
        Target orbit inclination [deg].
    branch : str
        ``"ascending"`` or ``"descending"``.
    v_ref_mps : float or None
        Reference inertial speed [m/s].
        If ``None``, computed as circular orbital speed at *target_altitude*.
    target_altitude : float or None
        Target orbital altitude [m].  Required when *v_ref_mps* is ``None``.

    Returns
    -------
    result : dict
        Dictionary with all computed values::

            lat_deg, lat_rad,
            inclination_deg, inclination_rad,
            branch,
            A_I_deg, A_I_rad,
            A_G_deg, A_G_rad,
            v_E, v_ref
    """
    # ── Latitude ──
    lat_rad, lat_deg = get_launch_latitude(site, custom_lat_deg)

    # ── Reference speed ──
    if v_ref_mps is not None:
        v_ref = float(v_ref_mps)
    else:
        if target_altitude is None:
            raise ValueError(
                "Either AZIMUTH_REFERENCE_SPEED_MPS or TARGET_ORBITAL_ALTITUDE "
                "must be provided to compute the reference speed."
            )
        r_target = c.R_EARTH + target_altitude
        v_ref = np.sqrt(c.MU_EARTH / r_target)

    # ── Inertial azimuth ──
    inclination_rad = np.deg2rad(inclination_deg)
    A_I = compute_inertial_azimuth(lat_rad, inclination_rad, branch)

    # ── Ground-relative heading ──
    A_G, v_E = compute_ground_relative_heading(A_I, lat_rad, v_ref)

    return {
        "lat_deg":          lat_deg,
        "lat_rad":          lat_rad,
        "inclination_deg":  inclination_deg,
        "inclination_rad":  inclination_rad,
        "branch":           branch,
        "A_I_deg":          np.rad2deg(A_I),
        "A_I_rad":          A_I,
        "A_G_deg":          np.rad2deg(A_G),
        "A_G_rad":          A_G,
        "v_E":              v_E,
        "v_ref":            v_ref,
    }


# ── Helpers ──────────────────────────────────────────────────────────────────

def _resolve_site_key(name):
    """Case-insensitive lookup into LAUNCH_SITES."""
    lower_map = {k.lower(): k for k in LAUNCH_SITES}
    key = lower_map.get(name.lower())
    if key is None:
        available = ", ".join(sorted(LAUNCH_SITES.keys()))
        raise ValueError(
            f"Unknown launch site '{name}'. "
            f"Available sites: {available}. "
            f"Use LAUNCH_SITE = 'custom' with CUSTOM_LATITUDE_DEG for "
            f"an unlisted site."
        )
    return key
