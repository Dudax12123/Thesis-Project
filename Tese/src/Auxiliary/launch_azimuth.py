"""Launch azimuth computation accounting for Earth rotation."""

import numpy as np
from Auxiliary import constants as c

# Launch-site geodetic latitudes [deg]
_SITE_LATITUDES = {
    "KSC":        28.573,
    "Vandenberg": 34.632,
    "Kourou":      5.236,
    "Baikonur":   45.965,
}


def compute_launch_azimuth(
    site="KSC",
    custom_lat_deg=None,
    inclination_deg=28.573,
    branch="ascending",
    v_ref_mps=None,
    target_altitude=500e3,
):
    """Compute inertial and ground-relative launch azimuths.

    Parameters
    ----------
    site : str
        Named launch site ("KSC", "Vandenberg", "Kourou", "Baikonur")
        or "custom" (requires *custom_lat_deg*).
    custom_lat_deg : float or None
        Latitude [deg] when *site* is "custom".
    inclination_deg : float
        Target orbit inclination [deg].
    branch : str
        "ascending" or "descending" node launch.
    v_ref_mps : float or None
        Reference inertial speed for the rotation heading correction.
        If None, defaults to circular orbital speed at *target_altitude*.
    target_altitude : float
        Target orbital altitude [m] (used only for default *v_ref_mps*).

    Returns
    -------
    dict with keys:
        v_E           – site eastward speed from Earth rotation [m/s]
        A_I_deg       – inertial launch azimuth [deg]
        A_G_deg       – ground-relative heading [deg]
        lat_deg       – launch-site latitude [deg]
        inclination_deg
        branch
        v_ref         – reference speed used [m/s]
    """
    # ── Latitude ────────────────────────────────────────────────
    if site.lower() == "custom":
        if custom_lat_deg is None:
            raise ValueError("custom_lat_deg is required when site='custom'")
        lat_deg = custom_lat_deg
    else:
        if site not in _SITE_LATITUDES:
            raise ValueError(
                f"Unknown site '{site}'. Choose from {list(_SITE_LATITUDES)} or 'custom'."
            )
        lat_deg = _SITE_LATITUDES[site]

    lat = np.deg2rad(lat_deg)
    inc = np.deg2rad(inclination_deg)

    # Feasibility: |lat| must be <= inclination for a direct-ascent trajectory
    if abs(lat_deg) > inclination_deg + 1e-6:
        raise ValueError(
            f"Inclination ({inclination_deg:.2f} deg) must be >= |latitude| "
            f"({abs(lat_deg):.2f} deg) for a direct-ascent trajectory."
        )

    # ── Inertial azimuth ────────────────────────────────────────
    sin_A_I = np.clip(np.cos(inc) / np.cos(lat), -1.0, 1.0)
    if branch == "ascending":
        A_I = np.arcsin(sin_A_I)
    elif branch == "descending":
        A_I = np.pi - np.arcsin(sin_A_I)
    else:
        raise ValueError(f"branch must be 'ascending' or 'descending', got '{branch}'")

    # ── Eastward velocity from Earth rotation ───────────────────
    v_E = c.OMEGA_EARTH * c.R_EARTH * np.cos(lat)

    # ── Reference speed (default: circular orbital speed) ───────
    if v_ref_mps is None:
        v_ref = np.sqrt(c.MU_EARTH / (c.R_EARTH + target_altitude))
    else:
        v_ref = v_ref_mps

    # ── Ground-relative heading ─────────────────────────────────
    #  v_ref * sin(A_G) = v_ref * sin(A_I) - v_E * cos(A_I)
    sin_A_G = np.sin(A_I) - (v_E * np.cos(A_I)) / v_ref
    sin_A_G = np.clip(sin_A_G, -1.0, 1.0)
    A_G = np.arcsin(sin_A_G)

    return {
        "v_E": v_E,
        "A_I_deg": np.rad2deg(A_I),
        "A_G_deg": np.rad2deg(A_G),
        "A_I_rad": A_I,
        "lat_deg": lat_deg,
        "inclination_deg": inclination_deg,
        "branch": branch,
        "v_ref": v_ref,
    }
