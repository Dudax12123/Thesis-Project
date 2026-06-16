G_0 = 9.81                  # Standard gravitational acceleration for Isp definition [m/s^2] — always Earth-standard, never changes with planet
R_EARTH = 6378e3            # Selected body radius [m]
MU_EARTH = 3.986004418e14   # Selected body gravitational constant [m^3/s^2]
OMEGA_EARTH = 7.2921159e-5  # Selected body rotation rate [rad/s]
RHO_0 = 1.225               # Selected body sea-level air density [kg/m^3]
H = 8500                    # Selected body atmospheric scale height [m]

# ---------------------------------------------------------------------------
# Planet catalog — add entries here to support additional bodies.
# R, MU, OMEGA, RHO_0, H are the same quantities as the module constants above.
# For airless bodies set RHO_0=0 and H=1 (dummy; avoids /0 in exp(-alt/H)).
# ---------------------------------------------------------------------------
PLANETS = {
    "earth": {
        "R":     6378e3,
        "MU":    3.986004418e14,
        "OMEGA": 7.2921159e-5,
        "RHO_0": 1.225,
        "H":     8500.0,
    },
    "moon": {
        "R":     1737.4e3,
        "MU":    4.9048695e12,
        "OMEGA": 2.6617e-6,
        "RHO_0": 0.0,
        "H":     1.0,
    },
    "mars": {
        "R":     3389.5e3,
        "MU":    4.282837e13,
        "OMEGA": 7.0882e-5,
        "RHO_0": 0.020,
        "H":     11100.0,
    },
}


def set_planet(name: str) -> None:
    """Override module-level body constants for the selected planet. Call once at startup."""
    global R_EARTH, MU_EARTH, OMEGA_EARTH, RHO_0, H
    body = PLANETS[name]
    R_EARTH     = body["R"]
    MU_EARTH    = body["MU"]
    OMEGA_EARTH = body["OMEGA"]
    RHO_0       = body["RHO_0"]
    H           = body["H"]
