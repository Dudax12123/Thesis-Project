import numpy as np

# ===================================================
# General parameters
# ===================================================

# -------------- Gravity Turn --------------
TIME_TO_START_KICK = 7.5                        # time to start gravity turn; [s]
DURATION_INITIAL_KICK = 45.                     # duration of gravity turn; [s]

# -------------- Desired Orbit --------------
TARGET_ORBITAL_ALTITUDE = 500e3                             # altitude of desired orbit; [m]

# -------------- Earth Rotation --------------
# If True, the component of Earth's surface rotation velocity aligned with
# the launch azimuth is added to the velocity before every orbital-parameter
# check (apogee targeting, circularization delta-v, final orbital elements).
# The flight dynamics (EOM) are NOT modified — only the orbital bookkeeping.
EARTH_ROTATION = True                           # include Earth rotation velocity boost

# -------------- Launch Site & Orbit Inclination --------------
# Select a named launch site or use "custom" with CUSTOM_LATITUDE_DEG.
# Named sites: "KSC", "Vandenberg", "Kourou", "Baikonur"
LAUNCH_SITE = "KSC"                            # launch site name or "custom"
CUSTOM_LATITUDE_DEG = None                      # latitude [deg]; only used if LAUNCH_SITE = "custom"

# Target orbit inclination [deg].
# Must satisfy: inclination >= |launch latitude| for a direct-ascent trajectory.
# Default 28.573° matches KSC latitude (due-east launch, maximum payload).
TARGET_INCLINATION_DEG = 51.6                 # target orbit inclination; [deg]

# Ascending or descending node launch.
#   "ascending"  -> A_I = asin(cos(i)/cos(phi))
#   "descending" -> A_I = pi - asin(cos(i)/cos(phi))
AZIMUTH_BRANCH = "ascending"                    # "ascending" or "descending"

# Reference inertial speed for the Earth-rotation heading correction [m/s].
# If None, defaults to the circular orbital speed at TARGET_ORBITAL_ALTITUDE:
#   v_ref = sqrt(mu / (R_E + h_target))
AZIMUTH_REFERENCE_SPEED_MPS = None

# -------------- Guidance Mode Selection --------------
# Choose the guidance strategy for the trajectory:
#   "gravity_turn": Pure gravity turn all the way (traditional method)
#                   - Initial kick maneuver, then zero angle of attack throughout
#                   - No active guidance after kick
#   "simple_poly":  Simplified polynomial guidance (linear gamma transition)
#                   - Initial kick until atmosphere exit
#                   - Linear transition from current flight path angle to horizontal
#                   - Simple, stable, but not optimal
#   "linear_tangent": Linear tangent steering law (classical guidance)
#                   - Initial kick until atmosphere exit
#                   - tan(α + γ) varies linearly with time-to-go
#                   - Classic ascent guidance method
#   "bilinear_tangent": Bilinear tangent steering law (advanced guidance)
#                   - Initial kick until atmosphere exit
#                   - tan(α + γ) = ratio of two linear functions of time-to-go
#                   - More flexible than linear tangent, controls value and derivative
#   "apollo":       Apollo polynomial guidance (classical explicit guidance)
#                   - Initial kick until atmosphere exit
#                   - Polynomial acceleration profiles in x and y directions
#                   - Enforces position and velocity terminal constraints
#                   - Used in Apollo missions, more accurate than simple_poly
GUIDANCE_MODE = "apollo"  # Options: "gravity_turn", "simple_poly", "linear_tangent", "bilinear_tangent", "apollo"

# -------------- Polynomial Guidance Parameters --------------
# (Only used if GUIDANCE_MODE is "simple_poly", "linear_tangent", or "apollo")
GUIDANCE_UPDATE_RATE = 0.5                      # How often to recompute guidance coefficients [s]
APOLLO_FREEZE_THRESHOLD = 10.0                  # Time-to-go threshold to freeze Apollo coefficients [s]
                                                 # (prevents numerical instability as tgo->0)
APOLLO_THRUST_MAGNITUDE_CONTROL = True          # Enable thrust magnitude control for Apollo guidance
                                                 # If True: Apollo commands both thrust angle AND magnitude
                                                 # If False: Apollo only commands angle (fixed thrust)

# -------------- Atmosphere Exit / Guidance Start Marker --------------
# Choose how to detect when the rocket exits the atmosphere and guidance should start:
#   "altitude": Use altitude threshold (traditional method)
#   "dynamic_pressure": Use dynamic pressure threshold (more physically meaningful)
ATMOSPHERE_EXIT_METHOD = "altitude"             # Options: "altitude", "dynamic_pressure"
ALT_NO_ATMOSPHERE = 65e3                        # altitude threshold for atmosphere exit; [m]
                                                 # (only used if ATMOSPHERE_EXIT_METHOD = "altitude")
DYNAMIC_PRESSURE_THRESHOLD = 1000.0             # dynamic pressure threshold [Pa]
                                                 # (only used if ATMOSPHERE_EXIT_METHOD = "dynamic_pressure")
                                                 # Typical value: 1000 Pa (fairly low, indicating thin atmosphere)

# -------------- Optimization --------------
ALPHA_LOWEST = -np.deg2rad(4.)                  # lowest possible kick angle to be tested; [rad]
ALPHA_HIGHEST = -np.deg2rad(2.5)                # highest possible kick angle to be tested; [rad]
MAX_ACCEPTED_BURN_TIME = 15.                    # maximum accepted burn time of delta-v; [s]

# -------------- Fast Run Mode --------------
# If True, skips optimization and uses pre-determined optimal kick angles
RUN_FAST = True   

# Optimal kick angles for each guidance mode (in radians)
# These values should be updated after running optimization for each mode
OPTIMAL_KICK_ANGLES = {
    "gravity_turn": -np.deg2rad(3.0),           # Update after optimization
    "simple_poly": -np.deg2rad(3.0),            # Update after optimization
    "linear_tangent": -np.deg2rad(3.0),         # Update after optimization
    "bilinear_tangent": -np.deg2rad(3.0),       # Update after optimization
    "apollo": -np.deg2rad(3.8273)                   # Update after optimization
}

# ===================================================
# Single Run specific parameters
# ===================================================
SS_THROTTLE = 1.0                               # Second Stage throttle 
INITIAL_KICK_ANGLE = - np.deg2rad(3.0)          # Initial kick angle [rad]


# ===================================================
# FOR SIMULATION
# ===================================================
TIME_STEP = 0.01                              # step size for integration; [s]
DURATION_AFTER_SIMULATION = 1000.               # duration of simulation after reaching desired orbit; [s]


# ===================================================
# FOR DEBUGGING
# ===================================================
INTERRUPTS_PRINT = False
EVENTS_PRINT = False