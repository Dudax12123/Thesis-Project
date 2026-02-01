import numpy as np

# ===================================================
# General parameters
# ===================================================

# -------------- Gravity Turn --------------
TIME_TO_START_KICK = 7.5                        # time to start gravity turn; [s]
DURATION_INITIAL_KICK = 45.                     # duration of gravity turn; [s]

# -------------- Desired Orbit --------------
TARGET_ORBITAL_ALTITUDE = 500e3                             # altitude of desired orbit; [m]

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
#   "apollo":       Apollo polynomial guidance (classical explicit guidance)
#                   - Initial kick until atmosphere exit
#                   - Polynomial acceleration profiles in x and y directions
#                   - Enforces position and velocity terminal constraints
#                   - Used in Apollo missions, more accurate than simple_poly
GUIDANCE_MODE = "simple_poly"  # Options: "gravity_turn", "simple_poly", "linear_tangent", "apollo"

# -------------- Polynomial Guidance Parameters --------------
# (Only used if GUIDANCE_MODE is "simple_poly", "linear_tangent", or "apollo")
GUIDANCE_UPDATE_RATE = 0.5                      # How often to recompute guidance coefficients [s]
APOLLO_FREEZE_THRESHOLD = 10.0                  # Time-to-go threshold to freeze Apollo coefficients [s]
                                                 # (prevents numerical instability as tgo->0)
APOLLO_THRUST_MAGNITUDE_CONTROL = True          # Enable thrust magnitude control for Apollo guidance
                                                 # If True: Apollo commands both thrust angle AND magnitude
                                                 # If False: Apollo only commands angle (fixed thrust)

# -------------- Optimization --------------
ALPHA_LOWEST = -np.deg2rad(4.)                  # lowest possible kick angle to be tested; [rad]
ALPHA_HIGHEST = -np.deg2rad(2.5)                # highest possible kick angle to be tested; [rad]
ALT_NO_ATMOSPHERE = 65e3                        # altitude where atmosphere can be neglected; [m]
MAX_ACCEPTED_BURN_TIME = 15.                    # maximum accepted burn time of delta-v; [s]

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
EVENTS_PRINT = True