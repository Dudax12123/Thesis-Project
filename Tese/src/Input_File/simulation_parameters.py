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
#   False: Pure gravity turn all the way (traditional method)
#          - Initial kick maneuver, then zero angle of attack throughout
#   True:  Gravity turn + Polynomial explicit guidance (advanced method)
#          - Initial kick maneuver until atmosphere exit
#          - Polynomial guidance takes over after leaving atmosphere (>65 km)
#          - Actively steers to optimize trajectory to target orbit
ENABLE_POLYNOMIAL_GUIDANCE = True               # Enable polynomial guidance after atmosphere exit

# -------------- Polynomial Guidance Parameters --------------
# (Only used if ENABLE_POLYNOMIAL_GUIDANCE = True)
POLY_GUIDANCE_ORDER = 3                         # Order of polynomial (1, 2, 3, etc.)
GUIDANCE_UPDATE_RATE = 0.1                      # How often to update guidance coefficients [s]

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