

"""
ROCKET SPECIFICATIONS - USER INPUT FILE

This file contains all the rocket specifications and parameters that define
the launch vehicle configuration. Users should modify the values in this file
to customize the rocket design for their specific mission requirements.

The default values provided correspond to the SpaceX Falcon 9 launch vehicle
specifications.

The file includes:
- Payload mass
- Event timing intervals (stage separation, engine ignition)
- Aerodynamic properties (cross-sectional area, drag coefficient)
- First and second stage specifications:
  * Engine properties (specific impulse, thrust)
  * Mass properties (structure mass, propellant mass)
- Calculated mass ratios for performance analysis
"""

# -------------- Payload Mass --------------
M_PAYLOAD = 0e3           # payload mass; [kg]

# -------------- Event Intervals --------------
# Define time steps for events after main engine cutoff
TIME_First_STAGE_SEPARATION = 3             # time when stage separation should take place after main engine cutoff [s]
TIME_SECOND_ENGINE_IGNITION = 8       # time when second stage should be ignited after main engine cutoff [s]

# -------------- Aerodynamic Properties --------------
A = 10.52               # cross sectional area [m^2]
C_D = 0.3               # drag coefficient [no unit]
C_L = 0.1               # lift coefficient [no unit] ---> lift is neglected in the simulation

# =======================================================
#  FIRST STAGE
# =======================================================

# -------------- Engine Properties --------------
ISP_1 = 283             # specific impulse [s]
F_THRUST_1 = 7600e3     # thrust of engine [N]

# -------------- Mass Properties --------------
M_STRUCTURE_1 = 25.6e3   # mass structure [kg]
M_PROP_1 = 395.7e3        # mass propellant [kg]


# =======================================================
#  SECOND STAGE
# =======================================================

# -------------- Engine Properties --------------
ISP_2 = 348              # specific impulse [s]
F_THRUST_2 = 934e3       # thrust of engine [N]

# -------------- Mass Properties --------------
M_STRUCTURE_2 = 3900            # mass structure [kg]
M_PROP_2 = 92670                # mass propellant [kg]


# =======================================================
#  MASS RATIOS
# =======================================================

# -------------- First Stage Mass Ratios --------------
# Total mass at stage 1 ignition (includes structure, propellant, stage 2, and payload)
M_TOTAL_1 = M_STRUCTURE_1 + M_PROP_1 + M_STRUCTURE_2 + M_PROP_2 + M_PAYLOAD

# Propellant mass ratio: M_prop / M_total
LAMBDA_1 = M_PROP_1 / M_TOTAL_1

# Structural ratio: M_structure / M_total
EPSILON_1 = M_STRUCTURE_1 / M_TOTAL_1

# Payload ratio: (M_payload + upper stages) / M_total
PI_1 = (M_PAYLOAD + M_STRUCTURE_2 + M_PROP_2) / M_TOTAL_1

# -------------- Second Stage Mass Ratios --------------
# Total mass at stage 2 ignition (includes structure, propellant, and payload)
M_TOTAL_2 = M_STRUCTURE_2 + M_PROP_2 + M_PAYLOAD

# Propellant mass ratio: M_prop / M_total
LAMBDA_2 = M_PROP_2 / M_TOTAL_2

# Structural ratio: M_structure / M_total
EPSILON_2 = M_STRUCTURE_2 / M_TOTAL_2

# Payload ratio: M_payload / M_total
PI_2 = M_PAYLOAD / M_TOTAL_2
