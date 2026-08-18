

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

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as _c

# -------------- Payload Mass --------------
M_PAYLOAD = 0e3           # payload mass; [kg]

# -------------- Fairing --------------
M_FAIRING = 1900          # payload fairing mass [kg] — jettisoned at atmosphere exit
                           # already included in M_STRUCTURE_1; stored here for the jettison event

# -------------- Event Intervals --------------
# Define time steps for events after main engine cutoff
TIME_First_STAGE_SEPARATION = 3             # time when stage separation should take place after main engine cutoff [s]
TIME_SECOND_ENGINE_IGNITION = 8       # time when second stage should be ignited after main engine cutoff [s]

# -------------- Aerodynamic Properties --------------
A = 10.52               # cross sectional area [m^2]
C_D = 0.3               # drag coefficient [no unit]
C_L = 0.1               # lift coefficient [no unit]; used when INCLUDE_LIFT (default True)

# =======================================================
#  FIRST STAGE
# =======================================================

# -------------- Engine Properties --------------
ISP_1_SL  = 283         # specific impulse at sea level [s]
ISP_1_VAC = 311         # specific impulse in vacuum [s]
ISP_1 = ISP_1_SL        # backward-compat alias (sea-level value)
F_THRUST_1_SL  = 7607e3 # thrust at sea level [N]
F_THRUST_1_VAC = 8227e3 # thrust in vacuum [N]
F_THRUST_1 = F_THRUST_1_SL  # backward-compat alias (sea-level value)

# Stage-1 nozzle exit area, derived rather than chosen. The pressure-dependent
# thrust model is F(h) = F_VAC - p_a(h)*A_E, so at sea level
# F_SL = F_VAC - p_0*A_E, which fixes A_E from the two published thrust figures
# above. It is therefore not an independent vehicle parameter: editing either
# thrust moves it, which is the intent.
# Only read when THRUST_1_MODE = "pressure".
A_E = (F_THRUST_1_VAC - F_THRUST_1_SL) / _c.P_0  # nozzle exit area [m^2]

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
