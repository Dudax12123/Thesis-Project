import sys
from pathlib import Path

# Add parent directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from Auxiliary import constants as c

def gravitational_acceleration(r):
    """
    Calculate gravitational acceleration at a given altitude using inverse-square law.

    Parameters:
    -----------
    r : float
        Distance from body center [m]

    Returns:
    --------
    g : float
        Gravitational acceleration at given altitude [m/s^2]

    Notes:
    ------
    Uses inverse-square law: g = MU / r^2
    """
    g = c.MU_EARTH / (r**2)
    return g
