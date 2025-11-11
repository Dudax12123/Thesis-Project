import numpy as np
import constants as c

def gravitational_acceleration(r, MU_EARTH=c.MU_EARTH):
    """
    Calculate gravitational acceleration at a given altitude using inverse-square law.
    
    Parameters:
    -----------
    r : float
        Distance from Earth's center [m]
    MU_EARTH : float
        Standard gravitational parameter for Earth [m^3/s^2]

    Returns:
    --------
    g : float
        Gravitational acceleration at given altitude [m/s^2]

    Notes:
    ------
    Uses inverse-square law: g = MU_EARTH / r^2
    """
    g = MU_EARTH / ((r)**2)
    return g
