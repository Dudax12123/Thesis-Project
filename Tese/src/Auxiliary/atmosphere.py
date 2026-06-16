"""
ATMOSPHERE MODULE

This module contains functions for calculating atmospheric properties and
aerodynamic forces acting on the rocket during flight.

Functions:
- atmospheric_density: Calculate air density at a given altitude
- speed_of_sound: Calculate speed of sound at a given altitude
- dynamic_pressure: Calculate dynamic pressure at given velocity and altitude
- drag_force: Calculate aerodynamic drag force
- lift_force: Calculate aerodynamic lift force
"""
import sys
from pathlib import Path

# Add parent directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from Auxiliary import rocket_specs as r
from Auxiliary import constants as c


def atmospheric_density(altitude):
    """
    Calculate atmospheric density at a given altitude using exponential model.

    Parameters:
    -----------
    altitude : float
        Altitude above sea level [m]

    Returns:
    --------
    rho : float
        Air density at given altitude [kg/m^3]

    Notes:
    ------
    Uses exponential atmospheric model: rho = rho_0 * exp(-h/H)
    where H is the scale height. rho_0 and H are read from constants at call
    time so that multi-planet support (c.set_planet) takes effect correctly.
    """
    rho = c.RHO_0 * np.exp(-altitude / c.H)
    return rho

def speed_of_sound(altitude):
    """
    Calculate speed of sound at a given altitude using standard atmosphere model.
    
    Parameters:
    -----------
    altitude : float
        Altitude above sea level [m]
    
    Returns:
    --------
    a : float
        Speed of sound [m/s]
    
    Notes:
    ------
    Uses simplified standard atmosphere model:
    - Below 11 km (troposphere): T = T0 - L*h, where L = 6.5 K/km
    - Above 11 km (lower stratosphere): T = constant = 216.65 K
    - Speed of sound: a = sqrt(gamma * R * T)
    - gamma = 1.4 (specific heat ratio for air)
    - R = 287.05 J/(kg·K) (specific gas constant for air)
    """
    # Constants
    T0 = 288.15  # Sea level temperature [K]
    L = 0.0065   # Temperature lapse rate [K/m]
    gamma = 1.4  # Specific heat ratio for air
    R = 287.05   # Specific gas constant for air [J/(kg·K)]
    h_tropopause = 11000.0  # Tropopause altitude [m]
    T_tropopause = 216.65   # Tropopause temperature [K]
    
    # Calculate temperature based on altitude
    if altitude <= h_tropopause:
        # Troposphere: temperature decreases linearly
        T = T0 - L * altitude
    else:
        # Lower stratosphere: constant temperature
        T = T_tropopause
    
    # Calculate speed of sound
    a = np.sqrt(gamma * R * T)
    
    return a

def dynamic_pressure(velocity, altitude):
    """
    Calculate dynamic pressure (q) at given velocity and altitude.
    
    Parameters:
    -----------
    velocity : float
        Rocket velocity magnitude [m/s]
    altitude : float
        Altitude above sea level [m]
    
    Returns:
    --------
    q : float
        Dynamic pressure [Pa or N/m^2]
    
    Notes:
    ------
    Dynamic pressure: q = 0.5 * rho * v^2
    This is often called "max-q" when at its maximum value during ascent.
    """
    rho = atmospheric_density(altitude)
    q = 0.5 * rho * velocity**2
    return q

def aerothermal_flux(velocity, altitude):
    """
    Calculate aerothermal flux (Phi) at given velocity and altitude.

    Parameters:
    -----------
    velocity : float
        Rocket velocity magnitude [m/s]
    altitude : float
        Altitude above sea level [m]

    Returns:
    --------
    phi : float
        Aerothermal flux [W/m^2]

    Notes:
    ------
    Aerothermal flux: Phi = 0.5 * rho * v^3
    """
    rho = atmospheric_density(altitude)
    phi = 0.5 * rho * velocity**3
    return phi


def drag_force(q, C_D=r.C_D, A=r.A):
    """
    Calculate aerodynamic drag force acting on the rocket.
    
    Parameters:
    -----------
    q : float
        Dynamic pressure [Pa or N/m^2]
    
    Returns:
    --------
    F_drag : float
        Drag force magnitude [N]
    
    Notes:
    ------
    Drag force equation: F_D = q * C_D * A
    where:
    - q is dynamic pressure (0.5 * rho * v^2)
    - C_D is drag coefficient
    - A is cross-sectional area
    """
    
    # Drag force
    F_drag = q * C_D * A
    
    return F_drag


def lift_force(q, C_L=r.C_L, A=r.A):
    """
    Calculate aerodynamic lift force acting on the rocket.
    
    Parameters:
    -----------
    q : float
        Dynamic pressure [Pa or N/m^2]
    
    Returns:
    --------
    F_lift : float
        Lift force magnitude [N]
    
    Notes:
    ------
    Lift force equation: F_L = q * C_L * A 
    where:
    - q is dynamic pressure (0.5 * rho * v^2)
    - C_L is lift coefficient
    - A is cross-sectional area
    
    Note: In the rocket_specs file, lift is typically neglected (C_L = 0.1 is small)
    """
    
    # Lift force (proportional to angle of attack for small angles)
    F_lift = q * C_L * A
    
    return F_lift



