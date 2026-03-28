"""
Simple Polynomial Guidance

Simplified polynomial guidance that linearly transitions the flight path angle
from the current value to the desired terminal condition (horizontal flight).

This is a basic explicit guidance law that provides smooth trajectory shaping
without the complexity of the full Apollo guidance equations.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from Auxiliary import constants as c


def compute_polynomial_coefficients(current_state, target_altitude, t_go):
    """
    Compute polynomial guidance coefficients based on current state and target.
    
    This implements a simplified polynomial guidance law that linearly transitions
    from the current flight path angle to the desired terminal conditions.
    
    The polynomial is of the form: alpha(tau) = a0 + a1*tau
    where tau is a normalized time parameter.
    
    Parameters:
    -----------
    current_state : array
        Current state [s, r, v, gamma, m]
        - s: downtrack [m]
        - r: radius from Earth's center [m]
        - v: velocity magnitude [m/s]
        - gamma: flight path angle [rad]
        - m: current mass [kg]
    target_altitude : float
        Target orbital altitude [m]
    t_go : float
        Time-to-go [s]
        
    Returns:
    --------
    coefficients : list
        Polynomial coefficients [a0, a1] for linear guidance law
    """
    s, r_val, v, gamma, m = current_state[:5]
    
    current_alt = r_val - c.R_EARTH
    
    # Terminal conditions for circular orbit
    gamma_terminal = 0.0  # Horizontal flight for circular orbit
    
    # Linear transition from current gamma to zero
    # alpha(tau) = a0 + a1*tau
    # At tau=1 (now): alpha = gamma
    # At tau=0 (end): alpha = 0
    a0 = gamma_terminal  # Terminal angle
    a1 = (gamma - gamma_terminal)  # Linear slope for transition
    
    return [a0, a1]


def polynomial_guidance(t, t_go, current_state, coefficients):
    """
    Polynomial explicit guidance for thrust angle control.
    
    Computes the commanded angle of attack using a linear polynomial function
    that smoothly transitions the flight path angle to the terminal condition.
    
    Parameters:
    -----------
    t : float
        Current time [s]
    t_go : float
        Time-to-go until target [s]
    current_state : array
        Current state [s, r, v, gamma, m]
        - s: downtrack [m]
        - r: radius from Earth's center [m]
        - v: velocity magnitude [m/s]
        - gamma: flight path angle [rad]
        - m: current mass [kg]
    coefficients : list
        Polynomial coefficients [a0, a1] for linear guidance law
        
    Returns:
    --------
    alpha : float
        Commanded angle of attack [rad]
    """
    s, r_val, v, gamma, m = current_state[:5]
    
    # Normalized time-to-go (1 at start, 0 at end)
    if t_go > 0.1:
        tau = np.clip(t_go / 100.0, 0.0, 1.0)  # Normalize by typical guidance duration
    else:
        tau = 0.0
    
    # Polynomial guidance law
    a0, a1 = coefficients
    alpha = a0 + a1*tau 
    
    # Limit angle of attack to reasonable values
    # alpha = np.clip(alpha, -np.deg2rad(10), np.deg2rad(10))
    
    return alpha
