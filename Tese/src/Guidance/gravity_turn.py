"""
Gravity Turn Guidance

Traditional gravity turn guidance with minimal active control.
The rocket naturally follows a trajectory shaped primarily by gravitational forces
after an initial pitch maneuver.

This is the simplest guidance mode - the thrust vector is aligned with the velocity
vector (zero angle of attack), allowing gravity to naturally curve the trajectory.
"""

import numpy as np


def gravity_turn_guidance(t, state, params=None):
    """
    Gravity turn guidance - zero angle of attack.
    
    In gravity turn mode, the rocket maintains zero angle of attack,
    meaning the thrust vector is aligned with the velocity vector.
    The trajectory curves naturally due to gravitational forces.
    
    Parameters:
    -----------
    t : float
        Current time [s]
    state : array
        Current state [s, r, v, gamma, m]
        - s: downtrack [m]
        - r: radius from Earth's center [m]
        - v: velocity magnitude [m/s]
        - gamma: flight path angle [rad]
        - m: current mass [kg]
    params : dict, optional
        Additional parameters (not used in gravity turn)
        
    Returns:
    --------
    alpha : float
        Commanded angle of attack [rad] - always 0.0 for gravity turn
    """
    # Zero angle of attack - thrust aligned with velocity
    alpha = 0.0
    
    return alpha


def gravity_turn_initialization(initial_state, target_altitude):
    """
    Initialize gravity turn guidance parameters.
    
    Gravity turn requires minimal initialization since it's passive guidance.
    This function is provided for consistency with other guidance modes.
    
    Parameters:
    -----------
    initial_state : array
        Initial state when guidance begins [s, r, v, gamma, m]
    target_altitude : float
        Target orbital altitude [m]
        
    Returns:
    --------
    params : dict
        Empty dictionary (no parameters needed for gravity turn)
    """
    return {}
