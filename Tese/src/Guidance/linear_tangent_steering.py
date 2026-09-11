"""
Linear Tangent Steering Law

Classical linear tangent steering guidance where the tangent of the velocity 
vector inclination varies linearly with time-to-go.

The steering law is: tan(α(t) + γ(t)) = a(t_f - t) + b

where:
- α(t) is the angle of attack
- γ(t) is the current flight path angle
- t_f is the target time (when guidance should end)
- a, b are coefficients determined from boundary conditions

References:
- Etkin, B. (1972). Dynamics of Atmospheric Flight
- Hull, D. G. (1997). Optimal Control Theory for Applications
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from Auxiliary import constants as c


# ---------------------------------------------------------------------------
# Open-loop form with optimiser-chosen constants (pso_coast, since 2026-09-11)
# ---------------------------------------------------------------------------
#
# This is the form the bibliography gives the law. In the flat-Earth, constant-
# gravity, no-drag problem the thrust direction that is optimal with the final
# downrange free satisfies tan(theta) = a*t_go + b, and the two constants are
# fixed by the TERMINAL boundary conditions of that problem (Perkins 1966, by
# the Lagrange technique; Bryson & Ho; Chapter 4: "a separate numerical
# optimisation ... determines the optimal gamma(t) and t_f; the linear or
# bilinear tangent law then provides the steering history"). The law has no
# targeting content of its own: whatever solves for its constants supplies it.
# Here that is the swarm, exactly as the exponential law is treated.
#
# The legacy closed-loop routine further down derives the constants from the
# CURRENT flight-path angle instead, with the terminal pitch pinned at zero.
# That boundary condition is this implementation's, not the literature's; it
# leaves the law nothing to target, and re-deriving it at every update returns
# alpha = 0 identically. It is kept only as the fallback for the architectures
# that supply no constants (apogee_check, direct, segmented waypoints).
#
# Two conventions, both measured against the indirect-PMP optimum of the
# archived results batch before being adopted:
#
#   Time runs CONTINUOUSLY from the first Stage-2 ignition t0 to the planned
#   final cutoff tf, through the coast. In the flat-Earth derivation the
#   costates propagate through a coast exactly as through a burn
#   (dlambda_vy/dt = -lambda_y in both), so tan(theta) is linear in absolute
#   time, not in powered time. Fitting the PMP steering: absolute time 0.69-0.84
#   deg rms, powered-only (re-epoched per arc) 4.4-8.1 deg rms.
#
#   Pitch is measured from the LOCAL horizontal, theta = alpha + gamma, as in
#   Chapter 4's equation. The flat-Earth derivation measures it from a fixed
#   horizontal; on the archived PMP optimum the two frames fit comparably
#   (linear 0.84 vs 0.69 deg, bilinear 0.03 vs 0.17 deg), so the chapter's frame
#   is kept.
#
# Parametrised by the pitch at the two ends of the span rather than by (a, b),
# so the swarm's bounds are physical angles:
#
#   tan theta(sigma) = (1 - sigma) tan theta0 + sigma tan thetaf,
#   sigma = (t - t0)/(tf - t0),
#
# which is tan(theta) = a*t_go + b with t_go = tf - t, b = tan thetaf and
# a = (tan theta0 - tan thetaf)/(tf - t0).

def open_loop_tan_pitch(sigma, theta0, thetaf):
    """Commanded pitch from the local horizontal at normalised time ``sigma``.

    ``sigma`` is clamped to [0, 1]: past the planned cutoff (an integrator
    endpoint a hair beyond tf) the law holds its terminal value.
    """
    sigma = min(max(float(sigma), 0.0), 1.0)
    return float(np.arctan((1.0 - sigma) * np.tan(theta0) + sigma * np.tan(thetaf)))


def open_loop_alpha(t, t0, tf, gamma, theta0, thetaf):
    """Angle of attack of the open-loop linear tangent law.

    Parameters
    ----------
    t      : float  current time [s]
    t0     : float  first Stage-2 ignition [s]
    tf     : float  planned final cutoff [s] (coast included)
    gamma  : float  current flight-path angle [rad]
    theta0 : float  commanded pitch at t0 [rad]
    thetaf : float  commanded pitch at tf [rad]
    """
    sigma = (t - t0) / (tf - t0) if tf > t0 else 0.0
    return open_loop_tan_pitch(sigma, theta0, thetaf) - gamma


def open_loop_coefficients(T, theta0, thetaf):
    """Chapter-4 form ``[a, b]`` of tan(alpha + gamma) = a*t_go + b over a span T."""
    return [(np.tan(theta0) - np.tan(thetaf)) / T, float(np.tan(thetaf))]


def compute_lts_coefficients(current_state, target_altitude, t_go):
    """
    Compute linear tangent steering coefficients from boundary conditions.
    
    Boundary conditions:
    - Current: tan(α + γ) = tan(α_current + γ_current) at t = t_current
    - Terminal: tan(α + γ) = 0 (horizontal flight) at t = t_f
    
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
        Linear tangent coefficients [a, b, t_f]
        where tan(α + γ) = a*(t_f - t) + b
    """
    s, r_val, v, gamma, m = current_state[:5]
    
    # Terminal condition: horizontal flight
    # tan(α_terminal + γ_terminal) = tan(0 + 0) = 0
    tan_terminal = 0.0
    
    # Current condition: assume we want to smoothly transition
    # At initialization (t = t_0), we'll assume α ≈ 0 for smooth handover
    # So tan(α_current + γ_current) ≈ tan(γ_current)
    tan_current = np.tan(gamma)
    
    # Linear relationship: tan(α + γ) = a*(t_f - t) + b
    # At t = t_f: tan_terminal = a*0 + b  →  b = tan_terminal = 0
    # At t = t_0: tan_current = a*t_go + b
    #             tan_current = a*t_go + 0
    #             a = tan_current / t_go
    
    b = tan_terminal
    
    if t_go > 0.1:
        a = (tan_current - b) / t_go
    else:
        # Near target, keep current slope
        a = 0.0
    
    # Store target time for use in guidance law
    # We'll use t_go and update it each call rather than absolute time
    t_f = t_go  # This will be time-to-go at each update
    
    return [a, b, t_f]


def linear_tangent_steering(t, t_go, current_state, coefficients):
    """
    Linear tangent steering guidance law.
    
    Implements: tan(α(t) + γ(t)) = a(t_f - t) + b
    
    Solves for angle of attack α given current flight path angle γ.
    
    Parameters:
    -----------
    t : float
        Current time [s]
    t_go : float
        Current time-to-go estimate [s]
    current_state : array
        Current state [s, r, v, gamma, m]
        - s: downtrack [m]
        - r: radius from Earth's center [m]
        - v: velocity magnitude [m/s]
        - gamma: flight path angle [rad]
        - m: current mass [kg]
    coefficients : list
        Linear tangent coefficients [a, b, t_f]
        
    Returns:
    --------
    alpha : float
        Commanded angle of attack [rad]
    """
    s, r_val, v, gamma, m = current_state[:5]
    a, b, t_f = coefficients
    
    # Compute commanded velocity vector inclination
    # tan(α + γ)_commanded = a*(t_f - t_current) + b
    # Since we're using t_go as our time reference:
    tan_alpha_plus_gamma_cmd = a * t_go + b
    
    # Compute commanded inclination angle
    # α + γ = arctan(tan(α + γ))
    alpha_plus_gamma_cmd = np.arctan(tan_alpha_plus_gamma_cmd)
    
    # Solve for angle of attack
    # α = (α + γ)_commanded - γ_current
    alpha = alpha_plus_gamma_cmd - gamma
    
    # Safety limits to prevent excessive maneuvering
    # Typical limits for ascent: ±15 degrees
    # alpha = np.clip(alpha, -np.deg2rad(15), np.deg2rad(15))
    
    return alpha


def compute_lts_coefficients_advanced(current_state, target_altitude, t_go, 
                                      terminal_gamma=0.0):
    """
    Advanced coefficient computation with specified terminal conditions.
    
    This version allows specifying different terminal flight path angles,
    useful for testing or specific mission requirements.
    
    Parameters:
    -----------
    current_state : array
        Current state [s, r, v, gamma, m]
    target_altitude : float
        Target orbital altitude [m]
    t_go : float
        Time-to-go [s]
    terminal_gamma : float, optional
        Desired terminal flight path angle [rad], default 0.0 (horizontal)
        
    Returns:
    --------
    coefficients : list
        Linear tangent coefficients [a, b, t_f]
    """
    s, r_val, v, gamma, m = current_state[:5]
    
    # Terminal condition with specified terminal angle
    # At terminal time, assume α ≈ 0 (thrust aligned with velocity)
    tan_terminal = np.tan(terminal_gamma)
    
    # Current condition
    # Assume smooth handover: α_initial ≈ 0
    tan_current = np.tan(gamma)
    
    # Solve for coefficients
    b = tan_terminal
    
    if t_go > 0.1:
        a = (tan_current - b) / t_go
    else:
        a = 0.0
    
    t_f = t_go
    
    return [a, b, t_f]


class LinearTangentSteeringState:
    """
    State management for linear tangent steering guidance.
    
    Manages coefficient updates and time reference for the steering law.
    """
    
    def __init__(self, update_interval=0.5):
        """
        Initialize linear tangent steering state.
        
        Parameters:
        -----------
        update_interval : float
            Minimum time between coefficient updates [s]
        """
        self.coefficients = [0.0, 0.0, 0.0]
        self.last_update_time = 0.0
        self.update_interval = update_interval
        self.initialized = False
        
    def reset(self):
        """Reset guidance state."""
        self.coefficients = [0.0, 0.0, 0.0]
        self.last_update_time = 0.0
        self.initialized = False
        
    def should_update_coefficients(self, t):
        """
        Check if coefficients should be updated.
        
        Parameters:
        -----------
        t : float
            Current time [s]
            
        Returns:
        --------
        bool : True if coefficients should be updated
        """
        if not self.initialized:
            return True
            
        return (t - self.last_update_time) >= self.update_interval
        
    def update_coefficients(self, t, state, target_altitude, t_go):
        """
        Update guidance coefficients.
        
        Parameters:
        -----------
        t : float
            Current time [s]
        state : array
            Current state [s, r, v, gamma, m]
        target_altitude : float
            Target altitude [m]
        t_go : float
            Time-to-go [s]
            
        Returns:
        --------
        coefficients : list
            Updated coefficients [a, b, t_f]
        """
        self.coefficients = compute_lts_coefficients(state, target_altitude, t_go)
        self.last_update_time = t
        self.initialized = True
        
        return self.coefficients
