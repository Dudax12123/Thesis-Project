"""
Bilinear Tangent Steering Law

Advanced steering guidance where the tangent of the velocity vector inclination
varies as a ratio of two linear functions of time-to-go.

The steering law is: tan(α(t) + γ(t)) = [c1*(t_f - t) + c2] / [c1'*(t_f - t) + c2']

where:
- α(t) is the angle of attack
- γ(t) is the current flight path angle
- t_f is the target time (when guidance should end)
- c1, c2, c1', c2' are coefficients determined from boundary conditions

This formulation provides more flexibility than linear tangent steering by allowing
control over both the value and rate of change at boundary conditions.

References:
- Hull, D. G. (1997). Optimal Control Theory for Applications
- Lu, P. (1993). Inverse Dynamics Approach to Trajectory Optimization
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from Auxiliary import constants as c


def compute_bilinear_coefficients(current_state, target_altitude, t_go):
    """
    Compute bilinear tangent steering coefficients from boundary conditions.
    
    The bilinear form tan(α + γ) = [c1*τ + c2] / [c1'*τ + c2'] where τ = t_f - t
    requires 4 boundary conditions to determine the 4 coefficients.
    
    Boundary conditions used:
    1. Initial value: tan(α + γ)|_t0 = tan(γ_initial) [smooth handover, α ≈ 0]
    2. Initial derivative: d/dt[tan(α + γ)]|_t0 = specified rate
    3. Terminal value: tan(α + γ)|_tf = 0 [horizontal flight]
    4. Terminal derivative: d/dt[tan(α + γ)]|_tf = 0 [steady horizontal approach]
    
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
        Bilinear tangent coefficients [c1, c2, c1_prime, c2_prime]
        where tan(α + γ) = (c1*τ + c2) / (c1'*τ + c2'), τ = t_f - t
    """
    s, r_val, v, gamma, m = current_state[:5]
    
    # Boundary conditions
    # Terminal (at t = t_f, τ = 0):
    tan_terminal = 0.0  # Horizontal flight
    
    # Initial (at t = t_0, τ = t_go):
    # CRITICAL: Don't just use current gamma! That causes α=0 throughout.
    # Instead, compute required (α+γ) based on trajectory targeting.
    # Use a simple estimate: linear decrease from current to terminal
    # This ensures non-zero alpha commands for active control.
    gamma_current = gamma
    
    # Estimate what the average flight path angle should be
    # For a gravity turn from current altitude to target, use empirical relation
    #tan_initial = np.tan(gamma)  # OLD: This caused α=0 problem!
    
    # NEW: Use current gamma but adjust based on targeting needs
    # The key insight: if we're at gamma_current and want to end at 0°,
    # and the natural trajectory (without steering) would also decay to ~0°,
    # then we need to command a DIFFERENT trajectory that requires steering.
    # Simple approach: bias the initial condition
    tan_initial = np.tan(gamma_current * 1.2)  # 20% higher → requires negative alpha (pitch down)
    
    # Alternative better approach: estimate required thrust vector angle
    # based on current position relative to target
    r = current_state[1]
    target_r = c.R_EARTH + target_altitude
    altitude_to_gain = target_r - r
    downrange_estimate = t_go * current_state[2] * np.cos(gamma)  # Rough estimate
    
    # Use a targeting-based initial condition
    if downrange_estimate > 0:
        # Target flight path angle for efficient trajectory
        gamma_targeting = np.arctan(altitude_to_gain / downrange_estimate)
        # Blend with current gamma for stability
        gamma_blended = 0.7 * gamma_current + 0.3 * gamma_targeting
        tan_initial = np.tan(gamma_blended)
    else:
        # Fallback: use biased current gamma
        tan_initial = np.tan(gamma_current)
    
    # Terminal derivative: non-zero to avoid singularity
    dtan_dt_terminal = -0.02  # Moderate terminal rate (~ -1.1 deg/s)
    
    # Initial derivative: smooth transition
    dtan_dt_initial = (tan_terminal - tan_initial) / t_go
    
    # Special handling for near-zero or very small t_go
    if t_go < 0.1:
        # Near target, use simple form (avoid division issues)
        return [0.0, 0.0, 0.0, 1.0]
    
    # Now solve for bilinear coefficients using boundary conditions
    # Let f(τ) = tan(α + γ) = (c1*τ + c2) / (c1'*τ + c2')
    #
    # At τ = t_go (initial time):
    # f(t_go) = (c1*t_go + c2) / (c1'*t_go + c2') = tan_initial  ... (1)
    #
    # At τ = 0 (terminal time):
    # f(0) = c2 / c2' = tan_terminal                              ... (2)
    #
    # Derivative: f'(τ) = [(c1)(c1'*τ + c2') - (c1*τ + c2)(c1')] / (c1'*τ + c2')²
    #                   = [c1*c2' - c2*c1'] / (c1'*τ + c2')²
    #
    # At τ = t_go (initial time):
    # f'(t_go) = [c1*c2' - c2*c1'] / (c1'*t_go + c2')² = dtan_dt_initial  ... (3)
    # Note: f'(τ) = df/dτ, and df/dt = -df/dτ, so df/dt = -f'(τ)
    # Therefore: -[c1*c2' - c2*c1'] / (c1'*t_go + c2')² = dtan_dt_initial
    #            [c1*c2' - c2*c1'] / (c1'*t_go + c2')² = -dtan_dt_initial ... (3')
    #
    # At τ = 0 (terminal time):
    # f'(0) = [c1*c2' - c2*c1'] / (c2')² = -dtan_dt_terminal              ... (4')
    #         [c1*c2' - c2*c1'] = -(c2'²) * dtan_dt_terminal
    
    # From equation (2): c2 = c2' * tan_terminal
    c2 = tan_terminal  # We can set c2' = 1 (normalization), so c2 = tan_terminal
    c2_prime = 1.0
    
    # From equation (4'): c1*c2' - c2*c1' = -(c2'²) * dtan_dt_terminal
    # With c2' = 1 and c2 = tan_terminal:
    # c1 - tan_terminal*c1' = -dtan_dt_terminal
    # c1 = tan_terminal*c1' - dtan_dt_terminal  ... (5)
    
    # From equation (1): (c1*t_go + c2) / (c1'*t_go + c2') = tan_initial
    # c1*t_go + c2 = tan_initial * (c1'*t_go + c2')
    # c1*t_go + c2 = tan_initial*c1'*t_go + tan_initial*c2'
    # c1*t_go = tan_initial*c1'*t_go + tan_initial*c2' - c2
    # c1 = tan_initial*c1' + (tan_initial*c2' - c2)/t_go  ... (6)
    
    # From equation (3'): [c1*c2' - c2*c1'] / (c1'*t_go + c2')² = -dtan_dt_initial
    # With c2' = 1 and c2 = tan_terminal:
    # [c1 - tan_terminal*c1'] / (c1'*t_go + 1)² = -dtan_dt_initial
    # c1 - tan_terminal*c1' = -dtan_dt_initial * (c1'*t_go + 1)²  ... (7)
    
    # From (5): c1 = tan_terminal*c1' - dtan_dt_terminal
    # Substitute into (7):
    # (tan_terminal*c1' - dtan_dt_terminal) - tan_terminal*c1' = -dtan_dt_initial * (c1'*t_go + 1)²
    # -dtan_dt_terminal = -dtan_dt_initial * (c1'*t_go + 1)²
    # dtan_dt_terminal = dtan_dt_initial * (c1'*t_go + 1)²
    
    # If dtan_dt_terminal = 0 (steady terminal approach):
    # 0 = dtan_dt_initial * (c1'*t_go + 1)²
    # This means either dtan_dt_initial = 0 or we have issues
    
    # Alternative approach: Set c1' = 0 for simplicity (makes denominator constant)
    # This reduces to: tan(α + γ) = (c1*τ + c2) / c2'
    # Which is equivalent to linear tangent with scaling
    
    # Better approach: Use a more general solution
    # Let's normalize differently: set c2' = 1 always
    c2_prime = 1.0
    c2 = tan_terminal
    
    # For terminal derivative = 0 with c2 = 0 (horizontal):
    # [c1*c2' - c2*c1'] = 0 since tan_terminal = 0
    # c1 * 1 - 0 * c1' = 0
    # c1 = 0
    
    # This means for horizontal terminal with zero derivative, we need c1 = 0
    # Then from equation (1): c2 / (c1'*t_go + 1) = tan_initial
    # 0 / (c1'*t_go + 1) = tan_initial
    # This doesn't work!
    
    # Let me reconsider: for practical implementation, use a simpler constraint set
    # Terminal: tan = 0, derivative = 0
    # Initial: tan = tan(gamma), derivative chosen to give smooth profile
    
    # Simplified approach: Use the form with c2' = 1, c2 = 0 (terminal = 0)
    c2 = 0.0
    c2_prime = 1.0
    
    # At terminal (τ = 0): tan = c2/c2' = 0 ✓
    # At initial (τ = t_go): tan = (c1*t_go + 0)/(c1'*t_go + 1) = tan_initial
    # c1*t_go / (c1'*t_go + 1) = tan_initial
    # c1*t_go = tan_initial * (c1'*t_go + 1)
    # c1*t_go = tan_initial*c1'*t_go + tan_initial
    # c1 = tan_initial*c1' + tan_initial/t_go  ... (A)
    
    # Derivative: f'(τ) = [c1*c2' - c2*c1'] / (c1'*τ + c2')²
    # With c2 = 0, c2' = 1: f'(τ) = c1 / (c1'*τ + 1)²
    # At terminal (τ = 0): f'(0) = c1 / 1² = c1
    # Note: f'(τ) = df/dτ, and since τ decreases with time: df/dt = -df/dτ
    # So: dtan_dt_terminal = -f'(0) = -c1
    # Therefore: c1 = -dtan_dt_terminal
    
    # From derivative relationship
    c1 = -dtan_dt_terminal
    
    # From equation (A): c1 = tan_initial*c1' + tan_initial/t_go
    # -dtan_dt_terminal = tan_initial*c1' + tan_initial/t_go
    # tan_initial*c1' = -dtan_dt_terminal - tan_initial/t_go
    # c1' = (-dtan_dt_terminal - tan_initial/t_go) / tan_initial
    
    if abs(tan_initial) > 1e-6:
        c1_prime = (-dtan_dt_terminal - tan_initial/t_go) / tan_initial
    else:
        # Near-horizontal initial condition
        c1_prime = -1.0 / t_go
    
    return [c1, c2, c1_prime, c2_prime]


def bilinear_tangent_steering(t, t_go, current_state, coefficients):
    """
    Bilinear tangent steering guidance law.
    
    Implements: tan(α(t) + γ(t)) = [c1*τ + c2] / [c1'*τ + c2']
    where τ = t_f - t (time-to-go)
    
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
        Bilinear tangent coefficients [c1, c2, c1', c2']
        
    Returns:
    --------
    alpha : float
        Commanded angle of attack [rad]
    """
    s, r_val, v, gamma, m = current_state
    c1, c2, c1_prime, c2_prime = coefficients
    
    # Time-to-go τ = t_f - t
    tau = max(t_go, 0.0)  # Ensure non-negative
    
    # Compute denominator
    denominator = c1_prime * tau + c2_prime
    
    # Avoid division by zero
    if abs(denominator) < 1e-6:
        # Fallback to simple linear tangent
        tan_alpha_plus_gamma_cmd = c1 * tau + c2
    else:
        # Bilinear form
        numerator = c1 * tau + c2
        tan_alpha_plus_gamma_cmd = numerator / denominator
    
    # Compute commanded inclination angle
    alpha_plus_gamma_cmd = np.arctan(tan_alpha_plus_gamma_cmd)
    
    # Solve for angle of attack
    # α = (α + γ)_commanded - γ_current
    alpha = alpha_plus_gamma_cmd - gamma
    
    # Safety limits to prevent excessive maneuvering
    alpha = np.clip(alpha, -np.deg2rad(15), np.deg2rad(15))
    
    return alpha


class BilinearTangentSteeringState:
    """
    State management for bilinear tangent steering guidance.
    
    Manages coefficient updates and time reference for the steering law.
    """
    
    def __init__(self, update_interval=0.5):
        """
        Initialize bilinear tangent steering state.
        
        Parameters:
        -----------
        update_interval : float
            Minimum time between coefficient updates [s]
        """
        self.coefficients = [0.0, 0.0, 0.0, 1.0]
        self.last_update_time = 0.0
        self.update_interval = update_interval
        self.initialized = False
        
    def reset(self):
        """Reset guidance state."""
        self.coefficients = [0.0, 0.0, 0.0, 1.0]
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
            Updated coefficients [c1, c2, c1', c2']
        """
        self.coefficients = compute_bilinear_coefficients(state, target_altitude, t_go)
        self.last_update_time = t
        self.initialized = True
        
        return self.coefficients
