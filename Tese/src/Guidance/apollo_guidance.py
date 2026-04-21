"""
Apollo Guidance

Classical Apollo polynomial explicit guidance with linear acceleration profiles.
Based on the guidance system used in the Apollo lunar missions.

This guidance mode computes linear acceleration commands (ax, ay) that satisfy
terminal position and velocity constraints. It includes thrust magnitude control
to enable independent control of horizontal and vertical accelerations.

References:
- Classical Apollo guidance equations 2.36-2.41
- Battin, R. H. (1987). An introduction to the mathematics and methods of astrodynamics
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from Auxiliary import constants as c
from Auxiliary import gravity as grav


def predict_target_downrange(state, target_altitude):
    """
    Predict the downrange distance at which target altitude will be reached.
    
    Uses orbital mechanics (energy and angular momentum conservation) to estimate
    where the rocket will be when it reaches the target altitude.
    
    Justification: Needed for Apollo guidance to set downrange target. Uses simplified
    orbital mechanics assuming no further thrust after current state.
    
    Parameters:
    -----------
    state : array
        Current state [s, r, v, gamma, m]
        - s: downtrack [m]
        - r: radius from Earth's center [m]
        - v: velocity magnitude [m/s]
        - gamma: flight path angle [rad]
        - m: current mass [kg]
    target_altitude : float
        Target altitude above Earth's surface [m]
        
    Returns:
    --------
    downrange_target : float
        Predicted downrange distance to target [m]
    """
    s, r_val, v, gamma, m = state[:5]
    
    # Current orbital elements
    # Specific orbital energy
    epsilon = (v**2) / 2.0 - c.MU_EARTH / r_val
    
    # Specific angular momentum
    h = r_val * v * np.cos(gamma)
    
    # Semi-major axis
    if abs(epsilon) > 1e-6:
        a = -c.MU_EARTH / (2.0 * epsilon)
    else:
        # Nearly parabolic
        a = 1e12  # Very large value
    
    # Eccentricity
    if abs(a) > 1e6:
        e = 1.0  # Parabolic/hyperbolic
    else:
        e_squared = 1.0 - (h**2) / (c.MU_EARTH * a)
        e = np.sqrt(max(e_squared, 0.0))
    
    # Current and target radii
    r_current = r_val
    r_target = c.R_EARTH + target_altitude
    
    # True anomaly at current position
    # From orbit equation: r = a(1-e^2) / (1 + e*cos(nu))
    if e < 0.99:  # Elliptical orbit
        cos_nu_current = (a * (1 - e**2) - r_current) / (e * r_current)
        cos_nu_current = np.clip(cos_nu_current, -1.0, 1.0)
        nu_current = np.arccos(cos_nu_current)
        
        # Determine if we're in ascending or descending flight
        if gamma < 0:
            nu_current = -nu_current
        
        # True anomaly at target position
        cos_nu_target = (a * (1 - e**2) - r_target) / (e * r_target)
        cos_nu_target = np.clip(cos_nu_target, -1.0, 1.0)
        nu_target = np.arccos(cos_nu_target)
        
        # Angular distance traveled
        d_nu = nu_target - nu_current
        
        # Convert to downrange distance (arc length on Earth's surface)
        # This is approximate - assumes small angles
        downrange_increment = abs(d_nu) * c.R_EARTH
        
    else:
        # Highly eccentric or parabolic - use simple approximation
        altitude_change = target_altitude - (r_current - c.R_EARTH)
        downrange_increment = altitude_change / np.tan(max(abs(gamma), 0.1))
    
    # Total downrange at target
    downrange_target = s + downrange_increment
    
    return downrange_target


def compute_apollo_coefficients(state, target_altitude, t_go):
    """
    Compute Apollo polynomial guidance coefficients (classical formulation).
    
    Implements equations 2.39 and 2.40 from Apollo guidance literature.
    Uses linear acceleration profiles to satisfy terminal position and velocity constraints.
    
    This version enforces both horizontal and vertical terminal constraints:
    - Horizontal: Targets orbital velocity and predicted downrange position
    - Vertical: Targets altitude and horizontal flight (gamma = 0)
    
    Justification: Classical Apollo guidance formulation. During ascent, we need to
    control both altitude and horizontal velocity to achieve circular orbit insertion.
    The downrange target is predicted based on the current trajectory to avoid
    overconstraining the problem.
    
    Parameters:
    -----------
    state : array
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
        Apollo coefficients [k1, k2, k3, k4]
        where: ax = k1*(t-tepoch) + k2, ay = k3*(t-tepoch) + k4
    """
    s, r_val, v, gamma, m = state[:5]
    
    # Convert to Cartesian coordinates (x=downrange, y=altitude)
    x = s
    y = r_val - c.R_EARTH
    vx = v * np.cos(gamma)
    vy = v * np.sin(gamma)
    
    # Define target conditions
    y_target = target_altitude
    
    # Terminal velocity components (horizontal for circular orbit)
    vy_target = 0.0  # Horizontal flight (gamma = 0)
    
    # Estimate target horizontal velocity (circular orbital velocity)
    r_target = c.R_EARTH + target_altitude
    vx_target = np.sqrt(c.MU_EARTH / r_target)
    
   # Estimate target downrange position based on current trajectory
    x_target = 2*predict_target_downrange(state, target_altitude)
    
    # Apply Apollo guidance equations (2.39, 2.40) for both horizontal and vertical channels
    # Horizontal channel coefficients (enforce downrange position and horizontal velocity)
    # Equation 2.39: k1 = 6*(vx_f + vx)*t_go - 12*(x_f - x) / t_go^3
    # Equation 2.40: k2 = -2*(vx_f + 2*vx)*t_go + 6*(x_f - x) / t_go^2
    k1 = (6 * (vx_target + vx) * t_go - 12 * (x_target - x)) / (t_go ** 3)
    k2 = (-2 * (vx_target + 2 * vx) * t_go + 6 * (x_target - x)) / (t_go ** 2)
    
    # Vertical channel coefficients (enforce altitude and vertical velocity)
    # Equation 2.39: k3 = 6*(vy_f + vy)*t_go - 12*(y_f - y) / t_go^3
    # Equation 2.40: k4 = -2*(vy_f + 2*vy)*t_go + 6*(y_f - y) / t_go^2
    k3 = (6 * (vy_target + vy) * t_go - 12 * (y_target - y)) / (t_go ** 3)
    k4 = (-2 * (vy_target + 2 * vy) * t_go + 6 * (y_target - y)) / (t_go ** 2)
    
    return [k1, k2, k3, k4]


def apollo_guidance(t, t_epoch, state, coefficients):
    """
    Apollo polynomial explicit guidance for thrust angle control.
    
    Implements equation 2.41: commanded accelerations as linear functions of time.
    Converts total acceleration commands to thrust angle commands and required
    thrust magnitude.
    
    The guidance commands total accelerations (including gravity), then extracts
    the thrust component by subtracting gravitational acceleration. Returns both
    the angle of attack and the required thrust magnitude.
    
    Justification: Follows classical Apollo implementation. Accounts for gravity
    to extract thrust-only contribution, then converts to angle of attack and
    magnitude for independent control of horizontal and vertical accelerations.
    
    Parameters:
    -----------
    t : float
        Current time [s]
    t_epoch : float
        Time when coefficients were last computed/frozen [s]
    state : array
        Current state [s, r, v, gamma, m]
        - s: downtrack [m]
        - r: radius from Earth's center [m]
        - v: velocity magnitude [m/s]
        - gamma: flight path angle [rad]
        - m: current mass [kg]
    coefficients : list
        Apollo coefficients [k1, k2, k3, k4]
        
    Returns:
    --------
    alpha : float
        Commanded angle of attack [rad]
    a_thrust_magnitude : float
        Required thrust acceleration magnitude [m/s²]
    """
    s, r_val, v, gamma, m = state[:5]
    k1, k2, k3, k4 = coefficients
    
    # Time since epoch (for frozen coefficients)
    dt = t - t_epoch
    
    # Commanded total accelerations (equation 2.41)
    # These include all accelerations (thrust + gravity)
    ax_total = k1 * dt + k2
    ay_total = k3 * dt + k4
    
    # Compute gravitational acceleration components
    # Gravity acts radially inward toward Earth center
    a_grav = grav.gravitational_acceleration(r_val)
    
    # Gravity components in downrange-altitude (x-y) frame
    # Angle from vertical to position vector
    theta_from_vertical = s / c.R_EARTH
    
    # Gravity components (note: gravity acts toward Earth center)
    ax_gravity = a_grav * np.sin(theta_from_vertical)
    ay_gravity = -a_grav * np.cos(theta_from_vertical)  # Negative because downward
    
    # Extract thrust acceleration (remove gravity contribution)
    # Total acceleration = thrust acceleration + gravity acceleration
    # Therefore: thrust acceleration = total acceleration - gravity acceleration
    ax_thrust = ax_total - ax_gravity
    ay_thrust = ay_total - ay_gravity
    
    # Convert thrust acceleration to angle of attack
    # Thrust direction in inertial frame
    thrust_angle_inertial = np.arctan2(ay_thrust, ax_thrust)
    
    # Velocity direction in inertial frame
    velocity_angle = np.arctan2(v * np.sin(gamma), v * np.cos(gamma))
    
    # Angle of attack = thrust direction - velocity direction
    alpha = thrust_angle_inertial - velocity_angle
    
    # Normalize to [-pi, pi]
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
    
    # Store the unclamped alpha for debugging
    alpha_unclamped = alpha
    
    # Safety limits (prevent excessive maneuvers)
    # Justification: Physical limits of vehicle control authority
   # alpha = np.clip(alpha, -np.deg2rad(15), np.deg2rad(15))
    
    # Debug output on first call (only once)
    if not hasattr(apollo_guidance, '_debug_printed'):
        if abs(alpha_unclamped) > np.deg2rad(15):
            print(f"\nWARNING: Apollo guidance DEBUG (first excessive command):")
            print(f"   Time: {t:.2f}s, dt: {dt:.2f}s")
            print(f"   Coefficients: k1={coefficients[0]:.4f}, k2={coefficients[1]:.4f}, k3={coefficients[2]:.4f}, k4={coefficients[3]:.4f}")
            print(f"   ax_total={ax_total:.2f}, ay_total={ay_total:.2f}")
            print(f"   ax_thrust={ax_thrust:.2f}, ay_thrust={ay_thrust:.2f}")
            print(f"   Thrust angle: {np.rad2deg(thrust_angle_inertial):.2f} deg, Velocity angle: {np.rad2deg(velocity_angle):.2f} deg")
            print(f"   Alpha unclamped: {np.rad2deg(alpha_unclamped):.2f} deg, clamped: {np.rad2deg(alpha):.2f} deg")
            print(f"   Gamma: {np.rad2deg(gamma):.2f} deg, Velocity: {v:.2f} m/s")
            apollo_guidance._debug_printed = True
    
    # Calculate required thrust magnitude
    # This is the magnitude of the thrust acceleration vector
    a_thrust_magnitude = np.sqrt(ax_thrust**2 + ay_thrust**2)
    
    return alpha, a_thrust_magnitude


class ApolloGuidanceState:
    """
    State management for Apollo guidance.
    
    Manages coefficient freezing and update timing for Apollo guidance.
    This class helps encapsulate the state variables that were previously global.
    """
    
    def __init__(self, freeze_threshold=10.0):
        """
        Initialize Apollo guidance state.
        
        Parameters:
        -----------
        freeze_threshold : float
            Time-to-go threshold below which coefficients are frozen [s]
        """
        self.coefficients_frozen = False
        self.freeze_time = None
        self.coefficients = [0.0, 0.0, 0.0, 0.0]
        self.freeze_threshold = freeze_threshold
        self.last_update_time = 0.0
        
    def reset(self):
        """Reset guidance state."""
        self.coefficients_frozen = False
        self.freeze_time = None
        self.coefficients = [0.0, 0.0, 0.0, 0.0]
        self.last_update_time = 0.0
        
    def should_update_coefficients(self, t, update_interval=0.5):
        """
        Check if coefficients should be updated.
        
        Parameters:
        -----------
        t : float
            Current time [s]
        update_interval : float
            Minimum time between updates [s]
            
        Returns:
        --------
        bool : True if coefficients should be updated
        """
        if self.coefficients_frozen:
            return False
        
        return (t - self.last_update_time) >= update_interval
        
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
            Updated Apollo coefficients [k1, k2, k3, k4]
        """
        # Check if we should freeze coefficients
        if not self.coefficients_frozen and t_go < self.freeze_threshold:
            self.coefficients_frozen = True
            self.freeze_time = t
            if hasattr(self, 'verbose') and self.verbose:
                print(f"Apollo coefficients frozen at t={t:.1f}s, t_go={t_go:.1f}s")
        
        # Update coefficients if not frozen
        if not self.coefficients_frozen:
            self.coefficients = compute_apollo_coefficients(state, target_altitude, t_go)
            self.last_update_time = t
        
        return self.coefficients
        
    def get_epoch_time(self, t):
        """
        Get the epoch time for coefficient evaluation.
        
        Parameters:
        -----------
        t : float
            Current time [s]
            
        Returns:
        --------
        t_epoch : float
            Epoch time [s]
        """
        if self.coefficients_frozen and self.freeze_time is not None:
            return self.freeze_time
        else:
            return t
