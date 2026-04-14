""" ===============================================
    ROCKET ASCENT SIMULATION - COASTING SINGLE BURN
    
    This module simulates rocket trajectory optimization using a
    single-burn coasting strategy for orbital insertion.
=============================================== """

import sys
from pathlib import Path

# Add parent directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import atmosphere as atm
from Auxiliary import gravity as grav
from Auxiliary import constants as c
from Input_File import simulation_parameters as sim_params
from Auxiliary import rocket_specs as r
import Guidance.gravity_turn as gravity_turn_guidance
import Guidance.simple_polynomial as simple_poly_guidance
import Guidance.linear_tangent_steering as lts_guidance
import Guidance.bilinear_tangent_steering as bts_guidance
import Guidance.apollo_guidance as apollo_guidance_module
import numpy as np
from scipy.integrate import solve_ivp

#===================================================
# Global Variables
#===================================================
time_kick_start = None
kick_performed = False
time_raise = sim_params.DURATION_INITIAL_KICK / 2.
main_engine_cutoff = False
second_engine_ignition = False
stage_2_burnt = False
time_main_engine_cutoff = None
second_stage_cutoff = False
flag_falling_single_burn = False
current_kick_angle = 0.0  # Store current kick angle for interrupt functions

# For single burn optimization
SINGLE_BURN_FULL_SIMULATION = False
TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = None

# Guidance phase flags
atmosphere_exited = False
guidance_phase_active = False
time_atmosphere_exit = None
last_guidance_update_time = 0.0
guidance_coefficients = [0.0, 0.0, 0.0, 0.0]  # For simple_poly: [a0, a1] or apollo: [k1, k2, k3, k4]
apollo_coefficients_frozen = False  # Flag to indicate if Apollo coefficients are frozen

# Thrust history for plotting
thrust_history = []  # Store thrust values during integration
time_history = []    # Store corresponding time values

# Earth rotation velocity boost [m/s]
# Set via set_earth_rotation_boost(); used as initial tangential speed in ECI mode.
earth_rotation_boost = 0.0

# Effective angular rate of the co-rotating atmosphere projected onto the
# orbital plane [rad/s].  omega_eff = OMEGA_EARTH * cos(lat) * sin(A_I).
# v_atm_tangential(r) = omega_eff * r.
omega_eff_rad = 0.0
apollo_freeze_time = None  # Time when coefficients were frozen (tepoch)

# Angle of attack history (full AoA including geometric baseline)
alpha_history = []  # Store full angle of attack during flight
alpha_time_history = []  # Store corresponding time values

# Steering angle history (deliberate control input only)
# 0 during vertical phase and coast; kick command during kick; guidance output during guidance
steering_history = []
steering_time_history = []

#===================================================
# Interrupt functions for simulation
#===================================================

def interrupt_radius_check(t, y):
    """
    Returns zero if the current radius exceeds the radius of the desired orbit.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    y : array
        Current state vector
        
    Returns:
    --------
    int : 0 if interrupt triggered, 1 otherwise
    """
    margin = 50e3
    r = np.sqrt(y[0]**2 + y[1]**2)
    if r > (sim_params.TARGET_ORBITAL_ALTITUDE + c.R_EARTH + margin):
        if sim_params.INTERRUPTS_PRINT: 
            print("Interrupt Radius Check happened at time ", t)
        return 0
    return 1


def interrupt_stage_separation(t, y):
    """
    Returns zero if the stage separation should be performed.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    y : array
        Current state vector
        
    Returns:
    --------
    int : 0 if interrupt triggered, 1 otherwise
    """
    global time_main_engine_cutoff, main_engine_cutoff

    if main_engine_cutoff:
        if t >= (time_main_engine_cutoff + r.TIME_First_STAGE_SEPARATION):
            if sim_params.INTERRUPTS_PRINT:
                print("Interrupt Stage Separation happened at time ", t)
            return 0
    return 1


def interrupt_stage_2_burnt(t, y):
    """
    Returns zero if the second stage is fully burnt.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    y : array
        Current state vector
        
    Returns:
    --------
    int : 0 if interrupt triggered, 1 otherwise
    """
    m = y[4]
    if m <= (r.M_PAYLOAD + r.M_STRUCTURE_2):
        if sim_params.INTERRUPTS_PRINT:
            print("Interrupt Stage 2 Burnt happened at time ", t)
        return 0
    return 1


def interrupt_ground_collision(t, y):
    """
    Returns zero if the current radius is below Earth's radius.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    y : array
        Current state vector
        
    Returns:
    --------
    int : 0 if interrupt triggered, 1 otherwise
    """
    r_val = np.sqrt(y[0]**2 + y[1]**2)
    if r_val < c.R_EARTH - 1e3:
        if sim_params.INTERRUPTS_PRINT:
            print("Interrupt Earth Collision happened at time ", t)
        return 0
    return 1


def interrupt_velocity_exceeded(t, y):
    """
    Returns zero if the current velocity exceeds the velocity of the desired orbit.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    y : array
        Current state vector
        
    Returns:
    --------
    float : Difference between current and desired velocity
    """
    # ECI speed: add v_boost to the horizontal component
    x_pos, y_pos, vx_ecef, vy_ecef = y[0], y[1], y[2], y[3]
    r_val = np.sqrt(x_pos**2 + y_pos**2)
    if r_val > 0:
        vx_eci = vx_ecef + earth_rotation_boost * (y_pos / r_val)
        vy_eci = vy_ecef + earth_rotation_boost * (-x_pos / r_val)
    else:
        vx_eci = vx_ecef + earth_rotation_boost
        vy_eci = vy_ecef
    v = np.sqrt(vx_eci**2 + vy_eci**2)
    r_desired = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    v_desired = np.sqrt(c.MU_EARTH / r_desired)
    return v - v_desired


def interrupt_single_burn_traj(t, y):
    """
    Checks if the current apogee matches the desired altitude if the rocket 
    would stop burning at the current time stamp.
    Only performed once above the atmosphere.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    y : array
        Current state vector
        
    Returns:
    --------
    float : Difference between apogee and target altitude
    """
    legacy = cartesian_to_legacy_state(y[0], y[1], y[2], y[3], y[4])
    r_val = legacy[1]
    v_eci = legacy[2]
    gamma_eci = legacy[3]
    alt = r_val - c.R_EARTH

    if alt < sim_params.ALT_NO_ATMOSPHERE:
        return 1
    else:
        # Compute current orbital elements (using ECI velocity)
        a, e, r_apo, r_peri, _ = get_orbital_elements(r_val, v_eci, gamma_eci)

        diff = r_apo - (sim_params.TARGET_ORBITAL_ALTITUDE + c.R_EARTH)
        
        return diff
    

def interrupt_horizontal_check(t, y):
    """
    Checks if the rocket reached horizontal flight direction.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    y : array
        Current state vector
        
    Returns:
    --------
    int : 0 if interrupt triggered, 1 otherwise
    """
    _, _, _, gamma_eci, _ = cartesian_to_legacy_state(y[0], y[1], y[2], y[3], y[4])
    epsilon = np.deg2rad(0.01)
    
    if gamma_eci < epsilon:
        if sim_params.INTERRUPTS_PRINT:
            print("Interrupt Horizontal Flight Direction happened at time ", t)
        return 0
    return 1

    
#===================================================
# Event functions
#===================================================

def event_main_engine_cutoff(t, y):
    """
    Checks if there is still propellant in the first stage and 
    triggers engine cutoff event when there isn't.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    y : array
        Current state vector
    """
    global main_engine_cutoff, time_main_engine_cutoff
    
    if main_engine_cutoff == True:
        return
    
    first_stage_leftover_propellant = y[4] - (r.M_STRUCTURE_1 + r.M_STRUCTURE_2 + 
                                                r.M_PROP_2 + r.M_PAYLOAD)
    
    if first_stage_leftover_propellant <= 0 and main_engine_cutoff == False:
        main_engine_cutoff = True
        time_main_engine_cutoff = t

        if sim_params.EVENTS_PRINT:
            print("Main engine cutoff at t = ", t)

    return


def event_second_engine_ignition(t):
    """
    Triggers second stage engine ignition.
    
    Parameters:
    -----------
    t : float
        Current time since launch [s]
    """
    global time_main_engine_cutoff, second_engine_ignition
    
    if t >= (r.TIME_SECOND_ENGINE_IGNITION + time_main_engine_cutoff):
        second_engine_ignition = True
        
        if sim_params.EVENTS_PRINT:
            print("Second engine ignited at t = ", t)
        
    return

#===================================================
# Utility Functions
#===================================================

def cartesian_coordinates(h, s):
    """
    Convert altitude and downtrack to Cartesian coordinates.
    
    Parameters:
    -----------
    h : float
        Altitude above Earth's surface [m]
    s : float
        Downtrack distance [m]
        
    Returns:
    --------
    x, y : float
        Cartesian coordinates [m]
    """
    theta = s / c.R_EARTH
    y = (h + c.R_EARTH) * np.cos(theta)
    x = (h + c.R_EARTH) * np.sin(theta)
    
    return x, y


def get_orbital_elements(r_val, v_inertial, gamma_inertial, mu=c.MU_EARTH):
    """
    Computes the orbital parameters given the input state.
    
    Note: Velocity and gamma should be relative to the inertial (ECI) 
    reference frame, not the ECEF frame.
    
    Parameters:
    -----------
    r_val : float
        Radial distance to Earth's center [m]
    v_inertial : float
        Velocity relative to ECI frame [m/s]
    gamma_inertial : float
        Flight path angle relative to ECI frame [rad]
    mu : float, optional
        Gravitational parameter [m^3/s^2]
    
    Returns:
    --------
    a : float
        Semi-major axis [m]
    e : float
        Eccentricity [-]
    r_apo : float
        Apoapsis radius [m]
    r_peri : float
        Periapsis radius [m]
    orbit_period : float
        Period of the orbit [s]
    """
    a = (mu * r_val) / ((2 * mu) - (r_val * v_inertial**2))
    e = (1 - (r_val * v_inertial * np.cos(gamma_inertial))**2 / (mu * a))**0.5
    r_apo = a * (1 + e)
    r_peri = a * (1 - e)
    orbit_period = 2 * np.pi * (np.pow(a, 1.5)) / (np.pow(mu, 0.5))
    
    return a, e, r_apo, r_peri, orbit_period


def surface_to_inertial(v, gamma, v_boost):
    """Convert surface-relative velocity to inertial by adding a tangential boost.

    Parameters
    ----------
    v : float
        Surface-relative velocity magnitude [m/s].
    gamma : float
        Surface-relative flight path angle [rad].
    v_boost : float
        Tangential (horizontal) velocity boost from Earth rotation [m/s].

    Returns
    -------
    v_inertial : float
        Inertial velocity magnitude [m/s].
    gamma_inertial : float
        Inertial flight path angle [rad].
    """
    v_tan = v * np.cos(gamma) + v_boost
    v_rad = v * np.sin(gamma)
    v_inertial = np.sqrt(v_tan**2 + v_rad**2)
    gamma_inertial = np.arctan2(v_rad, v_tan)
    return v_inertial, gamma_inertial


def set_earth_rotation_boost(azimuth_data):
    """Compute and store Earth-rotation parameters for the orbital plane.

    Parameters
    ----------
    azimuth_data : dict
        Dictionary returned by ``launch_azimuth.compute_launch_azimuth()``.
        Must contain keys ``v_E`` and ``A_I_rad``.
    """
    global earth_rotation_boost, omega_eff_rad
    if not sim_params.EARTH_ROTATION:
        earth_rotation_boost = 0.0
        omega_eff_rad = 0.0
        return
    v_E = azimuth_data["v_E"]
    A_I = azimuth_data["A_I_rad"]
    # Project eastward surface speed onto the in-plane (azimuth) direction
    earth_rotation_boost = v_E * np.sin(A_I)
    # Effective angular rate of the atmosphere in the orbital plane
    omega_eff_rad = earth_rotation_boost / c.R_EARTH


def thrust_Isp():
    """
    Returns the current thrust and specific impulse based on engine status.
    
    Returns:
    --------
    F_T : float
        Current thrust [N]
    Isp : float
        Current specific impulse [s]
    """
    global main_engine_cutoff, second_engine_ignition, second_stage_cutoff
    
    if not main_engine_cutoff:
        F_T = r.F_THRUST_1
        Isp = r.ISP_1
    elif main_engine_cutoff and not second_engine_ignition:
        F_T = 0
        Isp = r.ISP_1
    elif main_engine_cutoff and second_stage_cutoff:
        F_T = 0
        Isp = r.ISP_2
    elif main_engine_cutoff and second_engine_ignition:
        F_T = r.F_THRUST_2
        Isp = r.ISP_2
    else:
        print("Warning: Both first stage and second stage engines are running at the same time.")
        F_T = r.F_THRUST_1
        Isp = r.ISP_1
        
    return F_T, Isp


def pitch_program_linear(t, initial_kick_angle):
    """
    Returns the angle of attack for the initial kick.
    Increases the angle of attack to a certain value and decreases it 
    afterwards in a linear way.

    Parameters:
    -----------
    t : float
        Current time since launch [s]
    initial_kick_angle : float
        Maximum kick angle [rad]
        
    Returns:
    --------
    float : Current angle of attack [rad]
    """
    global time_kick_start, kick_performed, time_raise

    if time_kick_start == None:
        time_kick_start = t
        if sim_params.EVENTS_PRINT:
            print("\nInitial kick started at t = ", t)
        return 0.0
    
    elif t > (time_kick_start + sim_params.DURATION_INITIAL_KICK):
        kick_performed = True
        if sim_params.EVENTS_PRINT:
            print("\nInitial kick ended at t = ", t)
        return 0.0
    
    else:
        # Check if angle should raise or decrease
        if t < (time_kick_start + time_raise):
            # Define rate of angle change
            angle_rate = (t - time_kick_start) / time_raise
            return initial_kick_angle * angle_rate
        else:
            # Define rate of angle change
            angle_rate = (t - (time_kick_start + time_raise)) / time_raise
            return initial_kick_angle * (1 - angle_rate)


def estimate_time_to_target(state, target_altitude):
    """
    Estimate time remaining until reaching target altitude.
    
    Parameters:
    -----------
    state : array
        Current state [s, r, v, gamma, m] (legacy format from cartesian_to_legacy_state)
    target_altitude : float
        Target altitude [m]
        
    Returns:
    --------
    t_go : float
        Estimated time-to-go [s]
    """
    s, r_val, v, gamma, m = state
    
    current_alt = r_val - c.R_EARTH
    altitude_remaining = target_altitude - current_alt
    
    # Simple estimation based on current radial velocity
    v_radial = v * np.sin(gamma)
    
    if v_radial > 1e-3:
        t_go = altitude_remaining / v_radial
    else:
        # If not climbing much, estimate based on average velocity
        t_go = 1000.0  # Large default value
    
    return max(t_go, 0.1)  # Avoid division by zero issues


def calculate_burn_time(mass_initial, delta_v):
    """
    Calculate the burn time required for a given delta-v maneuver.
    
    Parameters:
    -----------
    mass_initial : float
        Initial mass before burn [kg]
    delta_v : float
        Required velocity change [m/s]
        
    Returns:
    --------
    burn_time : float
        Time required for the burn [s]
    """
    # Using rocket equation: m_final = m_initial * exp(-delta_v / (g0 * Isp))
    mass_final = mass_initial * np.exp(-delta_v / (c.G_0 * r.ISP_2))
    mass_propellant = mass_initial - mass_final
    
    # Burn time = mass_propellant / mass_flow_rate
    mass_flow_rate = r.F_THRUST_2 / (c.G_0 * r.ISP_2)
    burn_time = mass_propellant / mass_flow_rate
    
    return burn_time


def get_time_until_apogee(e, gamma, v, T, a, r_current):
    """
    Calculate time until the spacecraft reaches apogee.
    
    Parameters:
    -----------
    e : float
        Eccentricity [-]
    gamma : float
        Flight path angle [rad]
    v : float
        Velocity [m/s]
    T : float
        Orbital period [s]
    a : float
        Semi-major axis [m]
    r_current : float
        Current radius [m]
        
    Returns:
    --------
    time_until_apogee : float
        Time until apogee [s]
    """
    theta = np.arccos((a * (1 - e**2) - r_current) / (e * r_current))
    ecc_anomaly = 2 * np.arctan2(np.sqrt((1 - e) / (1 + e)) * (1 - np.cos(theta)), 
                                   np.sin(theta))
    mean_anomaly = ecc_anomaly - e * np.sin(ecc_anomaly)
    time_until_apogee = T / (2 * np.pi) * mean_anomaly
    time_until_apogee = (T / 2.) - time_until_apogee

    return time_until_apogee


#===================================================
# Cartesian Helpers & EOM
#===================================================

def cartesian_to_polar(x, y, vx, vy):
    """Derive polar/legacy quantities from Cartesian ECEF state.

    Parameters
    ----------
    x, y : float
        Position in orbital-plane Cartesian frame [m].
        y = radial at launch, x = downrange at launch.
    vx, vy : float
        Velocity components [m/s].

    Returns
    -------
    r_val : float   – radius from Earth centre [m]
    alt   : float   – altitude above surface [m]
    v     : float   – speed [m/s]
    gamma : float   – flight-path angle (from local horizontal) [rad]
    theta : float   – geocentric angle from launch site [rad]
    """
    r_val = np.sqrt(x**2 + y**2)
    alt = r_val - c.R_EARTH
    v = np.sqrt(vx**2 + vy**2)
    # theta: angle from the initial radial (y-axis)
    theta = np.arctan2(x, y)
    # Flight-path angle = angle of velocity above local horizontal.
    # Local radial unit vector:  r_hat = (x/r, y/r)
    # Local horizontal (prograde) unit vector: h_hat = (y/r, -x/r)
    # v_radial  = dot(v, r_hat) = (vx*x + vy*y)/r
    # v_horiz   = dot(v, h_hat) = (vx*y - vy*x)/r
    # gamma     = atan2(v_radial, v_horiz)
    if r_val > 0 and v > 1e-10:
        v_radial = (vx * x + vy * y) / r_val
        v_horiz  = (vx * y - vy * x) / r_val
        gamma = np.arctan2(v_radial, v_horiz)
    else:
        gamma = np.pi / 2.0  # default: radially outward
    return r_val, alt, v, gamma, theta


def cartesian_to_legacy_state(x, y, vx, vy, m):
    """Build a legacy-format state ``[s, r, v, gamma, m]`` from Cartesian.

    The velocity is converted to **ECI** by adding ``earth_rotation_boost``
    to the horizontal component before computing *v* and *gamma*.  This is
    the state that guidance modules expect.

    ``s`` is the ECEF ground-track distance = R_E * theta.
    """
    r_val, alt, _, _, theta = cartesian_to_polar(x, y, vx, vy)

    # ECI velocity: add Earth-rotation boost to the horizontal component.
    # Horizontal unit vector at the current position: h_hat = (y/r, -x/r)
    # Adding v_boost along h_hat:
    #   vx_eci = vx + v_boost * (y / r)
    #   vy_eci = vy + v_boost * (-x / r)
    if r_val > 0:
        vx_eci = vx + earth_rotation_boost * (y / r_val)
        vy_eci = vy + earth_rotation_boost * (-x / r_val)
    else:
        vx_eci = vx + earth_rotation_boost
        vy_eci = vy

    v_eci = np.sqrt(vx_eci**2 + vy_eci**2)
    if r_val > 0 and v_eci > 1e-10:
        v_radial_eci = (vx_eci * x + vy_eci * y) / r_val
        v_horiz_eci  = (vx_eci * y - vy_eci * x) / r_val
        gamma_eci = np.arctan2(v_radial_eci, v_horiz_eci)
    else:
        gamma_eci = np.pi / 2.0

    s = c.R_EARTH * theta  # ECEF ground-track
    return np.array([s, r_val, v_eci, gamma_eci, m])


def diff_eom_cartesian(x, y, vx, vy, m, F_D, F_T, alpha, Isp):
    """Equations of motion in Cartesian (approximate ECEF) coordinates.

    State: [x, y, vx, vy, m].

    Forces included: gravity (inverse-square), thrust (at angle *alpha*
    from velocity direction), aerodynamic drag (opposes velocity, i.e.
    the atmosphere co-rotates with ECEF).  No Coriolis / centrifugal.

    Parameters
    ----------
    x, y : float       – position [m]
    vx, vy : float     – ECEF velocity [m/s]
    m : float           – mass [kg]
    F_D : float         – drag force magnitude [N]
    F_T : float         – thrust force magnitude [N]
    alpha : float       – angle of attack (from velocity direction) [rad]
    Isp : float         – specific impulse [s]

    Returns
    -------
    list : [dxdt, dydt, dvxdt, dvydt, dmdt]
    """
    r_val = np.sqrt(x**2 + y**2)
    v = np.sqrt(vx**2 + vy**2)

    # --- Gravity (toward Earth centre) ---
    g_mag = c.MU_EARTH / (r_val**2)
    ax_grav = -g_mag * x / r_val
    ay_grav = -g_mag * y / r_val

    # --- Thrust direction ---
    # Alpha is the angle of attack measured from the velocity vector.
    # When v ≈ 0 (pre-launch / very start), thrust radially outward.
    if v > 1e-6:
        vel_angle = np.arctan2(vy, vx)
        thrust_angle = vel_angle + alpha
    else:
        # Thrust radially outward
        thrust_angle = np.arctan2(y, x)

    ax_thrust = (F_T / m) * np.cos(thrust_angle)
    ay_thrust = (F_T / m) * np.sin(thrust_angle)

    # --- Drag (opposes ECEF velocity directly) ---
    if v > 1e-6:
        ax_drag = -(F_D / m) * (vx / v)
        ay_drag = -(F_D / m) * (vy / v)
    else:
        ax_drag = 0.0
        ay_drag = 0.0

    # --- Assemble ---
    dxdt  = vx
    dydt  = vy
    dvxdt = ax_thrust + ax_drag + ax_grav
    dvydt = ay_thrust + ay_drag + ay_grav
    dmdt  = -F_T / (Isp * c.G_0) if Isp > 0 else 0.0

    return [dxdt, dydt, dvxdt, dvydt, dmdt]


#===================================================
# Dynamics Functions
#===================================================

def rocket_dynamics(t, state):
    """
    Simulates the dynamics of the rocket. This function will be integrated 
    by the scipy.solve_ivp function.
    
    Parameters:
    -----------
    t : float
        Time variable (necessary for solve_ivp function)
    state : array
        Current state vector of the rocket (Cartesian ECEF)
        [x, y, vx, vy, m]
        - x:  horizontal position (downrange at launch) [m]
        - y:  vertical position (radial at launch) [m]
        - vx: horizontal velocity [m/s]
        - vy: vertical velocity [m/s]
        - m:  current mass [kg]

    Returns:
    --------
    list : Derivatives of the state vector
    """
    global time_kick_start, kick_performed, main_engine_cutoff, flag_falling_single_burn
    global current_kick_angle
    global atmosphere_exited, guidance_phase_active, time_atmosphere_exit
    global last_guidance_update_time, guidance_coefficients
    global apollo_coefficients_frozen, apollo_freeze_time
    global thrust_history, time_history
    global alpha_history, alpha_time_history
    global steering_history, steering_time_history

    # Get Cartesian state components
    x_pos, y_pos, vx, vy, m = state

    # Derive polar/legacy quantities needed by guidance and events
    r_val, alt, v, gamma, theta = cartesian_to_polar(x_pos, y_pos, vx, vy)

    # Build legacy state for guidance modules (ECI velocity = ECEF + v_boost)
    legacy_state = cartesian_to_legacy_state(x_pos, y_pos, vx, vy, m)
    s = legacy_state[0]
    v_eci = legacy_state[2]
    gamma_eci = legacy_state[3]

    # Check main engine state and second engine state
    event_main_engine_cutoff(t, state)
    if main_engine_cutoff:
        event_second_engine_ignition(t)
    
    # --- Get current thrust, Isp ---
    F_T, Isp = thrust_Isp()

    # --- Calculate dynamic pressure (needed for atmosphere exit check and drag) ---
    # In ECEF frame, velocity IS atmosphere-relative, so v is correct for drag
    q = atm.dynamic_pressure(v, alt)

    # --- Check atmosphere exit condition based on selected method ---
    atmosphere_exit_detected = False
    if sim_params.ATMOSPHERE_EXIT_METHOD == "altitude":
        atmosphere_exit_detected = (alt > sim_params.ALT_NO_ATMOSPHERE)
    elif sim_params.ATMOSPHERE_EXIT_METHOD == "dynamic_pressure":
        atmosphere_exit_detected = (q < sim_params.DYNAMIC_PRESSURE_THRESHOLD)

    # --- Get current angle of attack (GUIDANCE LOGIC) ---
    # All guidance uses the legacy_state (ECI quantities) internally.
    # Alpha is the angle of attack = angle from velocity direction.
    # steering_angle tracks only the deliberate control input (0 when no command)
    steering_angle = 0.0
    
    if t >= sim_params.TIME_TO_START_KICK and (not kick_performed):
        # Phase 1: Initial gravity turn (pitchover) - COMMON TO ALL MODES
        kick_alpha = pitch_program_linear(t, current_kick_angle)
        steering_angle = kick_alpha
        alpha = kick_alpha
        
    elif (kick_performed and sim_params.GUIDANCE_MODE in ["simple_poly", "linear_tangent", "bilinear_tangent", "apollo"] and 
          atmosphere_exit_detected and (not atmosphere_exited) and F_T > 0):
        # Detect atmosphere exit and initialize guidance (only if engines burning)
        atmosphere_exited = True
        time_atmosphere_exit = t
        guidance_phase_active = True
        last_guidance_update_time = t
        
        # Initialize guidance coefficients based on mode
        t_go = estimate_time_to_target(legacy_state, sim_params.TARGET_ORBITAL_ALTITUDE)
        
        if sim_params.GUIDANCE_MODE == "simple_poly":
            guidance_coefficients = simple_poly_guidance.compute_polynomial_coefficients(legacy_state, 
                                                                 sim_params.TARGET_ORBITAL_ALTITUDE, 
                                                                 t_go)
            alpha = simple_poly_guidance.polynomial_guidance(t, t_go, legacy_state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nAtmosphere exit at t = {t:.2f} s, alt = {alt/1000:.2f} km, q = {q:.2f} Pa")
                print(f"  Exit method: {sim_params.ATMOSPHERE_EXIT_METHOD}")
                print(f"  Switching to SIMPLE POLYNOMIAL guidance mode")
                print(f"  Initial t_go = {t_go:.2f} s")
        
        elif sim_params.GUIDANCE_MODE == "linear_tangent":
            guidance_coefficients = lts_guidance.compute_lts_coefficients(legacy_state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            alpha = lts_guidance.linear_tangent_steering(t, t_go, legacy_state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nAtmosphere exit at t = {t:.2f} s, alt = {alt/1000:.2f} km, q = {q:.2f} Pa")
                print(f"  Exit method: {sim_params.ATMOSPHERE_EXIT_METHOD}")
                print(f"  Switching to LINEAR TANGENT STEERING guidance mode")
                print(f"  Initial t_go = {t_go:.2f} s")
                print(f"  LTS coefficients: a={guidance_coefficients[0]:.6f}, b={guidance_coefficients[1]:.6f}")
                print(f"  Initial alpha command: {np.rad2deg(alpha):.2f} deg")
                
        elif sim_params.GUIDANCE_MODE == "bilinear_tangent":
            guidance_coefficients = bts_guidance.compute_bilinear_coefficients(legacy_state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            alpha = bts_guidance.bilinear_tangent_steering(t, t_go, legacy_state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nAtmosphere exit at t = {t:.2f} s, alt = {alt/1000:.2f} km, q = {q:.2f} Pa")
                print(f"  Exit method: {sim_params.ATMOSPHERE_EXIT_METHOD}")
                print(f"  Switching to BILINEAR TANGENT STEERING guidance mode")
                print(f"  Initial t_go = {t_go:.2f} s")
                print(f"  BTS coefficients: c1={guidance_coefficients[0]:.6f}, c2={guidance_coefficients[1]:.6f}, c1'={guidance_coefficients[2]:.6f}, c2'={guidance_coefficients[3]:.6f}")
                print(f"  Initial alpha command: {np.rad2deg(alpha):.2f} deg")
                
        elif sim_params.GUIDANCE_MODE == "apollo":
            guidance_coefficients = apollo_guidance_module.compute_apollo_coefficients(legacy_state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            apollo_freeze_time = t
            apollo_coefficients_frozen = False
            alpha, a_thrust_cmd = apollo_guidance_module.apollo_guidance(t, apollo_freeze_time, legacy_state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nAtmosphere exit at t = {t:.2f} s, alt = {alt/1000:.2f} km, q = {q:.2f} Pa")
                print(f"  Exit method: {sim_params.ATMOSPHERE_EXIT_METHOD}")
                print(f"  Switching to APOLLO POLYNOMIAL guidance mode")
                print(f"  Thrust magnitude control: {sim_params.APOLLO_THRUST_MAGNITUDE_CONTROL}")
                print(f"  Current downrange: {s/1000:.2f} km")
                print(f"  Initial t_go = {t_go:.2f} s")
                print(f"  Apollo coefficients: k1={guidance_coefficients[0]:.6f}, k2={guidance_coefficients[1]:.6f}, k3={guidance_coefficients[2]:.6f}, k4={guidance_coefficients[3]:.6f}")
                print(f"  Initial alpha command: {np.rad2deg(alpha):.2f} deg")
                if sim_params.APOLLO_THRUST_MAGNITUDE_CONTROL:
                    print(f"  Commanded thrust accel: {a_thrust_cmd:.2f} m/s²")
        
        steering_angle = alpha
        
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "simple_poly" and F_T > 0:
        if (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            t_go = estimate_time_to_target(legacy_state, sim_params.TARGET_ORBITAL_ALTITUDE)
            guidance_coefficients = simple_poly_guidance.compute_polynomial_coefficients(legacy_state,
                                                                 sim_params.TARGET_ORBITAL_ALTITUDE,
                                                                 t_go)
            last_guidance_update_time = t
        t_go = estimate_time_to_target(legacy_state, sim_params.TARGET_ORBITAL_ALTITUDE)
        alpha = simple_poly_guidance.polynomial_guidance(t, t_go, legacy_state, guidance_coefficients)
        steering_angle = alpha
    
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "linear_tangent" and F_T > 0:
        if (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            t_go = estimate_time_to_target(legacy_state, sim_params.TARGET_ORBITAL_ALTITUDE)
            guidance_coefficients = lts_guidance.compute_lts_coefficients(legacy_state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            last_guidance_update_time = t
        t_go = estimate_time_to_target(legacy_state, sim_params.TARGET_ORBITAL_ALTITUDE)
        alpha = lts_guidance.linear_tangent_steering(t, t_go, legacy_state, guidance_coefficients)
        steering_angle = alpha
        
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "bilinear_tangent" and F_T > 0:
        if (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            t_go = estimate_time_to_target(legacy_state, sim_params.TARGET_ORBITAL_ALTITUDE)
            guidance_coefficients = bts_guidance.compute_bilinear_coefficients(legacy_state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            last_guidance_update_time = t
        t_go = estimate_time_to_target(legacy_state, sim_params.TARGET_ORBITAL_ALTITUDE)
        alpha = bts_guidance.bilinear_tangent_steering(t, t_go, legacy_state, guidance_coefficients)
        steering_angle = alpha
        
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "apollo" and F_T > 0:
        t_go = estimate_time_to_target(legacy_state, sim_params.TARGET_ORBITAL_ALTITUDE)
        
        if t_go < sim_params.APOLLO_FREEZE_THRESHOLD and not apollo_coefficients_frozen:
            apollo_coefficients_frozen = True
            apollo_freeze_time = t
            
            if sim_params.EVENTS_PRINT:
                print(f"\n  Apollo coefficients FROZEN at t = {t:.2f} s (t_go = {t_go:.2f} s)")
        
        if (not apollo_coefficients_frozen) and (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            guidance_coefficients = apollo_guidance_module.compute_apollo_coefficients(legacy_state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            apollo_freeze_time = t
            last_guidance_update_time = t
        
        alpha, a_thrust_cmd = apollo_guidance_module.apollo_guidance(t, apollo_freeze_time, legacy_state, guidance_coefficients)
        
        if sim_params.APOLLO_THRUST_MAGNITUDE_CONTROL:
            F_T_commanded = m * a_thrust_cmd
            F_T_nominal, _ = thrust_Isp()
            F_T = min(F_T_commanded, F_T_nominal)
        steering_angle = alpha
        
    else:
        # Default: zero angle of attack (gravity turn mode or coasting)
        alpha = 0.

    # Store angle of attack and steering angle throughout the entire flight (for plotting)
    alpha_history.append(alpha)
    alpha_time_history.append(t)
    steering_history.append(steering_angle)
    steering_time_history.append(t)

    # --- Calculate drag force ---
    F_D = atm.drag_force(q)

    # --- Compute state derivatives using Cartesian EOM ---
    state_differentiated = diff_eom_cartesian(x_pos, y_pos, vx, vy, m, F_D, F_T, alpha, Isp)

    # Check if falling (radial velocity < 0 in ECEF)
    # v_radial = (vx*x + vy*y)/r corresponds to old state_differentiated[2]
    v_radial = (vx * x_pos + vy * y_pos) / r_val if r_val > 0 else 0.0
    if v_radial < 0:
        flag_falling_single_burn = True

    # Store thrust and time for later retrieval
    thrust_history.append(F_T)
    time_history.append(t)

    return state_differentiated


#===================================================
# Simulation Functions
#===================================================

def simulate_trajectory(init_time, time_stamp, state_init, stage_1_flag, 
                       stage_2_flag):
    """
    Simulates the trajectory of the rocket until a given time stamp or 
    until a certain interrupt function is called.

    Parameters:
    -----------
    init_time : float
        Initial time [s]
    time_stamp : float
        Time stamp until the simulation should be performed [s]
    state_init : array
        Initial state vector of the rocket
    stage_1_flag : bool
        True if simulating stage 1
    stage_2_flag : bool
        True if simulating stage 2
    
    Returns:
    --------
    solution : OdeResult
        Solution object from scipy.solve_ivp
    """
    t_span = (init_time, init_time + time_stamp + 1)
    t_eval = np.arange(init_time, init_time + time_stamp + sim_params.TIME_STEP, 
                       sim_params.TIME_STEP)

    if stage_1_flag:
        interrupt_list = [interrupt_stage_separation, interrupt_ground_collision, 
                         interrupt_velocity_exceeded]
    
    elif stage_2_flag:
        # Coasting single burn trajectory
        interrupt_list = [interrupt_radius_check, interrupt_stage_2_burnt, 
                         interrupt_ground_collision, interrupt_single_burn_traj, 
                         interrupt_horizontal_check]
    else:
        interrupt_list = [interrupt_ground_collision]
    
    for interrupt in interrupt_list:
        interrupt.terminal = True
        interrupt.direction = 0
    
    return solve_ivp(rocket_dynamics, y0=state_init, t_span=t_span, t_eval=t_eval, 
                    max_step=1, events=interrupt_list, atol=1e-8)


def run(initial_kick_angle):
    """
    Main function to run the rocket trajectory simulation with coasting single burn.
    
    Parameters:
    -----------
    initial_kick_angle : float
        Initial kick angle for gravity turn [rad]
        
    Returns:
    --------
    time_steps_simulation : array
        Time steps of the simulation [s]
    data : array
        State data over time
    alt_stopped : float or None
        Altitude where engine stopped [m]
    delta_v : float
        Delta-v required for circularization [m/s]
    m_propellant_total_used_2nd_stage : float
        Total propellant used in 2nd stage [kg]
    """
    global time_kick_start, kick_performed, time_raise, main_engine_cutoff
    global second_engine_ignition, stage_2_burnt, time_main_engine_cutoff
    global second_stage_cutoff, flag_falling_single_burn, current_kick_angle
    global atmosphere_exited, guidance_phase_active, time_atmosphere_exit
    global last_guidance_update_time, guidance_coefficients
    global thrust_history, time_history
    global alpha_history, alpha_time_history
    global steering_history, steering_time_history
    
    #===================================================
    # Reset global variables
    #===================================================
    time_kick_start = None
    kick_performed = False
    time_raise = sim_params.DURATION_INITIAL_KICK / 2.
    main_engine_cutoff = False
    second_engine_ignition = False
    stage_2_burnt = False
    time_main_engine_cutoff = None
    second_stage_cutoff = False
    flag_falling_single_burn = False
    current_kick_angle = initial_kick_angle  # Store for use in dynamics
    
    # Reset guidance phase variables
    atmosphere_exited = False
    guidance_phase_active = False
    time_atmosphere_exit = None
    last_guidance_update_time = 0.0
    guidance_coefficients = [0.0, 0.0, 0.0, 0.0]
    
    # Reset thrust and time history
    thrust_history = []
    time_history = []
    
    # Reset angle of attack history
    alpha_history = []
    alpha_time_history = []
    
    # Reset steering angle history (deliberate control input)
    steering_history = []
    steering_time_history = []

    #===================================================
    # Simulation until stage separation
    #===================================================

    # Define initial state
    # Cartesian ECEF: [x, y, vx, vy, m] — at rest on surface
    # y = radial (up at launch site), x = downrange (in orbital plane)
    initial_mass = (r.M_STRUCTURE_1 + r.M_PROP_1 + r.M_STRUCTURE_2 + 
                   r.M_PROP_2 + r.M_PAYLOAD)
    initial_state_1 = [0., c.R_EARTH, 0., 0., initial_mass]

    # Define time of simulation 1
    time_1 = 500.

    # Call simulation for stage 1
    sol_1 = simulate_trajectory(0, time_1, initial_state_1, True, False)

    #===================================================
    # Simulation after stage separation
    #===================================================
    
    # Define new initial state
    initial_state_2 = sol_1.y[:, -1]

    # Adjust mass -> perform stage separation
    initial_state_2[4] = initial_state_2[4] - r.M_STRUCTURE_1
    
    # Define time of simulation 2
    init_time_2 = sol_1.t[-1]
    time_2 = 4000.
    
    # Call simulation for stage 2
    sol_2 = simulate_trajectory(init_time_2, time_2, initial_state_2, False, 
                               True)

    data = np.concatenate((sol_1.y, sol_2.y), axis=1)
    time_steps_simulation = np.concatenate((sol_1.t, sol_2.t))

    # --- Process results for coasting single burn ---
    # Convert Cartesian terminal state to ECI legacy quantities for orbital analysis
    legacy_stop = cartesian_to_legacy_state(sol_2.y[0, -1], sol_2.y[1, -1],
                                            sol_2.y[2, -1], sol_2.y[3, -1], sol_2.y[4, -1])
    r_stop = legacy_stop[1]
    v_stop = legacy_stop[2]
    gamma_stop = legacy_stop[3]

    # Calculate altitude to stop burning
    alt_stop = r_stop - c.R_EARTH
    
    # Calculate orbital elements at stop
    a_stop, e_stop, r_apo_stop, r_peri_stop, orbit_period_stop = get_orbital_elements(
        r_stop, v_stop, gamma_stop)

    epsilon = (c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE) * 0.002
    diff = abs(r_apo_stop - (c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE))

    if diff < epsilon:
        # ----- Calculate delta v -----
        r_desired = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
        v_desired = np.sqrt(c.MU_EARTH / r_desired)
        
        # Get velocity at apogee from vis-viva
        # In ECI mode v_apo is already inertial; in surface mode boost = 0.
        v_apo = np.sqrt(c.MU_EARTH * a_stop * (1 - e_stop**2)) / r_apo_stop
        delta_v = np.abs(v_apo - v_desired)

        # ----- Calculate total propellant required -----
        m_propellant_left = sol_2.y[4, -1] - (r.M_STRUCTURE_2 + r.M_PAYLOAD)
        m_propellant_used = r.M_PROP_2 - m_propellant_left
        m_propellant_required = sol_2.y[4, -1] * (1 - np.exp(-delta_v / 
                                                    (c.G_0 * r.ISP_2)))

        # Check if the propellant required is less than the propellant left
        if m_propellant_required < m_propellant_left:
            m_propellant_total_used_2nd_stage = m_propellant_used + m_propellant_required
        else:
            m_propellant_total_used_2nd_stage = 999999999.

        # Calculate burn time for that delta_v
        burn_time_delta_v = calculate_burn_time(sol_2.y[4, -1], delta_v)
        if burn_time_delta_v > sim_params.MAX_ACCEPTED_BURN_TIME:
            m_propellant_total_used_2nd_stage = 999999999.

        if not SINGLE_BURN_FULL_SIMULATION:
            # Convert Cartesian data [x,y,vx,vy,m] to legacy [s,r,v_eci,gamma_eci,m]
            legacy_data = np.zeros_like(data)
            for i in range(data.shape[1]):
                legacy_data[:, i] = cartesian_to_legacy_state(data[0, i], data[1, i],
                                                              data[2, i], data[3, i], data[4, i])
            data = legacy_data
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
            steering_data = np.array(steering_history)
            steering_time_data = np.array(steering_time_history)
            return time_steps_simulation, data, alt_stop, delta_v, m_propellant_total_used_2nd_stage, thrust_data, time_thrust, alpha_data, alpha_time_data, steering_data, steering_time_data
        else:
            global TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
            TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = sol_2.t[-1]
            r.C_D = 0.
            
            # Print result of masses
            print("\t* Optimal altitude to stop burning: \t\t", alt_stop / 1000, "km")
            print("\t* Optimal time to stop burning: \t\t", 
                  TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL, "s")
            print("\t* Optimal delta-v: \t\t\t\t", delta_v, "m/s")
            print("\nPropellant Overview of 2nd Stage:")
            print("\t* Propellant left: \t\t\t\t", m_propellant_left, "kg")
            print("\t* Propellant used: \t\t\t\t", m_propellant_used, "kg")
            print("\t* Propellant required by circularization:\t", 
                  m_propellant_required, "kg")
            print("\t* Total propellant used: \t\t\t", 
                  m_propellant_total_used_2nd_stage, "kg")
            print("\t* Possible Payload: \t\t\t\t", 
                  (r.M_PROP_2 - m_propellant_total_used_2nd_stage), "kg")

            # Print launch azimuth info if available
            az = getattr(sim_params, "LAUNCH_AZIMUTH_DATA", None)
            if az is not None:
                print(f"\nLaunch Azimuth (ascent plane selection):")
                print(f"\t* Inertial azimuth  (A_I):  {az['A_I_deg']:.4f} deg")
                print(f"\t* Ground-rel heading (A_G): {az['A_G_deg']:.4f} deg")

            # ----- Simulate the rest of the trajectory -----
            # 1. Coasting
            second_stage_cutoff = True
            initial_state_3 = sol_2.y[:, -1]
            
            init_time_3 = sol_2.t[-1]
            time_3 = get_time_until_apogee(e_stop, gamma_stop, 
                                           v_stop, orbit_period_stop, 
                                           a_stop, r_stop)
            
            sol_3 = simulate_trajectory(init_time_3, time_3, initial_state_3, 
                                       False, False)

            # 2. Circularization burn (instantaneous delta-v along velocity direction)
            initial_state_4 = sol_3.y[:, -1]
            vx_circ = initial_state_4[2]
            vy_circ = initial_state_4[3]
            v_circ_mag = np.sqrt(vx_circ**2 + vy_circ**2)
            if v_circ_mag > 1e-10:
                initial_state_4[2] += delta_v * vx_circ / v_circ_mag
                initial_state_4[3] += delta_v * vy_circ / v_circ_mag
            else:
                initial_state_4[2] += delta_v  # fallback

            burn_time_delta_v = calculate_burn_time(initial_state_4[4], delta_v)
            print("\nBurn times:")
            print("\t* Time for delta-v:\t\t\t\t", burn_time_delta_v, "s\n")

            initial_state_4[4] -= m_propellant_required

            # 3. Simulation after circularization burn
            init_time_4 = sol_3.t[-1]
            time_4 = sim_params.DURATION_AFTER_SIMULATION
            
            sol_4 = simulate_trajectory(init_time_4, time_4, initial_state_4, 
                                       False, False)

            # Collect data and time steps
            data = np.concatenate((sol_1.y, sol_2.y, sol_3.y, sol_4.y), axis=1)
            time_steps_simulation = np.concatenate((sol_1.t, sol_2.t, sol_3.t, 
                                                   sol_4.t))

            # Convert Cartesian data [x,y,vx,vy,m] to legacy [s,r,v_eci,gamma_eci,m]
            legacy_data = np.zeros_like(data)
            for i in range(data.shape[1]):
                legacy_data[:, i] = cartesian_to_legacy_state(data[0, i], data[1, i],
                                                              data[2, i], data[3, i], data[4, i])
            data = legacy_data
    
            # Convert thrust history to numpy array for easier handling
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
            steering_data = np.array(steering_history)
            steering_time_data = np.array(steering_time_history)
    
            return time_steps_simulation, data, alt_stop, delta_v, m_propellant_total_used_2nd_stage, thrust_data, time_thrust, alpha_data, alpha_time_data, steering_data, steering_time_data
    
    else:
        # Convert Cartesian data [x,y,vx,vy,m] to legacy [s,r,v_eci,gamma_eci,m]
        legacy_data = np.zeros_like(data)
        for i in range(data.shape[1]):
            legacy_data[:, i] = cartesian_to_legacy_state(data[0, i], data[1, i],
                                                          data[2, i], data[3, i], data[4, i])
        data = legacy_data
        thrust_data = np.array(thrust_history)
        time_thrust = np.array(time_history)
        alpha_data = np.array(alpha_history)
        alpha_time_data = np.array(alpha_time_history)
        steering_data = np.array(steering_history)
        steering_time_data = np.array(steering_time_history)
        return time_steps_simulation, data, None, 9999999.0, 9999999.0, thrust_data, time_thrust, alpha_data, alpha_time_data, steering_data, steering_time_data

