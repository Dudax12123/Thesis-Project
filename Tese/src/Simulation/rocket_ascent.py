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
apollo_freeze_time = None  # Time when coefficients were frozen (tepoch)

# Steering angle history for plotting (guidance phase)
alpha_history = []  # Store steering angles during guidance phase
alpha_time_history = []  # Store corresponding time values for steering angles

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
    r = y[1]
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
    r_val = y[1]
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
    v = y[2]
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
    r_val = y[1]
    v = y[2]
    gamma = y[3]
    alt = r_val - c.R_EARTH

    if alt < sim_params.ALT_NO_ATMOSPHERE:
        return 1
    else:
        # Compute current orbital elements
        a, e, r_apo, r_peri, _ = get_orbital_elements(r_val, v, gamma)

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
    gamma = y[3]
    epsilon = np.deg2rad(0.01)
    
    if gamma < epsilon:
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
        Current state [s, r, v, gamma, m]
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
        Current state vector of the rocket
        [s, r, v, gamma, m]
        - s: downtrack [m]
        - r: radius from Earth's center [m]
        - v: velocity norm [m/s]
        - gamma: flight path angle [rad]
        - m: current mass [kg]

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

    # Get state components
    s, r_val, v, gamma, m = state

    # Compute altitude above Earth's surface
    alt = r_val - c.R_EARTH

    # Check main engine state and second engine state
    event_main_engine_cutoff(t, state)
    if main_engine_cutoff:
        event_second_engine_ignition(t)
    
    # --- Get current thrust, Isp ---
    F_T, Isp = thrust_Isp()

    # --- Get current angle of attack (GUIDANCE LOGIC) ---
    # Three-mode guidance system based on simulation_parameters.GUIDANCE_MODE
    
    if t >= sim_params.TIME_TO_START_KICK and (not kick_performed):
        # Phase 1: Initial gravity turn (pitchover) - COMMON TO ALL MODES
        alpha = pitch_program_linear(t, current_kick_angle)
        
    elif (kick_performed and sim_params.GUIDANCE_MODE in ["simple_poly", "linear_tangent", "apollo"] and 
          alt > sim_params.ALT_NO_ATMOSPHERE and (not atmosphere_exited) and F_T > 0):
        # Detect atmosphere exit and initialize guidance (only if engines burning)
        atmosphere_exited = True
        time_atmosphere_exit = t
        guidance_phase_active = True
        last_guidance_update_time = t
        
        # Initialize guidance coefficients based on mode
        t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
        
        if sim_params.GUIDANCE_MODE == "simple_poly":
            # Simple polynomial: linear gamma transition
            guidance_coefficients = simple_poly_guidance.compute_polynomial_coefficients(state, 
                                                                 sim_params.TARGET_ORBITAL_ALTITUDE, 
                                                                 t_go)
            alpha = simple_poly_guidance.polynomial_guidance(t, t_go, state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nAtmosphere exit at t = {t:.2f} s, alt = {alt/1000:.2f} km")
                print(f"  Switching to SIMPLE POLYNOMIAL guidance mode")
                print(f"  Initial t_go = {t_go:.2f} s")
        
        elif sim_params.GUIDANCE_MODE == "linear_tangent":
            # Linear tangent steering: tan(α + γ) varies linearly with time
            guidance_coefficients = lts_guidance.compute_lts_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            alpha = lts_guidance.linear_tangent_steering(t, t_go, state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nAtmosphere exit at t = {t:.2f} s, alt = {alt/1000:.2f} km")
                print(f"  Switching to LINEAR TANGENT STEERING guidance mode")
                print(f"  Initial t_go = {t_go:.2f} s")
                print(f"  LTS coefficients: a={guidance_coefficients[0]:.6f}, b={guidance_coefficients[1]:.6f}")
                print(f"  Initial alpha command: {np.rad2deg(alpha):.2f} deg")
                
        elif sim_params.GUIDANCE_MODE == "apollo":
            # Apollo polynomial: acceleration profiles with terminal constraints
            guidance_coefficients = apollo_guidance_module.compute_apollo_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            apollo_freeze_time = t  # Initialize freeze time
            apollo_coefficients_frozen = False
            alpha, a_thrust_cmd = apollo_guidance_module.apollo_guidance(t, apollo_freeze_time, state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nAtmosphere exit at t = {t:.2f} s, alt = {alt/1000:.2f} km")
                print(f"  Switching to APOLLO POLYNOMIAL guidance mode")
                print(f"  Thrust magnitude control: {sim_params.APOLLO_THRUST_MAGNITUDE_CONTROL}")
                print(f"  Current downrange: {s/1000:.2f} km")
                print(f"  Initial t_go = {t_go:.2f} s")
                print(f"  Apollo coefficients: k1={guidance_coefficients[0]:.6f}, k2={guidance_coefficients[1]:.6f}, k3={guidance_coefficients[2]:.6f}, k4={guidance_coefficients[3]:.6f}")
                print(f"  Initial alpha command: {np.rad2deg(alpha):.2f} deg")
                if sim_params.APOLLO_THRUST_MAGNITUDE_CONTROL:
                    print(f"  Commanded thrust accel: {a_thrust_cmd:.2f} m/s²")
        
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "simple_poly" and F_T > 0:
        # Phase 2a: Simple polynomial guidance (only while engines burning)
        
        # Update guidance coefficients periodically
        if (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            guidance_coefficients = simple_poly_guidance.compute_polynomial_coefficients(state,
                                                                 sim_params.TARGET_ORBITAL_ALTITUDE,
                                                                 t_go)
            last_guidance_update_time = t
        
        # Compute guidance angle
        t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
        alpha = simple_poly_guidance.polynomial_guidance(t, t_go, state, guidance_coefficients)
    
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "linear_tangent" and F_T > 0:
        # Phase 2b: Linear tangent steering guidance (only while engines burning)
        
        # Update guidance coefficients periodically
        if (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            guidance_coefficients = lts_guidance.compute_lts_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            last_guidance_update_time = t
        
        # Compute guidance angle
        t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
        alpha = lts_guidance.linear_tangent_steering(t, t_go, state, guidance_coefficients)
        
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "apollo" and F_T > 0:
        # Phase 2b: Apollo polynomial guidance (only while engines burning)
        
        t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
        
        # Check if we should freeze coefficients (t_go below threshold)
        if t_go < sim_params.APOLLO_FREEZE_THRESHOLD and not apollo_coefficients_frozen:
            # Freeze coefficients to prevent numerical instability
            # Justification: As t_go->0, denominators in k1,k2,k3,k4 cause unbounded growth
            apollo_coefficients_frozen = True
            apollo_freeze_time = t
            
            if sim_params.EVENTS_PRINT:
                print(f"\n  Apollo coefficients FROZEN at t = {t:.2f} s (t_go = {t_go:.2f} s)")
        
        # Update coefficients if not frozen and update interval reached
        if (not apollo_coefficients_frozen) and (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            guidance_coefficients = apollo_guidance_module.compute_apollo_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            apollo_freeze_time = t  # Update epoch time
            last_guidance_update_time = t
        
        # Compute guidance angle using current or frozen coefficients
        alpha, a_thrust_cmd = apollo_guidance_module.apollo_guidance(t, apollo_freeze_time, state, guidance_coefficients)
        
        # Apply thrust magnitude control if enabled
        if sim_params.APOLLO_THRUST_MAGNITUDE_CONTROL:
            # Override thrust with commanded magnitude
            # Convert acceleration command to force
            F_T_commanded = m * a_thrust_cmd
            # Get the nominal (maximum) thrust available
            F_T_nominal, _ = thrust_Isp()
            # Use commanded thrust but limit to maximum available
            F_T = min(F_T_commanded, F_T_nominal)
        
    else:
        # Default: zero angle of attack (gravity turn mode or coasting)
        alpha = 0.

    # Store steering angle throughout the entire flight (for plotting)
    # This captures initial kick, guidance phase, and coasting
    alpha_history.append(alpha)
    alpha_time_history.append(t)

    # --- Determine current accelerations and forces ---
    a_grav = grav.gravitational_acceleration(r_val)
    
    # Calculate dynamic pressure and drag
    q = atm.dynamic_pressure(v, alt)
    F_D = atm.drag_force(q)
    
    # Lift force (typically neglected)
    F_L = 0.0

    state_differentiated = diff_eom_base(s, r_val, v, gamma, m, F_L, F_D, F_T, 
                                         a_grav, alpha, Isp)

    if time_kick_start == None:
        state_differentiated[3] = 0.0

    if state_differentiated[2] < 0:
        flag_falling_single_burn = True

    # Store thrust and time for later retrieval
    thrust_history.append(F_T)
    time_history.append(t)

    return state_differentiated


def diff_eom_base(s, r_val, v, gamma, m, F_L, F_D, F_T, a_grav, alpha, Isp):
    """
    Differential equations of motion for the rocket WITHOUT Earth rotation.
    
    Parameters:
    -----------
    s : float
        Downtrack [m]
    r_val : float
        Radius from Earth's center [m]
    v : float
        Velocity norm [m/s]
    gamma : float
        Flight path angle [rad]
    m : float
        Current mass [kg]
    F_L : float
        Lift force [N]
    F_D : float
        Drag force [N]
    F_T : float
        Thrust force [N]
    a_grav : float
        Gravity acceleration [m/s^2]
    alpha : float
        Angle of attack [rad]
    Isp : float
        Specific impulse [s]

    Returns:
    --------
    list : Derivatives of the state vector [dsdt, drdt, dvdt, dgammadt, dmdt]
    """
    # --- Get trigonometric operations of gamma and alpha ---
    c_gamma = np.cos(gamma)
    s_gamma = np.sin(gamma)
    c_alpha = np.cos(alpha)
    s_alpha = np.sin(alpha)

    # --- Compute the derivatives ---
    dsdt = (c.R_EARTH / r_val) * v * c_gamma

    # Radial velocity
    drdt = v * np.sin(gamma)

    # Velocity magnitude change
    dvdt = (F_T / m) * c_alpha - (F_D / m) - a_grav * s_gamma

    # Catch the case of zero velocity to avoid division by zero
    epsilon = 1e-6
    if v < epsilon:
        dgammadt = 0.
    else:
        # Flight path angle change
        dgammadt = (1. / v) * ((F_T / m) * s_alpha + F_L / m - 
                               (a_grav - (v**2 / r_val)) * c_gamma)
    
    # Derivative of mass
    dmdt = -F_T / (Isp * c.G_0)

    return [dsdt, drdt, dvdt, dgammadt, dmdt]


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
    
    # Reset steering angle history
    alpha_history = []
    alpha_time_history = []

    #===================================================
    # Simulation until stage separation
    #===================================================

    # Define initial state
    initial_mass = (r.M_STRUCTURE_1 + r.M_PROP_1 + r.M_STRUCTURE_2 + 
                   r.M_PROP_2 + r.M_PAYLOAD)
    initial_state_1 = [0., c.R_EARTH, 0., np.deg2rad(90.), initial_mass]

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
    r_stop = sol_2.y[1, -1]
    v_stop = sol_2.y[2, -1]
    gamma_stop = sol_2.y[3, -1]

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
        
        # Get velocity at apogee
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
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
            return time_steps_simulation, data, alt_stop, delta_v, m_propellant_total_used_2nd_stage, thrust_data, time_thrust, alpha_data, alpha_time_data
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

            # ----- Simulate the rest of the trajectory -----
            # 1. Coasting
            second_stage_cutoff = True
            initial_state_3 = sol_2.y[:, -1]
            
            init_time_3 = sol_2.t[-1]
            time_3 = get_time_until_apogee(e_stop, initial_state_3[3], 
                                           initial_state_3[2], orbit_period_stop, 
                                           a_stop, initial_state_3[1])
            
            sol_3 = simulate_trajectory(init_time_3, time_3, initial_state_3, 
                                       False, False)

            # 2. Circularization burn (instantaneous delta-v)
            initial_state_4 = sol_3.y[:, -1]
            initial_state_4[2] += delta_v

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
    
            # Convert thrust history to numpy array for easier handling
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
    
            return time_steps_simulation, data, alt_stop, delta_v, m_propellant_total_used_2nd_stage, thrust_data, time_thrust, alpha_data, alpha_time_data
    
    else:
        thrust_data = np.array(thrust_history)
        time_thrust = np.array(time_history)
        alpha_data = np.array(alpha_history)
        alpha_time_data = np.array(alpha_time_history)
        return time_steps_simulation, data, None, 9999999.0, 9999999.0, thrust_data, time_thrust, alpha_data, alpha_time_data

