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
from Auxiliary import earth_rotation as earth_rot
from Input_File import simulation_parameters as sim_params
from Auxiliary import rocket_specs as r
import Guidance.gravity_turn as gravity_turn_guidance
import Guidance.simple_polynomial as simple_poly_guidance
import Guidance.linear_tangent_steering as lts_guidance
import Guidance.bilinear_tangent_steering as bts_guidance
import Guidance.apollo_guidance as apollo_guidance_module
import Guidance.cpr_guidance as cpr_guidance_module
import Guidance.peg_guidance as peg_guidance_mod
import Guidance.peg_guidance_new as peg_new_mod
import Guidance.exp_shooting_guidance as exp_shoot_mod
import Guidance.pso_paper_guidance as pso_paper_mod
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

# Inertial-frame flag: set True after ECEF→ECI transition at SECO
# so that pseudo-forces are no longer applied during coast/orbit phases.
PROPAGATING_IN_INERTIAL_FRAME = False

# Guidance phase flags
atmosphere_exited = False
guidance_phase_active = False
time_atmosphere_exit = None
time_guidance_start = None
last_guidance_update_time = 0.0
guidance_initial_tgo = None   # t_go stored at guidance start (used when GUIDANCE_TGO_FIXED)
guidance_coefficients = [0.0, 0.0, 0.0, 0.0]  # For simple_poly: [a0, a1] or apollo: [k1, k2, k3, k4]
cpr_theta_dot = None    # [rad/s] constant pitch rate, computed at CPR guidance start
cpr_t_start = None      # [s] time when CPR guidance began
apollo_coefficients_frozen = False  # Flag to indicate if Apollo coefficients are frozen

# Thrust history for plotting
thrust_history = []  # Store thrust values during integration
time_history = []    # Store corresponding time values
apollo_freeze_time = None  # Time when coefficients were frozen (tepoch)
apollo_previous_tgo = None  # Previous Apollo t_go estimate, used as fallback
lts_previous_tgo = None     # Previous t_go for linear/bilinear propellant method (fallback)

# Steering angle history for plotting (guidance phase)
alpha_history = []  # Store steering angles during guidance phase
alpha_time_history = []  # Store corresponding time values for steering angles
theta_history = []       # pitch angle θ = α + γ, recorded every ODE call
theta_time_history = []  # corresponding time values for theta
tgo_history = []    # Store Apollo t_go estimates during guidance phase
tgo_time_history = []  # Store corresponding time values for t_go

# Crash detection
CRASH_DETECTED = False
CRASH_TIME = None

# Pseudo-force acceleration history for plotting
coriolis_mag_history = []      # Store Coriolis acceleration magnitude
centrifugal_mag_history = []   # Store centrifugal acceleration magnitude
cross_heading_counter_force_history = []  # Store lateral counter-force [N]
cross_heading_accel_history = []          # Store cross-heading acceleration [m/s²]

# Fairing jettison state
fairing_jettisoned = False
time_fairing_jettison = None

# PEG guidance state
peg_A       = 0.0
peg_B       = 0.0
peg_T       = None   # burn-time estimate [s]
peg_t_epoch = None   # time of last major-loop update [s]
peg_frozen  = False

# Exponential-shooting guidance state
exp_shoot_a     = None   # coefficient a in θ(t) = a·exp(b·t_rel)
exp_shoot_b     = None   # coefficient b
exp_shoot_epoch = None   # absolute t when (a, b) were computed

# PSO paper-mode guidance state (Morgado, Marta, Gil 2022)
# These are set by Simulation/pso_paper_solver.py before each ra.run() call.
pso_paper_lam0          = None    # tuple (lam_h0, lam_V0, lam_g0) — initial costates
pso_paper_gamma_p       = None    # initial pitch angle [rad] (≈ 1.55, used for kick mapping)
pso_paper_dt_coast      = 0.0     # coast duration during Stage 2 [s]
pso_paper_coast_pct     = 0.0     # fraction of Δt_T spent thrusting BEFORE coast
pso_paper_burn_pct      = 1.0     # fraction of m_prop_S2 / m_dot consumed in Stage 2
# These are computed once Stage 2 ignites (or coast/seco are needed):
pso_paper_coast_start_t = None    # absolute time coast window opens [s]
pso_paper_coast_end_t   = None    # absolute time coast window closes [s]
pso_paper_seco_t        = None    # absolute time of commanded SECO [s]
# Per-step history (cleared in run()):
pso_paper_costate_history       = []   # list of (lam_h, lam_V, lam_g) tuples
pso_paper_costate_time_history  = []   # matching absolute times [s]

# PEG_new guidance state (analytical predictor-corrector)
peg_new_vgo_r     = 0.0
peg_new_vgo_theta = 0.0
peg_new_L0        = 1.0
peg_new_tgo       = None
peg_new_t_lambda  = 0.0
peg_new_lambda_r  = 0.0
peg_new_t_epoch   = None
peg_new_frozen    = False

# Stage 1 Isp linear-ramp state (only active when ISP_1_MODE = "linear")
_isp1_last_update_time = 0.0   # Last time Isp was stepped
_isp1_current = r.ISP_1_SL    # Current Isp value used by the ramp

# Stage 1 Thrust linear-ramp state (only active when THRUST_1_MODE = "linear")
_thrust1_last_update_time = 0.0   # Last time thrust was stepped
_thrust1_current = r.F_THRUST_1_SL  # Current thrust value used by the ramp

# Earth rotation launch geometry (set in run())
LAUNCH_AZIMUTH = np.deg2rad(90.0)   # Active azimuth in rotating frame [rad]
LAUNCH_AZIMUTH_INERTIAL = np.deg2rad(90.0)  # Geometric azimuth in inertial frame [rad]
LAUNCH_LATITUDE_RAD = 0.0           # Current launch-site latitude [rad]
LAUNCH_ROTATION_SPEED = 0.0         # Surface rotation speed at launch latitude [m/s]
AZIMUTH_MODE_USED = "corrected"     # Active azimuth mode used during current run

# Final inclination metrics from the latest run
LAST_ACHIEVED_INCLINATION_DEG = np.nan
LAST_INCLINATION_DRIFT_DEG = np.nan

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
    # Paper-mode: let the trajectory propagate even if it overshoots the
    # target radius — the PSO objective penalises altitude error softly, so
    # a hard interrupt would just hide bad designs from the cost function.
    if sim_params.GUIDANCE_MODE == "pso_paper":
        return 1

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


def interrupt_fairing_jettison(t, y):
    """
    Returns zero when the atmosphere is exited (fairing jettison point).
    Fires once; stays at 1.0 afterwards via the fairing_jettisoned flag.
    """
    if fairing_jettisoned:
        return 1.0
    if atmosphere_exited:
        if sim_params.INTERRUPTS_PRINT:
            print("Interrupt Fairing Jettison happened at time", t)
        return 0.0
    return 1.0


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
    # Paper-mode: stop integration at the PSO-commanded SECO so the
    # simulation doesn't coast for thousands of seconds afterward.  Without
    # this, interrupt_stage_2_burnt (propellant-exhausted) never fires when
    # last_burn_pct < 1 and the integration runs out the full 4000-second
    # time span — wasting ~8× computation and giving a meaningless final state.
    if (sim_params.GUIDANCE_MODE == "pso_paper"
            and pso_paper_seco_t is not None
            and t >= pso_paper_seco_t):
        if sim_params.INTERRUPTS_PRINT:
            print(f"[pso_paper] Stage-2 SECO interrupt at t={t:.2f}s")
        return 0

    m = y[4]
    fairing_correction = r.M_FAIRING if not fairing_jettisoned else 0.0
    if m <= (r.M_PAYLOAD + r.M_STRUCTURE_2 + fairing_correction):
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
    r_val = y[1]
    v = y[2]
    gamma = y[3]
    r_desired = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
    v_desired = np.sqrt(c.MU_EARTH / r_desired)

    if sim_params.ENABLE_EARTH_ROTATION:
        lat = get_latitude_from_downrange(y[0])
        heading = get_heading_from_state(y, lat)
        v_inertial, _ = earth_rot.ecef_to_eci_velocity(v, gamma, heading, lat, r_val)
        return v_inertial - v_desired

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
    # Paper-mode (Morgado et al. 2022): SECO is commanded from the PSO design
    # vector via thrust_Isp(); the apogee-crossing check is therefore disabled
    # so that the mid-burn coast and terminal impulse can run to completion.
    if sim_params.GUIDANCE_MODE == "pso_paper":
        return 1

    r_val = y[1]
    v = y[2]
    gamma = y[3]
    lat = get_latitude_from_downrange(y[0]) if sim_params.ENABLE_EARTH_ROTATION else LAUNCH_LATITUDE_RAD
    alt = r_val - c.R_EARTH

    if alt < sim_params.ALT_NO_ATMOSPHERE:
        return 1
    else:
        if sim_params.ENABLE_EARTH_ROTATION:
            heading = get_heading_from_state(y, lat)
            v, gamma = earth_rot.ecef_to_eci_velocity(v, gamma, heading, lat, r_val)

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
    # Paper-mode: gamma is allowed to evolve freely toward target via the
    # costate-driven steering, so the horizontal-flight interrupt is disabled.
    if sim_params.GUIDANCE_MODE == "pso_paper":
        return 1

    gamma = y[3]
    r_val = y[1]
    alt = r_val - c.R_EARTH
    epsilon = np.deg2rad(0.01)

    # Only trigger near the target altitude to prevent false firing during the
    # coast between stage-1 separation and stage-2 ignition (where gamma
    # naturally crosses zero on the parabolic arc at low altitude).
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
    
    stage1_dry = r.M_STRUCTURE_1 - (r.M_FAIRING if fairing_jettisoned else 0.0)
    first_stage_leftover_propellant = y[4] - (stage1_dry + r.M_STRUCTURE_2 +
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
        if not second_engine_ignition:
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


def get_latitude_from_downrange(s):
    """
    Compute geocentric latitude from downrange along the launch great-circle.

    This keeps latitude physically bounded in [-pi/2, pi/2] and avoids drift
    that can appear when integrating latitude with a fixed-heading assumption.
    """
    if not sim_params.ENABLE_EARTH_ROTATION:
        return LAUNCH_LATITUDE_RAD

    sigma = s / c.R_EARTH
    sin_phi0 = np.sin(LAUNCH_LATITUDE_RAD)
    cos_phi0 = np.cos(LAUNCH_LATITUDE_RAD)

    # Great-circle relation from launch site with initial inertial azimuth.
    sin_lat = (sin_phi0 * np.cos(sigma) +
               cos_phi0 * np.sin(sigma) * np.cos(LAUNCH_AZIMUTH_INERTIAL))
    sin_lat = np.clip(sin_lat, -1.0, 1.0)
    return np.arcsin(sin_lat)


def get_latitude_rate_from_downrange(s, dsdt):
    """
    Compute d(latitude)/dt from great-circle geometry and ds/dt.
    """
    if not sim_params.ENABLE_EARTH_ROTATION:
        return 0.0

    sigma = s / c.R_EARTH
    sin_phi0 = np.sin(LAUNCH_LATITUDE_RAD)
    cos_phi0 = np.cos(LAUNCH_LATITUDE_RAD)
    cos_beta0 = np.cos(LAUNCH_AZIMUTH_INERTIAL)

    u = sin_phi0 * np.cos(sigma) + cos_phi0 * np.sin(sigma) * cos_beta0
    u = np.clip(u, -1.0, 1.0)
    du_dsigma = -sin_phi0 * np.sin(sigma) + cos_phi0 * np.cos(sigma) * cos_beta0

    cos_lat = np.sqrt(max(1.0 - u**2, 0.0))
    cos_lat = max(cos_lat, 1e-10)
    dlat_dsigma = du_dsigma / cos_lat
    dsigma_dt = dsdt / c.R_EARTH

    return dlat_dsigma * dsigma_dt


def get_heading_from_state(state, lat_rad=None):
    """
    Return heading (azimuth) used for Earth-rotation contributions.

    If heading propagation is enabled and available in the state vector,
    use that tracked value. Otherwise fall back to the active launch azimuth.
    """
    if not sim_params.ENABLE_EARTH_ROTATION:
        return LAUNCH_AZIMUTH

    if sim_params.TRACK_HEADING_STATE and len(state) > 6:
        return state[6]

    return LAUNCH_AZIMUTH


def get_heading_rate_from_latitude(lat_rad, dlatdt, heading_rad):
    """
    Compute d(heading)/dt from great-circle geometry and latitude rate.

    This keeps heading consistent with the same spherical-geometry assumption
    used for latitude propagation in the 2D ascent model.
    """
    if not sim_params.ENABLE_EARTH_ROTATION:
        return 0.0

    return np.tan(lat_rad) * np.tan(heading_rad) * dlatdt


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


def get_inertial_state_components(r_val, v_ecef, gamma_ecef, lat_rad, heading_rad=None):
    """
    Return velocity and flight-path angle in ECI frame.

    Parameters:
    -----------
    r_val : float
        Radial distance to Earth's center [m]
    v_ecef : float
        Velocity in rotating frame [m/s]
    gamma_ecef : float
        Flight-path angle in rotating frame [rad]
    lat_rad : float
        Current latitude [rad]
    heading_rad : float, optional
        Current heading/azimuth [rad]. If None, uses active launch azimuth.

    Returns:
    --------
    v_eci : float
        Inertial velocity [m/s]
    gamma_eci : float
        Inertial flight-path angle [rad]
    """
    if sim_params.ENABLE_EARTH_ROTATION:
        heading = LAUNCH_AZIMUTH if heading_rad is None else heading_rad
        return earth_rot.ecef_to_eci_velocity(v_ecef, gamma_ecef, heading, lat_rad, r_val)
    return v_ecef, gamma_ecef


def _get_stage1_isp(t):
    """
    Returns the effective stage-1 Isp based on the ISP_1_MODE setting in simulation_parameters.

    Modes
    -----
    "sea_level"  : constant ISP_1_SL throughout stage 1
    "vacuum"     : constant ISP_1_VAC throughout stage 1
    "average"    : constant average of ISP_1_SL and ISP_1_VAC
    "linear"     : ramps from ISP_1_SL at t=0 to ISP_1_VAC at estimated stage-1 burnout,
                   updated in discrete steps of ISP_1_LINEAR_UPDATE_RATE seconds
    """
    global _isp1_last_update_time, _isp1_current

    mode = sim_params.ISP_1_MODE

    if mode == "sea_level":
        return r.ISP_1_SL

    if mode == "vacuum":
        return r.ISP_1_VAC

    if mode == "average":
        return (r.ISP_1_SL + r.ISP_1_VAC) / 2.0

    if mode == "linear":
        # Estimated stage-1 burnout time (using sea-level mdot as reference)
        mdot_sl = r.F_THRUST_1_SL / (r.ISP_1_SL * c.G_0)
        t_burnout = r.M_PROP_1 / mdot_sl

        # Only update Isp at the requested step interval
        if t - _isp1_last_update_time >= sim_params.ISP_1_LINEAR_UPDATE_RATE:
            frac = min(t / t_burnout, 1.0)
            _isp1_current = r.ISP_1_SL + frac * (r.ISP_1_VAC - r.ISP_1_SL)
            _isp1_last_update_time = t

        return _isp1_current

    # Fallback
    return r.ISP_1_SL


def _get_stage1_thrust(t):
    """
    Returns the effective stage-1 thrust based on the THRUST_1_MODE setting in simulation_parameters.

    Modes
    -----
    "sea_level"  : constant F_THRUST_1_SL throughout stage 1
    "vacuum"     : constant F_THRUST_1_VAC throughout stage 1
    "average"    : constant average of F_THRUST_1_SL and F_THRUST_1_VAC
    "linear"     : ramps from F_THRUST_1_SL at t=0 to F_THRUST_1_VAC at estimated stage-1 burnout,
                   updated in discrete steps of THRUST_1_LINEAR_UPDATE_RATE seconds
    """
    global _thrust1_last_update_time, _thrust1_current

    mode = sim_params.THRUST_1_MODE

    if mode == "sea_level":
        return r.F_THRUST_1_SL

    if mode == "vacuum":
        return r.F_THRUST_1_VAC

    if mode == "average":
        return (r.F_THRUST_1_SL + r.F_THRUST_1_VAC) / 2.0

    if mode == "linear":
        # Estimated stage-1 burnout time (using sea-level mdot as reference)
        mdot_sl = r.F_THRUST_1_SL / (r.ISP_1_SL * c.G_0)
        t_burnout = r.M_PROP_1 / mdot_sl

        # Only update thrust at the requested step interval
        if t - _thrust1_last_update_time >= sim_params.THRUST_1_LINEAR_UPDATE_RATE:
            frac = min(t / t_burnout, 1.0)
            _thrust1_current = r.F_THRUST_1_SL + frac * (r.F_THRUST_1_VAC - r.F_THRUST_1_SL)
            _thrust1_last_update_time = t

        return _thrust1_current

    # Fallback
    return r.F_THRUST_1_SL


def _paper_costate_offset(state_len):
    """Return the index where costates begin in the state vector for paper mode.

    Layout: [s, r, v, gamma, m] (+ [lat]) (+ [heading]) + [lam_h, lam_V, lam_g]
    Returns None if the state isn't extended with costates yet.
    """
    base = 5  # s, r, v, gamma, m
    if sim_params.ENABLE_EARTH_ROTATION:
        base += 1
        if sim_params.TRACK_HEADING_STATE:
            base += 1
    if state_len >= base + 3:
        return base
    return None


def _paper_setup_stage2_schedule(t_ignition):
    """Compute the absolute-time coast/SECO schedule from PSO design vars.

    Called once when Stage 2 ignites (in paper mode).  Uses:
      - pso_paper_burn_pct  -> total active-thrust duration Δt_T
      - pso_paper_coast_pct -> fraction of Δt_T thrusting before coast
      - pso_paper_dt_coast  -> coast duration Δt_c
    """
    global pso_paper_coast_start_t, pso_paper_coast_end_t, pso_paper_seco_t
    if pso_paper_burn_pct is None:
        return
    mdot_s2 = r.F_THRUST_2 / (r.ISP_2 * c.G_0)
    t_max_full_burn = r.M_PROP_2 / mdot_s2
    dt_T = float(pso_paper_burn_pct) * t_max_full_burn
    dt_before_coast = float(pso_paper_coast_pct) * dt_T
    pso_paper_coast_start_t = t_ignition + dt_before_coast
    pso_paper_coast_end_t   = pso_paper_coast_start_t + float(pso_paper_dt_coast)
    pso_paper_seco_t        = pso_paper_coast_end_t + (dt_T - dt_before_coast)


def thrust_Isp(t):
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
        F_T = _get_stage1_thrust(t)
        Isp = _get_stage1_isp(t)
    elif main_engine_cutoff and not second_engine_ignition:
        F_T = 0
        Isp = _get_stage1_isp(t)
    elif main_engine_cutoff and second_stage_cutoff:
        F_T = 0
        Isp = r.ISP_2
    elif main_engine_cutoff and second_engine_ignition:
        F_T = r.F_THRUST_2
        Isp = r.ISP_2
    else:
        print("Warning: Both first stage and second stage engines are running at the same time.")
        F_T = _get_stage1_thrust(t)
        Isp = _get_stage1_isp(t)

    # --- Paper-mode mid-burn coast / commanded SECO (paper sec 4.1) ---
    if (sim_params.GUIDANCE_MODE == "pso_paper"
            and second_engine_ignition
            and pso_paper_burn_pct is not None):
        if pso_paper_seco_t is not None and t >= pso_paper_seco_t:
            # Commanded SECO from PSO design var.  Mark stage as cut so the
            # downstream coast/circularisation logic engages naturally.
            second_stage_cutoff = True
            return 0.0, r.ISP_2
        if (pso_paper_coast_start_t is not None
                and pso_paper_coast_start_t <= t < pso_paper_coast_end_t):
            # Mid-burn coast — engine off, no propellant consumption
            return 0.0, r.ISP_2

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

    # Paper-mode: the pitch maneuver is an instantaneous gamma state-jump
    # applied inside run() at t = PSO_PAPER_T_PITCHOVER.  This function must
    # never produce an AOA pulse — return zero always.
    # kick_performed is set True here (not at t=3s in run()) so that the
    # atmosphere-exit gate fires at the right altitude.  By t≈7.5s (when this
    # function is first called), v≈115 m/s → q≈7800 Pa > threshold → no false
    # atmosphere-exit trigger from the low-velocity vertical-liftoff phase.
    if sim_params.GUIDANCE_MODE == "pso_paper":
        if time_kick_start is None:
            time_kick_start = t
        kick_performed = True
        return 0.0

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

    Uses remaining propellant burn time as the primary estimate, which is far
    more accurate for Apollo-style guidance than altitude / radial-velocity.
    The altitude-based formula gives a t_go that is far too large early in
    ascent (when gamma is large and altitude gain is slow), leading to nearly-
    vertical thrust commands and an under-powered horizontal channel.

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
    s, r_val, v, gamma, m = state[:5]

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


def _compute_apollo_tgo(state, F_T, Isp, target_altitude, previous_tgo):
    """
    Compute Apollo-style time-to-go for the Apollo guidance mode.

    Returns a tuple ``(guidance_tgo, display_tgo)``:

    * ``guidance_tgo`` — t_go passed to the guidance law (compute_apollo_coefficients
      and apollo_guidance).  During **stage 1** this is the *total* remaining
      ascent time: stage-1 remaining burn + coast + full stage-2 insertion burn.
      This ensures the polynomial plans the full remaining trajectory, not just
      the few seconds left on stage 1.  During **stage 2** it equals the
      propellant-derived stage-2 t_go directly.

    * ``display_tgo`` — identical to ``guidance_tgo``; kept as a separate return
      value for clarity and to avoid changing call-site signatures.

    Only called when GUIDANCE_MODE == "apollo".
    """
    s, r_val, v, gamma, m = state[:5]

    # Dry mass depends on which stage is currently burning
    if not main_engine_cutoff:
        stage1_dry = r.M_STRUCTURE_1 - (r.M_FAIRING if fairing_jettisoned else 0.0)
        dry_mass = stage1_dry + r.M_STRUCTURE_2 + r.M_PROP_2 + r.M_PAYLOAD
    else:
        fairing_correction = r.M_FAIRING if not fairing_jettisoned else 0.0
        dry_mass = r.M_STRUCTURE_2 + r.M_PAYLOAD + fairing_correction

    remaining_prop = m - dry_mass
    if remaining_prop <= 0.0:
        return 0.0, 0.0

    if F_T <= 0.0 or Isp <= 0.0:
        fallback = float(previous_tgo) if previous_tgo is not None else 0.0
        return fallback, fallback

    Ve = Isp * c.G_0
    mdot = F_T / Ve
    T_BUP = remaining_prop / mdot

    # Convert current velocity to inertial (ECI) frame if Earth rotation is enabled,
    # so that VG is computed consistently against the inertial orbital target velocity.
    v_inertial = v
    gamma_inertial = gamma
    if sim_params.ENABLE_EARTH_ROTATION:
        lat = get_latitude_from_downrange(s)
        heading = get_heading_from_state(state)
        v_inertial, gamma_inertial = earth_rot.ecef_to_eci_velocity(
            v, gamma, heading, lat, r_val
        )

    # Velocity-to-be-gained in the (downrange, altitude) 2-D frame
    # — same frame used by compute_apollo_coefficients
    vx_current = v_inertial * np.cos(gamma_inertial)
    vy_current = v_inertial * np.sin(gamma_inertial)
    r_target = c.R_EARTH + target_altitude
    vx_target = np.sqrt(c.MU_EARTH / r_target)
    vy_target = 0.0
    VG_vec = np.array([vx_target - vx_current, vy_target - vy_current])

    # TODO: Add gravity compensation once sign convention is verified:
    #   VG_vec -= g_eff_vector * previous_tgo

    VG = float(np.linalg.norm(VG_vec))

    if not main_engine_cutoff:
        # ---------------------------------------------------------------
        # STAGE 1: guidance_tgo = total remaining ascent time
        #   T_BUP_1_remaining + coast + stage-2 insertion burn
        #
        # The truncated rocket equation (used in estimate_apollo_time_to_go)
        # is only accurate when VG/Ve << 1.  At guidance activation, VG is
        # typically ~5000-6000 m/s while Ve_2 = 3413 m/s, so VG/Ve ≈ 1.5-1.7.
        # The truncated formula always clamps to 0.5 * T_BUP and produces a
        # constant t_go that never declines.
        #
        # Instead, use the exact rocket-equation burn-time formula:
        #   t_burn = T_BUP * (1 - exp(-VG/Ve))
        # which is valid for any VG/Ve ratio.
        # ---------------------------------------------------------------
        Ve_2 = r.ISP_2 * c.G_0
        mdot_2 = r.F_THRUST_2 / Ve_2
        T_BUP_2_full = r.M_PROP_2 / mdot_2          # max stage-2 burn budget [s]
        coast_time = float(r.TIME_SECOND_ENGINE_IGNITION)

        if VG > 0.0 and Ve_2 > 0.0:
            stage2_tgo = T_BUP_2_full * (1.0 - np.exp(-VG / Ve_2))
        else:
            stage2_tgo = 0.0

        # Total remaining time: stage-1 remaining burn + coast + stage-2 insertion
        guidance_tgo = T_BUP + coast_time + stage2_tgo
        guidance_tgo = max(guidance_tgo, 0.1)

    else:
        # ---------------------------------------------------------------
        # STAGE 2: use exact rocket-equation burn time for remaining VG.
        # ---------------------------------------------------------------
        if VG > 0.0 and Ve > 0.0:
            guidance_tgo = T_BUP * (1.0 - np.exp(-VG / Ve))
        else:
            guidance_tgo = float(previous_tgo) if previous_tgo is not None else 0.0
        guidance_tgo = max(guidance_tgo, 0.1)
        guidance_tgo = min(guidance_tgo, T_BUP)  # can't exceed remaining prop time

    display_tgo = guidance_tgo
    return guidance_tgo, display_tgo


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
    # For a nearly circular orbit (e ≈ 0) the formula divides by e*r_current ≈ 0
    # producing NaN. A circular orbit has no preferred apogee; any point is
    # equivalent, so returning 0 (circularise immediately) is correct.
    if e < 1e-6:
        return 0.0

    theta = np.arccos((a * (1 - e**2) - r_current) / (e * r_current))
    ecc_anomaly = 2 * np.arctan2(np.sqrt((1 - e) / (1 + e)) * (1 - np.cos(theta)), 
                                   np.sin(theta))
    mean_anomaly = ecc_anomaly - e * np.sin(ecc_anomaly)
    time_until_apogee = T / (2 * np.pi) * mean_anomaly
    time_until_apogee = (T / 2.) - time_until_apogee

    # arccos maps theta to [0, π] (ascending arc only). When gamma < 0 the
    # rocket is on the descending arc and must coast through perigee before
    # reaching apogee. By orbital symmetry the correct time is T minus the
    # ascending-arc estimate.
    if gamma < 0:
        time_until_apogee = T - time_until_apogee

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
        [s, r, v, gamma, m], [s, r, v, gamma, m, lat], or
        [s, r, v, gamma, m, lat, heading]
        - s: downtrack [m]
        - r: radius from Earth's center [m]
        - v: velocity norm [m/s]
        - gamma: flight path angle [rad]
        - m: current mass [kg]
        - lat: current latitude [rad] (only when Earth rotation is enabled)
        - heading: current heading [rad] (optional when heading tracking is enabled)

    Returns:
    --------
    list : Derivatives of the state vector
    """
    global time_kick_start, kick_performed, main_engine_cutoff, flag_falling_single_burn
    global current_kick_angle
    global atmosphere_exited, guidance_phase_active, time_atmosphere_exit, time_guidance_start
    global last_guidance_update_time, guidance_initial_tgo, guidance_coefficients
    global apollo_coefficients_frozen, apollo_freeze_time, apollo_previous_tgo, lts_previous_tgo
    global cpr_theta_dot, cpr_t_start
    global peg_A, peg_B, peg_T, peg_t_epoch, peg_frozen
    global peg_new_vgo_r, peg_new_vgo_theta, peg_new_L0, peg_new_tgo
    global peg_new_t_lambda, peg_new_lambda_r, peg_new_t_epoch, peg_new_frozen
    global exp_shoot_a, exp_shoot_b, exp_shoot_epoch
    global thrust_history, time_history
    global alpha_history, alpha_time_history, theta_history, theta_time_history
    global tgo_history, tgo_time_history
    global cross_heading_counter_force_history, cross_heading_accel_history
    global LAUNCH_AZIMUTH, LAUNCH_LATITUDE_RAD

    # Get state components
    s, r_val, v, gamma, m = state[:5]
    lat = LAUNCH_LATITUDE_RAD
    heading = LAUNCH_AZIMUTH

    if sim_params.ENABLE_EARTH_ROTATION:
        if len(state) > 5:
            lat = state[5]
        else:
            lat = get_latitude_from_downrange(s)

        if sim_params.TRACK_HEADING_STATE and len(state) > 6:
            heading = state[6]

    # Compute altitude above Earth's surface
    alt = r_val - c.R_EARTH

    # Check main engine state and second engine state
    event_main_engine_cutoff(t, state)
    if main_engine_cutoff:
        was_ignited = second_engine_ignition
        event_second_engine_ignition(t)
        # Paper-mode: lock in coast/SECO schedule the instant Stage 2 ignites.
        if (sim_params.GUIDANCE_MODE == "pso_paper"
                and second_engine_ignition and not was_ignited
                and pso_paper_seco_t is None):
            _paper_setup_stage2_schedule(t)
            if sim_params.EVENTS_PRINT:
                print(f"[pso_paper] Stage-2 schedule: "
                      f"coast_start={pso_paper_coast_start_t:.1f}s, "
                      f"coast_end={pso_paper_coast_end_t:.1f}s, "
                      f"seco={pso_paper_seco_t:.1f}s")

    # --- Get current thrust, Isp ---
    F_T, Isp = thrust_Isp(t)

    # --- Calculate dynamic pressure (needed for atmosphere exit check and drag) ---
    q = atm.dynamic_pressure(v, alt)

    # --- Check atmosphere exit condition based on selected method ---
    atmosphere_exit_detected = False
    if sim_params.ATMOSPHERE_EXIT_METHOD == "altitude":
        atmosphere_exit_detected = (alt > sim_params.ALT_NO_ATMOSPHERE)
    elif sim_params.ATMOSPHERE_EXIT_METHOD == "dynamic_pressure":
        atmosphere_exit_detected = (q < sim_params.DYNAMIC_PRESSURE_THRESHOLD)
    elif sim_params.ATMOSPHERE_EXIT_METHOD == "aerothermal_flux":
        phi = atm.aerothermal_flux(v, alt)
        atmosphere_exit_detected = (phi < sim_params.AEROTHERMAL_FLUX_THRESHOLD)

    # --- Record atmosphere exit event (independent of guidance start) ---
    # Guard with kick_performed to avoid false trigger at t=0 when q=0 (v=0).
    # pso_paper extra guard: kick_performed is set at t≈7.5s when v≈40 m/s,
    # i.e. q≈980 Pa < threshold — a false trigger at low altitude.  Require
    # v > 200 m/s so the exit only registers when genuinely above the atmosphere.
    if kick_performed and atmosphere_exit_detected and not atmosphere_exited:
        _v_ok = (v > 200.0 if sim_params.GUIDANCE_MODE == "pso_paper" else True)
        if _v_ok:
            atmosphere_exited = True
            time_atmosphere_exit = t

    # --- Determine if guidance should start (mode-dependent) ---
    if sim_params.GUIDANCE_START_MODE == "after_kick":
        guidance_start_ready = kick_performed
    else:  # "after_atmosphere_exit" (default)
        guidance_start_ready = atmosphere_exit_detected

    # --- Get current angle of attack (GUIDANCE LOGIC) ---
    # Three-mode guidance system based on simulation_parameters.GUIDANCE_MODE
    
    if t >= sim_params.TIME_TO_START_KICK and (not kick_performed):
        if sim_params.GUIDANCE_MODE == "cpr":
            # CPR has no kick maneuver — vertical phase ends immediately.
            # Set time_kick_start to release the dgammadt=0 guard (line ~1285).
            kick_performed = True
            time_kick_start = t
            alpha = 0.0
        else:
            # Phase 1: Initial gravity turn (pitchover) - COMMON TO ALL OTHER MODES
            alpha = pitch_program_linear(t, current_kick_angle)

    elif (kick_performed and sim_params.GUIDANCE_MODE == "cpr"
          and not guidance_phase_active and F_T > 0):
        # CPR guidance initialisation
        guidance_phase_active = True
        cpr_t_start = t
        time_guidance_start = t
        cpr_theta_initial = np.pi / 2.0                            # 90° — vertical
        if sim_params.CPR_THETA_DOT_MODE == "manual":
            cpr_theta_dot = np.deg2rad(sim_params.CPR_THETA_DOT)   # user-defined [rad/s]
            t_go = cpr_theta_initial / cpr_theta_dot                # derived duration [s]
        else:  # "tgo"
            t_go, _ = _compute_apollo_tgo(state, F_T, Isp,
                                           sim_params.TARGET_ORBITAL_ALTITUDE, None)
            cpr_theta_dot = cpr_theta_initial / max(t_go, 0.1)     # rad/s
        alpha = cpr_guidance_module.cpr_alpha(t, cpr_t_start,
                                               cpr_theta_initial, cpr_theta_dot, gamma)
        if sim_params.EVENTS_PRINT:
            print(f"\nCPR guidance start at t = {t:.2f} s")
            print(f"  mode = {sim_params.CPR_THETA_DOT_MODE}")
            print(f"  duration = {t_go:.2f} s,  θ_dot = {np.rad2deg(cpr_theta_dot):.4f} deg/s")

    elif (kick_performed and sim_params.GUIDANCE_MODE in ["simple_poly", "linear_tangent", "bilinear_tangent", "apollo"] and
          guidance_start_ready and (not guidance_phase_active) and F_T > 0):
        # Initialize guidance (only if engines burning)
        guidance_phase_active = True
        time_guidance_start = t
        last_guidance_update_time = t
        
        # Initialize guidance coefficients based on mode
        if sim_params.LTS_TGO_METHOD == "propellant" and sim_params.GUIDANCE_MODE in ["linear_tangent", "bilinear_tangent"]:
            t_go, _ = _compute_apollo_tgo(state, F_T, Isp, sim_params.TARGET_ORBITAL_ALTITUDE, lts_previous_tgo)
            lts_previous_tgo = t_go
        else:
            t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
        guidance_initial_tgo = t_go   # Store for GUIDANCE_TGO_FIXED mode
        
        if sim_params.GUIDANCE_MODE == "simple_poly":
            # Simple polynomial: linear gamma transition
            guidance_coefficients = simple_poly_guidance.compute_polynomial_coefficients(state, 
                                                                 sim_params.TARGET_ORBITAL_ALTITUDE, 
                                                                 t_go)
            alpha = simple_poly_guidance.polynomial_guidance(t, t_go, state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nGuidance start at t = {t:.2f} s, alt = {alt/1000:.2f} km, q = {q:.2f} Pa")
                print(f"  Start mode: {sim_params.GUIDANCE_START_MODE}")
                print(f"  Switching to SIMPLE POLYNOMIAL guidance mode")
                print(f"  Initial t_go = {t_go:.2f} s")
        
        elif sim_params.GUIDANCE_MODE == "linear_tangent":
            # Linear tangent steering: tan(α + γ) varies linearly with time
            guidance_coefficients = lts_guidance.compute_lts_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            alpha = lts_guidance.linear_tangent_steering(t, t_go, state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nGuidance start at t = {t:.2f} s, alt = {alt/1000:.2f} km, q = {q:.2f} Pa")
                print(f"  Start mode: {sim_params.GUIDANCE_START_MODE}")
                print(f"  Switching to LINEAR TANGENT STEERING guidance mode")
                print(f"  Initial t_go = {t_go:.2f} s")
                print(f"  LTS coefficients: a={guidance_coefficients[0]:.6f}, b={guidance_coefficients[1]:.6f}")
                print(f"  Initial alpha command: {np.rad2deg(alpha):.2f} deg")
                
        elif sim_params.GUIDANCE_MODE == "bilinear_tangent":
            # Bilinear tangent steering: tan(α + γ) = ratio of two linear functions
            guidance_coefficients = bts_guidance.compute_bilinear_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            alpha = bts_guidance.bilinear_tangent_steering(t, t_go, state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nGuidance start at t = {t:.2f} s, alt = {alt/1000:.2f} km, q = {q:.2f} Pa")
                print(f"  Start mode: {sim_params.GUIDANCE_START_MODE}")
                print(f"  Switching to BILINEAR TANGENT STEERING guidance mode")
                print(f"  Initial t_go = {t_go:.2f} s")
                print(f"  BTS coefficients: c1={guidance_coefficients[0]:.6f}, c2={guidance_coefficients[1]:.6f}, c1'={guidance_coefficients[2]:.6f}, c2'={guidance_coefficients[3]:.6f}")
                print(f"  Initial alpha command: {np.rad2deg(alpha):.2f} deg")
                
        elif sim_params.GUIDANCE_MODE == "apollo":
            # Compute t_go using the selected method
            if sim_params.APOLLO_TGO_METHOD == "altitude":
                t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
                t_go_display = t_go
            else:  # "propellant" (default)
                t_go, t_go_display = _compute_apollo_tgo(state, F_T, Isp, sim_params.TARGET_ORBITAL_ALTITUDE, apollo_previous_tgo)
            apollo_previous_tgo = t_go
            # Apollo polynomial: acceleration profiles with terminal constraints
            # t_go already reflects the full remaining ascent timeline (stage 1 +
            # coast + stage 2), so the polynomial plans the complete trajectory.
            guidance_coefficients = apollo_guidance_module.compute_apollo_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go,
                                                               use_downrange_constraint=(sim_params.GUIDANCE_START_MODE == "after_atmosphere_exit"))
            apollo_freeze_time = t  # Initialize freeze time
            apollo_coefficients_frozen = False
            alpha, a_thrust_cmd = apollo_guidance_module.apollo_guidance(t, apollo_freeze_time, state, guidance_coefficients)
            
            if sim_params.EVENTS_PRINT:
                print(f"\nGuidance start at t = {t:.2f} s, alt = {alt/1000:.2f} km, q = {q:.2f} Pa")
                print(f"  Start mode: {sim_params.GUIDANCE_START_MODE}")
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

        # Update guidance coefficients periodically (skip if coefficients are fixed)
        if not sim_params.GUIDANCE_COEFFICIENTS_FIXED and \
                (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            guidance_coefficients = lts_guidance.compute_lts_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            last_guidance_update_time = t

        # Compute guidance angle
        if sim_params.GUIDANCE_TGO_FIXED:
            t_go = guidance_initial_tgo
        elif sim_params.LTS_TGO_METHOD == "propellant":
            t_go, _ = _compute_apollo_tgo(state, F_T, Isp, sim_params.TARGET_ORBITAL_ALTITUDE, lts_previous_tgo)
            lts_previous_tgo = t_go
        else:
            t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
        tgo_history.append(t_go)
        tgo_time_history.append(t)
        alpha = lts_guidance.linear_tangent_steering(t, t_go, state, guidance_coefficients)

    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "bilinear_tangent" and F_T > 0:
        # Phase 2c: Bilinear tangent steering guidance (only while engines burning)

        # Update guidance coefficients periodically (skip if coefficients are fixed)
        if not sim_params.GUIDANCE_COEFFICIENTS_FIXED and \
                (t - last_guidance_update_time) >= sim_params.GUIDANCE_UPDATE_RATE:
            t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            guidance_coefficients = bts_guidance.compute_bilinear_coefficients(state,
                                                               sim_params.TARGET_ORBITAL_ALTITUDE,
                                                               t_go)
            last_guidance_update_time = t

        # Compute guidance angle
        if sim_params.GUIDANCE_TGO_FIXED:
            t_go = guidance_initial_tgo
        elif sim_params.LTS_TGO_METHOD == "propellant":
            t_go, _ = _compute_apollo_tgo(state, F_T, Isp, sim_params.TARGET_ORBITAL_ALTITUDE, lts_previous_tgo)
            lts_previous_tgo = t_go
        else:
            t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
        tgo_history.append(t_go)
        tgo_time_history.append(t)
        alpha = bts_guidance.bilinear_tangent_steering(t, t_go, state, guidance_coefficients)
        
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "apollo" and F_T > 0:
        # Phase 2b: Apollo polynomial guidance (only while engines burning)

        if sim_params.APOLLO_TGO_METHOD == "altitude":
            t_go = estimate_time_to_target(state, sim_params.TARGET_ORBITAL_ALTITUDE)
            t_go_display = t_go
        else:  # "propellant" (default)
            t_go, t_go_display = _compute_apollo_tgo(state, F_T, Isp, sim_params.TARGET_ORBITAL_ALTITUDE, apollo_previous_tgo)
        apollo_previous_tgo = t_go

        # Check if we should freeze coefficients (t_go below threshold).
        # During stage 1, t_go now reflects the full remaining timeline
        # (coast + stage-2 burn), so it will not be small until near orbit
        # insertion — the freeze threshold is safe to apply in both stages.
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
                                                               t_go,
                                                               use_downrange_constraint=(sim_params.GUIDANCE_START_MODE == "after_atmosphere_exit"))
            apollo_freeze_time = t  # Update epoch time
            last_guidance_update_time = t
            # Record t_go once per guidance update cycle (not every ODE sub-step)
            tgo_history.append(t_go_display)
            tgo_time_history.append(t)
        
        # Compute guidance angle using current or frozen coefficients
        alpha, a_thrust_cmd = apollo_guidance_module.apollo_guidance(t, apollo_freeze_time, state, guidance_coefficients)
        
        # Apply thrust magnitude control if enabled.
        # The commanded thrust is capped at the nominal (maximum) available,
        # so stage 1 effectively always runs at full thrust when the guidance
        # commands more acceleration than the engine can provide.
        if sim_params.APOLLO_THRUST_MAGNITUDE_CONTROL:
            # Override thrust with commanded magnitude
            # Convert acceleration command to force
            F_T_commanded = m * a_thrust_cmd
            # Get the nominal (maximum) thrust available
            F_T_nominal, _ = thrust_Isp(t)
            # Use commanded thrust but limit to maximum available
            F_T = min(F_T_commanded, F_T_nominal)
        
    # --- PEG initialisation ---
    elif (kick_performed and sim_params.GUIDANCE_MODE == "peg"
          and guidance_start_ready and second_engine_ignition
          and not guidance_phase_active and F_T > 0):
        guidance_phase_active     = True
        time_guidance_start       = t
        last_guidance_update_time = t
        peg_t_epoch               = t
        peg_frozen                = False

        Ve    = r.ISP_2 * c.G_0
        r_tgt = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
        fairing_corr = r.M_FAIRING if not fairing_jettisoned else 0.0
        dry  = r.M_STRUCTURE_2 + r.M_PAYLOAD + fairing_corr
        T_seed = max(m - dry, 0.1) / (F_T / Ve)  # propellant-based seed
        _peg_damping = (sim_params.PEG_CONVERGENCE_DAMPING
                        if sim_params.PEG_CONVERGENCE_MODE == "damped" else 1.0)
        _peg_tol     = (sim_params.PEG_CONVERGENCE_TOL
                        if sim_params.PEG_CONVERGENCE_MODE == "damped" else 0.0)
        peg_A, peg_B, peg_T = peg_guidance_mod.converge_peg(
            state[:5], T_seed, Ve, F_T, r_tgt, c.MU_EARTH,
            max_iter=sim_params.PEG_CONVERGENCE_MAX_ITER,
            tol=_peg_tol, damping=_peg_damping)

        alpha = peg_guidance_mod.peg_alpha(0.0, peg_A, peg_B, gamma)

    # --- PEG per-step ---
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "peg" and F_T > 0:
        Ve    = r.ISP_2 * c.G_0
        r_tgt = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE

        if (not peg_frozen
                and (t - last_guidance_update_time) >= sim_params.PEG_MAJOR_LOOP_RATE):
            dt    = t - last_guidance_update_time
            peg_T = max(peg_T - dt, 0.1)

            if peg_T < sim_params.APOLLO_FREEZE_THRESHOLD:
                peg_frozen = True
                if sim_params.EVENTS_PRINT:
                    print(f"PEG coefficients frozen at t = {t:.1f} s, T = {peg_T:.1f} s")
            else:
                _peg_damping = (sim_params.PEG_CONVERGENCE_DAMPING
                                if sim_params.PEG_CONVERGENCE_MODE == "damped" else 1.0)
                _peg_tol     = (sim_params.PEG_CONVERGENCE_TOL
                                if sim_params.PEG_CONVERGENCE_MODE == "damped" else 0.0)
                peg_A, peg_B, peg_T = peg_guidance_mod.converge_peg(
                    state[:5], peg_T, Ve, F_T, r_tgt, c.MU_EARTH,
                    max_iter=sim_params.PEG_CONVERGENCE_MAX_ITER,
                    tol=_peg_tol, damping=_peg_damping)
                peg_t_epoch = t
            last_guidance_update_time = t

        t_since = t - peg_t_epoch
        alpha   = peg_guidance_mod.peg_alpha(t_since, peg_A, peg_B, gamma)

    # --- PEG_NEW initialisation ---
    elif (kick_performed and sim_params.GUIDANCE_MODE == "peg_new"
          and guidance_start_ready and second_engine_ignition
          and not guidance_phase_active and F_T > 0):
        guidance_phase_active     = True
        time_guidance_start       = t
        last_guidance_update_time = t
        peg_new_t_epoch           = t
        peg_new_frozen            = False

        Ve    = r.ISP_2 * c.G_0
        r_tgt = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
        (peg_new_vgo_r, peg_new_vgo_theta,
         peg_new_L0, peg_new_tgo,
         peg_new_t_lambda, peg_new_lambda_r) = peg_new_mod.peg_new_major_loop(
             state[:5], r_tgt, c.MU_EARTH, Ve, F_T)

        alpha = peg_new_mod.peg_new_alpha(
            0.0, peg_new_vgo_r, peg_new_vgo_theta,
            peg_new_L0, peg_new_lambda_r, peg_new_t_lambda, gamma)

    # --- PEG_NEW per-step ---
    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "peg_new" and F_T > 0:
        Ve    = r.ISP_2 * c.G_0
        r_tgt = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE

        if (not peg_new_frozen
                and (t - last_guidance_update_time) >= sim_params.PEG_MAJOR_LOOP_RATE):
            if peg_new_tgo is not None and peg_new_tgo < sim_params.APOLLO_FREEZE_THRESHOLD:
                peg_new_frozen = True
                if sim_params.EVENTS_PRINT:
                    print(f"PEG_new frozen at t = {t:.1f} s, tgo = {peg_new_tgo:.1f} s")
            else:
                (peg_new_vgo_r, peg_new_vgo_theta,
                 peg_new_L0, peg_new_tgo,
                 peg_new_t_lambda, peg_new_lambda_r) = peg_new_mod.peg_new_major_loop(
                     state[:5], r_tgt, c.MU_EARTH, Ve, F_T)
                peg_new_t_epoch = t
            last_guidance_update_time = t

        t_since = t - peg_new_t_epoch
        alpha   = peg_new_mod.peg_new_alpha(
            t_since, peg_new_vgo_r, peg_new_vgo_theta,
            peg_new_L0, peg_new_lambda_r, peg_new_t_lambda, gamma)

    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "cpr" and F_T > 0:
        # CPR per-step: command pitch angle ramp, derive alpha
        alpha = cpr_guidance_module.cpr_alpha(t, cpr_t_start,
                                               np.pi / 2.0, cpr_theta_dot, gamma)

    elif guidance_phase_active and sim_params.GUIDANCE_MODE == "exp_shooting" and F_T > 0:
        # Exponential pitch law: θ(t_rel) = a·exp(b·t_rel), α = θ − γ
        # Optimize (a, b) once at guidance start, then hold fixed.
        if exp_shoot_a is None:
            isp_active = Isp  # already computed by thrust_Isp(t) for current stage
            if second_engine_ignition:
                m_dry_active = r.M_STRUCTURE_2
            elif fairing_jettisoned:
                m_dry_active = r.M_STRUCTURE_1 - r.M_FAIRING
            else:
                m_dry_active = r.M_STRUCTURE_1
            r_tgt = c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE
            exp_shoot_a, exp_shoot_b = exp_shoot_mod.optimize_exp_pitch(
                state[:5], r_tgt, c.MU_EARTH, F_T, isp_active, m_dry_active, c.G_0
            )
            exp_shoot_epoch = t
            if sim_params.EVENTS_PRINT:
                print(f"[exp_shooting] initialized at t={t:.1f}s: a={exp_shoot_a:.4f} rad, b={exp_shoot_b:.6f} 1/s")
        alpha = exp_shoot_mod.exp_pitch_alpha(t - exp_shoot_epoch,
                                               exp_shoot_a, exp_shoot_b, gamma)

    elif (sim_params.GUIDANCE_MODE == "pso_paper" and kick_performed
          and second_engine_ignition and F_T > 0):
        # Paper-mode (Pontryagin + PSO): steering from live costates.
        # Gated on second_engine_ignition to match PEG/PEG_new precedent —
        # the indirect method (paper §4.1) governs only the free-flight phase
        # which coincides with Stage 2.  Costates remain frozen (dλ/dt = 0)
        # through Stage 1, MECO, and the inter-stage coast; they start
        # evolving per paper eq. (30) only from Stage 2 ignition onward.
        if not guidance_phase_active:
            guidance_phase_active = True
            time_guidance_start = t
            if sim_params.EVENTS_PRINT:
                print(f"[pso_paper] guidance activated at t={t:.2f}s, "
                      f"alt={alt/1000:.2f} km")
        cs_offset = _paper_costate_offset(len(state))
        if cs_offset is not None:
            lam_h_now = state[cs_offset]
            lam_V_now = state[cs_offset + 1]
            lam_g_now = state[cs_offset + 2]
            alpha = pso_paper_mod.steering_from_costates(lam_V_now, lam_g_now, v)
        else:
            # State not extended (shouldn't happen in paper mode) — gravity turn
            alpha = 0.0

    else:
        # Default: zero angle of attack (gravity turn mode or coasting)
        alpha = 0.

    # Store steering angle and pitch angle throughout the entire flight (for plotting)
    alpha_history.append(alpha)
    alpha_time_history.append(t)
    theta_history.append(alpha + gamma)
    theta_time_history.append(t)

    # --- Determine current accelerations and forces ---
    a_grav = grav.gravitational_acceleration(r_val)
    
    # Calculate drag (dynamic pressure already calculated earlier)
    F_D = atm.drag_force(q)
    
    # Lift force
    if sim_params.INCLUDE_LIFT:
        F_L = atm.lift_force(q)
    else:
        F_L = 0.0

    state_differentiated = diff_eom_base(s, r_val, v, gamma, m, F_L, F_D, F_T, 
                                         a_grav, alpha, Isp)

    delta_dheadingdt_pseudo = 0.0
    a_cross_heading_pseudo = 0.0
    coriolis_mag_val = 0.0
    centrifugal_mag_val = 0.0
    if (sim_params.ENABLE_EARTH_ROTATION and sim_params.INCLUDE_PSEUDO_FORCES
            and not PROPAGATING_IN_INERTIAL_FRAME
            and sim_params.GUIDANCE_MODE != "pso_paper"):
        delta_dvdt, delta_dgammadt, delta_dheadingdt_pseudo, a_cross_heading_pseudo, coriolis_mag_val, centrifugal_mag_val = earth_rot.rotating_frame_pseudoforce_rates(
            v,
            gamma,
            heading,
            lat,
            r_val,
        )
        state_differentiated[2] += delta_dvdt
        state_differentiated[3] += delta_dgammadt

    cross_heading_counter_force_history.append(m * abs(a_cross_heading_pseudo))
    cross_heading_accel_history.append(abs(a_cross_heading_pseudo))

    if sim_params.ENABLE_EARTH_ROTATION:
        dsdt = state_differentiated[0]
        dlatdt = get_latitude_rate_from_downrange(s, dsdt)
        state_differentiated.append(dlatdt)

        if sim_params.TRACK_HEADING_STATE:
            if PROPAGATING_IN_INERTIAL_FRAME:
                dheadingdt = 0.0
            else:
                dheadingdt = get_heading_rate_from_latitude(lat, dlatdt, heading)
                if sim_params.INCLUDE_CROSS_HEADING_PSEUDO_FORCE:
                    dheadingdt += delta_dheadingdt_pseudo
            state_differentiated.append(dheadingdt)

    # --- Paper-mode costate ODEs (Morgado et al. 2022 eq. 30) ---
    if sim_params.GUIDANCE_MODE == "pso_paper":
        cs_off = _paper_costate_offset(len(state))
        if cs_off is not None:
            if guidance_phase_active:
                lam_h_s = state[cs_off]
                lam_V_s = state[cs_off + 1]
                lam_g_s = state[cs_off + 2]
                dlam_h, dlam_V, dlam_g = pso_paper_mod.costate_derivatives(
                    r_val, v, gamma, alpha, lam_h_s, lam_V_s, lam_g_s,
                    F_T, m, c.MU_EARTH)
                # Record costate history (sampled at every solve_ivp evaluation)
                pso_paper_costate_history.append((lam_h_s, lam_V_s, lam_g_s))
                pso_paper_costate_time_history.append(t)
            else:
                # Pre-guidance: costates are frozen at their PSO initial values.
                dlam_h = dlam_V = dlam_g = 0.0
            state_differentiated.append(dlam_h)
            state_differentiated.append(dlam_V)
            state_differentiated.append(dlam_g)

    if time_kick_start == None:
        state_differentiated[3] = 0.0

    if state_differentiated[2] < 0:
        flag_falling_single_burn = True

    # Store thrust, pseudo-force and time for later retrieval
    thrust_history.append(F_T)
    coriolis_mag_history.append(coriolis_mag_val)
    centrifugal_mag_history.append(centrifugal_mag_val)
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
                       stage_2_flag, override_events=None):
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
    override_events : list or None
        If provided, use this event list instead of the default for the given stage flags.

    Returns:
    --------
    solution : OdeResult
        Solution object from scipy.solve_ivp
    """
    t_span = (init_time, init_time + time_stamp + 1)
    t_eval = np.arange(init_time, init_time + time_stamp + sim_params.TIME_STEP,
                       sim_params.TIME_STEP)

    if override_events is not None:
        interrupt_list = override_events

    elif stage_1_flag:
        interrupt_list = [interrupt_stage_separation, interrupt_ground_collision,
                         interrupt_velocity_exceeded]

    elif stage_2_flag:
        # Coasting single burn trajectory.  Paper-mode (Morgado et al. 2022)
        # neutralises interrupt_single_burn_traj and interrupt_horizontal_check
        # internally (they return a constant non-zero value), so the event list
        # layout stays identical and downstream t_events indexing is preserved.
        interrupt_list = [interrupt_radius_check, interrupt_stage_2_burnt,
                         interrupt_ground_collision, interrupt_single_burn_traj,
                         interrupt_horizontal_check]
    else:
        interrupt_list = [interrupt_ground_collision]
    
    for interrupt in interrupt_list:
        interrupt.terminal = True
        interrupt.direction = 0

    # Only fire SECO when apogee is crossing upward through the target (negative→positive).
    # This prevents spurious re-triggers caused by numerical fluctuations once the orbit
    # already exceeds the target apogee (direction=0 would fire on the downward crossing too).
    if stage_2_flag:
        interrupt_single_burn_traj.direction = 1
    
    return solve_ivp(rocket_dynamics, y0=state_init, t_span=t_span, t_eval=t_eval, 
                    max_step=1, events=interrupt_list, atol=1e-8)


def run(initial_kick_angle, azimuth_override=None):
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
    global last_guidance_update_time, guidance_initial_tgo, guidance_coefficients
    global apollo_coefficients_frozen, apollo_freeze_time, apollo_previous_tgo, lts_previous_tgo
    global cpr_theta_dot, cpr_t_start
    global thrust_history, time_history
    global alpha_history, alpha_time_history, theta_history, theta_time_history
    global tgo_history, tgo_time_history
    global coriolis_mag_history, centrifugal_mag_history
    global cross_heading_counter_force_history, cross_heading_accel_history
    global fairing_jettisoned, time_fairing_jettison
    global peg_A, peg_B, peg_T, peg_t_epoch, peg_frozen
    global peg_new_vgo_r, peg_new_vgo_theta, peg_new_L0, peg_new_tgo
    global peg_new_t_lambda, peg_new_lambda_r, peg_new_t_epoch, peg_new_frozen
    global exp_shoot_a, exp_shoot_b, exp_shoot_epoch
    global CRASH_DETECTED, CRASH_TIME
    global LAUNCH_AZIMUTH, LAUNCH_AZIMUTH_INERTIAL, LAUNCH_LATITUDE_RAD, LAUNCH_ROTATION_SPEED
    global AZIMUTH_MODE_USED
    global LAST_ACHIEVED_INCLINATION_DEG, LAST_INCLINATION_DRIFT_DEG
    global PROPAGATING_IN_INERTIAL_FRAME
    global _isp1_last_update_time, _isp1_current
    global _thrust1_last_update_time, _thrust1_current
    
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
    PROPAGATING_IN_INERTIAL_FRAME = False
    current_kick_angle = initial_kick_angle  # Store for use in dynamics

    LAUNCH_AZIMUTH = np.deg2rad(90.0)
    LAUNCH_AZIMUTH_INERTIAL = np.deg2rad(90.0)
    LAUNCH_LATITUDE_RAD = np.deg2rad(sim_params.LAUNCH_LATITUDE)
    LAUNCH_ROTATION_SPEED = 0.0
    AZIMUTH_MODE_USED = "corrected"

    LAST_ACHIEVED_INCLINATION_DEG = np.nan
    LAST_INCLINATION_DRIFT_DEG = np.nan

    if sim_params.ENABLE_EARTH_ROTATION:
        (LAUNCH_AZIMUTH,
         LAUNCH_AZIMUTH_INERTIAL,
         LAUNCH_ROTATION_SPEED) = earth_rot.select_launch_azimuth(
            sim_params.TARGET_ORBIT_INCLINATION,
            sim_params.LAUNCH_LATITUDE,
            sim_params.TARGET_ORBITAL_ALTITUDE,
        )
        if azimuth_override is not None:
            LAUNCH_AZIMUTH = azimuth_override
            LAUNCH_AZIMUTH_INERTIAL = azimuth_override
        AZIMUTH_MODE_USED = getattr(sim_params, "AZIMUTH_INCLINATION_MODE", "formula_compare")
    
    # Reset guidance phase variables
    atmosphere_exited = False
    guidance_phase_active = False
    time_atmosphere_exit = None
    time_guidance_start = None
    last_guidance_update_time = 0.0
    guidance_initial_tgo = None
    guidance_coefficients = [0.0, 0.0, 0.0, 0.0]
    apollo_coefficients_frozen = False
    apollo_freeze_time = None
    apollo_previous_tgo = None
    lts_previous_tgo = None
    cpr_theta_dot = None
    cpr_t_start = None

    # Reset fairing state
    fairing_jettisoned = False
    time_fairing_jettison = None

    # Reset PEG state
    peg_A = peg_B = 0.0
    peg_T = peg_t_epoch = None
    peg_frozen = False

    # Reset PEG_new state
    peg_new_vgo_r = peg_new_vgo_theta = 0.0
    peg_new_L0 = 1.0
    peg_new_tgo = peg_new_t_epoch = None
    peg_new_t_lambda = peg_new_lambda_r = 0.0
    peg_new_frozen = False

    # Reset exponential-shooting state
    exp_shoot_a = exp_shoot_b = exp_shoot_epoch = None

    # Reset paper-mode (Morgado et al. 2022) per-run state.
    # NOTE: pso_paper_lam0/_gamma_p/_dt_coast/_coast_pct/_burn_pct are NOT reset here —
    # they are injected by Simulation/pso_paper_solver.py before each ra.run() call.
    global pso_paper_coast_start_t, pso_paper_coast_end_t, pso_paper_seco_t
    global pso_paper_costate_history, pso_paper_costate_time_history
    pso_paper_coast_start_t = None
    pso_paper_coast_end_t   = None
    pso_paper_seco_t        = None
    pso_paper_costate_history       = []
    pso_paper_costate_time_history  = []

    # Reset thrust, pseudo-force, and time history
    thrust_history = []
    coriolis_mag_history = []
    centrifugal_mag_history = []
    cross_heading_counter_force_history = []
    cross_heading_accel_history = []
    time_history = []
    
    # Reset steering angle and pitch angle history
    alpha_history = []
    alpha_time_history = []
    theta_history = []
    theta_time_history = []

    # Reset Apollo t_go history
    tgo_history = []
    tgo_time_history = []

    # Reset crash detection flags
    CRASH_DETECTED = False
    CRASH_TIME = None

    # Reset stage-1 Isp linear-ramp state
    _isp1_last_update_time = 0.0
    _isp1_current = r.ISP_1_SL

    # Reset stage-1 thrust linear-ramp state
    _thrust1_last_update_time = 0.0
    _thrust1_current = r.F_THRUST_1_SL

    #===================================================
    # Simulation until stage separation
    #===================================================

    # Define initial state
    initial_mass = (r.M_STRUCTURE_1 + r.M_PROP_1 + r.M_STRUCTURE_2 + 
                   r.M_PROP_2 + r.M_PAYLOAD)
    initial_state_1 = [0., c.R_EARTH, 0., np.deg2rad(90.), initial_mass]
    if sim_params.ENABLE_EARTH_ROTATION:
        initial_state_1.append(LAUNCH_LATITUDE_RAD)
        if sim_params.TRACK_HEADING_STATE:
            initial_state_1.append(LAUNCH_AZIMUTH)
    # Paper-mode: extend state with 3 costate slots (lam_h, lam_V, lam_gamma).
    # Initial values come from the PSO design vector via module globals; the
    # default zero-vector is the safe fallback when paper-mode is enabled but
    # no PSO injection has happened yet (e.g. unit-test smoke run).
    if sim_params.GUIDANCE_MODE == "pso_paper":
        if pso_paper_lam0 is not None:
            initial_state_1.extend([float(pso_paper_lam0[0]),
                                    float(pso_paper_lam0[1]),
                                    float(pso_paper_lam0[2])])
        else:
            initial_state_1.extend([0.0, 0.0, 0.0])

    # Define time of simulation 1
    time_1 = 500.

    # === Paper-mode: vertical liftoff [0, t_pitch] then instantaneous pitch-over ===
    # Integrates t_pitch seconds of pure vertical flight (pitch_program_linear is
    # short-circuited → alpha = 0), then jumps gamma to gamma_p in the state
    # snapshot.  This replaces the simulator's 45 s triangular kick entirely.
    sol_1a_pre   = None
    init_time_1a = 0.0
    if sim_params.GUIDANCE_MODE == "pso_paper":
        t_pitch = float(sim_params.PSO_PAPER_T_PITCHOVER)
        sol_1a_pre = simulate_trajectory(
            0.0, t_pitch, initial_state_1, False, False,
            override_events=[interrupt_ground_collision,
                             interrupt_velocity_exceeded],
        )
        # Guard: crash during the 3-second vertical liftoff (extremely rare).
        if len(sol_1a_pre.t_events[0]) > 0:
            CRASH_DETECTED = True
            CRASH_TIME = sol_1a_pre.t_events[0][0]
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
            coriolis_mag_data = np.array(coriolis_mag_history)
            centrifugal_mag_data = np.array(centrifugal_mag_history)
            return (sol_1a_pre.t, sol_1a_pre.y, None, None, 9999999.0,
                    thrust_data, time_thrust, alpha_data, alpha_time_data,
                    coriolis_mag_data, centrifugal_mag_data)

        # Inject the pitch-over: gamma → gamma_p, all other state slots unchanged.
        initial_state_1 = sol_1a_pre.y[:, -1].copy().tolist()
        gamma_p_val = (float(pso_paper_gamma_p)
                       if pso_paper_gamma_p is not None else np.pi / 2.0)
        initial_state_1[3] = gamma_p_val
        # NOTE: do NOT set time_kick_start here. Leaving it as None freezes dγ/dt
        # in diff_eom (line ~1637 guard) until pitch_program_linear is called at
        # t = TIME_TO_START_KICK (~7.5 s). This avoids the 1/V singularity in the
        # gravity-turn rate at low velocity — at t = 3 s, V ≈ 15 m/s gives
        # dγ/dt ~ -10°/s for gamma_p = 1.3 rad, which over-pitches the rocket to
        # γ < 0 and triggers ground-collision crashes (1e20 PSO penalty).
        # Real rockets only enter their gravity turn after v > ~100 m/s anyway.
        init_time_1a = t_pitch
        if sim_params.EVENTS_PRINT:
            print(f"[pso_paper] instant pitch-over at t={t_pitch:.1f}s  "
                  f"gamma -> {np.rad2deg(gamma_p_val):.3f} deg  "
                  f"(|v|={initial_state_1[2]:.2f} m/s)")

    # === Stage 1A: ascent until fairing jettison OR stage separation ===
    # interrupt_stage_separation is included so Stage 1A never runs past MECO when
    # the atmosphere exit (and thus fairing jettison) happens after MECO — e.g. when
    # ATMOSPHERE_EXIT_METHOD = "aerothermal_flux" with a threshold crossed at ~137 km.
    sol_1a = simulate_trajectory(init_time_1a, time_1, initial_state_1, False, False,
        override_events=[interrupt_fairing_jettison, interrupt_ground_collision,
                         interrupt_velocity_exceeded, interrupt_stage_separation])
    # event indices: 0=fairing, 1=crash, 2=velocity, 3=stage_sep

    # Paper-mode: prepend the pre-pitchover arc so downstream code sees [0, end].
    if sol_1a_pre is not None:
        class _MergedSol1a:
            t        = np.concatenate([sol_1a_pre.t, sol_1a.t])
            y        = np.concatenate([sol_1a_pre.y, sol_1a.y], axis=1)
            t_events = sol_1a.t_events   # event indices unchanged
        sol_1a = _MergedSol1a()

    if len(sol_1a.t_events[1]) > 0:  # crash before any other event
        CRASH_DETECTED = True
        CRASH_TIME = sol_1a.t_events[1][0]
        thrust_data = np.array(thrust_history)
        time_thrust = np.array(time_history)
        alpha_data = np.array(alpha_history)
        alpha_time_data = np.array(alpha_time_history)
        coriolis_mag_data = np.array(coriolis_mag_history)
        centrifugal_mag_data = np.array(centrifugal_mag_history)
        return sol_1a.t, sol_1a.y, None, None, 9999999.0, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data

    if len(sol_1a.t_events[0]) > 0:
        # Fairing jettison fired before stage separation — normal case for
        # dynamic_pressure / altitude methods (atmosphere exit during Stage 1 burn).
        fairing_jettisoned = True
        time_fairing_jettison = sol_1a.t[-1]
        initial_state_1b = sol_1a.y[:, -1].copy()
        initial_state_1b[4] -= r.M_FAIRING
        if sim_params.EVENTS_PRINT:
            print(f"Fairing jettisoned at t = {time_fairing_jettison:.1f} s, "
                  f"alt = {(initial_state_1b[1] - c.R_EARTH)/1e3:.1f} km")

        # === Stage 1B: continue burn from fairing jettison until stage separation ===
        sol_1b = simulate_trajectory(time_fairing_jettison, time_1, initial_state_1b,
                                      True, False)

        if len(sol_1b.t_events[1]) > 0:  # crash after fairing jettison
            CRASH_DETECTED = True
            CRASH_TIME = sol_1b.t_events[1][0]
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
            coriolis_mag_data = np.array(coriolis_mag_history)
            centrifugal_mag_data = np.array(centrifugal_mag_history)
            t_combined = np.concatenate([sol_1a.t, sol_1b.t])
            y_combined = np.concatenate([sol_1a.y, sol_1b.y], axis=1)
            return t_combined, y_combined, None, None, 9999999.0, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data

        # Merge Stage 1A and 1B into a single result for downstream code
        class _Sol1:
            t = np.concatenate([sol_1a.t, sol_1b.t])
            y = np.concatenate([sol_1a.y, sol_1b.y], axis=1)
            t_events = sol_1b.t_events  # stage sep is t_events[0]; crash handled above

        sol_1 = _Sol1()

    else:
        # Stage separation fired before fairing jettison — atmosphere exit has not
        # occurred during Stage 1 (e.g. aerothermal flux threshold crossed in Stage 2).
        # Use Stage 1A directly as the complete Stage 1 result; the fairing mass is
        # already part of M_STRUCTURE_1 and will be dropped with it at staging.
        sol_1 = sol_1a

    #===================================================
    # Simulation after stage separation
    #===================================================
    
    # Define new initial state
    initial_state_2 = sol_1.y[:, -1]

    # Adjust mass -> perform stage separation.
    # Always drop only the actual Stage-1 structure (M_STRUCTURE_1 - M_FAIRING).
    # If the fairing was already jettisoned in Stage 1, it is not in the mass at this
    # point, so the result is correct. If the fairing is still on (atmosphere exit in
    # Stage 2), Stage 2 starts with M_FAIRING included and drops it later.
    initial_state_2[4] = initial_state_2[4] - (r.M_STRUCTURE_1 - r.M_FAIRING)
    
    # Define time of simulation 2
    init_time_2 = sol_1.t[-1]
    time_2 = 4000.
    
    if not fairing_jettisoned:
        # === Stage 2A: burn with fairing until jettison (atmosphere exit) or normal end ===
        # Fairing interrupt fires when atmosphere_exited becomes True in Stage 2.
        # Event indices: fairing=0, radius=1, burnt=2, crash=3, single=4, horiz=5
        sol_2a = simulate_trajectory(init_time_2, time_2, initial_state_2, False, False,
            override_events=[interrupt_fairing_jettison, interrupt_radius_check,
                             interrupt_stage_2_burnt, interrupt_ground_collision,
                             interrupt_single_burn_traj, interrupt_horizontal_check])

        if len(sol_2a.t_events[3]) > 0:  # crash in Stage 2A (index 3 = ground collision)
            CRASH_DETECTED = True
            CRASH_TIME = sol_2a.t_events[3][0]
            data = np.concatenate((sol_1.y, sol_2a.y), axis=1)
            time_steps_simulation = np.concatenate((sol_1.t, sol_2a.t))
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
            coriolis_mag_data = np.array(coriolis_mag_history)
            centrifugal_mag_data = np.array(centrifugal_mag_history)
            return time_steps_simulation, data, None, None, 9999999.0, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data

        if len(sol_2a.t_events[0]) > 0:  # fairing jettison fired in Stage 2A
            fairing_jettisoned = True
            time_fairing_jettison = sol_2a.t[-1]
            initial_state_2b = sol_2a.y[:, -1].copy()
            initial_state_2b[4] -= r.M_FAIRING
            if sim_params.EVENTS_PRINT:
                print(f"Fairing jettisoned in Stage 2 at t = {time_fairing_jettison:.1f} s, "
                      f"alt = {(initial_state_2b[1] - c.R_EARTH)/1e3:.1f} km")

            # === Stage 2B: continue burn without fairing until normal end ===
            sol_2b = simulate_trajectory(time_fairing_jettison, time_2,
                                          initial_state_2b, False, True)

            if len(sol_2b.t_events[2]) > 0:  # crash in Stage 2B (index 2 = ground collision)
                CRASH_DETECTED = True
                CRASH_TIME = sol_2b.t_events[2][0]
                t_combined = np.concatenate([sol_1.t, sol_2a.t, sol_2b.t])
                y_combined = np.concatenate([sol_1.y, sol_2a.y, sol_2b.y], axis=1)
                thrust_data = np.array(thrust_history)
                time_thrust = np.array(time_history)
                alpha_data = np.array(alpha_history)
                alpha_time_data = np.array(alpha_time_history)
                coriolis_mag_data = np.array(coriolis_mag_history)
                centrifugal_mag_data = np.array(centrifugal_mag_history)
                return t_combined, y_combined, None, None, 9999999.0, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data

            # Merge Stage 2A and 2B; downstream crash check at t_events[2] still works
            class _Sol2:
                t = np.concatenate([sol_2a.t, sol_2b.t])
                y = np.concatenate([sol_2a.y, sol_2b.y], axis=1)
                t_events = sol_2b.t_events

            sol_2 = _Sol2()

        else:
            # Orbit/burnout reached before atmosphere exit — fairing never dropped.
            # Crash already handled above; patch t_events for the downstream check.
            sol_2a.t_events = [[], [], [], [], []]
            sol_2 = sol_2a

    else:
        # === Normal Stage 2: fairing already dropped in Stage 1 ===
        sol_2 = simulate_trajectory(init_time_2, time_2, initial_state_2, False, True)

        if len(sol_2.t_events[2]) > 0:  # interrupt_ground_collision fired in stage 2
            CRASH_DETECTED = True
            CRASH_TIME = sol_2.t_events[2][0]
            data = np.concatenate((sol_1.y, sol_2.y), axis=1)
            time_steps_simulation = np.concatenate((sol_1.t, sol_2.t))
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
            coriolis_mag_data = np.array(coriolis_mag_history)
            centrifugal_mag_data = np.array(centrifugal_mag_history)
            return time_steps_simulation, data, None, None, 9999999.0, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data

    data = np.concatenate((sol_1.y, sol_2.y), axis=1)
    time_steps_simulation = np.concatenate((sol_1.t, sol_2.t))

    # --- Process results for coasting single burn ---
    r_stop = sol_2.y[1, -1]
    v_stop = sol_2.y[2, -1]
    gamma_stop = sol_2.y[3, -1]
    lat_stop = get_latitude_from_downrange(sol_2.y[0, -1]) if sim_params.ENABLE_EARTH_ROTATION else LAUNCH_LATITUDE_RAD
    heading_stop = LAUNCH_AZIMUTH
    if sim_params.ENABLE_EARTH_ROTATION and sim_params.TRACK_HEADING_STATE and sol_2.y.shape[0] > 6:
        heading_stop = sol_2.y[6, -1]

    # Calculate altitude to stop burning
    alt_stop = r_stop - c.R_EARTH

    if sim_params.ENABLE_EARTH_ROTATION:
        LAST_ACHIEVED_INCLINATION_DEG = earth_rot.achieved_inclination_from_local_state(
            v_stop,
            gamma_stop,
            heading_stop,
            lat_stop,
            r_stop,
        )
        LAST_INCLINATION_DRIFT_DEG = LAST_ACHIEVED_INCLINATION_DEG - sim_params.TARGET_ORBIT_INCLINATION

    # Convert from rotating frame to inertial frame for orbital mechanics.
    v_stop, gamma_stop = get_inertial_state_components(r_stop, v_stop, gamma_stop, lat_stop, heading_stop)
    
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
        fairing_in_mass = r.M_FAIRING if not fairing_jettisoned else 0.0
        m_propellant_left = sol_2.y[4, -1] - (r.M_STRUCTURE_2 + r.M_PAYLOAD + fairing_in_mass)
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
            # Penalise kick angles whose coast phase would crash before reaching
            # apogee: rocket heading toward perigee (gamma < 0) that is inside Earth.
            if gamma_stop < 0 and r_peri_stop < c.R_EARTH:
                m_propellant_total_used_2nd_stage = 9999999.0
            thrust_data = np.array(thrust_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
            coriolis_mag_data = np.array(coriolis_mag_history)
            centrifugal_mag_data = np.array(centrifugal_mag_history)
            return time_steps_simulation, data, alt_stop, delta_v, m_propellant_total_used_2nd_stage, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data
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

            # If Earth rotation is enabled, the ascent state is in rotating-frame
            # speed/flight-path-angle. Convert to inertial components before coast
            # propagation and circularization so post-SECO dynamics are consistent
            # with the orbital-element and delta-v calculations above.
            if sim_params.ENABLE_EARTH_ROTATION:
                lat_state_3 = get_latitude_from_downrange(initial_state_3[0])
                heading_state_3 = LAUNCH_AZIMUTH
                if sim_params.TRACK_HEADING_STATE and len(initial_state_3) > 6:
                    heading_state_3 = initial_state_3[6]
                v_eci_3, gamma_eci_3 = get_inertial_state_components(
                    initial_state_3[1],
                    initial_state_3[2],
                    initial_state_3[3],
                    lat_state_3,
                    heading_state_3,
                )
                initial_state_3[2] = v_eci_3
                initial_state_3[3] = gamma_eci_3
            
            init_time_3 = sol_2.t[-1]
            time_3 = get_time_until_apogee(e_stop, initial_state_3[3],
                                           initial_state_3[2], orbit_period_stop,
                                           a_stop, initial_state_3[1])

            print("\n[COAST DIAGNOSTICS]")
            print(f"  SECO time         : T+{init_time_3:.2f} s")
            print(f"  SECO altitude     : {(initial_state_3[1] - c.R_EARTH)/1000:.2f} km")
            print(f"  gamma_stop        : {np.rad2deg(initial_state_3[3]):.4f} deg")
            print(f"  e_stop            : {e_stop:.6f}")
            print(f"  r_peri_stop       : {(r_peri_stop - c.R_EARTH)/1000:.2f} km (above Earth surface)")
            print(f"  r_apo_stop        : {(r_apo_stop - c.R_EARTH)/1000:.2f} km")
            print(f"  orbit_period      : {orbit_period_stop:.2f} s ({orbit_period_stop/60:.2f} min)")
            print(f"  time_3 (coast)    : {time_3:.2f} s ({time_3/60:.2f} min)")
            print(f"  Expected T/2      : {orbit_period_stop/2:.2f} s")
            
            # The state has been converted to the inertial frame above.
            # Mark that we are now propagating in the inertial frame so
            # pseudo-forces (Coriolis / centrifugal) are automatically
            # skipped by rocket_dynamics().
            PROPAGATING_IN_INERTIAL_FRAME = True
            
            sol_3 = simulate_trajectory(init_time_3, time_3, initial_state_3,
                                       False, False)

            if len(sol_3.t_events[0]) > 0:  # interrupt_ground_collision in coast
                CRASH_DETECTED = True
                CRASH_TIME = sol_3.t_events[0][0]
                data = np.concatenate((sol_1.y, sol_2.y, sol_3.y), axis=1)
                time_steps_simulation = np.concatenate((sol_1.t, sol_2.t, sol_3.t))
                thrust_data = np.array(thrust_history)
                time_thrust = np.array(time_history)
                alpha_data = np.array(alpha_history)
                alpha_time_data = np.array(alpha_time_history)
                coriolis_mag_data = np.array(coriolis_mag_history)
                centrifugal_mag_data = np.array(centrifugal_mag_history)
                return time_steps_simulation, data, None, None, None, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data

            print(f"  Sol_3 ended at    : T+{sol_3.t[-1]:.2f} s  "
                  f"alt={( sol_3.y[1,-1] - c.R_EARTH)/1000:.2f} km  "
                  f"gamma={np.rad2deg(sol_3.y[3,-1]):.3f} deg")

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

            if len(sol_4.t_events[0]) > 0:  # interrupt_ground_collision post-circ
                CRASH_DETECTED = True
                CRASH_TIME = sol_4.t_events[0][0]
                data = np.concatenate((sol_1.y, sol_2.y, sol_3.y, sol_4.y), axis=1)
                time_steps_simulation = np.concatenate((sol_1.t, sol_2.t, sol_3.t, sol_4.t))
                thrust_data = np.array(thrust_history)
                time_thrust = np.array(time_history)
                alpha_data = np.array(alpha_history)
                alpha_time_data = np.array(alpha_time_history)
                coriolis_mag_data = np.array(coriolis_mag_history)
                centrifugal_mag_data = np.array(centrifugal_mag_history)
                return time_steps_simulation, data, None, None, None, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data

            # Collect data and time steps
            data = np.concatenate((sol_1.y, sol_2.y, sol_3.y, sol_4.y), axis=1)
            time_steps_simulation = np.concatenate((sol_1.t, sol_2.t, sol_3.t, 
                                                   sol_4.t))
    
            # Convert thrust history to numpy array for easier handling
            thrust_data = np.array(thrust_history)
            coriolis_mag_data = np.array(coriolis_mag_history)
            centrifugal_mag_data = np.array(centrifugal_mag_history)
            time_thrust = np.array(time_history)
            alpha_data = np.array(alpha_history)
            alpha_time_data = np.array(alpha_time_history)
    
            return time_steps_simulation, data, alt_stop, delta_v, m_propellant_total_used_2nd_stage, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data
    
    else:
        thrust_data = np.array(thrust_history)
        coriolis_mag_data = np.array(coriolis_mag_history)
        centrifugal_mag_data = np.array(centrifugal_mag_history)
        time_thrust = np.array(time_history)
        alpha_data = np.array(alpha_history)
        alpha_time_data = np.array(alpha_time_history)
        return time_steps_simulation, data, None, 9999999.0, 9999999.0, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data

