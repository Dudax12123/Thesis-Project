""" ===============================================
    TRAJECTORY OPTIMIZATION SOLVERS
    
    This module contains optimization algorithms for finding
    optimal gravity turn parameters for coasting single burn
    trajectories.
=============================================== """

import sys
from pathlib import Path

# Add parent directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from scipy.optimize import brute
import time
from Auxiliary import constants as c
from Input_File import simulation_parameters as sim_params
from Simulation import rocket_ascent as ra


#===================================================
# Coasting Single Burn Optimization
#===================================================

def coasting_single_burn_objective(kick_angle):
    """
    Objective function to find the initial kick angle for the gravity turn 
    which minimizes the used propellant in the second stage.
    
    Parameters:
    -----------
    kick_angle : float
        Initial kick angle [rad]
    
    Returns:
    --------
    m_propellant_total_used_2nd_stage : float
        Total mass of propellant used in the second stage [kg]
    """
    time_steps, data, alt_stopped, delta_v, m_propellant_total_used_2nd_stage, thrust_data, time_thrust, alpha_data, alpha_time_data = ra.run(kick_angle)

    # Debugging output
    print("Kick angle:\t\t", np.rad2deg(kick_angle))
    print("Propellant used:\t", m_propellant_total_used_2nd_stage, "kg")
    print("\n")

    return m_propellant_total_used_2nd_stage


def find_initial_kick_angle_coast_single_burn():
    """
    Finds the initial kick angle for the gravity turn using brute force optimization.
    
    Returns:
    --------
    alpha_optimal : float
        Optimal initial kick angle [rad]
    """
    bounds = [(sim_params.ALPHA_LOWEST, sim_params.ALPHA_HIGHEST)]

    print("\nFinding initial kick angle for coasting single burn using Brute Force...\n")

    # Time measurement
    start_time = time.time()

    # Brute force grid search
    result = brute(
        lambda x: abs(coasting_single_burn_objective(x[0])),
        ranges=bounds,
        Ns=1000,
        finish=None,
        full_output=True
    )
    alpha_optimal = result[0]

    # Time measurement
    end_time = time.time()
    print("-----------------------------------------------------\n")
    print(f"Optimization finished after {np.round(end_time - start_time, 2)} seconds.")

    return alpha_optimal


def find_initial_azimuth_for_inclination(initial_kick_angle):
    """
    Find an initial azimuth that reduces terminal inclination error.

    This is used for pseudo-3DOF (`coriolis_centrifugal`) mode and performs a
    bracketed bisection around corrected launch azimuth.
    """
    if not sim_params.ENABLE_AZIMUTH_ITERATION:
        beta_corrected, _, _ = ra.earth_rot.corrected_azimuth(
            sim_params.TARGET_ORBIT_INCLINATION,
            sim_params.LAUNCH_LATITUDE,
            sim_params.TARGET_ORBITAL_ALTITUDE,
        )
        return beta_corrected

    beta_corrected, _, _ = ra.earth_rot.corrected_azimuth(
        sim_params.TARGET_ORBIT_INCLINATION,
        sim_params.LAUNCH_LATITUDE,
        sim_params.TARGET_ORBITAL_ALTITUDE,
    )

    bracket = np.deg2rad(sim_params.AZIMUTH_BRACKET_DEG)
    low = beta_corrected - bracket
    high = beta_corrected + bracket
    tol = sim_params.AZIMUTH_ITERATION_TOL_DEG

    def inc_error(azimuth):
        _, data, *_ = ra.run(initial_kick_angle, initial_azimuth_override=azimuth)
        achieved = ra.achieved_inclination_deg(data)
        return achieved - sim_params.TARGET_ORBIT_INCLINATION, achieved

    err_low, inc_low = inc_error(low)
    err_high, inc_high = inc_error(high)

    if err_low * err_high > 0:
        # Fallback when bracket does not straddle the root.
        _, inc_mid = inc_error(beta_corrected)
        cands = [(abs(inc_low - sim_params.TARGET_ORBIT_INCLINATION), low),
                 (abs(inc_mid - sim_params.TARGET_ORBIT_INCLINATION), beta_corrected),
                 (abs(inc_high - sim_params.TARGET_ORBIT_INCLINATION), high)]
        return min(cands, key=lambda x: x[0])[1]

    for _ in range(sim_params.AZIMUTH_ITERATION_MAX_ITERS):
        mid = 0.5 * (low + high)
        err_mid, inc_mid = inc_error(mid)
        if abs(err_mid) <= tol:
            return mid

        if err_low * err_mid <= 0:
            high = mid
            err_high = err_mid
        else:
            low = mid
            err_low = err_mid

    return 0.5 * (low + high)


#===================================================
# Utility Functions for Orbital Mechanics
#===================================================

def hohman_transfer(v_initial, r_initial, r_final):
    """
    Calculate delta-v required for a Hohmann transfer.
    
    Parameters:
    -----------
    v_initial : float
        Initial velocity at starting orbit [m/s]
    r_initial : float
        Initial orbital radius [m]
    r_final : float
        Final orbital radius [m]
        
    Returns:
    --------
    delta_v_total : float
        Total delta-v required [m/s]
    delta_v1 : float
        First burn delta-v [m/s]
    delta_v2 : float
        Second burn delta-v [m/s]
    """
    # Transfer orbit semi-major axis
    a_transfer = (r_initial + r_final) / 2.0
    
    # Delta-v for first burn (periapsis)
    v_transfer_peri = np.sqrt(c.MU_EARTH * (2.0 / r_initial - 1.0 / a_transfer))
    delta_v1 = v_transfer_peri - v_initial
    
    # Velocity at apoapsis of transfer orbit
    v_transfer_apo = np.sqrt(c.MU_EARTH * (2.0 / r_final - 1.0 / a_transfer))
    
    # Circular velocity at final orbit
    v_final = np.sqrt(c.MU_EARTH / r_final)
    
    # Delta-v for second burn (apoapsis)
    delta_v2 = v_final - v_transfer_apo
    
    # Total delta-v
    delta_v_total = abs(delta_v1) + abs(delta_v2)
    
    return delta_v_total, delta_v1, delta_v2


def circularize_delta_v(r_val, v):
    """
    Computes the delta-v required to circularize an orbit at radius r with velocity v.

    Parameters:
    -----------
    r_val : float
        Orbital radius [m]
    v : float
        Current velocity [m/s]

    Returns:
    --------
    delta_v : float
        Delta-v required for circularization [m/s]
    """
    v_circular = np.sqrt(c.MU_EARTH / r_val)
    delta_v = abs(v_circular - v)
    
    return delta_v

