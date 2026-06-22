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
    time_steps, data, alt_stopped, delta_v, m_propellant_total_used_2nd_stage, thrust_data, time_thrust, alpha_data, alpha_time_data, _cor, _cent = ra.run(kick_angle)

    print("Kick angle:\t\t", np.rad2deg(kick_angle))
    if ra.CRASH_DETECTED:
        print(f"  [GROUND IMPACT at T+{ra.CRASH_TIME:.1f}s — propellant set to sentinel]")
    print("Propellant used:\t", m_propellant_total_used_2nd_stage, "kg")
    print("\n")

    return m_propellant_total_used_2nd_stage


def find_initial_kick_angle_coast_single_burn():
    """
    Finds the initial kick angle for the gravity turn using brute force optimization.

    When ``KICK_PROFILE_MODE == "instantaneous"``, the search is performed over
    gamma_p in [1.54, 1.57] rad (the pso_coast pitch-over convention) and the
    result is converted back to a kick angle (kick_angle = gamma_p - pi/2)
    before being returned, so callers (ra.run(kick_angle_optimal, ...)) need no
    changes.

    Returns:
    --------
    kick_angle_optimal : float
        Optimal initial kick angle [rad]
    """
    instantaneous = getattr(sim_params, 'KICK_PROFILE_MODE', 'triangular') == 'instantaneous'

    if instantaneous:
        bounds = [(1.54, 1.57)]   # gamma_p [rad]
        objective = lambda x: abs(coasting_single_burn_objective(x[0] - np.pi / 2.0))
    else:
        bounds = [(sim_params.ALPHA_LOWEST, sim_params.ALPHA_HIGHEST)]
        objective = lambda x: abs(coasting_single_burn_objective(x[0]))

    print("\nFinding initial kick angle for coasting single burn using Brute Force...\n")

    # Time measurement
    start_time = time.time()

    # Brute force grid search
    result = brute(
        objective,
        ranges=bounds,
        Ns=1000,
        finish=None,
        full_output=True
    )
    x_optimal = float(result[0])
    best_obj  = float(result[1])

    # When Stage-1 over-performs (commonly THRUST_1_MODE="vacuum"), the osculating
    # apogee never falls to the target for ANY kick, so every grid point returns
    # the apogee-match propellant sentinel (9.999e6). brute then hands back a
    # meaningless boundary kick and the full run circularizes mid-ascent into a
    # near-escape orbit. Fail fast instead. (Same infeasible-apogee-match class as
    # the apollo + apogee_check guard; real Stage-2 propellant is a few 1e3 kg, so
    # the 1e6 threshold cannot false-trigger.)
    if best_obj >= 1.0e6:
        raise ValueError(
            "apogee_check found no kick that reaches the target apogee — Stage-1 "
            "over-performs (commonly THRUST_1_MODE='vacuum'). Use COAST_METHOD="
            "'pso_coast' or 'direct', or set THRUST_1_MODE to 'sea_level'/'average'.")

    # Time measurement
    end_time = time.time()
    print("-----------------------------------------------------\n")
    print(f"Optimization finished after {np.round(end_time - start_time, 2)} seconds.")

    return (x_optimal - np.pi / 2.0) if instantaneous else x_optimal


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

