""" ===============================================
    MAIN SCRIPT - COASTING SINGLE BURN OPTIMIZATION
    
    This script executes the coasting single burn trajectory
    optimization to find the optimal gravity turn kick angle
    that minimizes propellant usage.
=============================================== """

import numpy as np
import solver
import rocket_ascent as ra
import simulation_parameters as sim_params


def execute():
    """
    Main execution function for coasting single burn optimization.
    
    This function:
    1. Finds the optimal kick angle that minimizes propellant usage
    2. Runs a full simulation with the optimal parameters
    3. Prints the results
    """
    
    print("="*60)
    print("COASTING SINGLE BURN TRAJECTORY OPTIMIZATION")
    print("="*60)
    
    # Set to optimization mode
    ra.SINGLE_BURN_FULL_SIMULATION = False
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = None

    # Find optimal kick angle
    kick_angle_optimal = solver.find_initial_kick_angle_coast_single_burn()
    
    print("\n" + "="*60)
    print("OPTIMIZATION RESULTS")
    print("="*60)
    print(f"\nOptimal kick angle: {np.rad2deg(kick_angle_optimal):.4f} degrees")
    
    # Run full simulation with optimal parameters
    print("\n" + "="*60)
    print("RUNNING FULL TRAJECTORY SIMULATION")
    print("="*60 + "\n")
    
    ra.SINGLE_BURN_FULL_SIMULATION = True
    time, data, alt_stopped, delta_v, m_propellant_total = ra.run(kick_angle_optimal)

    # Calculate final orbital elements
    r_final = data[1, -1]
    v_final = data[2, -1]
    gamma_final = data[3, -1]
    
    a, e, r_apo, r_peri, T = ra.get_orbital_elements(r_final, v_final, gamma_final)
    
    print("\n" + "="*60)
    print("FINAL ORBITAL ELEMENTS")
    print("="*60)
    print(f"\t* Semi-major axis:\t\t\t{a/1000:.2f} km")
    print(f"\t* Eccentricity:\t\t\t\t{e:.6f}")
    print(f"\t* Apoapsis altitude:\t\t\t{(r_apo - sim_params.TARGET_ORBITAL_ALTITUDE)/1000:.2f} km")
    print(f"\t* Periapsis altitude:\t\t\t{(r_peri - sim_params.TARGET_ORBITAL_ALTITUDE)/1000:.2f} km")
    print(f"\t* Orbital period:\t\t\t{T/60:.2f} minutes")
    
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60 + "\n")
    
    return time, data, kick_angle_optimal


if __name__ == "__main__":
    execute()
