""" ===============================================
    MAIN SCRIPT - COASTING SINGLE BURN OPTIMIZATION
    
    This script executes the coasting single burn trajectory
    optimization to find the optimal gravity turn kick angle
    that minimizes propellant usage.
=============================================== """

import sys
from pathlib import Path

# Add current directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
import matplotlib.pyplot as plt
from Simulation import solver
from Simulation import rocket_ascent as ra
from Input_File import simulation_parameters as sim_params
from Auxiliary import constants as c
from Auxiliary import launch_azimuth
import Plots.plots as plots
import Plots.guidance_phase_plots as guidance_plots

def execute():
    """
    Main execution function for coasting single burn optimization.
    
    This function:
    1. Finds the optimal kick angle that minimizes propellant usage
    2. Runs a full simulation with the optimal parameters
    3. Prints the results including final orbital elements
    4. Returns the time history, state data, and optimal kick angle
    """
    
    print("="*60)
    print("COASTING SINGLE BURN TRAJECTORY OPTIMIZATION")
    print("="*60)
    
    # Display guidance mode
    guidance_mode_names = {
        "gravity_turn": "Pure Gravity Turn",
        "simple_poly": "Simplified Polynomial Guidance",
        "linear_tangent": "Linear Tangent Steering",
        "bilinear_tangent": "Bilinear Tangent Steering",
        "apollo": "Apollo Polynomial Guidance"
    }
    
    mode_name = guidance_mode_names.get(sim_params.GUIDANCE_MODE, "Unknown")
    print(f"Guidance Mode: {mode_name}")
    
    if sim_params.GUIDANCE_MODE == "gravity_turn":
        print("  - Traditional gravity turn throughout flight")
        print("  - Zero angle of attack after initial kick")
    elif sim_params.GUIDANCE_MODE == "simple_poly":
        print("  - Gravity turn until atmosphere exit (65 km)")
        print("  - Linear flight path angle transition to horizontal")
        print("  - Simple and stable")
    elif sim_params.GUIDANCE_MODE == "linear_tangent":
        print("  - Gravity turn until atmosphere exit (65 km)")
        print("  - Classical linear tangent steering law")
        print("  - tan(alpha + gamma) varies linearly with time-to-go")
    elif sim_params.GUIDANCE_MODE == "bilinear_tangent":
        print("  - Gravity turn until atmosphere exit (65 km)")
        print("  - Bilinear tangent steering law")
        print("  - tan(alpha + gamma) = ratio of two linear functions of t-to-go")
        print("  - Controls both value and derivative at boundaries")
    elif sim_params.GUIDANCE_MODE == "apollo":
        print("  - Gravity turn until atmosphere exit (65 km)")
        print("  - Apollo-style acceleration command profiles")
        print("  - Enforces position & velocity terminal constraints")
        print("  - Coefficient freezing at t_go < 10s for stability")
    
    print("="*60)

    # ── Launch azimuth computation ──────────────────────────────
    azimuth_data = launch_azimuth.compute_launch_azimuth(
        site=sim_params.LAUNCH_SITE,
        custom_lat_deg=sim_params.CUSTOM_LATITUDE_DEG,
        inclination_deg=sim_params.TARGET_INCLINATION_DEG,
        branch=sim_params.AZIMUTH_BRANCH,
        v_ref_mps=sim_params.AZIMUTH_REFERENCE_SPEED_MPS,
        target_altitude=sim_params.TARGET_ORBITAL_ALTITUDE,
    )
    # Store on sim_params module so downstream code can access if needed
    sim_params.LAUNCH_AZIMUTH_DATA = azimuth_data

    # Compute Earth rotation parameters (initial velocity & omega_eff for ECI)
    ra.set_earth_rotation_boost(azimuth_data)

    print("\n" + "="*60)
    print("LAUNCH AZIMUTH")
    print("="*60)
    print(f"  Launch site:               {sim_params.LAUNCH_SITE} "
          f"(lat {azimuth_data['lat_deg']:.3f} deg)")
    print(f"  Target inclination:        {azimuth_data['inclination_deg']:.2f} deg")
    print(f"  Azimuth branch:            {azimuth_data['branch']}")
    print(f"  Reference speed (v_ref):   {azimuth_data['v_ref']:.2f} m/s")
    print(f"  Site eastward speed (v_E): {azimuth_data['v_E']:.2f} m/s")
    print(f"  Inertial azimuth  (A_I):   {azimuth_data['A_I_deg']:.4f} deg")
    print(f"  Ground-rel heading (A_G):  {azimuth_data['A_G_deg']:.4f} deg")
    frame_label = "ECI" if sim_params.EARTH_ROTATION else "surface"
    print(f"  In-plane v_boost:          {ra.earth_rotation_boost:.2f} m/s"
          f"  (frame: {frame_label})")
    print("="*60)
    
    # Set to optimization mode
    ra.SINGLE_BURN_FULL_SIMULATION = False
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = None

    # Determine kick angle (either from optimization or pre-set optimal value)
    if sim_params.RUN_FAST:
        print("\n" + "="*60)
        print("FAST RUN MODE")
        print("="*60)
        kick_angle_optimal = sim_params.OPTIMAL_KICK_ANGLES.get(sim_params.GUIDANCE_MODE, sim_params.INITIAL_KICK_ANGLE)
        print(f"\nUsing pre-determined optimal kick angle: {np.rad2deg(kick_angle_optimal):.4f} degrees")
        print("(Skipping optimization)")
    else:
        # Find optimal kick angle through optimization
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
    time, data, alt_stopped, delta_v, m_propellant_total, thrust_data, time_thrust, alpha_data, alpha_time_data = ra.run(kick_angle_optimal)

    # Calculate final orbital elements
    # Post-circularisation state already includes the Earth rotation boost,
    # so no additional conversion is needed.
    r_final = data[1, -1]
    v_final = data[2, -1]
    gamma_final = data[3, -1]
    
    a, e, r_apo, r_peri, T = ra.get_orbital_elements(r_final, v_final, gamma_final)
    
    print("\n" + "="*60)
    print("MISSION EVENT TIMELINE")
    print("="*60)
    
    # Get event timestamps from the simulation
    print(f"\t* T+{0.0:.2f}s\t\t\tLiftoff")
    
    if ra.time_kick_start is not None:
        print(f"\t* T+{ra.time_kick_start:.2f}s\t\tKick maneuver start")
        kick_end_time = ra.time_kick_start + sim_params.DURATION_INITIAL_KICK
        print(f"\t* T+{kick_end_time:.2f}s\t\tKick maneuver end")
    
    if ra.time_atmosphere_exit is not None:
        print(f"\t* T+{ra.time_atmosphere_exit:.2f}s\t\tAtmosphere exit (65 km)")
        if sim_params.GUIDANCE_MODE != "gravity_turn":
            guidance_activation_msg = {
                "simple_poly": "Simple polynomial guidance",
                "linear_tangent": "Linear tangent steering",
                "bilinear_tangent": "Bilinear tangent steering",
                "apollo": "Apollo polynomial guidance"
            }.get(sim_params.GUIDANCE_MODE, "Guidance")
            print(f"\t* T+{ra.time_atmosphere_exit:.2f}s\t\t{guidance_activation_msg} activation")
    
    if ra.time_main_engine_cutoff is not None:
        print(f"\t* T+{ra.time_main_engine_cutoff:.2f}s\t\tStage 1 engine cutoff (MECO)")
        stage_sep_time = ra.time_main_engine_cutoff + 3.0  # TIME_First_STAGE_SEPARATION
        print(f"\t* T+{stage_sep_time:.2f}s\t\tStage separation")
        stage2_ignition_time = ra.time_main_engine_cutoff + 8.0  # TIME_SECOND_ENGINE_IGNITION
        print(f"\t* T+{stage2_ignition_time:.2f}s\t\tStage 2 ignition")
    
    if ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL is not None:
        print(f"\t* T+{ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL:.2f}s\t\tStage 2 cutoff (SECO)")
        print(f"\t* T+{ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL:.2f}s\t\tCoast phase to apogee begins")
        
        # Find orbit insertion time by detecting velocity discontinuity
        time_insertion = None
        velocity_full = data[2]
        for i in range(1, len(velocity_full)):
            if time[i] > ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL:
                velocity_jump = velocity_full[i] - velocity_full[i-1]
                time_diff = time[i] - time[i-1]
                if time_diff > 0:
                    accel = velocity_jump / time_diff
                    if accel > 100.0:  # Delta-v application shows high acceleration
                        time_insertion = time[i]
                        break
        
        if time_insertion is not None:
            print(f"\t* T+{time_insertion:.2f}s\t\t\tOrbit insertion (circularization burn)")
    
    # Final state at end of simulation
    print(f"\t* T+{time[-1]:.2f}s\t\t\tSimulation end (stable orbit)")

    
    print("\n" + "="*60)
    print("FINAL ORBITAL ELEMENTS")
    print("="*60)
    print(f"\t* Semi-major axis:\t\t\t{a/1000:.2f} km")
    print(f"\t* Eccentricity:\t\t\t\t{e:.6f}")
    print(f"\t* Apoapsis altitude:\t\t\t{((r_apo - c.R_EARTH)/1000):.2f} km")
    print(f"\t* Periapsis altitude:\t\t\t{((r_peri - c.R_EARTH)/1000):.2f} km")
    print(f"\t* Orbital period:\t\t\t{T/60:.2f} minutes")
    print(f"\t* Inclination (target):  \t\t{azimuth_data['inclination_deg']:.2f} deg")
    print(f"\t* Inertial azimuth (A_I):\t\t{azimuth_data['A_I_deg']:.4f} deg")
    
    print("\n" + "="*60)
    print("PROPELLANT USAGE")
    print("="*60)
    print(f"\t* Total propellant consumed:\t\t{m_propellant_total:.2f} kg")
    print(f"\t* Total delta-v:\t\t\t{delta_v:.2f} m/s")
    
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60 + "\n")
    
    # Plot the results
    print("Generating plots...")
    
    # Key parameters plot (always shown)
    guidance_plots.plot_key_parameters(time, data, thrust_data, time_thrust)
    
    # Ascent phase plot (launch to SECO + 100s)
    guidance_plots.plot_ascent_phase(time, data, thrust_data, time_thrust)
    
    # Full mission plots
    plots.single_run(time, data, kick_angle_optimal, thrust_data, time_thrust, alpha_data, alpha_time_data)
    plots.plot_trajectory_xy(data, time)
    
    # Generate detailed guidance phase plots
    if sim_params.GUIDANCE_MODE != "gravity_turn" and ra.time_atmosphere_exit is not None:
        print("\nGenerating detailed guidance phase analysis...")
        # guidance_plots.plot_guidance_phase(time, data, thrust_data, time_thrust)  # Commented out: Detailed analysis and rates/performances
        guidance_plots.plot_trajectory_to_seco(time, data)
    
    # Generate steering angle plot (shows entire flight profile)
    print("\nGenerating steering angle plot...")
    guidance_plots.plot_apollo_steering_angles(alpha_data, alpha_time_data, time, data)
    
    # Keep all plot windows open until user closes them
    print("\nAll plots generated. Close plot windows to exit.")
    plt.show()
    
    return time, data, kick_angle_optimal

if __name__ == "__main__":
    execute()
