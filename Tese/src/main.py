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
from Auxiliary import earth_rotation as earth_rot
from Auxiliary import rocket_specs as r_specs
import Plots.new_plot_runner as new_plot_runner

# ---------------------------------------------------------------------------
# Back-pressure thrust loss lookup
# ---------------------------------------------------------------------------
# Digitised from the Ka (ft/s) vs Isp_SL/Isp_VAC chart.
# Source curve starts at (1.0, 0) and rises as the ratio decreases.
# Points are (Isp_SL/Isp_VAC, Ka [ft/s]).
_KA_RATIO = np.array([1.00, 0.95, 0.90, 0.85, 0.80, 0.75, 0.70, 0.65, 0.60, 0.55])
_KA_FT_S  = np.array([   0,  120,  285,  490,  730,  960, 1130, 1400, 1620, 1900])
_FT_S_TO_KM_S = 0.0003048   # 1 ft/s = 0.0003048 km/s

def _back_pressure_thrust_loss_kms(isp_sl: float, isp_vac: float) -> float:
    """
    Return the back-pressure thrust loss Ka [km/s] by interpolating the
    Ka vs (Isp_SL / Isp_VAC) reference chart.

    The curve represents the integrated velocity loss suffered by a rocket
    ascending through the atmosphere when its nozzle is not optimally expanded
    at every altitude — i.e. the difference between vacuum-Isp performance and
    the actual performance at ambient pressure.

    Parameters
    ----------
    isp_sl  : sea-level specific impulse [s]
    isp_vac : vacuum specific impulse [s]

    Returns
    -------
    Ka [km/s]
    """
    ratio = isp_sl / isp_vac
    ka_ft_s = float(np.interp(ratio, _KA_RATIO[::-1], _KA_FT_S[::-1]))
    return ka_ft_s * _FT_S_TO_KM_S

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

    if sim_params.ENABLE_EARTH_ROTATION:
        beta_corrected, beta_inertial, v_rot_surface = earth_rot.corrected_azimuth(
            sim_params.TARGET_ORBIT_INCLINATION,
            sim_params.LAUNCH_LATITUDE,
            sim_params.TARGET_ORBITAL_ALTITUDE,
        )
        active_beta = beta_corrected if sim_params.EARTH_ROTATION_AZIMUTH_MODE.lower().strip() == "corrected" else beta_inertial
        implied_inclination = earth_rot.orbit_inclination(sim_params.LAUNCH_LATITUDE, beta_inertial)
        expected_gain = earth_rot.delta_v_gain(
            sim_params.LAUNCH_LATITUDE,
            active_beta,
            c.R_EARTH + sim_params.TARGET_ORBITAL_ALTITUDE,
        )

        print("\n" + "="*60)
        print("EARTH ROTATION CONFIGURATION")
        print("="*60)
        print(f"Azimuth mode: {sim_params.EARTH_ROTATION_AZIMUTH_MODE}")
        print(f"Pseudo-forces in EOM: {sim_params.INCLUDE_PSEUDO_FORCES}")
        print(f"Heading state tracking: {sim_params.TRACK_HEADING_STATE}")
        print(f"Launch site latitude: {sim_params.LAUNCH_LATITUDE:.4f} deg")
        print(f"Launch site longitude: {sim_params.LAUNCH_LONGITUDE:.4f} deg")
        print(f"Target inclination: {sim_params.TARGET_ORBIT_INCLINATION:.4f} deg")
        print(f"Geometric inertial azimuth: {np.rad2deg(beta_inertial):.4f} deg")
        print(f"Corrected rotating-frame azimuth: {np.rad2deg(beta_corrected):.4f} deg")
        print(f"Active rotating-frame azimuth: {np.rad2deg(active_beta):.4f} deg")
        print(f"Surface rotation speed at launch site: {v_rot_surface:.2f} m/s")
        print(f"Estimated inertial delta-v gain: {expected_gain:.2f} m/s")
        print(f"Inclination implied by geometric azimuth/latitude: {implied_inclination:.4f} deg")
    
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
        # Suppress event prints during optimization to reduce noise
        _events_print_saved = sim_params.EVENTS_PRINT
        sim_params.EVENTS_PRINT = False
        
        # Find optimal kick angle through optimization
        kick_angle_optimal = solver.find_initial_kick_angle_coast_single_burn()
        
        # Restore event prints for the full simulation
        sim_params.EVENTS_PRINT = _events_print_saved
        
        print("\n" + "="*60)
        print("OPTIMIZATION RESULTS")
        print("="*60)
        print(f"\nOptimal kick angle: {np.rad2deg(kick_angle_optimal):.4f} degrees")
    
    # Run full simulation with optimal parameters
    print("\n" + "="*60)
    print("RUNNING FULL TRAJECTORY SIMULATION")
    print("="*60 + "\n")
    
    ra.SINGLE_BURN_FULL_SIMULATION = True
    time, data, alt_stopped, delta_v, m_propellant_total, thrust_data, time_thrust, alpha_data, alpha_time_data, coriolis_mag_data, centrifugal_mag_data = ra.run(kick_angle_optimal)

    # Check for failed simulation (sentinel value means apogee missed target or insufficient propellant)
    max_possible_propellant = r_specs.M_PROP_1 + r_specs.M_PROP_2
    if False and m_propellant_total > max_possible_propellant:
        print("\n" + "!"*60)
        print("SIMULATION FAILED")
        print("!"*60)
        print(f"Propellant metric returned: {m_propellant_total:.0f} kg (sentinel value)")
        print("The trajectory did not achieve the target orbit.")
        print("Possible causes:")
        print("  - Kick angle produces an apogee that misses the target altitude")
        print("  - Insufficient propellant for circularization burn")
        print("Skipping plots and orbital element display.")
        print("!"*60 + "\n")
        return time, data, kick_angle_optimal

    # Calculate final orbital elements
    r_final = data[1, -1]
    s_final = data[0, -1]
    v_final = data[2, -1]
    gamma_final = data[3, -1]
    lat_final = ra.get_latitude_from_downrange(s_final) if sim_params.ENABLE_EARTH_ROTATION else np.deg2rad(sim_params.LAUNCH_LATITUDE)
    heading_final = ra.LAUNCH_AZIMUTH
    if sim_params.ENABLE_EARTH_ROTATION and sim_params.TRACK_HEADING_STATE and data.shape[0] > 6:
        heading_final = data[6, -1]

    # In full simulation mode, post-SECO coast/circularization phases are already
    # propagated in inertial speed/FPA when Earth rotation is enabled.
    state_already_inertial = (
        sim_params.ENABLE_EARTH_ROTATION
        and ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL is not None
        and time[-1] > ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    )

    if not state_already_inertial:
        v_final, gamma_final = ra.get_inertial_state_components(r_final, v_final, gamma_final, lat_final, heading_final)
    
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
    if ra.time_guidance_start is not None and sim_params.GUIDANCE_MODE != "gravity_turn":
        guidance_activation_msg = {
            "simple_poly": "Simple polynomial guidance",
            "linear_tangent": "Linear tangent steering",
            "bilinear_tangent": "Bilinear tangent steering",
            "apollo": "Apollo polynomial guidance"
        }.get(sim_params.GUIDANCE_MODE, "Guidance")
        print(f"\t* T+{ra.time_guidance_start:.2f}s\t\t{guidance_activation_msg} activation")
    
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
    if sim_params.ENABLE_EARTH_ROTATION:
        print(f"\t* Azimuth mode used:\t\t\t{ra.AZIMUTH_MODE_USED}")
        if sim_params.TRACK_HEADING_STATE and data.shape[0] > 6:
            print(f"\t* Final tracked heading:\t\t{np.rad2deg(heading_final):.4f} deg")
        if np.isfinite(ra.LAST_ACHIEVED_INCLINATION_DEG):
            print(f"\t* Achieved inclination:\t\t{ra.LAST_ACHIEVED_INCLINATION_DEG:.4f} deg")
            if sim_params.PRINT_INCLINATION_DRIFT:
                print(f"\t* Inclination drift (achieved-target):\t{ra.LAST_INCLINATION_DRIFT_DEG:+.4f} deg")
        print(f"\t* Cross-heading pseudo-forces:\t\t{'ON' if sim_params.INCLUDE_CROSS_HEADING_PSEUDO_FORCE else 'OFF'}")
    
    print("\n" + "="*60)
    print("PROPELLANT USAGE")
    print("="*60)
    print(f"\t* Optimal kick angle:\t\t\t{np.rad2deg(kick_angle_optimal):.4f} degrees")
    print(f"\t* Total propellant consumed:\t\t{m_propellant_total:.2f} kg")
    print(f"\t* Total delta-v:\t\t\t{delta_v:.2f} m/s")

    # Back-pressure thrust loss
    ka_kms = _back_pressure_thrust_loss_kms(r_specs.ISP_1_SL, r_specs.ISP_1_VAC)
    isp_ratio = r_specs.ISP_1_SL / r_specs.ISP_1_VAC
    print(f"\n\t* Stage 1 Isp ratio (SL/VAC):\t\t{isp_ratio:.4f}")
    print(f"\t* Back-pressure thrust loss (Ka):\t{ka_kms:.4f} km/s  ({ka_kms*1000:.1f} m/s)")
    
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60 + "\n")
    
    # Plot the results
    print("Generating new plot suite...")

    _tgo_time = (np.array(ra.tgo_time_history)
                 if sim_params.GUIDANCE_MODE == "apollo" and len(ra.tgo_time_history) > 0
                 else None)
    _tgo = (np.array(ra.tgo_history)
            if sim_params.GUIDANCE_MODE == "apollo" and len(ra.tgo_history) > 0
            else None)
    _freeze_threshold = (getattr(sim_params, "APOLLO_FREEZE_THRESHOLD", None)
                         if sim_params.GUIDANCE_MODE == "apollo" else None)

    new_plot_runner.run_new_plot_suite(
        time,
        data,
        thrust_data,
        time_thrust,
        alpha_data,
        alpha_time_data,
        output_dir=None,
        show=True,
        close_after=False,
        coriolis_mag_data=coriolis_mag_data,
        centrifugal_mag_data=centrifugal_mag_data,
        tgo_time_data=_tgo_time,
        tgo_data=_tgo,
        apollo_freeze_threshold=_freeze_threshold,
    )

    # --- Heading comparison plot: with vs without cross-heading pseudo-force ---
    if (sim_params.ENABLE_EARTH_ROTATION and sim_params.TRACK_HEADING_STATE
            and data.shape[0] > 6):
        print("\nRunning heading comparison (cross-heading pseudo-force ON vs OFF)...")
        heading_comparison_plot(time, data, kick_angle_optimal,
                                ra.LAST_ACHIEVED_INCLINATION_DEG)

    # Keep all plot windows open until user closes them
    print("\nAll plots generated. Close plot windows to exit.")
    plt.show()
    
    return time, data, kick_angle_optimal


def heading_comparison_plot(time_ref, data_ref, kick_angle, inc_on):
    """
    Run a second simulation with cross-heading pseudo-force disabled and
    plot heading vs time for both cases on the same axes.
    """
    heading_on = np.rad2deg(data_ref[6, :])

    # Re-run with cross-heading pseudo-force disabled
    _saved_cross = sim_params.INCLUDE_CROSS_HEADING_PSEUDO_FORCE
    sim_params.INCLUDE_CROSS_HEADING_PSEUDO_FORCE = False
    ra.SINGLE_BURN_FULL_SIMULATION = True
    _, data_off, _, _, _, _, _, _, _, _, _ = ra.run(kick_angle)
    sim_params.INCLUDE_CROSS_HEADING_PSEUDO_FORCE = _saved_cross

    if data_off.shape[0] <= 6:
        print("  Heading state not available in comparison run — skipping plot.")
        return

    heading_off = np.rad2deg(data_off[6, :])
    n = min(len(time_ref), data_off.shape[1])

    # Truncate at SECO — heading is not propagated after ECI transition
    from Plots.plot_state_utils import event_times, cutoff_index
    idx = cutoff_index(time_ref[:n], event_times().get('seco'))

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(time_ref[:idx], heading_on[:idx],
            label="With cross-heading pseudo-force", linewidth=1.2)
    ax.plot(time_ref[:idx], heading_off[:idx],
            label="Without cross-heading pseudo-force", linewidth=1.2,
            linestyle="--")
    from Plots.plot_state_utils import add_event_markers
    add_event_markers(ax)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Heading [deg]")
    ax.set_title("Heading Evolution: Effect of Cross-Heading Pseudo-Force")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    # Inclination comparison
    lat_off = ra.get_latitude_from_downrange(data_off[0, -1])
    heading_off_final = data_off[6, -1]
    inc_off = earth_rot.achieved_inclination_from_local_state(
        data_off[2, -1], data_off[3, -1], heading_off_final,
        lat_off, data_off[1, -1])
    print(f"\n  Inclination WITH cross-heading pseudo-force:    {inc_on:.4f} deg")
    print(f"  Inclination WITHOUT cross-heading pseudo-force: {inc_off:.4f} deg")
    print(f"  Difference: {inc_on - inc_off:+.4f} deg")


if __name__ == "__main__":
    execute()
