""" ===============================================
    RUN ALL GUIDANCE METHODS
    
    This script runs the trajectory optimization for all
    available guidance methods and saves the plots to their
    respective folders in the Images directory.
=============================================== """

import sys
from pathlib import Path

# Add current directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for saving figures
import matplotlib.pyplot as plt
from Simulation import solver
from Simulation import rocket_ascent as ra
from Input_File import simulation_parameters as sim_params
from Auxiliary import constants as c
import Plots.plots as plots
import Plots.guidance_phase_plots as guidance_plots


# Define guidance modes and their corresponding folder names
GUIDANCE_MODES = {
    "gravity_turn": "Gravity_Turn",
    "simple_poly": "Simple_Polynomial",
    "linear_tangent": "Linear_Tangent_Steering",
    "bilinear_tangent": "Bilinear_Tangent_Steering",
    "apollo": "Apollo_Guidance"
}


def run_guidance_method(guidance_mode, save_folder):
    """
    Run trajectory optimization for a specific guidance method and save plots.
    
    Args:
        guidance_mode: The guidance mode to run
        save_folder: Path to folder where plots should be saved
    """
    print("\n" + "="*70)
    print(f"RUNNING GUIDANCE METHOD: {guidance_mode.upper()}")
    print("="*70)
    
    # Set guidance mode (modify module attribute)
    sim_params.GUIDANCE_MODE = guidance_mode
    
    # Display guidance mode information
    guidance_mode_names = {
        "gravity_turn": "Pure Gravity Turn",
        "simple_poly": "Simplified Polynomial Guidance",
        "linear_tangent": "Linear Tangent Steering",
        "bilinear_tangent": "Bilinear Tangent Steering",
        "apollo": "Apollo Polynomial Guidance"
    }
    
    mode_name = guidance_mode_names.get(guidance_mode, "Unknown")
    print(f"Guidance Mode: {mode_name}")
    
    # Set to optimization mode
    ra.SINGLE_BURN_FULL_SIMULATION = False
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = None

    # Find optimal kick angle
    print("Finding optimal kick angle...")
    kick_angle_optimal = solver.find_initial_kick_angle_coast_single_burn()
    
    print(f"\nOptimal kick angle: {np.rad2deg(kick_angle_optimal):.4f} degrees")
    
    # Run full simulation with optimal parameters
    print("Running full trajectory simulation...")
    ra.SINGLE_BURN_FULL_SIMULATION = True
    time, data, alt_stopped, delta_v, m_propellant_total, thrust_data, time_thrust, alpha_data, alpha_time_data = ra.run(kick_angle_optimal)

    # Calculate final orbital elements
    r_final = data[1, -1]
    v_final = data[2, -1]
    gamma_final = data[3, -1]
    a, e, r_apo, r_peri, T = ra.get_orbital_elements(r_final, v_final, gamma_final)
    
    print(f"\nTotal propellant consumed: {m_propellant_total:.2f} kg")
    print(f"Total delta-v: {delta_v:.2f} m/s")
    
    # Create save folder if it doesn't exist
    save_path = Path(save_folder)
    save_path.mkdir(parents=True, exist_ok=True)
    
    print(f"\nGenerating and saving plots to: {save_folder}")
    
    # Generate and save all 6 plots
    
    # Plot 1: Key Parameters
    print("  - Saving Plot 1: Key Trajectory Parameters")
    guidance_plots.plot_key_parameters(time, data, thrust_data, time_thrust)
    plt.savefig(save_path / "01_key_parameters.png", dpi=300, bbox_inches="tight")
    plt.close('all')
    
    # Plot 2: Ascent Phase
    print("  - Saving Plot 2: Ascent Phase Analysis")
    guidance_plots.plot_ascent_phase(time, data, thrust_data, time_thrust)
    plt.savefig(save_path / "02_ascent_phase.png", dpi=300, bbox_inches="tight")
    plt.close('all')
    
    # Plot 3: Trajectory Losses (from single_run)
    print("  - Saving Plot 3: Trajectory Losses")
    plots.single_run(time, data, kick_angle_optimal, thrust_data, time_thrust)
    plt.savefig(save_path / "03_trajectory_losses.png", dpi=300, bbox_inches="tight")
    plt.close('all')
    
    # Plot 4: Trajectory XY
    print("  - Saving Plot 4: Trajectory X-Y Coordinates")
    plots.plot_trajectory_xy(data, time)
    plt.savefig(save_path / "04_trajectory_xy.png", dpi=300, bbox_inches="tight")
    plt.close('all')
    
    # Plot 5: Trajectory to SECO (for non-gravity-turn modes)
    if guidance_mode != "gravity_turn" and ra.time_atmosphere_exit is not None:
        print("  - Saving Plot 5: Trajectory to SECO")
        guidance_plots.plot_trajectory_to_seco(time, data)
        plt.savefig(save_path / "05_trajectory_to_seco.png", dpi=300, bbox_inches="tight")
        plt.close('all')
    else:
        print("  - Plot 5: Trajectory to SECO (skipped for gravity turn)")
    
    # Plot 6: Steering Angles
    print("  - Saving Plot 6: Steering Angles")
    guidance_plots.plot_apollo_steering_angles(alpha_data, alpha_time_data, time, data)
    plt.savefig(save_path / "06_steering_angles.png", dpi=300, bbox_inches="tight")
    plt.close('all')
    
    print(f"\nAll plots saved successfully to {save_folder}")
    print("="*70 + "\n")


def main():
    """
    Main function to run all guidance methods and save their plots.
    """
    # Get the base Images folder path (two levels up from src, then into Images)
    base_path = Path(__file__).parent.parent.parent / "Images"
    
    print("="*70)
    print("AUTOMATED GUIDANCE METHODS RUNNER")
    print("="*70)
    print(f"Base output folder: {base_path}")
    print(f"Number of guidance methods to run: {len(GUIDANCE_MODES)}")
    print("="*70)
    
    # Run each guidance method
    for idx, (guidance_mode, folder_name) in enumerate(GUIDANCE_MODES.items(), 1):
        save_folder = base_path / folder_name
        print(f"\n[{idx}/{len(GUIDANCE_MODES)}] Processing: {guidance_mode}")
        
        try:
            run_guidance_method(guidance_mode, save_folder)
        except Exception as e:
            print(f"\nERROR running {guidance_mode}: {str(e)}")
            import traceback
            traceback.print_exc()
            print(f"\nContinuing with next guidance method...\n")
            continue
    
    print("\n" + "="*70)
    print("ALL GUIDANCE METHODS COMPLETED")
    print("="*70)
    print("\nPlots have been saved to the following folders:")
    for guidance_mode, folder_name in GUIDANCE_MODES.items():
        folder_path = base_path / folder_name
        print(f"  - {guidance_mode}: {folder_path}")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
