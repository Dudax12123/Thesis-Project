""" ===============================================
    COMPARE GUIDANCE METHODS
    
    This script runs all guidance methods and creates
    comparison plots for propellant consumption and
    time to orbit insertion.
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


# Define guidance modes and their display names
GUIDANCE_MODES = {
    "gravity_turn": "Gravity Turn",
    "simple_poly": "Simple Polynomial",
    "linear_tangent": "Linear Tangent",
    "bilinear_tangent": "Bilinear Tangent",
    "apollo": "Apollo"
}


def run_guidance_method_for_comparison(guidance_mode):
    """
    Run trajectory optimization for a specific guidance method and extract metrics.
    
    Args:
        guidance_mode: The guidance mode to run
        
    Returns:
        dict: Dictionary containing propellant used and time to insertion
    """
    print(f"\nRunning {guidance_mode}...")
    
    # Set guidance mode
    sim_params.GUIDANCE_MODE = guidance_mode
    
    # Set to optimization mode
    ra.SINGLE_BURN_FULL_SIMULATION = False
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = None

    # Find optimal kick angle
    kick_angle_optimal = solver.find_initial_kick_angle_coast_single_burn()
    
    # Run full simulation with optimal parameters
    ra.SINGLE_BURN_FULL_SIMULATION = True
    time, data, alt_stopped, delta_v, m_propellant_total, thrust_data, time_thrust, alpha_data, alpha_time_data = ra.run(kick_angle_optimal)

    # Find orbit insertion time by detecting velocity discontinuity
    time_insertion = None
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    
    if time_seco is not None:
        velocity_full = data[2]
        for i in range(1, len(velocity_full)):
            if time[i] > time_seco:
                velocity_jump = velocity_full[i] - velocity_full[i-1]
                time_diff = time[i] - time[i-1]
                if time_diff > 0:
                    accel = velocity_jump / time_diff
                    if accel > 100.0:  # Delta-v application shows high acceleration
                        time_insertion = time[i]
                        break
    
    # If no insertion found, use final time
    if time_insertion is None:
        time_insertion = time[-1]
    
    results = {
        'propellant_kg': m_propellant_total,
        'time_to_insertion_s': time_insertion,
        'kick_angle_deg': np.rad2deg(kick_angle_optimal),
        'delta_v': delta_v
    }
    
    print(f"  Propellant: {m_propellant_total:.2f} kg")
    print(f"  Time to insertion: {time_insertion:.2f} s")
    print(f"  Optimal kick angle: {np.rad2deg(kick_angle_optimal):.4f}°")
    
    return results


def create_comparison_plots(results_dict, save_path):
    """
    Create comparison plots for all guidance methods.
    
    Args:
        results_dict: Dictionary with guidance method names as keys and results as values
        save_path: Path to save the comparison plot
    """
    # Extract data
    methods = list(results_dict.keys())
    propellants = [results_dict[m]['propellant_kg'] for m in methods]
    times = [results_dict[m]['time_to_insertion_s'] for m in methods]
    kick_angles = [results_dict[m]['kick_angle_deg'] for m in methods]
    delta_vs = [results_dict[m]['delta_v'] for m in methods]
    
    # Create figure with 2x2 subplots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Guidance Methods Comparison', fontsize=18, fontweight='bold', y=0.995)
    
    # Colors for each method
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    
    # Plot 1: Propellant Consumption
    bars1 = ax1.bar(methods, propellants, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax1.set_ylabel('Total Propellant Consumed [kg]', fontsize=12, fontweight='bold')
    ax1.set_title('Propellant Consumption', fontsize=14, fontweight='bold')
    ax1.grid(axis='y', alpha=0.3, linestyle='--')
    ax1.tick_params(axis='x', rotation=15)
    
    # Add value labels on bars
    for i, (bar, val) in enumerate(zip(bars1, propellants)):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{val:.1f} kg',
                ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Find best (minimum) propellant
    min_prop_idx = propellants.index(min(propellants))
    bars1[min_prop_idx].set_edgecolor('green')
    bars1[min_prop_idx].set_linewidth(3)
    
    # Plot 2: Time to Insertion
    bars2 = ax2.bar(methods, times, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax2.set_ylabel('Time to Orbit Insertion [s]', fontsize=12, fontweight='bold')
    ax2.set_title('Time to Orbit Insertion', fontsize=14, fontweight='bold')
    ax2.grid(axis='y', alpha=0.3, linestyle='--')
    ax2.tick_params(axis='x', rotation=15)
    
    # Add value labels on bars
    for i, (bar, val) in enumerate(zip(bars2, times)):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{val:.1f} s',
                ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Find best (minimum) time
    min_time_idx = times.index(min(times))
    bars2[min_time_idx].set_edgecolor('green')
    bars2[min_time_idx].set_linewidth(3)
    
    # Plot 3: Optimal Kick Angle
    bars3 = ax3.bar(methods, kick_angles, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax3.set_ylabel('Optimal Kick Angle [degrees]', fontsize=12, fontweight='bold')
    ax3.set_title('Optimal Kick Angle', fontsize=14, fontweight='bold')
    ax3.grid(axis='y', alpha=0.3, linestyle='--')
    ax3.tick_params(axis='x', rotation=15)
    ax3.axhline(y=0, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
    
    # Add value labels on bars
    for i, (bar, val) in enumerate(zip(bars3, kick_angles)):
        height = bar.get_height()
        y_pos = height if height > 0 else height
        va = 'bottom' if height > 0 else 'top'
        ax3.text(bar.get_x() + bar.get_width()/2., y_pos,
                f'{val:.3f}°',
                ha='center', va=va, fontsize=10, fontweight='bold')
    
    # Plot 4: Delta-V
    bars4 = ax4.bar(methods, delta_vs, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax4.set_ylabel('Circularization Delta-V [m/s]', fontsize=12, fontweight='bold')
    ax4.set_title('Circularization Delta-V Required', fontsize=14, fontweight='bold')
    ax4.grid(axis='y', alpha=0.3, linestyle='--')
    ax4.tick_params(axis='x', rotation=15)
    
    # Add value labels on bars
    for i, (bar, val) in enumerate(zip(bars4, delta_vs)):
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height,
                f'{val:.2f} m/s',
                ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Find best (minimum) delta-v
    min_dv_idx = delta_vs.index(min(delta_vs))
    bars4[min_dv_idx].set_edgecolor('green')
    bars4[min_dv_idx].set_linewidth(3)
    
    plt.tight_layout()
    
    # Save figure
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\nComparison plot saved to: {save_path}")
    
    # Also create a simple 2-plot version (propellant and time only)
    fig2, (ax_prop, ax_time) = plt.subplots(1, 2, figsize=(14, 6))
    fig2.suptitle('Guidance Methods Comparison: Key Metrics', fontsize=16, fontweight='bold')
    
    # Propellant
    bars_p = ax_prop.bar(methods, propellants, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax_prop.set_ylabel('Total Propellant Consumed [kg]', fontsize=12, fontweight='bold')
    ax_prop.set_xlabel('Guidance Method', fontsize=11, fontweight='bold')
    ax_prop.set_title('Propellant Consumption', fontsize=13, fontweight='bold')
    ax_prop.grid(axis='y', alpha=0.3, linestyle='--')
    ax_prop.tick_params(axis='x', rotation=15)
    bars_p[min_prop_idx].set_edgecolor('green')
    bars_p[min_prop_idx].set_linewidth(3)
    
    for bar, val in zip(bars_p, propellants):
        height = bar.get_height()
        ax_prop.text(bar.get_x() + bar.get_width()/2., height,
                    f'{val:.1f}\nkg',
                    ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    # Time
    bars_t = ax_time.bar(methods, times, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax_time.set_ylabel('Time to Orbit Insertion [s]', fontsize=12, fontweight='bold')
    ax_time.set_xlabel('Guidance Method', fontsize=11, fontweight='bold')
    ax_time.set_title('Time to Orbit Insertion', fontsize=13, fontweight='bold')
    ax_time.grid(axis='y', alpha=0.3, linestyle='--')
    ax_time.tick_params(axis='x', rotation=15)
    bars_t[min_time_idx].set_edgecolor('green')
    bars_t[min_time_idx].set_linewidth(3)
    
    for bar, val in zip(bars_t, times):
        height = bar.get_height()
        ax_time.text(bar.get_x() + bar.get_width()/2., height,
                    f'{val:.1f}\ns',
                    ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    plt.tight_layout()
    
    # Save simplified version
    save_path_simple = save_path.parent / "guidance_comparison_simple.png"
    plt.savefig(save_path_simple, dpi=300, bbox_inches='tight')
    print(f"Simplified comparison plot saved to: {save_path_simple}")
    
    plt.close('all')


def print_comparison_table(results_dict):
    """
    Print a formatted comparison table.
    
    Args:
        results_dict: Dictionary with guidance method names as keys and results as values
    """
    print("\n" + "="*90)
    print("GUIDANCE METHODS COMPARISON TABLE")
    print("="*90)
    print(f"{'Method':<20} {'Propellant [kg]':<18} {'Time [s]':<12} {'Kick Angle [°]':<18} {'Delta-V [m/s]':<15}")
    print("-"*90)
    
    for method, results in results_dict.items():
        display_name = GUIDANCE_MODES[method]
        print(f"{display_name:<20} {results['propellant_kg']:>15.2f}   "
              f"{results['time_to_insertion_s']:>9.2f}   "
              f"{results['kick_angle_deg']:>15.4f}   "
              f"{results['delta_v']:>12.2f}")
    
    print("="*90)
    
    # Find best methods
    methods = list(results_dict.keys())
    propellants = [results_dict[m]['propellant_kg'] for m in methods]
    times = [results_dict[m]['time_to_insertion_s'] for m in methods]
    
    min_prop_method = methods[propellants.index(min(propellants))]
    min_time_method = methods[times.index(min(times))]
    
    print(f"\nBest for Propellant Efficiency: {GUIDANCE_MODES[min_prop_method]} ({min(propellants):.2f} kg)")
    print(f"Best for Time Efficiency: {GUIDANCE_MODES[min_time_method]} ({min(times):.2f} s)")
    print("="*90 + "\n")


def main():
    """
    Main function to compare all guidance methods.
    """
    print("="*70)
    print("GUIDANCE METHODS COMPARISON")
    print("="*70)
    
    results = {}
    
    # Run each guidance method and collect results
    for idx, guidance_mode in enumerate(GUIDANCE_MODES.keys(), 1):
        print(f"\n[{idx}/{len(GUIDANCE_MODES)}] Processing: {guidance_mode}")
        
        try:
            results[guidance_mode] = run_guidance_method_for_comparison(guidance_mode)
        except Exception as e:
            print(f"\nERROR running {guidance_mode}: {str(e)}")
            import traceback
            traceback.print_exc()
            continue
    
    # Print comparison table
    print_comparison_table(results)
    
    # Create comparison plots
    base_path = Path(__file__).parent.parent.parent / "Images"
    save_path = base_path / "guidance_comparison.png"
    
    create_comparison_plots(results, save_path)
    
    print("\n" + "="*70)
    print("COMPARISON COMPLETE")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
