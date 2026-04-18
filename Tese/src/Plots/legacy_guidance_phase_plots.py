""" ===============================================
    GUIDANCE PHASE DETAILED PLOTTING
    
    Plots focusing on the active guidance phase between
    atmosphere exit and SECO (coasting start).
=============================================== """

import matplotlib.pyplot as plt
import numpy as np
import sys
from pathlib import Path

# Add parent directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Input_File import simulation_parameters as sim_params
from Simulation import rocket_ascent as ra
from Auxiliary import rocket_specs as r


def _prepare_monotonic_series(time_array, value_array):
    """
    Sort by time and keep the last sample for duplicate timestamps.
    """
    t = np.asarray(time_array)
    v = np.asarray(value_array)

    if len(t) == 0:
        return t, v

    order = np.argsort(t, kind='stable')
    t_sorted = t[order]
    v_sorted = v[order]

    unique_t, first_idx = np.unique(t_sorted, return_index=True)
    next_idx = np.r_[first_idx[1:], len(t_sorted)]
    last_idx = next_idx - 1

    return unique_t, v_sorted[last_idx]


def plot_key_parameters(time_steps, data, thrust_data, time_thrust):
    """
    Creates a 4-panel plot showing key trajectory parameters over time.
    
    Plots:
        - Altitude vs Time
        - Thrust vs Time
        - Propellant Mass vs Time
        - Flight Path Angle vs Time
    
    Inputs:
        - time_steps: array of time steps [s]
        - data: array of data points with structure:
            * data[0]: downtrack s; [m]
            * data[1]: current radius r from Earth's center; [m]
            * data[2]: velocity norm; [m/s]
            * data[3]: flight path angle; [rad]
            * data[4]: mass of the rocket; [kg]
    """

    # Enforce monotonic samples before interpolation.
    time_thrust, thrust_data = _prepare_monotonic_series(time_thrust, thrust_data)
    
    # Get phase transition times
    time_guidance = ra.time_guidance_start
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    time_meco = ra.time_main_engine_cutoff
    
    # Reduce data for plotting
    reduction_factor = 10
    time_reduced = time_steps[::reduction_factor]
    data_reduced = data[:, ::reduction_factor]
    
    # Prepare data
    h = (data_reduced[1] - c.R_EARTH) / 1000.       # altitude h; [km]
    gamma = data_reduced[3]                          # flight path angle; [rad]
    m_total = data_reduced[4]                        # total mass; [kg]
    
    # Compute propellant mass (total mass minus structure and payload)
    m_prop = m_total - (r.M_STRUCTURE_1 + r.M_STRUCTURE_2 + r.M_PAYLOAD)
    
    # Interpolate actual thrust data to match reduced time steps
    thrust = np.interp(time_reduced, time_thrust, thrust_data) / 1000.  # Convert to kN
    
    # Create figure with single plot and multiple y-axes
    fig, ax1 = plt.subplots(figsize=(14, 8))
    fig.suptitle('Key Trajectory Parameters Over Time', fontsize=18, fontweight='bold')
    
    # First y-axis: Altitude (left side)
    color1 = 'tab:blue'
    ax1.set_xlabel('Time [s]', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Altitude [km]', color=color1, fontsize=14, fontweight='bold')
    line1 = ax1.plot(time_reduced, h, color=color1, linewidth=2.5, label='Altitude')
    ax1.tick_params(axis='y', labelcolor=color1, labelsize=12)
    ax1.tick_params(axis='x', labelsize=12)
    ax1.grid(True, alpha=0.3)
    
    # Second y-axis: Thrust (move to left side)
    ax2 = ax1.twinx()
    ax2.spines['left'].set_position(('outward', 60))
    ax2.spines['left'].set_visible(True)
    ax2.spines['right'].set_visible(False)
    ax2.yaxis.set_label_position('left')
    ax2.yaxis.set_ticks_position('left')
    color2 = 'tab:red'
    ax2.set_ylabel('Thrust [kN]', color=color2, fontsize=14, fontweight='bold')
    line2 = ax2.plot(time_reduced, thrust, color=color2, linewidth=2.5, label='Thrust')
    ax2.tick_params(axis='y', labelcolor=color2, labelsize=12)
    
    # Third y-axis: Propellant Mass (move to left side)
    ax3 = ax1.twinx()
    ax3.spines['left'].set_position(('outward', 120))
    ax3.spines['left'].set_visible(True)
    ax3.spines['right'].set_visible(False)
    ax3.yaxis.set_label_position('left')
    ax3.yaxis.set_ticks_position('left')
    color3 = 'tab:green'
    ax3.set_ylabel('Propellant Mass [kg]', color=color3, fontsize=14, fontweight='bold')
    line3 = ax3.plot(time_reduced, m_prop, color=color3, linewidth=2.5, label='Propellant Mass')
    ax3.tick_params(axis='y', labelcolor=color3, labelsize=12)
    
    # Fourth y-axis: Flight Path Angle (move to left side)
    ax4 = ax1.twinx()
    ax4.spines['left'].set_position(('outward', 180))
    ax4.spines['left'].set_visible(True)
    ax4.spines['right'].set_visible(False)
    ax4.yaxis.set_label_position('left')
    ax4.yaxis.set_ticks_position('left')
    color4 = 'tab:purple'
    ax4.set_ylabel('Flight Path Angle [deg]', color=color4, fontsize=14, fontweight='bold')
    line4 = ax4.plot(time_reduced, np.rad2deg(gamma), color=color4, linewidth=2.5, label='Flight Path Angle')
    ax4.tick_params(axis='y', labelcolor=color4, labelsize=12)
    ax4.axhline(y=0, color=color4, linestyle=':', linewidth=1, alpha=0.3)
    
    # Align zeros of all y-axes while maintaining good visibility
    h_min, h_max = h.min(), h.max()
    thrust_min, thrust_max = thrust.min(), thrust.max()
    m_prop_min, m_prop_max = m_prop.min(), m_prop.max()
    gamma_deg = np.rad2deg(gamma)
    gamma_min, gamma_max = gamma_deg.min(), gamma_deg.max()
    
    # Calculate target zero position (10% from bottom for nice padding)
    zero_fraction = 0.1
    
    def get_limits_with_zero_aligned(data_min, data_max, zero_frac):
        """Calculate y-limits so zero appears at zero_frac from bottom"""
        # Add some padding to data range
        data_range = data_max - data_min
        padded_min = data_min - 0.05 * data_range
        padded_max = data_max + 0.05 * data_range
        
        # Calculate required range to place zero at zero_frac position
        # zero_frac = (0 - ymin) / (ymax - ymin)
        # Rearranging: ymin = -zero_frac * (ymax - ymin)
        # We want: ymin <= padded_min and ymax >= padded_max
        
        if padded_min >= 0:
            # Data is all positive, extend downward to include zero
            ymax = padded_max
            ymin = -zero_frac / (1 - zero_frac) * ymax
        elif padded_max <= 0:
            # Data is all negative, extend upward to include zero  
            ymin = padded_min
            ymax = ymin * (1 - zero_frac) / (-zero_frac)
        else:
            # Data crosses zero - adjust to align zero at target fraction
            total_range_needed = max(padded_max / (1 - zero_frac), -padded_min / zero_frac)
            ymin = -zero_frac * total_range_needed
            ymax = (1 - zero_frac) * total_range_needed
        
        return ymin, ymax
    
    # Apply aligned limits to all axes
    ax1.set_ylim(get_limits_with_zero_aligned(h_min, h_max, zero_fraction))
    ax2.set_ylim(get_limits_with_zero_aligned(thrust_min, thrust_max, zero_fraction))
    ax3.set_ylim(get_limits_with_zero_aligned(m_prop_min, m_prop_max, zero_fraction))
    ax4.set_ylim(get_limits_with_zero_aligned(gamma_min, gamma_max, zero_fraction))
    
    # Add vertical lines for phase transitions
    if time_guidance is not None:
        ax1.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=2, alpha=1.0)
    if time_meco is not None:
        ax1.axvline(x=time_meco, color='orange', linestyle='--', linewidth=2, alpha=1.0)
    if time_seco is not None:
        ax1.axvline(x=time_seco, color='black', linestyle='--', linewidth=2, alpha=1.0)
    
    # Create combined legend
    lines = line1 + line2 + line3 + line4
    labels = [l.get_label() for l in lines]
    
    # Add phase markers to legend (numbered)
    from matplotlib.lines import Line2D
    legend_elements = lines.copy()
    legend_labels = labels.copy()
    
    if time_guidance is not None:
        legend_elements.append(Line2D([0], [0], color='cyan', linestyle='--', linewidth=2))
        legend_labels.append('Guidance Activation')
    if time_meco is not None:
        legend_elements.append(Line2D([0], [0], color='orange', linestyle='--', linewidth=2))
        legend_labels.append('MECO')
    if time_seco is not None:
        legend_elements.append(Line2D([0], [0], color='black', linestyle='--', linewidth=2))
        legend_labels.append('SECO')
    
    ax1.legend(legend_elements, legend_labels, loc='center right', fontsize=12, framealpha=0.9, draggable=True)
    
    plt.tight_layout()
    plt.show(block=False)



def plot_guidance_phase(time_steps, data, thrust_data, time_thrust):
    """
    Creates detailed plots for the active guidance phase only.
    
    Plots from atmosphere exit (guidance activation) to SECO (coasting start).
    
    Inputs:
        - time_steps: array of time steps (for the data array); [s]
        - data: array of data points with structure:
            * data[0]: downtrack s; [m]
            * data[1]: current radius r from Earth's center; [m]
            * data[2]: velocity norm; [m/s]
            * data[3]: flight path angle; [rad]
            * data[4]: mass of the rocket; [kg]
    """

    # Enforce monotonic samples before interpolation.
    time_thrust, thrust_data = _prepare_monotonic_series(time_thrust, thrust_data)
    
    # Get phase transition times
    time_guidance = ra.time_guidance_start
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    
    if time_guidance is None or time_seco is None:
        print("Cannot plot guidance phase: transition times not available")
        return
    
    # Find indices for the guidance phase
    idx_start = np.argmin(np.abs(time_steps - time_guidance))
    idx_end = np.argmin(np.abs(time_steps - time_seco))
    
    # Extract guidance phase data
    time_guidance_phase = time_steps[idx_start:idx_end+1]
    data_guidance_phase = data[:, idx_start:idx_end+1]
    
    # Reduce data for plotting (every 5th point for clarity)
    reduction_factor = 5
    time_reduced = time_guidance_phase[::reduction_factor]
    data_reduced = data_guidance_phase[:, ::reduction_factor]
    
    print("\n" + "="*60)
    print("GUIDANCE PHASE ANALYSIS")
    print("="*60)
    print(f"Guidance start time:\t\t{time_guidance:.2f} s")
    print(f"Guidance end time (SECO):\t{time_seco:.2f} s")
    print(f"Guidance duration:\t\t{time_seco - time_guidance:.2f} s")
    print(f"Data points in phase:\t\t{len(time_guidance_phase)}")
    print("="*60 + "\n")
    
    # -------------- Prepare data --------------
    h = (data_reduced[1] - c.R_EARTH) / 1000.       # altitude h; [km]
    s = data_reduced[0] / 1000.                     # downtrack s; [km]
    v = data_reduced[2]                              # velocity; [m/s]
    gamma = data_reduced[3]                          # flight path angle; [rad]
    m = data_reduced[4]                              # mass; [kg]
    
    # Compute derived quantities
    h_dot = v * np.sin(gamma)                        # vertical velocity; [m/s]
    s_dot = v * np.cos(gamma)                        # horizontal velocity; [m/s]
    
    # Compute dynamic pressure
    q = np.zeros(len(time_reduced))
    for i in range(len(time_reduced)):
        alt = h[i] * 1000
        rho = c.RHO_0 * np.exp(-alt / c.H)
        q[i] = 0.5 * rho * v[i]**2
    
    # Compute thrust acceleration from actual simulated thrust history.
    F_T = np.interp(time_reduced, time_thrust, thrust_data)
    thrust_accel = F_T / m
    
    # -------------- Plotting --------------
    fig1, axs1 = plt.subplots(3, 3, figsize=(18, 15))
    fig1.suptitle('Guidance Phase Detailed Analysis', fontsize=18, fontweight='bold')
    
    # Row 1: Position and Trajectory
    # Trajectory plot: altitude vs downtrack
    axs1[0, 0].plot(s, h, 'b-', linewidth=2)
    # Numbered markers
    axs1[0, 0].plot(s[0], h[0], 'o', color='green', markersize=8, 
                   markeredgecolor='black', markeredgewidth=1, zorder=5)
    axs1[0, 0].text(s[0], h[0], '1', color='white', fontsize=9, 
                   fontweight='bold', ha='center', va='center', zorder=6)
    axs1[0, 0].plot(s[-1], h[-1], 'o', color='red', markersize=8, 
                   markeredgecolor='black', markeredgewidth=1, zorder=5)
    axs1[0, 0].text(s[-1], h[-1], '2', color='white', fontsize=9, 
                   fontweight='bold', ha='center', va='center', zorder=6)
    axs1[0, 0].set_xlabel('Downtrack [km]')
    axs1[0, 0].set_ylabel('Altitude [km]')
    axs1[0, 0].set_title('Trajectory During Guidance')
    # Custom legend
    from matplotlib.lines import Line2D
    legend_elements = [Line2D([0], [0], color='b', linewidth=2, label='Trajectory'),
                      Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
                            markersize=6, label='① Guidance Start'),
                      Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                            markersize=6, label='② SECO')]
    axs1[0, 0].legend(handles=legend_elements)
    axs1[0, 0].grid(True, alpha=0.3)
    
    # Altitude over time
    axs1[0, 1].plot(time_reduced, h, 'b-', linewidth=2)
    axs1[0, 1].set_xlabel('Time [s]')
    axs1[0, 1].set_ylabel('Altitude [km]')
    axs1[0, 1].set_title('Altitude Evolution')
    axs1[0, 1].grid(True, alpha=0.3)
    
    # Downtrack over time
    axs1[0, 2].plot(time_reduced, s, 'b-', linewidth=2)
    axs1[0, 2].set_xlabel('Time [s]')
    axs1[0, 2].set_ylabel('Downtrack [km]')
    axs1[0, 2].set_title('Downtrack Evolution')
    axs1[0, 2].grid(True, alpha=0.3)
    
    # Row 2: Velocity Components
    # Velocity magnitude
    axs1[1, 0].plot(time_reduced, v, 'b-', linewidth=2, label='Total Velocity')
    axs1[1, 0].set_xlabel('Time [s]')
    axs1[1, 0].set_ylabel('Velocity [m/s]')
    axs1[1, 0].set_title('Velocity Magnitude')
    axs1[1, 0].legend()
    axs1[1, 0].grid(True, alpha=0.3)
    
    # Velocity components
    axs1[1, 1].plot(time_reduced, h_dot, 'r-', linewidth=2, label='Vertical (ḣ)')
    axs1[1, 1].plot(time_reduced, s_dot, 'g-', linewidth=2, label='Horizontal (ṡ)')
    axs1[1, 1].set_xlabel('Time [s]')
    axs1[1, 1].set_ylabel('Velocity Component [m/s]')
    axs1[1, 1].set_title('Velocity Components')
    axs1[1, 1].legend()
    axs1[1, 1].grid(True, alpha=0.3)
    
    # Flight path angle
    axs1[1, 2].plot(time_reduced, np.rad2deg(gamma), 'b-', linewidth=2)
    axs1[1, 2].axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Horizontal (target)')
    axs1[1, 2].set_xlabel('Time [s]')
    axs1[1, 2].set_ylabel('Flight Path Angle [deg]')
    axs1[1, 2].set_title('Flight Path Angle (γ)')
    axs1[1, 2].legend()
    axs1[1, 2].grid(True, alpha=0.3)
    
    # Row 3: Dynamics
    # Dynamic pressure
    axs1[2, 0].plot(time_reduced, q, 'b-', linewidth=2)
    axs1[2, 0].set_xlabel('Time [s]')
    axs1[2, 0].set_ylabel('Dynamic Pressure [Pa]')
    axs1[2, 0].set_title('Dynamic Pressure')
    axs1[2, 0].grid(True, alpha=0.3)
    
    # Mass
    axs1[2, 1].plot(time_reduced, m, 'b-', linewidth=2)
    axs1[2, 1].set_xlabel('Time [s]')
    axs1[2, 1].set_ylabel('Mass [kg]')
    axs1[2, 1].set_title('Vehicle Mass')
    axs1[2, 1].grid(True, alpha=0.3)
    
    # Thrust acceleration
    axs1[2, 2].plot(time_reduced, thrust_accel, 'b-', linewidth=2)
    axs1[2, 2].set_xlabel('Time [s]')
    axs1[2, 2].set_ylabel('Thrust Acceleration [m/s²]')
    axs1[2, 2].set_title('Thrust-to-Weight Acceleration')
    axs1[2, 2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show(block=False)
    
    # -------------- Second Figure: Rates and Performance --------------
    fig2, axs2 = plt.subplots(2, 2, figsize=(14, 10))
    fig2.suptitle('Guidance Phase Rates and Performance', fontsize=18, fontweight='bold')
    
    # Compute rates (using finite differences)
    dt = np.diff(time_reduced)
    dh_dt = np.diff(h) / (dt / 1000.)  # Convert to m/s (h is in km)
    ds_dt = np.diff(s) / (dt / 1000.)  # Convert to m/s
    dv_dt = np.diff(v) / dt
    dgamma_dt = np.diff(gamma) / dt
    
    time_diff = time_reduced[:-1]  # Time array for derivatives
    
    # Altitude rate
    axs2[0, 0].plot(time_diff, dh_dt, 'b-', linewidth=2)
    axs2[0, 0].set_xlabel('Time [s]')
    axs2[0, 0].set_ylabel('Altitude Rate [m/s]')
    axs2[0, 0].set_title('Climb Rate (dh/dt)')
    axs2[0, 0].grid(True, alpha=0.3)
    
    # Downtrack rate
    axs2[0, 1].plot(time_diff, ds_dt, 'b-', linewidth=2)
    axs2[0, 1].set_xlabel('Time [s]')
    axs2[0, 1].set_ylabel('Downtrack Rate [m/s]')
    axs2[0, 1].set_title('Horizontal Velocity (ds/dt)')
    axs2[0, 1].grid(True, alpha=0.3)
    
    # Acceleration
    axs2[1, 0].plot(time_diff, dv_dt, 'b-', linewidth=2)
    axs2[1, 0].set_xlabel('Time [s]')
    axs2[1, 0].set_ylabel('Acceleration [m/s²]')
    axs2[1, 0].set_title('Velocity Rate of Change (dv/dt)')
    axs2[1, 0].grid(True, alpha=0.3)
    
    # Flight path angle rate
    axs2[1, 1].plot(time_diff, np.rad2deg(dgamma_dt), 'b-', linewidth=2)
    axs2[1, 1].set_xlabel('Time [s]')
    axs2[1, 1].set_ylabel('Angle Rate [deg/s]')
    axs2[1, 1].set_title('Flight Path Angle Rate (dγ/dt)')
    axs2[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show(block=False)
    
    # -------------- Print Summary Statistics --------------
    print("GUIDANCE PHASE STATISTICS")
    print("="*60)
    print("\nAltitude:")
    print(f"\t* Initial:\t\t{h[0]:.2f} km")
    print(f"\t* Final:\t\t{h[-1]:.2f} km")
    print(f"\t* Gain:\t\t\t{h[-1] - h[0]:.2f} km")
    
    print("\nVelocity:")
    print(f"\t* Initial:\t\t{v[0]:.2f} m/s")
    print(f"\t* Final:\t\t{v[-1]:.2f} m/s")
    print(f"\t* Gain:\t\t\t{v[-1] - v[0]:.2f} m/s")
    
    print("\nFlight Path Angle:")
    print(f"\t* Initial:\t\t{np.rad2deg(gamma[0]):.2f} deg")
    print(f"\t* Final:\t\t{np.rad2deg(gamma[-1]):.2f} deg")
    print(f"\t* Change:\t\t{np.rad2deg(gamma[-1] - gamma[0]):.2f} deg")
    
    print("\nMass:")
    print(f"\t* Initial:\t\t{m[0]:.2f} kg")
    print(f"\t* Final:\t\t{m[-1]:.2f} kg")
    print(f"\t* Propellant used:\t{m[0] - m[-1]:.2f} kg")
    
    print("\nDowntrack:")
    print(f"\t* Distance covered:\t{s[-1] - s[0]:.2f} km")
    
    print("="*60 + "\n")


def plot_trajectory_to_seco(time_steps, data):
    """
    Plots the rocket trajectory from launch to SECO with Earth shown as a blue disk.
    The guidance phase is highlighted in a different color.
    
    Inputs:
        - time_steps: array of time steps [s]
        - data: array of data points with structure:
            * data[0]: downtrack s; [m]
            * data[1]: current radius r from Earth's center; [m]
    """
    
    # Get phase transition times
    time_guidance = ra.time_guidance_start
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    
    if time_seco is None:
        print("Cannot plot trajectory to SECO: SECO time not available")
        return
    
    # Find index for SECO
    idx_seco = np.argmin(np.abs(time_steps - time_seco))
    
    # Extract data up to SECO
    data_to_seco = data[:, :idx_seco+1]
    
    # Reduce data for plotting
    reduction_factor = 10
    data_reduced = data_to_seco[:, ::reduction_factor]
    time_reduced = time_steps[:idx_seco+1:reduction_factor]
    
    # Prepare data
    h = (data_reduced[1] - c.R_EARTH)       # altitude h; [m]
    s = data_reduced[0]                     # downtrack s; [m]
    
    # Convert to cartesian coordinates
    x, y = ra.cartesian_coordinates(h, s)
    x = x / 1000.
    y = y / 1000.
    
    # Find guidance phase indices in reduced data
    if time_guidance is not None:
        idx_guidance_reduced = np.argmin(np.abs(time_reduced - time_guidance))
    else:
        idx_guidance_reduced = 0
    
    # Plot setup
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_facecolor("white")
    
    # Plot trajectory in two segments with different colors
    # Segment 1: Launch to atmosphere exit (gravity turn phase) - white
    ax.plot(x[:idx_guidance_reduced+1], y[:idx_guidance_reduced+1], 
           color="black", linewidth=2, label="Initial Gravity Turn", zorder=3)
    
    # Segment 2: Atmosphere exit to SECO (guidance phase) - cyan
    if time_guidance is not None:
        ax.plot(x[idx_guidance_reduced:], y[idx_guidance_reduced:], 
               color="blue", linewidth=2.5, label="Active Guidance Phase", zorder=4)
    
    # Add numbered markers
    ax.plot(x[0], y[0], 'o', color='green', markersize=12, 
           markeredgecolor='white', markeredgewidth=1, zorder=5)
    ax.text(x[0], y[0], '1', color='white', fontsize=10, 
           fontweight='bold', ha='center', va='center', zorder=6)
    
    if time_guidance is not None:
        ax.plot(x[idx_guidance_reduced], y[idx_guidance_reduced], 'o', color='yellow', 
               markersize=12, markeredgecolor='white', markeredgewidth=1, zorder=5)
        ax.text(x[idx_guidance_reduced], y[idx_guidance_reduced], '2', color='black', fontsize=10, 
               fontweight='bold', ha='center', va='center', zorder=6)
    
    ax.plot(x[-1], y[-1], 'o', color='red', markersize=12, 
           markeredgecolor='white', markeredgewidth=1, zorder=5)
    ax.text(x[-1], y[-1], '3', color='white', fontsize=10, 
           fontweight='bold', ha='center', va='center', zorder=6)
    
    # Create custom legend with numbered markers
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='black', linewidth=2.5, label='Launcher Trajectory'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
              markersize=7, label='① Launch'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='yellow', 
              markersize=7, label='② Guidance Activation')
    ]
    if time_guidance is not None:
        legend_elements.insert(1, Line2D([0], [0], color='blue', linewidth=2.5, label='Active Guidance Phase'))
    legend_elements.append(Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                                 markersize=7, label='③ SECO (Coasting Start)'))
    

    # Create Earth representation (circular disk)
    earth_radius_km = c.R_EARTH / 1000.0
    earth = plt.Circle((0, 0), earth_radius_km, color='blue', alpha=0.5, zorder=1)
    
    # Show Earth
    ax.add_patch(earth)
    
    # Labels and aesthetics
    ax.set_xlabel("Downtrack Distance (km)", color="white", fontsize=18)
    ax.set_ylabel("Altitude (km)", color="white", fontsize=18)
    ax.set_title("Powered Ascent Trajectory (Launch to SECO)", color="black", fontsize=20, fontweight='bold')
    ax.tick_params(colors='white', labelsize=12)
    #ax.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.3)

    # Add legend with styling for black background
    legend = ax.legend(handles=legend_elements, loc='upper right', fontsize=12, 
                      facecolor='black', edgecolor='white', framealpha=0.8)
    for text in legend.get_texts():
        text.set_color('white')

    # Set limits to show the trajectory clearly
    margin = 500
    ax.set_xlim(min(x) - margin, max(x) + margin)
    ax.set_ylim(min(y) - margin, max(y) + margin)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    plt.show(block=False)


def plot_ascent_phase(time_steps, data, thrust_data, time_thrust):
    """
    Creates a single plot with 4 overlaid y-axes showing key trajectory parameters 
    from launch to 100s after SECO.
    
    Parameters overlaid with different colors:
        - Altitude (blue, left y-axis)
        - Thrust (red, right y-axis 1)
        - Total Mass (green, right y-axis 2, offset)
        - Flight Path Angle (purple, right y-axis 3, offset more)
    
    Time range: T+0 to SECO + 100 seconds
    
    Inputs:
        - time_steps: array of time steps [s]
        - data: array of data points
        - thrust_data: array of thrust values [N]
        - time_thrust: array of time steps for thrust [s]
    """
    
    # Get SECO time
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    if time_seco is None:
        print("Warning: SECO time not available, cannot create ascent phase plot")
        return
    
    # Define time limit: SECO + 100 seconds
    time_limit = time_seco + 100.0
    
    # Filter data to time limit
    mask = time_steps <= time_limit
    time_filtered = time_steps[mask]
    data_filtered = data[:, mask]
    
    # Filter thrust data
    thrust_mask = time_thrust <= time_limit
    time_thrust_filtered = time_thrust[thrust_mask]
    thrust_filtered = thrust_data[thrust_mask]
    
    # Reduce data for plotting
    reduction_factor = 5
    time_reduced = time_filtered[::reduction_factor]
    data_reduced = data_filtered[:, ::reduction_factor]
    
    # Prepare data
    h = (data_reduced[1] - c.R_EARTH) / 1000.       # altitude; [km]
    gamma = data_reduced[3]                          # flight path angle; [rad]
    m_total = data_reduced[4]                        # total mass; [kg]
    
    # Interpolate thrust to match reduced time steps
    thrust = np.interp(time_reduced, time_thrust_filtered, thrust_filtered) / 1000.  # kN
    
    # Get phase transition times
    time_guidance = ra.time_guidance_start
    time_meco = ra.time_main_engine_cutoff
    
    # Create figure with single plot
    fig, ax1 = plt.subplots(figsize=(12, 7))
    fig.suptitle(f'Ascent Phase: Key Parameters (Launch to SECO + 100s)', fontsize=18, fontweight='bold')
    
    # Plot 1: Altitude (left y-axis)
    color1 = 'tab:blue'
    ax1.set_xlabel('Time [s]', fontsize=14)
    ax1.set_ylabel('Altitude [km]', color=color1, fontsize=14)
    line1 = ax1.plot(time_reduced, h, color=color1, linewidth=2, label='Altitude')
    ax1.tick_params(axis='y', labelcolor=color1, labelsize=12)
    ax1.tick_params(axis='x', labelsize=12)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Thrust (move to left side)
    ax2 = ax1.twinx()
    ax2.spines['left'].set_position(('outward', 60))
    ax2.spines['left'].set_visible(True)
    ax2.spines['right'].set_visible(False)
    ax2.yaxis.set_label_position('left')
    ax2.yaxis.set_ticks_position('left')
    color2 = 'tab:red'
    ax2.set_ylabel('Thrust [kN]', color=color2, fontsize=14)
    line2 = ax2.plot(time_reduced, thrust, color=color2, linewidth=2, label='Thrust')
    ax2.tick_params(axis='y', labelcolor=color2, labelsize=12)
    
    # Plot 3: Total Mass (move to left side)
    ax3 = ax1.twinx()
    ax3.spines['left'].set_position(('outward', 120))
    ax3.spines['left'].set_visible(True)
    ax3.spines['right'].set_visible(False)
    ax3.yaxis.set_label_position('left')
    ax3.yaxis.set_ticks_position('left')
    color3 = 'tab:green'
    ax3.set_ylabel('Total Mass [kg]', color=color3, fontsize=14)
    line3 = ax3.plot(time_reduced, m_total, color=color3, linewidth=2, label='Total Mass')
    ax3.tick_params(axis='y', labelcolor=color3, labelsize=12)
    
    # Plot 4: Flight Path Angle (move to left side)
    ax4 = ax1.twinx()
    ax4.spines['left'].set_position(('outward', 180))
    ax4.spines['left'].set_visible(True)
    ax4.spines['right'].set_visible(False)
    ax4.yaxis.set_label_position('left')
    ax4.yaxis.set_ticks_position('left')
    color4 = 'tab:purple'
    ax4.set_ylabel('Flight Path Angle [deg]', color=color4, fontsize=14)
    line4 = ax4.plot(time_reduced, np.rad2deg(gamma), color=color4, linewidth=2, label='Flight Path Angle')
    ax4.tick_params(axis='y', labelcolor=color4, labelsize=12)
    ax4.axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.3)
    
    # Align zeros of all y-axes while maintaining good visibility
    h_min, h_max = h.min(), h.max()
    thrust_min, thrust_max = thrust.min(), thrust.max()
    m_total_min, m_total_max = m_total.min(), m_total.max()
    gamma_deg = np.rad2deg(gamma)
    gamma_min, gamma_max = gamma_deg.min(), gamma_deg.max()
    
    # Calculate target zero position (10% from bottom for nice padding)
    zero_fraction = 0.1
    
    def get_limits_with_zero_aligned(data_min, data_max, zero_frac):
        """Calculate y-limits so zero appears at zero_frac from bottom"""
        # Add some padding to data range
        data_range = data_max - data_min
        padded_min = data_min - 0.05 * data_range
        padded_max = data_max + 0.05 * data_range
        
        # Calculate required range to place zero at zero_frac position
        # zero_frac = (0 - ymin) / (ymax - ymin)
        # Rearranging: ymin = -zero_frac * (ymax - ymin)
        # We want: ymin <= padded_min and ymax >= padded_max
        
        if padded_min >= 0:
            # Data is all positive, extend downward to include zero
            ymax = padded_max
            ymin = -zero_frac / (1 - zero_frac) * ymax
        elif padded_max <= 0:
            # Data is all negative, extend upward to include zero  
            ymin = padded_min
            ymax = ymin * (1 - zero_frac) / (-zero_frac)
        else:
            # Data crosses zero - adjust to align zero at target fraction
            total_range_needed = max(padded_max / (1 - zero_frac), -padded_min / zero_frac)
            ymin = -zero_frac * total_range_needed
            ymax = (1 - zero_frac) * total_range_needed
        
        return ymin, ymax
    
    # Apply aligned limits to all axes
    ax1.set_ylim(get_limits_with_zero_aligned(h_min, h_max, zero_fraction))
    ax2.set_ylim(get_limits_with_zero_aligned(thrust_min, thrust_max, zero_fraction))
    ax3.set_ylim(get_limits_with_zero_aligned(m_total_min, m_total_max, zero_fraction))
    ax4.set_ylim(get_limits_with_zero_aligned(gamma_min, gamma_max, zero_fraction))
    
    # Add vertical lines for phase transitions
    if time_guidance is not None and time_guidance <= time_limit:
        ax1.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=1.5, alpha=0.7, label='Guidance')
    if time_meco is not None and time_meco <= time_limit:
        ax1.axvline(x=time_meco, color='magenta', linestyle='--', linewidth=1.5, alpha=0.7, label='MECO')
    if time_seco is not None and time_seco <= time_limit:
        ax1.axvline(x=time_seco, color='red', linestyle='--', linewidth=1.5, alpha=0.7, label='SECO')
    
    # Create combined legend
    lines = line1 + line2 + line3 + line4
    labels = [l.get_label() for l in lines]
    
    # Add phase transition labels (numbered)
    if time_guidance is not None and time_guidance <= time_limit:
        labels.append('① Guidance')
        lines.append(plt.Line2D([0], [0], color='cyan', linestyle='--', linewidth=1.5))
    if time_meco is not None and time_meco <= time_limit:
        labels.append('② MECO')
        lines.append(plt.Line2D([0], [0], color='magenta', linestyle='--', linewidth=1.5))
    if time_seco is not None and time_seco <= time_limit:
        labels.append('③ SECO')
        lines.append(plt.Line2D([0], [0], color='red', linestyle='--', linewidth=1.5))
    
    legend = ax1.legend(lines, labels, loc='upper center', fontsize=12, draggable=True)
    
    plt.tight_layout()
    plt.show(block=False)

def plot_apollo_steering_angles(alpha_data, alpha_time_data, time_steps, data):
    """
    Plot steering angles (angle of attack) throughout the entire flight.
    
    This function creates a detailed plot showing:
    - Steering angle (alpha) throughout the flight
    - Flight path angle (gamma) for reference  
    - Different flight phases marked (kick, guidance, coast)
    
    Parameters:
    -----------
    alpha_data : array
        Steering angle history throughout flight [rad]
    alpha_time_data : array
        Time values corresponding to steering angles [s]
    time_steps : array
        Full simulation time steps [s]
    data : array
        Full state data with structure:
            * data[0]: downtrack s; [m]
            * data[1]: current radius r from Earth's center; [m]
            * data[2]: velocity norm; [m/s]
            * data[3]: flight path angle; [rad]
            * data[4]: mass of the rocket; [kg]
    """

    # Enforce monotonic samples before plotting.
    alpha_time_data, alpha_data = _prepare_monotonic_series(alpha_time_data, alpha_data)
    
    # Check if we have steering angle data
    if len(alpha_data) == 0 or len(alpha_time_data) == 0:
        print("No steering angle data available - skipping steering angle plot")
        return
    
    # Check if Apollo guidance angles are hitting the limits
    if sim_params.GUIDANCE_MODE == "apollo":
        alpha_limit = np.deg2rad(15)
        
        # Only check angles during guidance phase
        time_guidance = ra.time_guidance_start
        time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
        
        if time_guidance is not None and time_seco is not None:
            # Find indices for guidance phase
            guidance_mask = (alpha_time_data >= time_guidance) & (alpha_time_data <= time_seco)
            alpha_guidance = alpha_data[guidance_mask]
            
            if len(alpha_guidance) > 0:
                num_at_limit = np.sum(np.abs(alpha_guidance) >= alpha_limit * 0.99)
                pct_at_limit = 100.0 * num_at_limit / len(alpha_guidance)
                
                if pct_at_limit > 50:
                    print(f"\nWARNING: {pct_at_limit:.1f}% of Apollo guidance commands are at the +/-15 deg safety limit!")
                    print("   This indicates the guidance is commanding angles larger than the limit.")
                    print("   Possible causes:")
                    print("   - Horizontal coefficients creating large horizontal thrust requirements")
                    print("   - Time-to-go estimate might be inaccurate")
                    print("   - Target conditions might be inconsistent with current trajectory")
    
    # Get phase transition times
    time_kick_start = ra.time_kick_start
    time_guidance = ra.time_guidance_start
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    #apollo_freeze_time = ra.apollo_freeze_time
    
    # Create figure and axis
    fig, ax1 = plt.subplots(figsize=(12, 7))
    
    # ============= SUBPLOT 1: Steering Angle (Alpha) =============
    ax1.set_xlabel('Time [s]', fontsize=18)
    ax1.set_ylabel('Angle [deg]', fontsize=18, fontweight='bold')
    ax1.set_title('Steering Angle (Angle of Attack) vs Time', fontsize=20, fontweight='bold')
    
    # Plot steering angle throughout the flight
    ax1.plot(alpha_time_data, np.rad2deg(alpha_data), 'b-', linewidth=2, label='Steering Angle (α)', alpha=0.8)
    ax1.axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.5)
    ax1.tick_params(axis='both', which='major', labelsize=12)
    ax1.grid(True, alpha=0.3)
    
    # Add phase transition markers
    if time_kick_start is not None:
        ax1.axvline(x=time_kick_start, color='green', linestyle='--', linewidth=1.5, alpha=0.6, label='Kick Start')
        kick_end = time_kick_start + sim_params.DURATION_INITIAL_KICK
        ax1.axvline(x=kick_end, color='lime', linestyle='--', linewidth=1.5, alpha=0.6, label='Kick End')
        # Add shaded region for kick phase
        #ax1.axvspan(time_kick_start, kick_end, alpha=0.1, color='green', label='Kick Phase')
    
    if time_guidance is not None:
        ax1.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=1.5, alpha=0.7, label='Guidance Start')
       # if time_seco is not None:
            # Add shaded region for guidance phase
            #ax1.axvspan(time_guidance, time_seco, alpha=0.1, color='cyan', label='Guidance Phase')
    
    #if apollo_freeze_time is not None and sim_params.GUIDANCE_MODE == "apollo":
        #ax1.axvline(x=apollo_freeze_time, color='orange', linestyle='--', linewidth=1.5, alpha=0.7, label='Coeff. Frozen')
    
    if time_seco is not None:
        ax1.axvline(x=time_seco, color='red', linestyle='--', linewidth=1.5, alpha=0.7, label='SECO')
    
    ax1.legend(loc='best', fontsize=12, ncol=2)
    
    # ============= SUBPLOT 2: Flight Path Angle and Steering Angle =============
    #ax2.set_xlabel('Time [s]', fontsize=14)
    #ax2.set_ylabel('Angle [deg]', fontsize=14, fontweight='bold')
    #ax2.set_title('Flight Path Angle vs Steering Angle', fontsize=16, fontweight='bold')
    
    # Interpolate flight path angle to match steering angle time points
    #gamma_full = data[3]  # Full flight path angle array [rad]
    #gamma_interp = np.interp(alpha_time_data, time_steps, gamma_full)
    
    # Plot both angles
    #ax2.plot(alpha_time_data, np.rad2deg(gamma_interp), 'g-', linewidth=2, label='Flight Path Angle (γ)', alpha=0.8)
    #ax2.plot(alpha_time_data, np.rad2deg(alpha_data), 'b-', linewidth=2, label='Steering Angle (α)', alpha=0.8)
    
    # Plot the sum (thrust vector inertial angle)
    #thrust_angle_inertial = gamma_interp + alpha_data
    #ax2.plot(alpha_time_data, np.rad2deg(thrust_angle_inertial), 'r--', linewidth=1.5, 
            #label='Thrust Direction (γ + α)', alpha=0.7)
    
    #ax2.axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.5, label='Horizontal')
    #ax2.tick_params(axis='both', which='major', labelsize=12)
    #ax2.grid(True, alpha=0.3)
    
    # Add phase transition markers (same as subplot 1)
    #if time_kick_start is not None:
        #ax2.axvline(x=time_kick_start, color='green', linestyle='--', linewidth=1.5, alpha=0.5)
        #kick_end = time_kick_start + sim_params.DURATION_INITIAL_KICK
        #ax2.axvline(x=kick_end, color='lime', linestyle='--', linewidth=1.5, alpha=0.5)
        #ax2.axvspan(time_kick_start, kick_end, alpha=0.1, color='green')
    
    #if time_guidance is not None:
        #ax2.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=1.5, alpha=0.5)
        #if time_seco is not None:
            #ax2.axvspan(time_guidance, time_seco, alpha=0.1, color='cyan')
    
    #if apollo_freeze_time is not None and sim_params.GUIDANCE_MODE == "apollo":
        #ax2.axvline(x=apollo_freeze_time, color='orange', linestyle='--', linewidth=1.5, alpha=0.5)
    
    #if time_seco is not None:
        #ax2.axvline(x=time_seco, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
    
    #ax2.legend(loc='best', fontsize=12)
    
    # Add text box with comprehensive statistics
    #if len(alpha_data) > 0:
        #stats_lines = ['Steering Angle Statistics:']
        #stats_lines.append(f'Overall - Mean |α|: {np.rad2deg(np.mean(np.abs(alpha_data))):.3f}°, Max |α|: {np.rad2deg(np.max(np.abs(alpha_data))):.3f}°')
        
        # Kick phase statistics
        #if time_kick_start is not None:
        #    kick_end = time_kick_start + sim_params.DURATION_INITIAL_KICK
        #    kick_mask = (alpha_time_data >= time_kick_start) & (alpha_time_data <= kick_end)
        #    if np.any(kick_mask):
        #        alpha_kick = alpha_data[kick_mask]
        #        stats_lines.append(f'Kick Phase - Mean |α|: {np.rad2deg(np.mean(np.abs(alpha_kick))):.3f}°, Max |α|: {np.rad2deg(np.max(np.abs(alpha_kick))):.3f}°')
        
        # Guidance phase statistics
        #if time_guidance is not None and time_seco is not None:
        #    guidance_mask = (alpha_time_data >= time_guidance) & (alpha_time_data <= time_seco)
        #    if np.any(guidance_mask):
        #        alpha_guidance = alpha_data[guidance_mask]
        #        stats_lines.append(f'Guidance Phase - Mean |α|: {np.rad2deg(np.mean(np.abs(alpha_guidance))):.3f}°, Max |α|: {np.rad2deg(np.max(np.abs(alpha_guidance))):.3f}°')
        
        #stats_text = '\n'.join(stats_lines)
        
        #ax2.text(0.02, 0.02, stats_text, transform=ax2.transAxes, fontsize=11,
        #        verticalalignment='bottom', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Set x-axis limit to 1000 seconds
    ax1.set_xlim(0, 1000)
    
    plt.tight_layout()
    plt.show(block=False)


def plot_latitude_over_time(time_steps, data):
    """
    Plot propagated latitude over time.

    Inputs:
        - time_steps: array of simulation time steps [s]
        - data: array of state history
            * data[0]: downtrack s [m]
            * data[5]: propagated latitude [rad] (when Earth rotation is enabled)
    """
    if data.shape[0] <= 5:
        print("Latitude history not available (Earth rotation disabled or latitude state not present).")
        return

    # Reduce data for plotting clarity.
    reduction_factor = 10
    time_reduced = time_steps[::reduction_factor]
    lat_reduced_deg = np.rad2deg(data[5, ::reduction_factor])

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(time_reduced, lat_reduced_deg, color='tab:blue', linewidth=2.5, label='Propagated Latitude')

    # Reference lines for launch latitude and physical bounds.
    ax.axhline(sim_params.LAUNCH_LATITUDE, color='tab:orange', linestyle='--', linewidth=1.5,
               label=f'Launch Latitude ({sim_params.LAUNCH_LATITUDE:.2f} deg)')

    ax.set_xlabel('Time [s]', fontsize=18)
    ax.set_ylabel('Latitude [deg]', fontsize=18)
    ax.set_title('Propagated Latitude Over Time', fontsize=20, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=14)

    plt.tight_layout()
    plt.show(block=False)