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
    
    # Get phase transition times
    time_guidance = ra.time_atmosphere_exit
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
    fig.suptitle('Key Trajectory Parameters Over Time', fontsize=16, fontweight='bold')
    
    # First y-axis: Altitude (left side)
    color1 = 'tab:blue'
    ax1.set_xlabel('Time [s]', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Altitude [km]', color=color1, fontsize=12, fontweight='bold')
    line1 = ax1.plot(time_reduced, h, color=color1, linewidth=2.5, label='Altitude')
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.grid(True, alpha=0.3)
    
    # Second y-axis: Thrust
    ax2 = ax1.twinx()
    color2 = 'tab:red'
    ax2.set_ylabel('Thrust [kN]', color=color2, fontsize=12, fontweight='bold')
    line2 = ax2.plot(time_reduced, thrust, color=color2, linewidth=2.5, label='Thrust')
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Third y-axis: Propellant Mass
    ax3 = ax1.twinx()
    ax3.spines['right'].set_position(('outward', 60))
    color3 = 'tab:green'
    ax3.set_ylabel('Propellant Mass [kg]', color=color3, fontsize=12, fontweight='bold')
    line3 = ax3.plot(time_reduced, m_prop, color=color3, linewidth=2.5, label='Propellant Mass')
    ax3.tick_params(axis='y', labelcolor=color3)
    
    # Fourth y-axis: Flight Path Angle
    ax4 = ax1.twinx()
    ax4.spines['right'].set_position(('outward', 120))
    color4 = 'tab:purple'
    ax4.set_ylabel('Flight Path Angle [deg]', color=color4, fontsize=12, fontweight='bold')
    line4 = ax4.plot(time_reduced, np.rad2deg(gamma), color=color4, linewidth=2.5, label='Flight Path Angle')
    ax4.tick_params(axis='y', labelcolor=color4)
    ax4.axhline(y=0, color=color4, linestyle=':', linewidth=1, alpha=0.3)
    
    # Add vertical lines for phase transitions
    if time_guidance is not None:
        ax1.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=2, alpha=0.7)
    if time_meco is not None:
        ax1.axvline(x=time_meco, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if time_seco is not None:
        ax1.axvline(x=time_seco, color='black', linestyle='--', linewidth=2, alpha=0.7)
    
    # Create combined legend
    lines = line1 + line2 + line3 + line4
    labels = [l.get_label() for l in lines]
    
    # Add phase markers to legend
    from matplotlib.lines import Line2D
    legend_elements = lines.copy()
    legend_labels = labels.copy()
    
    if time_guidance is not None:
        legend_elements.append(Line2D([0], [0], color='cyan', linestyle='--', linewidth=2))
        legend_labels.append('Guidance Activation')
    if time_meco is not None:
        legend_elements.append(Line2D([0], [0], color='orange', linestyle='--', linewidth=2))
        legend_labels.append('MECO (Stage 1 Cutoff)')
    if time_seco is not None:
        legend_elements.append(Line2D([0], [0], color='black', linestyle='--', linewidth=2))
        legend_labels.append('SECO (Coasting Start)')
    
    ax1.legend(legend_elements, legend_labels, loc='upper left', fontsize=10, framealpha=0.9)
    
    plt.tight_layout()
    plt.show()



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
    
    # Get phase transition times
    time_guidance = ra.time_atmosphere_exit
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
    
    # Compute thrust and acceleration (approximate)
    F_T = ra.r.F_THRUST_2  # Second stage thrust
    Isp = ra.r.ISP_2
    thrust_accel = F_T / m
    
    # -------------- Plotting --------------
    fig1, axs1 = plt.subplots(3, 3, figsize=(18, 15))
    fig1.suptitle('Guidance Phase Detailed Analysis', fontsize=16, fontweight='bold')
    
    # Row 1: Position and Trajectory
    # Trajectory plot: altitude vs downtrack
    axs1[0, 0].plot(s, h, 'b-', linewidth=2)
    axs1[0, 0].plot(s[0], h[0], 'go', markersize=10, label='Guidance Start')
    axs1[0, 0].plot(s[-1], h[-1], 'ro', markersize=10, label='SECO')
    axs1[0, 0].set_xlabel('Downtrack [km]')
    axs1[0, 0].set_ylabel('Altitude [km]')
    axs1[0, 0].set_title('Trajectory During Guidance')
    axs1[0, 0].legend()
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
    fig2.suptitle('Guidance Phase Rates and Performance', fontsize=16, fontweight='bold')
    
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
    time_guidance = ra.time_atmosphere_exit
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
    ax.set_facecolor("black")
    
    # Plot trajectory in two segments with different colors
    # Segment 1: Launch to atmosphere exit (gravity turn phase) - white
    ax.plot(x[:idx_guidance_reduced+1], y[:idx_guidance_reduced+1], 
           color="white", linewidth=2, label="Initial Gravity Turn", zorder=3)
    
    # Segment 2: Atmosphere exit to SECO (guidance phase) - cyan
    if time_guidance is not None:
        ax.plot(x[idx_guidance_reduced:], y[idx_guidance_reduced:], 
               color="cyan", linewidth=2.5, label="Active Guidance Phase", zorder=4)
    
    # Add markers
    ax.plot(x[0], y[0], 'go', markersize=12, label='Launch', zorder=5)
    
    if time_guidance is not None:
        ax.plot(x[idx_guidance_reduced], y[idx_guidance_reduced], 'y^', 
               markersize=14, label='Guidance Activation', zorder=5)
    
    ax.plot(x[-1], y[-1], 'ro', markersize=14, label='SECO (Coasting Start)', zorder=5)
    
    # Create Earth representation (circular disk)
    earth_radius_km = c.R_EARTH / 1000.0
    earth = plt.Circle((0, 0), earth_radius_km, color='blue', zorder=1)
    
    # Show Earth
    ax.add_patch(earth)
    
    # Labels and aesthetics
    ax.set_xlabel("Downtrack Distance (km)", color="white", fontsize=12)
    ax.set_ylabel("Altitude (km)", color="white", fontsize=12)
    ax.set_title("Powered Ascent Trajectory (Launch to SECO)", color="white", fontsize=14, fontweight='bold')
    ax.tick_params(colors='white')
    ax.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.3)
    ax.legend(loc='upper left', fontsize=10)
    
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
    time_guidance = ra.time_atmosphere_exit
    time_meco = ra.time_main_engine_cutoff
    
    # Create figure with single plot
    fig, ax1 = plt.subplots(figsize=(12, 7))
    fig.suptitle(f'Ascent Phase: Key Parameters (Launch to SECO + 100s)', fontsize=14, fontweight='bold')
    
    # Plot 1: Altitude (left y-axis)
    color1 = 'tab:blue'
    ax1.set_xlabel('Time [s]', fontsize=12)
    ax1.set_ylabel('Altitude [km]', color=color1, fontsize=12)
    line1 = ax1.plot(time_reduced, h, color=color1, linewidth=2, label='Altitude')
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Thrust (right y-axis 1)
    ax2 = ax1.twinx()
    color2 = 'tab:red'
    ax2.set_ylabel('Thrust [kN]', color=color2, fontsize=12)
    line2 = ax2.plot(time_reduced, thrust, color=color2, linewidth=2, label='Thrust')
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Plot 3: Total Mass (right y-axis 2, offset)
    ax3 = ax1.twinx()
    ax3.spines['right'].set_position(('outward', 60))
    color3 = 'tab:green'
    ax3.set_ylabel('Total Mass [kg]', color=color3, fontsize=12)
    line3 = ax3.plot(time_reduced, m_total, color=color3, linewidth=2, label='Total Mass')
    ax3.tick_params(axis='y', labelcolor=color3)
    
    # Plot 4: Flight Path Angle (right y-axis 3, offset more)
    ax4 = ax1.twinx()
    ax4.spines['right'].set_position(('outward', 120))
    color4 = 'tab:purple'
    ax4.set_ylabel('Flight Path Angle [deg]', color=color4, fontsize=12)
    line4 = ax4.plot(time_reduced, np.rad2deg(gamma), color=color4, linewidth=2, label='Flight Path Angle')
    ax4.tick_params(axis='y', labelcolor=color4)
    ax4.axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.3)
    
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
    
    # Add phase transition labels
    if time_guidance is not None and time_guidance <= time_limit:
        labels.append('Guidance')
        lines.append(plt.Line2D([0], [0], color='cyan', linestyle='--', linewidth=1.5))
    if time_meco is not None and time_meco <= time_limit:
        labels.append('MECO')
        lines.append(plt.Line2D([0], [0], color='magenta', linestyle='--', linewidth=1.5))
    if time_seco is not None and time_seco <= time_limit:
        labels.append('SECO')
        lines.append(plt.Line2D([0], [0], color='red', linestyle='--', linewidth=1.5))
    
    ax1.legend(lines, labels, loc='best', fontsize=10)
    
    plt.tight_layout()
    plt.show(block=False)

def plot_apollo_steering_angles(alpha_data, alpha_time_data, time_steps, data):
    """
    Plot Apollo guidance steering angles (angle of attack) during the guidance phase.
    
    This function creates a detailed plot showing:
    - Steering angle (alpha) commanded by Apollo guidance
    - Flight path angle (gamma) for reference
    - Angle of attack relative to velocity vector
    
    Parameters:
    -----------
    alpha_data : array
        Steering angle history during guidance phase [rad]
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
    
    # Check if we have Apollo guidance data
    if len(alpha_data) == 0 or len(alpha_time_data) == 0:
        print("No Apollo guidance data available - skipping steering angle plot")
        return
    
    # Get phase transition times
    time_guidance = ra.time_atmosphere_exit
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    apollo_freeze_time = ra.apollo_freeze_time
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle('Apollo Guidance: Steering Angles During Guidance Phase', fontsize=16, fontweight='bold')
    
    # ============= SUBPLOT 1: Steering Angle (Alpha) =============
    ax1.set_xlabel('Time [s]', fontsize=12)
    ax1.set_ylabel('Angle [deg]', fontsize=12, fontweight='bold')
    ax1.set_title('Commanded Steering Angle (Angle of Attack)', fontsize=13, fontweight='bold')
    
    # Plot steering angle commanded by Apollo guidance
    ax1.plot(alpha_time_data, np.rad2deg(alpha_data), 'b-', linewidth=2.5, label='Steering Angle (α)')
    ax1.axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.5)
    ax1.grid(True, alpha=0.3)
    
    # Add phase transition markers
    if time_guidance is not None:
        ax1.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=1.5, alpha=0.7, label='Guidance Start')
        ax1.text(time_guidance, ax1.get_ylim()[1]*0.9, 'Guidance\nActivation', 
                ha='center', va='top', fontsize=9, bbox=dict(boxstyle='round', facecolor='cyan', alpha=0.3))
    
    if apollo_freeze_time is not None:
        ax1.axvline(x=apollo_freeze_time, color='orange', linestyle='--', linewidth=1.5, alpha=0.7, label='Coeff. Frozen')
        ax1.text(apollo_freeze_time, ax1.get_ylim()[1]*0.75, 'Coefficients\nFrozen', 
                ha='center', va='top', fontsize=9, bbox=dict(boxstyle='round', facecolor='orange', alpha=0.3))
    
    if time_seco is not None:
        ax1.axvline(x=time_seco, color='red', linestyle='--', linewidth=1.5, alpha=0.7, label='SECO')
        ax1.text(time_seco, ax1.get_ylim()[1]*0.6, 'SECO\n(Guidance End)', 
                ha='center', va='top', fontsize=9, bbox=dict(boxstyle='round', facecolor='red', alpha=0.3))
    
    ax1.legend(loc='best', fontsize=11)
    
    # ============= SUBPLOT 2: Flight Path Angle and Steering Angle =============
    ax2.set_xlabel('Time [s]', fontsize=12)
    ax2.set_ylabel('Angle [deg]', fontsize=12, fontweight='bold')
    ax2.set_title('Flight Path Angle vs Steering Angle', fontsize=13, fontweight='bold')
    
    # Interpolate flight path angle to match guidance time points
    gamma_full = data[3]  # Full flight path angle array [rad]
    gamma_guidance = np.interp(alpha_time_data, time_steps, gamma_full)
    
    # Plot both angles
    ax2.plot(alpha_time_data, np.rad2deg(gamma_guidance), 'g-', linewidth=2.5, label='Flight Path Angle (γ)', alpha=0.8)
    ax2.plot(alpha_time_data, np.rad2deg(alpha_data), 'b-', linewidth=2.5, label='Steering Angle (α)', alpha=0.8)
    
    # Plot the sum (thrust vector inertial angle)
    thrust_angle_inertial = gamma_guidance + alpha_data
    ax2.plot(alpha_time_data, np.rad2deg(thrust_angle_inertial), 'r--', linewidth=2, 
            label='Thrust Direction (γ + α)', alpha=0.7)
    
    ax2.axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.5, label='Horizontal')
    ax2.grid(True, alpha=0.3)
    
    # Add phase transition markers
    if time_guidance is not None:
        ax2.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=1.5, alpha=0.5)
    if apollo_freeze_time is not None:
        ax2.axvline(x=apollo_freeze_time, color='orange', linestyle='--', linewidth=1.5, alpha=0.5)
    if time_seco is not None:
        ax2.axvline(x=time_seco, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
    
    ax2.legend(loc='best', fontsize=11)
    
    # Add text box with statistics
    if len(alpha_data) > 0:
        alpha_mean = np.mean(np.abs(alpha_data))
        alpha_max = np.max(np.abs(alpha_data))
        alpha_std = np.std(alpha_data)
        
        stats_text = (f'Steering Angle Statistics:\n'
                     f'Mean |α|: {np.rad2deg(alpha_mean):.3f}°\n'
                     f'Max |α|: {np.rad2deg(alpha_max):.3f}°\n'
                     f'Std Dev: {np.rad2deg(alpha_std):.3f}°')
        
        ax2.text(0.02, 0.02, stats_text, transform=ax2.transAxes, fontsize=10,
                verticalalignment='bottom', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.show(block=False)