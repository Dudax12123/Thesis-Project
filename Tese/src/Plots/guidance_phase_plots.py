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


def plot_guidance_phase(time_steps, data):
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
    plt.show()
    
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
    plt.show()
    
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
