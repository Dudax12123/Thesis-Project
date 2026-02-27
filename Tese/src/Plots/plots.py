""" ===============================================
                Plotting
=============================================== """

import matplotlib.pyplot as plt
import numpy as np
import sys
from pathlib import Path

# Add parent directory to path to enable imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Input_File import simulation_parameters as sim_params
from Auxiliary import rocket_specs as r
from Auxiliary import gravity as grav
from Auxiliary import atmosphere as atm
from Simulation import rocket_ascent as ra


def single_run(time_steps, data, INITIAL_KICK_ANGLE, thrust_data, time_thrust):
    """
    Inputs:
        - time_steps: array of time steps (for the data array); [s]
        - data: array of data points. The data array has the following structure:
            * data[0]: downtrack s; [m]
            * data[1]: current radius r from Earth's center; [m]
            * data[2]: velocity norm; [m/s]
            * data[3]: flight path angle; [rad]
            * data[4]: mass of the rocket; [kg]
        - thrust_data: array of actual thrust values from simulation; [N]
        - time_thrust: array of time steps corresponding to thrust values; [s]

    Plots the following data over time:
        - altitude over downtrack
        - downtrack over time
        - altitude over time
        - velocity norm  over time
        - flight path angle (gamma) over time
        - mass of the rocket over time
        - dynamic pressure over time (based on velocity norm)
        - angle of attack over time
        
    Phase transition markers are added to show:
        - Guidance activation (atmosphere exit at 65 km)
        - Powered ascent to coasting transition (SECO)
        - Orbit insertion (circularization burn at apogee)
    """

    # Reduce data array
    data_reduced = data[:, ::10]
    time_reduced = time_steps[::10]
    
    # Get phase transition times from rocket_ascent module
    time_guidance = ra.time_atmosphere_exit  # Guidance activation (atmosphere exit)
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL  # End of powered ascent
    
    # Find orbit insertion (apogee) by detecting velocity discontinuity
    # The instantaneous circularization burn causes a velocity jump
    time_insertion = None
    if time_seco is not None:
        # Look for velocity discontinuity after SECO
        velocity_full = data[2]  # Full velocity array (not reduced)
        for i in range(1, len(velocity_full)):
            if time_steps[i] > time_seco:
                # Check for sudden velocity increase (delta-v application)
                velocity_jump = velocity_full[i] - velocity_full[i-1]
                time_diff = time_steps[i] - time_steps[i-1]
                if time_diff > 0:
                    accel = velocity_jump / time_diff
                    # Delta-v application shows as very high acceleration (>100 m/s²)
                    if accel > 100.0:
                        time_insertion = time_steps[i]
                        break
    
    # Helper function to find closest index in reduced time array
    def find_closest_index(time_array, target_time):
        if target_time is None:
            return None
        return np.argmin(np.abs(time_array - target_time))
    
    # Find indices for phase transitions
    idx_guidance = find_closest_index(time_reduced, time_guidance)
    idx_seco = find_closest_index(time_reduced, time_seco)
    idx_insertion = find_closest_index(time_reduced, time_insertion)

    # -------------- Prepare data --------------
    h = (data_reduced[1] - c.R_EARTH) / 1000.       # altitude h; [km]
    s = data_reduced[0] / 1000.                     # downtrack s; [km]
    
    # Compute propellant mass (total mass minus structure and payload)
    m_total = data_reduced[4]
    m_prop = m_total - (r.M_STRUCTURE_1 + r.M_STRUCTURE_2 + r.M_PAYLOAD)
    
    # Interpolate actual thrust data to match reduced time steps
    thrust = np.interp(time_reduced, time_thrust, thrust_data) / 1000.  # Convert to kN

    q = [0.0] * len(time_reduced)          # dynamic pressure; [Pa]
    for i in range(len(time_reduced)):
        alt = h[i] * 1000
        v = data_reduced[2][i]
        rho = c.RHO_0 * np.exp(-alt / c.H)
        q[i] = 0.5 * rho * v**2
    max_q = max(q)
    print("Maximum Dynamic Pressure:")
    print("\t* Max-Q:\t\t\t\t\t", max_q, "Pa")
    
    # Recreate angle of attack values
    # Initialize empty list with length of t
    angle_of_attacks = [0.0] * len(time_reduced)
    
    # Note: time_kick_start is set during simulation in rocket_ascent module
    # For plotting purposes, we'll approximate it
    time_kick_start_approx = sim_params.TIME_TO_START_KICK
    time_raise = sim_params.DURATION_INITIAL_KICK / 2.

    for i, t in enumerate(time_reduced):
        if t < time_kick_start_approx:
            angle_of_attacks[i] = 0.0
        elif t > (time_kick_start_approx + sim_params.DURATION_INITIAL_KICK):
            angle_of_attacks[i] = 0.0
        elif t > (time_kick_start_approx + (sim_params.DURATION_INITIAL_KICK / 2.)):
            angle_rate = (t - (time_kick_start_approx + time_raise)) / (time_raise)
            angle_of_attacks[i] = INITIAL_KICK_ANGLE * (1 - angle_rate)
        else:
            angle_rate = (t - time_kick_start_approx) / (time_raise)
            angle_of_attacks[i] = INITIAL_KICK_ANGLE * angle_rate


    # ----- COMPUTE LOSSES -----
    # Only compute losses if we have a time to stop burning
    if ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL is not None:
        time_to_stop_burning = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL

        # Gravity loss
        grav_loss = []
        drag_loss = []
        steering_loss = []
        time_loss = []
        
        # Use the actual drag coefficient used during ascent (0.3)
        # Note: r.C_D gets set to 0 after burn stops in the simulation
        C_D_actual = 0.3
        A_actual = r.A
        
        print(f"\nUsing C_D = {C_D_actual} for loss calculations (actual during ascent)")
        print(f"Cross-sectional area A: {A_actual} m^2")

        for i in range(len(time_reduced)):

            if time_reduced[i] > time_to_stop_burning:
                break

            current_radius = data_reduced[1][i]
            current_theta = data_reduced[3][i]
            current_mass = data_reduced[4][i]
        
            grav_accel = grav.gravitational_acceleration(current_radius)
            drag_accel = (q[i] * C_D_actual * A_actual) / current_mass
            
            # Steering loss: thrust not aligned with velocity
            # Interpolate thrust at this time point
            thrust_N = np.interp(time_reduced[i], time_thrust, thrust_data)  # Thrust in N
            thrust_accel = thrust_N / current_mass
            alpha = angle_of_attacks[i]
            steering_accel = thrust_accel * (1.0 - np.cos(alpha))  # Loss due to angle of attack
            
            # Debug output for first few iterations
            if i < 3:
                print(f"\nIteration {i}:")
                print(f"  Time: {time_reduced[i]:.2f} s")
                print(f"  Altitude: {(current_radius - c.R_EARTH)/1000:.2f} km")
                print(f"  Velocity: {data_reduced[2][i]:.2f} m/s")
                print(f"  Dynamic pressure q: {q[i]:.2f} Pa")
                print(f"  Mass: {current_mass:.2f} kg")
                print(f"  Angle of attack: {np.rad2deg(alpha):.4f} deg")
                print(f"  Drag accel: {drag_accel:.6f} m/s^2")
                print(f"  Gravity accel component: {grav_accel * np.sin(current_theta):.6f} m/s^2")
                print(f"  Steering accel loss: {steering_accel:.6f} m/s^2")

            grav_loss.append(grav_accel * np.sin(current_theta) * (time_reduced[i] - time_reduced[i-1]) + grav_loss[i-1] if i > 0 else 0.0)
            drag_loss.append(drag_accel * (time_reduced[i] - time_reduced[i-1]) + drag_loss[i-1] if i > 0 else 0.0)
            steering_loss.append(steering_accel * (time_reduced[i] - time_reduced[i-1]) + steering_loss[i-1] if i > 0 else 0.0)

            time_loss.append(time_reduced[i])


    # -------------- Plotting --------------
    # COMMENTED OUT: Full Mission Trajectory plot
    # fig1, axs1 = plt.subplots(2, 2, figsize=(14, 10))
    # fig1.suptitle('Full Mission Trajectory', fontsize=16, fontweight='bold')

    # # Altitude over time
    # axs1[0, 0].plot(time_reduced, h, 'b-', linewidth=2)
    # if idx_guidance is not None:
    #     axs1[0, 0].plot(time_reduced[idx_guidance], h[idx_guidance], 'b^', markersize=10, label='Guidance', zorder=5)
    # if idx_seco is not None:
    #     axs1[0, 0].plot(time_reduced[idx_seco], h[idx_seco], 'ro', markersize=10, label='SECO', zorder=5)
    # if idx_insertion is not None:
    #     axs1[0, 0].plot(time_reduced[idx_insertion], h[idx_insertion], 'gs', markersize=10, label='Insertion', zorder=5)
    # axs1[0, 0].set_xlabel('Time [s]', fontsize=11)
    # axs1[0, 0].set_ylabel('Altitude [km]', fontsize=11)
    # axs1[0, 0].set_title('Altitude over Time', fontsize=12, fontweight='bold')
    # axs1[0, 0].legend(fontsize=9)
    # axs1[0, 0].grid(True, alpha=0.3)

    # # Thrust over time
    # axs1[0, 1].plot(time_reduced, thrust, 'r-', linewidth=2)
    # if idx_guidance is not None:
    #     axs1[0, 1].plot(time_reduced[idx_guidance], thrust[idx_guidance], 'b^', markersize=10, label='Guidance', zorder=5)
    # if idx_seco is not None:
    #     axs1[0, 1].plot(time_reduced[idx_seco], thrust[idx_seco], 'ro', markersize=10, label='SECO', zorder=5)
    # if idx_insertion is not None:
    #     axs1[0, 1].plot(time_reduced[idx_insertion], thrust[idx_insertion], 'gs', markersize=10, label='Insertion', zorder=5)
    # axs1[0, 1].set_xlabel('Time [s]', fontsize=11)
    # axs1[0, 1].set_ylabel('Thrust [kN]', fontsize=11)
    # axs1[0, 1].set_title('Thrust over Time', fontsize=12, fontweight='bold')
    # axs1[0, 1].legend(fontsize=9)
    # axs1[0, 1].grid(True, alpha=0.3)

    # # Total mass over time
    # m_total = data_reduced[4]
    # axs1[1, 0].plot(time_reduced, m_total, 'b-', linewidth=2)
    # if idx_guidance is not None:
    #     axs1[1, 0].plot(time_reduced[idx_guidance], m_total[idx_guidance], 'c^', markersize=10, label='Guidance', zorder=5)
    # if idx_seco is not None:
    #     axs1[1, 0].plot(time_reduced[idx_seco], m_total[idx_seco], 'ro', markersize=10, label='SECO', zorder=5)
    # if idx_insertion is not None:
    #     axs1[1, 0].plot(time_reduced[idx_insertion], m_total[idx_insertion], 'gs', markersize=10, label='Insertion', zorder=5)
    # axs1[1, 0].set_xlabel('Time [s]', fontsize=11)
    # axs1[1, 0].set_ylabel('Total Mass [kg]', fontsize=11)
    # axs1[1, 0].set_title('Total Mass over Time', fontsize=12, fontweight='bold')
    # axs1[1, 0].legend(fontsize=9)
    # axs1[1, 0].grid(True, alpha=0.3)

    # # Flight path angle over time
    # axs1[1, 1].plot(time_reduced, np.rad2deg(data_reduced[3]), 'm-', linewidth=2)
    # if idx_guidance is not None:
    #     axs1[1, 1].plot(time_reduced[idx_guidance], np.rad2deg(data_reduced[3][idx_guidance]), 'b^', markersize=10, label='Guidance', zorder=5)
    # if idx_seco is not None:
    #     axs1[1, 1].plot(time_reduced[idx_seco], np.rad2deg(data_reduced[3][idx_seco]), 'ro', markersize=10, label='SECO', zorder=5)
    # if idx_insertion is not None:
    #     axs1[1, 1].plot(time_reduced[idx_insertion], np.rad2deg(data_reduced[3][idx_insertion]), 'gs', markersize=10, label='Insertion', zorder=5)
    # axs1[1, 1].axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.5)
    # axs1[1, 1].set_xlabel('Time [s]', fontsize=11)
    # axs1[1, 1].set_ylabel('Flight Path Angle [deg]', fontsize=11)
    # axs1[1, 1].set_title('Flight Path Angle over Time', fontsize=12, fontweight='bold')
    # axs1[1, 1].legend(fontsize=9)
    # axs1[1, 1].grid(True, alpha=0.3)

    if ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL is not None:
        # Find SECO point in loss time array
        idx_seco_loss = find_closest_index(np.array(time_loss), time_seco) if time_seco is not None else None
        
        # Get event times
        time_meco = ra.time_main_engine_cutoff
        
        # Plot the gravity loss, the drag loss, steering loss and the total loss in one plot
        fig2, axs2 = plt.subplots(figsize=(12, 6))
        axs2.plot(time_loss, grav_loss, label="Gravity Loss", color="blue", linewidth=2)
        axs2.plot(time_loss, drag_loss, label="Drag Loss", color="orange", linewidth=2)
        axs2.plot(time_loss, steering_loss, label="Steering Loss", color="green", linewidth=2)
        axs2.plot(time_loss, np.array(grav_loss) + np.array(drag_loss) + np.array(steering_loss), 
                 label="Total Loss", color="red", linewidth=2.5, linestyle='--')
        
        # Add vertical lines for phase transitions
        if time_guidance is not None:
            axs2.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=2, alpha=0.7, 
                        label=f'Guidance ({time_guidance:.1f}s)')
        if time_meco is not None:
            axs2.axvline(x=time_meco, color='magenta', linestyle='--', linewidth=2, alpha=0.7, 
                        label=f'MECO ({time_meco:.1f}s)')
        if time_seco is not None:
            axs2.axvline(x=time_seco, color='darkred', linestyle='--', linewidth=2, alpha=0.7, 
                        label=f'SECO ({time_seco:.1f}s)')
        
        axs2.set_xlabel('Time [s]', fontsize=11)
        axs2.set_ylabel('Delta-V Loss [m/s]', fontsize=11)
        axs2.set_title('Trajectory Losses over Time (Powered Ascent)', fontsize=12, fontweight='bold')
        axs2.legend(fontsize=10, loc='best')
        axs2.grid(True, alpha=0.3)
        
        print("\nLosses:")
        print("\t* Gravity loss:\t\t\t\t\t", grav_loss[-1], "m/s")
        print("\t* Drag loss:\t\t\t\t\t", drag_loss[-1], "m/s")
        print("\t* Steering loss:\t\t\t\t", steering_loss[-1], "m/s")
        print("\t* Total loss:\t\t\t\t\t", grav_loss[-1] + drag_loss[-1] + steering_loss[-1], "m/s")
        print("\n\n")

    # COMMENTED OUT: Angle of Attack (Steering Angle) over Time plot
    # fig3, axs3 = plt.subplots(figsize=(12, 6))
    
    # # Convert angle of attack to degrees for plotting
    # alpha_deg = np.rad2deg(angle_of_attacks)
    
    # # Plot alpha
    # axs3.plot(time_reduced, alpha_deg, 'b-', linewidth=2, label='Angle of Attack (α)')
    
    # # Add phase transition markers
    # if time_guidance is not None and idx_guidance is not None:
    #     axs3.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=2, alpha=0.7, 
    #                 label=f'Atmosphere Exit ({time_guidance:.1f}s)')
    # if time_seco is not None and idx_seco is not None:
    #     axs3.axvline(x=time_seco, color='red', linestyle='--', linewidth=2, alpha=0.7,
    #                 label=f'SECO ({time_seco:.1f}s)')
    
    # # Mark the kick maneuver period
    # kick_start = sim_params.TIME_TO_START_KICK
    # kick_end = kick_start + sim_params.DURATION_INITIAL_KICK
    # axs3.axvspan(kick_start, kick_end, alpha=0.2, color='yellow', label='Pitch Kick Maneuver')
    
    # # Add horizontal line at zero
    # axs3.axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.5)
    
    # axs3.set_xlabel('Time [s]', fontsize=11)
    # axs3.set_ylabel('Angle of Attack [deg]', fontsize=11)
    # axs3.set_title('Angle of Attack (Steering Angle) over Time', fontsize=12, fontweight='bold')
    # axs3.legend(fontsize=10, loc='best')
    # axs3.grid(True, alpha=0.3)
    
    # # Add annotation about guidance phases
    # axs3.text(0.02, 0.98, 
    #          'Phase 1: Pitch Kick\nPhase 2: Gravity Turn (α=0)\nPhase 3: Active Guidance',
    #          transform=axs3.transAxes, fontsize=9, verticalalignment='top',
    #          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    plt.show(block=False)
    
    
    
def plot_trajectory_xy(data, time_steps):
    """
    Plots the rocket trajectory in x-y coordinates with Earth shown as a blue disk.
    Phase transition markers show guidance activation, powered-to-coasting, and orbit insertion.
    
    Inputs:
        - data: array of data points with the following structure:
            * data[0]: downtrack s; [m]
            * data[1]: current radius r from Earth's center; [m]
        - time_steps: array of time steps [s]
    """
    # Reduce data_reduced array
    data_reduced = data[:, ::10]
    time_reduced = time_steps[::10]
    
    # Get phase transition times
    time_guidance = ra.time_atmosphere_exit
    time_seco = ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL
    
    # Find orbit insertion by detecting velocity discontinuity
    time_insertion = None
    if time_seco is not None:
        velocity_full = data[2]
        for i in range(1, len(velocity_full)):
            if time_steps[i] > time_seco:
                velocity_jump = velocity_full[i] - velocity_full[i-1]
                time_diff = time_steps[i] - time_steps[i-1]
                if time_diff > 0:
                    accel = velocity_jump / time_diff
                    if accel > 100.0:
                        time_insertion = time_steps[i]
                        break
    
    # Helper function to find closest index
    def find_closest_index(time_array, target_time):
        if target_time is None:
            return None
        return np.argmin(np.abs(time_array - target_time))
    
    # Find indices for phase transitions
    idx_guidance = find_closest_index(time_reduced, time_guidance)
    idx_seco = find_closest_index(time_reduced, time_seco)
    idx_insertion = find_closest_index(time_reduced, time_insertion)

    # -------------- Prepare data --------------
    h = (data_reduced[1] - c.R_EARTH)       # altitude h; [m]
    s = data_reduced[0]                     # downtrack s; [m]
    
    # Convert to cartesian coordinates
    x, y = ra.cartesian_coordinates(h, s)
    x = x/1000.
    y = y/1000.

    # Plot setup
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_facecolor("black")

    # Plot trajectory
    ax.plot(x, y, color="white", linewidth=1, label="Rocket Trajectory")
    
    # Add phase transition markers (numbered)
    if idx_guidance is not None:
        ax.plot(x[idx_guidance], y[idx_guidance], 'o', color='cyan', markersize=8, 
               markeredgecolor='white', markeredgewidth=1, zorder=5)
        ax.text(x[idx_guidance], y[idx_guidance], '1', color='white', fontsize=7, 
               fontweight='bold', ha='center', va='center', zorder=6)
    if idx_seco is not None:
        ax.plot(x[idx_seco], y[idx_seco], 'o', color='red', markersize=8, 
               markeredgecolor='white', markeredgewidth=1, zorder=5)
        ax.text(x[idx_seco], y[idx_seco], '2', color='white', fontsize=7, 
               fontweight='bold', ha='center', va='center', zorder=6)
    if idx_insertion is not None:
        ax.plot(x[idx_insertion], y[idx_insertion], 'o', color='green', markersize=8, 
               markeredgecolor='white', markeredgewidth=1, zorder=5)
        ax.text(x[idx_insertion], y[idx_insertion], '3', color='white', fontsize=7, 
               fontweight='bold', ha='center', va='center', zorder=6)
    
    # Add custom legend for numbered markers
    from matplotlib.patches import Circle
    from matplotlib.lines import Line2D
    legend_elements = [Line2D([0], [0], color='white', linewidth=1, label='Rocket Trajectory')]
    if idx_guidance is not None:
        legend_elements.append(Line2D([0], [0], marker='o', color='w', markerfacecolor='cyan', 
                                     markersize=7, label='① Guidance Activation'))
    if idx_seco is not None:
        legend_elements.append(Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                                     markersize=7, label='② SECO (Coasting Start)'))
    if idx_insertion is not None:
        legend_elements.append(Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
                                     markersize=7, label='③ Orbit Insertion'))
    
    # Create Earth representation (circular disk)
    earth_radius_km = c.R_EARTH / 1000.0
    earth = plt.Circle((0, 0), earth_radius_km, color='blue', zorder=1)
    
    # Show Earth
    ax.add_patch(earth)

    # Labels and aesthetics
    ax.set_xlabel("Downtrack Distance (km)", color="white")
    ax.set_ylabel("Altitude (km)", color="white")
    ax.set_title("Rocket Trajectory", color="white")
    ax.tick_params(colors='white')
    ax.grid(color='gray', linestyle='--', linewidth=0.5)
    
    # Add legend with styling for black background
    legend = ax.legend(handles=legend_elements, loc='upper left', fontsize=9, 
                      facecolor='black', edgecolor='white', framealpha=0.8)
    for text in legend.get_texts():
        text.set_color('white')

    # Set limits to make sure Earth is fully visible
    ax.set_xlim(min(x) - 1200, max(x) + 1200)  # Adjust margins around trajectory
    ax.set_ylim(min(y) - 1200, max(y) + 1200)
    ax.set_aspect('equal')  # Keep aspect ratio realistic

    # plt.savefig("rocket_trajectory.jpg", dpi=1000, bbox_inches="tight", pad_inches=0)
    plt.show(block=False)
