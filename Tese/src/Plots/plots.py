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


def single_run(time_steps, data, INITIAL_KICK_ANGLE, thrust_data, time_thrust, alpha_data, alpha_time_data):
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
        - alpha_data: array of actual angle of attack values from simulation; [rad]
        - alpha_time_data: array of time steps corresponding to alpha values; [s]

    Currently plots:
        - Trajectory losses over time (gravity, drag, steering, and total)
        - Dynamic pressure over time with max-Q indication
        - Rocket acceleration over time (total and thrust components)
        - Mach number during atmospheric phase
        
    Phase transition markers are added to show:
        - Guidance activation (atmosphere exit)
        - Main engine cutoff (MECO)
        - Powered ascent to coasting transition (SECO)
        
    Note: Additional plots (altitude, velocity, mass, angle of attack) are
    available but currently commented out in the code.
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
        if sim_params.EARTH_ROTATION:
            v_drag = ra._atmosphere_relative_speed(
                v, data_reduced[3][i], data_reduced[1][i])
        else:
            v_drag = v
        rho = c.RHO_0 * np.exp(-alt / c.H)
        q[i] = 0.5 * rho * v_drag**2
    max_q = max(q)
    print("Maximum Dynamic Pressure:")
    print("\t* Max-Q:\t\t\t\t\t", max_q, "Pa")
    
    # Use actual angle of attack values from simulation
    # Interpolate alpha_data onto the reduced time grid
    angle_of_attacks = np.interp(time_reduced, alpha_time_data, alpha_data)

    # Convert gamma and alpha to surface-relative for loss / acceleration
    # formulas that assume: steering loss = (F/m)(1-cos α),
    #                       gravity loss  = g sin γ,
    # where α is measured from the velocity vector and γ from the local
    # horizontal.  In ECI both quantities use a different convention.
    gamma_for_losses = data_reduced[3].copy()
    alpha_for_losses = angle_of_attacks.copy()
    if sim_params.EARTH_ROTATION:
        gamma_for_losses = ra.inertial_to_surface_gamma(
            data_reduced[2], data_reduced[3], data_reduced[1])
        # Alpha: during kick phase the ECI alpha contains the radial baseline
        # (pi/2 - gamma).  Subtract it to recover the perturbation-only alpha.
        kick_end_time = (ra.time_kick_start + sim_params.DURATION_INITIAL_KICK
                         if ra.time_kick_start is not None else 0.0)
        mask_kick = time_reduced <= kick_end_time
        gamma_interp_kick = np.interp(time_reduced, time_steps, data[3])
        alpha_for_losses[mask_kick] = (angle_of_attacks[mask_kick]
                                       - (np.pi / 2. - gamma_interp_kick[mask_kick]))

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
            current_theta = gamma_for_losses[i]
            current_mass = data_reduced[4][i]
        
            grav_accel = grav.gravitational_acceleration(current_radius)
            drag_accel = (q[i] * C_D_actual * A_actual) / current_mass
            
            # Steering loss: thrust not aligned with velocity
            # Interpolate thrust at this time point
            thrust_N = np.interp(time_reduced[i], time_thrust, thrust_data)  # Thrust in N
            thrust_accel = thrust_N / current_mass
            alpha = alpha_for_losses[i]
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
        #idx_seco_loss = find_closest_index(np.array(time_loss), time_seco) if time_seco is not None else None
        
        # Get event times
        time_meco = ra.time_main_engine_cutoff
        
        # Plot the gravity loss, the drag loss, steering loss and the total loss in one plot
        fig2, axs2 = plt.subplots(figsize=(12, 6))
        axs2.plot(time_loss, grav_loss, label="Gravity Loss", color="blue", linewidth=2)
        axs2.plot(time_loss, drag_loss, label="Drag Loss", color="orange", linewidth=2)
        axs2.plot(time_loss, steering_loss, label="Steering Loss", color="green", linewidth=2)
        axs2.plot(time_loss, np.array(grav_loss) + np.array(drag_loss) + np.array(steering_loss), 
                 label="Total Loss", color="red", linewidth=2.5)
        
        # Add vertical lines for phase transitions
        #if time_guidance is not None:
        #    axs2.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=2, alpha=0.7, 
        #                label=f'Guidance ({time_guidance:.1f}s)')
        #if time_meco is not None:
        #    axs2.axvline(x=time_meco, color='magenta', linestyle='--', linewidth=2, alpha=0.7, 
        #                label=f'MECO ({time_meco:.1f}s)')
        #if time_seco is not None:
        #    axs2.axvline(x=time_seco, color='darkred', linestyle='--', linewidth=2, alpha=0.7, 
        #               label=f'SECO ({time_seco:.1f}s)')
        
        axs2.set_xlabel('Time [s]', fontsize=18)
        axs2.set_ylabel('Delta-V Loss [m/s]', fontsize=18)
        axs2.set_title('Trajectory Losses over Time (Powered Ascent)', fontsize=20, fontweight='bold')
        axs2.tick_params(axis='both', which='major', labelsize=12)
        axs2.legend(fontsize=12, loc='best')
        axs2.grid(True, alpha=0.3)
        
        print("\nLosses:")
        print("\t* Gravity loss:\t\t\t\t\t", grav_loss[-1], "m/s")
        print("\t* Drag loss:\t\t\t\t\t", drag_loss[-1], "m/s")
        print("\t* Steering loss:\t\t\t\t", steering_loss[-1], "m/s")
        print("\t* Total loss:\t\t\t\t\t", grav_loss[-1] + drag_loss[-1] + steering_loss[-1], "m/s")
        print("\n\n")

    # Plot dynamic pressure over time
    fig3, axs3 = plt.subplots(figsize=(12, 6))
    
    # Convert dynamic pressure to kPa for better readability
    q_kPa = np.array(q) / 1000.0
    max_q_kPa = max(q_kPa)
    
    # Find time and index of max-q
    idx_max_q = np.argmax(q_kPa)
    time_max_q = time_reduced[idx_max_q]
    
    # Plot dynamic pressure
    axs3.plot(time_reduced, q_kPa, 'b-', linewidth=2.5, label='Dynamic Pressure')
    
    # Mark max-q
    axs3.plot(time_max_q, max_q_kPa, 'ro', markersize=5, label=f'Max-Q ({max_q_kPa:.2f} kPa at {time_max_q:.1f}s)', zorder=5)
    
    # Add vertical lines for phase transitions
    if time_guidance is not None:
        axs3.axvline(x=time_guidance, color='green', linestyle='--', linewidth=2, alpha=1, 
                    label=f'Atmosphere Exit ({time_guidance:.1f}s)')
    time_meco = ra.time_main_engine_cutoff
    if time_meco is not None:
        axs3.axvline(x=time_meco, color='red', linestyle='--', linewidth=2, alpha=1, 
                    label=f'MECO ({time_meco:.1f}s)')
    if time_seco is not None:
        axs3.axvline(x=time_seco, color='brown', linestyle='--', linewidth=2, alpha=1,
                    label=f'SECO ({time_seco:.1f}s)')
    
    # Mark the kick maneuver period
    #kick_start = sim_params.TIME_TO_START_KICK
    #kick_end = kick_start + sim_params.DURATION_INITIAL_KICK
    #axs3.axvspan(kick_start, kick_end, alpha=0.15, color='yellow', label='Pitch Kick Maneuver')
    
    # Add dynamic pressure threshold line if using dynamic pressure method for atmosphere exit
    if sim_params.ATMOSPHERE_EXIT_METHOD == "dynamic_pressure":
        q_threshold_kPa = sim_params.DYNAMIC_PRESSURE_THRESHOLD / 1000.0
        axs3.axhline(y=q_threshold_kPa, color='orange', linestyle=':', linewidth=2, alpha=0.7,
                    label=f'Atmosphere Exit Threshold ({q_threshold_kPa:.2f} kPa)')
    
    axs3.set_xlabel('Time [s]', fontsize=18)
    axs3.set_ylabel('Dynamic Pressure [kPa]', fontsize=18)
    axs3.set_title('Dynamic Pressure over Time', fontsize=20, fontweight='bold')
    axs3.tick_params(axis='both', which='major', labelsize=12)
    axs3.legend(fontsize=12, loc='best')
    axs3.grid(True, alpha=0.3)
    
    # Add information text box
    #info_text = f'Max-Q: {max_q_kPa:.2f} kPa\nOccurs at: {time_max_q:.1f} s'
    #if time_guidance is not None:
    #    q_at_guidance = q_kPa[find_closest_index(time_reduced, time_guidance)]
    #    info_text += f'\nQ at atmosphere exit: {q_at_guidance:.2f} kPa'
    #axs3.text(0.98, 0.98, info_text,
    #         transform=axs3.transAxes, fontsize=12, verticalalignment='top', horizontalalignment='right',
    #         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    # Plot acceleration over time
    fig4, axs4 = plt.subplots(figsize=(12, 6))
    
    # Calculate acceleration components
    thrust_accel_arr = []  # Thrust acceleration (F_T/m)
    drag_accel_arr = []    # Drag deceleration
    grav_accel_arr = []    # Gravitational acceleration (magnitude)
    total_accel_arr = []   # Total acceleration along velocity direction
    
    for i in range(len(time_reduced)):
        # Get current state
        v = data_reduced[2][i]
        m = data_reduced[4][i]
        r_val = data_reduced[1][i]
        gamma = gamma_for_losses[i]
        alt = (r_val - c.R_EARTH)
        
        # Interpolate thrust at this time point
        thrust_N = np.interp(time_reduced[i], time_thrust, thrust_data)
        
        # Calculate acceleration components
        a_thrust = thrust_N / m if m > 0 else 0.0
        a_grav = grav.gravitational_acceleration(r_val)
        a_drag = (q[i] * r.C_D * r.A) / m if m > 0 else 0.0
        
        # Get angle of attack for thrust direction
        alpha = alpha_for_losses[i]
        
        # Total acceleration along velocity direction (from equations of motion)
        # dvdt = (F_T/m)*cos(alpha) - F_D/m - g*sin(gamma)
        a_total = a_thrust * np.cos(alpha) - a_drag - a_grav * np.sin(gamma)
        
        thrust_accel_arr.append(a_thrust)
        drag_accel_arr.append(a_drag)
        grav_accel_arr.append(a_grav)
        total_accel_arr.append(a_total)
    
    # Convert to numpy arrays
    thrust_accel_arr = np.array(thrust_accel_arr)
    drag_accel_arr = np.array(drag_accel_arr)
    grav_accel_arr = np.array(grav_accel_arr)
    total_accel_arr = np.array(total_accel_arr)
    
    # Convert to G's for better readability
    thrust_accel_g = thrust_accel_arr / c.G_0
    total_accel_g = total_accel_arr / c.G_0
    
    # Determine plot cutoff time (a few seconds after SECO)
    plot_buffer_after_seco = 5.0  # seconds
    if time_seco is not None:
        time_cutoff = time_seco + plot_buffer_after_seco
        # Find index where time exceeds cutoff
        idx_cutoff = np.searchsorted(time_reduced, time_cutoff)
        if idx_cutoff >= len(time_reduced):
            idx_cutoff = len(time_reduced) - 1
        
        # Slice arrays to cutoff point
        time_plot = time_reduced[:idx_cutoff+1]
        thrust_accel_g_plot = thrust_accel_g[:idx_cutoff+1]
        total_accel_g_plot = total_accel_g[:idx_cutoff+1]
    else:
        # If no SECO, plot everything
        time_plot = time_reduced
        thrust_accel_g_plot = thrust_accel_g
        total_accel_g_plot = total_accel_g
        time_cutoff = time_reduced[-1]
    
    # Find maximum acceleration (within plot range)
    #idx_max_accel = np.argmax(total_accel_g_plot)
    #time_max_accel = time_plot[idx_max_accel]
    #max_accel_g = total_accel_g_plot[idx_max_accel]
    
    # Plot total acceleration and thrust acceleration
    axs4.plot(time_plot, total_accel_g_plot, 'r-', linewidth=2.5, label='Total Accel: $(F_T/m)·cos(α) - F_D/m - g·sin(γ)$')
    axs4.plot(time_plot, thrust_accel_g_plot, 'b-', linewidth=2, label='Thrust Acceleration (F_T/m)', alpha=0.7)
    
    # Mark max acceleration
    #axs4.plot(time_max_accel, max_accel_g, 'r*', markersize=15, 
    #         label=f'Max Accel ({max_accel_g:.2f} g at {time_max_accel:.1f}s)', zorder=5)
    
    # Add vertical lines for phase transitions
    #if time_guidance is not None:
    #    axs4.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=2, alpha=0.7, 
    #                label=f'Atmosphere Exit ({time_guidance:.1f}s)')
    #time_meco = ra.time_main_engine_cutoff
    #if time_meco is not None:
    #    axs4.axvline(x=time_meco, color='magenta', linestyle='--', linewidth=2, alpha=0.7, 
    #                label=f'MECO ({time_meco:.1f}s)')
    #if time_seco is not None:
    #    axs4.axvline(x=time_seco, color='darkred', linestyle='--', linewidth=2, alpha=0.7,
    #                label=f'SECO ({time_seco:.1f}s)')
    
    # Mark the kick maneuver period
    #kick_start = sim_params.TIME_TO_START_KICK
    #kick_end = kick_start + sim_params.DURATION_INITIAL_KICK
    #axs4.axvspan(kick_start, kick_end, alpha=0.15, color='yellow', label='Pitch Kick Maneuver')
    
    # Add horizontal line at 1g for reference
    axs4.axhline(y=1.0, color='black', linestyle='--', linewidth=1.5, alpha=0.7, label='1g Reference')
    
    # Set x-axis limit to cutoff time
    axs4.set_xlim(0, time_cutoff)
    
    axs4.set_xlabel('Time [s]', fontsize=18)
    axs4.set_ylabel('Acceleration [g]', fontsize=18)
    axs4.set_title('Rocket Acceleration over Time (Powered Ascent)', fontsize=20, fontweight='bold')
    axs4.tick_params(axis='both', which='major', labelsize=14)
    axs4.legend(fontsize=14, loc='best')
    axs4.grid(True, alpha=0.3)
    
    # Add information text box
    #accel_info = f'Max Acceleration: {max_accel_g:.2f} g\nOccurs at: {time_max_accel:.1f} s'
    #if time_guidance is not None:
    #    accel_at_guidance = total_accel_g[find_closest_index(time_reduced, time_guidance)]
    #    accel_info += f'\nAccel at atmosphere exit: {accel_at_guidance:.2f} g'
    #if time_seco is not None:
    #    idx_seco_accel = find_closest_index(time_reduced, time_seco)
    #    if idx_seco_accel is not None and idx_seco_accel < len(total_accel_g):
    #        accel_at_seco = total_accel_g[idx_seco_accel]
    #        accel_info += f'\nAccel at SECO: {accel_at_seco:.2f} g'
    #axs4.text(0.98, 0.98, accel_info,
    #         transform=axs4.transAxes, fontsize=12, verticalalignment='top', horizontalalignment='right',
    #         bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.8))

    # Plot Mach number over time (atmospheric phase only)
    fig5, axs5 = plt.subplots(figsize=(12, 6))
    
    # Calculate Mach number
    mach_numbers = []
    for i in range(len(time_reduced)):
        v = data_reduced[2][i]
        alt = (data_reduced[1][i] - c.R_EARTH)
        a_sound = atm.speed_of_sound(alt)
        mach = v / a_sound
        mach_numbers.append(mach)
    
    mach_numbers = np.array(mach_numbers)
    
    # Determine atmospheric phase cutoff (atmosphere exit + 5 seconds buffer)
    atm_cutoff_buffer = 1  # seconds
    if time_guidance is not None:
        time_atm_cutoff = time_guidance + atm_cutoff_buffer
        idx_atm_cutoff = np.searchsorted(time_reduced, time_atm_cutoff)
        if idx_atm_cutoff >= len(time_reduced):
            idx_atm_cutoff = len(time_reduced) - 1
        
        # Slice arrays to atmospheric phase
        time_atm_plot = time_reduced[:idx_atm_cutoff+1]
        mach_atm_plot = mach_numbers[:idx_atm_cutoff+1]
    else:
        # If no atmosphere exit detected, use altitude threshold (65 km)
        for i, alt_km in enumerate(h):
            if alt_km > 65:
                idx_atm_cutoff = i
                break
        else:
            idx_atm_cutoff = len(time_reduced) - 1
        
        time_atm_plot = time_reduced[:idx_atm_cutoff+1]
        mach_atm_plot = mach_numbers[:idx_atm_cutoff+1]
        time_atm_cutoff = time_atm_plot[-1]
    
    # Find maximum Mach number in atmospheric phase
    #idx_max_mach = np.argmax(mach_atm_plot)
    #time_max_mach = time_atm_plot[idx_max_mach]
    #max_mach = mach_atm_plot[idx_max_mach]
    
    # Plot Mach number
    axs5.plot(time_atm_plot, mach_atm_plot, 'b-', linewidth=2.5, label='Mach Number')
    
    # Add vertical lines for phase transitions
    time_meco = ra.time_main_engine_cutoff
    if time_meco is not None:
        axs5.axvline(x=time_meco, color='red', linestyle='--', linewidth=2, alpha=1, 
                    label=f'MECO ({time_meco:.1f}s)')
    
    # Mark max Mach
    #axs5.plot(time_max_mach, max_mach, 'r*', markersize=15, 
    #         label=f'Max Mach ({max_mach:.2f} at {time_max_mach:.1f}s)', zorder=5)
    
    # Add horizontal line at Mach 1 (transonic)
    axs5.axhline(y=1.0, color='black', linestyle='--', linewidth=2, alpha=0.7, label='Mach 1 (Transonic)')
    
    # Add vertical lines for phase transitions (within atmospheric phase)
    #if time_guidance is not None and time_guidance <= time_atm_cutoff:
    #    axs5.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=2, alpha=0.7, 
    #                label=f'Atmosphere Exit ({time_guidance:.1f}s)')
    #time_meco = ra.time_main_engine_cutoff
    #if time_meco is not None and time_meco <= time_atm_cutoff:
    #    axs5.axvline(x=time_meco, color='magenta', linestyle='--', linewidth=2, alpha=0.7, 
    #                label=f'MECO ({time_meco:.1f}s)')
    
    # Mark the kick maneuver period
    #kick_start = sim_params.TIME_TO_START_KICK
    #kick_end = kick_start + sim_params.DURATION_INITIAL_KICK
    #if kick_end <= time_atm_cutoff:
    #    axs5.axvspan(kick_start, kick_end, alpha=0.15, color='yellow', label='Pitch Kick Maneuver')
    
    # Add dynamic pressure threshold line if using it for atmosphere exit
    if sim_params.ATMOSPHERE_EXIT_METHOD == "dynamic_pressure":
        # Find when Mach crosses threshold (informational)
        pass
    
    # Set x-axis limit to atmospheric cutoff time
    axs5.set_xlim(0, time_atm_cutoff)
    
    axs5.set_xlabel('Time [s]', fontsize=18)
    axs5.set_ylabel('Mach Number', fontsize=18)
    axs5.set_title('Mach Number during Atmospheric Phase', fontsize=20, fontweight='bold')
    axs5.tick_params(axis='both', which='major', labelsize=14)
    axs5.legend(fontsize=14, loc='best')
    axs5.grid(True, alpha=0.3)
    
    # Add information text box
    #mach_info = f'Max Mach: {max_mach:.2f}\nOccurs at: {time_max_mach:.1f} s'
    # Find when vehicle goes supersonic (Mach > 1)
    #supersonic_idx = np.where(mach_atm_plot > 1.0)[0]
    #if len(supersonic_idx) > 0:
    #    time_supersonic = time_atm_plot[supersonic_idx[0]]
    #    mach_info += f'\nGoes supersonic at: {time_supersonic:.1f} s'
    #if time_guidance is not None and time_guidance <= time_atm_cutoff:
    #    idx_guidance_mach = find_closest_index(time_atm_plot, time_guidance)
    #    if idx_guidance_mach is not None:
    #        mach_at_exit = mach_atm_plot[idx_guidance_mach]
    #        mach_info += f'\nMach at atm exit: {mach_at_exit:.2f}'
    #axs5.text(0.98, 0.98, mach_info,
    #         transform=axs5.transAxes, fontsize=12, verticalalignment='top', horizontalalignment='right',
    #         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    # COMMENTED OUT: Angle of Attack (Steering Angle) over Time plot
    # fig6, axs6 = plt.subplots(figsize=(12, 6))
    
    # # Convert angle of attack to degrees for plotting
    # alpha_deg = np.rad2deg(angle_of_attacks)
    
    # # Plot alpha
    # axs6.plot(time_reduced, alpha_deg, 'b-', linewidth=2, label='Angle of Attack (α)')
    
    # # Add phase transition markers
    # if time_guidance is not None and idx_guidance is not None:
    #     axs6.axvline(x=time_guidance, color='cyan', linestyle='--', linewidth=2, alpha=0.7, 
    #                 label=f'Atmosphere Exit ({time_guidance:.1f}s)')
    # if time_seco is not None and idx_seco is not None:
    #     axs6.axvline(x=time_seco, color='red', linestyle='--', linewidth=2, alpha=0.7,
    #                 label=f'SECO ({time_seco:.1f}s)')
    
    # # Mark the kick maneuver period
    # kick_start = sim_params.TIME_TO_START_KICK
    # kick_end = kick_start + sim_params.DURATION_INITIAL_KICK
    # axs6.axvspan(kick_start, kick_end, alpha=0.2, color='yellow', label='Pitch Kick Maneuver')
    
    # # Add horizontal line at zero
    # axs6.axhline(y=0, color='k', linestyle=':', linewidth=1, alpha=0.5)
    
    # axs6.set_xlabel('Time [s]', fontsize=11)
    # axs6.set_ylabel('Angle of Attack [deg]', fontsize=11)
    # axs6.set_title('Angle of Attack (Steering Angle) over Time', fontsize=12, fontweight='bold')
    # axs6.legend(fontsize=10, loc='best')
    # axs6.grid(True, alpha=0.3)
    
    # # Add annotation about guidance phases
    # axs6.text(0.02, 0.98, 
    #          'Phase 1: Pitch Kick\nPhase 2: Gravity Turn (α=0)\nPhase 3: Active Guidance',
    #          transform=axs6.transAxes, fontsize=9, verticalalignment='top',
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
    ax.set_facecolor("white")  # Set background to white for better contrast with Earth and trajectory
    ax.set_aspect('equal', adjustable='datalim')  # Keep aspect ratio

    # Plot trajectory
    ax.plot(x, y, color="black", linewidth=1, label="Rocket Trajectory")
    
    # Add phase transition markers (numbered)
    if idx_guidance is not None:
        ax.plot(x[idx_guidance], y[idx_guidance], 'o', color='cyan', markersize=12, 
               markeredgecolor='white', markeredgewidth=1, zorder=5)
        ax.text(x[idx_guidance], y[idx_guidance], '1', color='white', fontsize=10, 
               fontweight='bold', ha='center', va='center', zorder=6)
    if idx_seco is not None:
        ax.plot(x[idx_seco], y[idx_seco], 'o', color='red', markersize=12, 
               markeredgecolor='white', markeredgewidth=1, zorder=5)
        ax.text(x[idx_seco], y[idx_seco], '2', color='white', fontsize=10, 
               fontweight='bold', ha='center', va='center', zorder=6)
    if idx_insertion is not None:
        ax.plot(x[idx_insertion], y[idx_insertion], 'o', color='green', markersize=12, 
               markeredgecolor='white', markeredgewidth=1, zorder=5)
        ax.text(x[idx_insertion], y[idx_insertion], '3', color='white', fontsize=10, 
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
    earth = plt.Circle((0, 0), earth_radius_km, color='blue', alpha=0.5, zorder=1)
    
    # Show Earth
    ax.add_patch(earth)

    # Labels and aesthetics
    ax.set_xlabel("Downrange Distance [km]", color="white", fontsize=12)
    ax.set_ylabel("Altitude [km]", color="white", fontsize=12)
    ax.set_title("Rocket Trajectory", color="black", fontsize=16, fontweight='bold')
    ax.tick_params(colors='white', labelsize=12)
    #ax.grid(color='gray', linestyle='--', linewidth=0.5)
    
    # Add legend with styling for black background
    legend = ax.legend(handles=legend_elements, loc='upper right', fontsize=12, 
                      facecolor='black', edgecolor='white', framealpha=0.8)
    for text in legend.get_texts():
        text.set_color('white')

    # Set limits to make sure Earth is fully visible
    ax.set_xlim(min(x) - 1200, max(x) + 1200)  # Adjust margins around trajectory
    ax.set_ylim(min(y) - 1200, max(y) + 1200)
    ax.set_aspect('equal')  # Keep aspect ratio realistic

    # plt.savefig("rocket_trajectory.jpg", dpi=1000, bbox_inches="tight", pad_inches=0)
    plt.show(block=False)
