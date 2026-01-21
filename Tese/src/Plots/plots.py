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


def single_run(time_steps, data, INITIAL_KICK_ANGLE):
    """
    Inputs:
        - time_steps: array of time steps (for the data array); [s]
        - data: array of data points. The data array has the following structure:
            * data[0]: downtrack s; [m]
            * data[1]: current radius r from Earth's center; [m]
            * data[2]: velocity norm; [m/s]
            * data[3]: flight path angle; [rad]
            * data[4]: mass of the rocket; [kg]

    Plots the following data over time:
        - altitude over downtrack
        - downtrack over time
        - altitude over time
        - velocity norm  over time
        - flight path angle (gamma) over time
        - mass of the rocket over time
        - dynamic pressure over time (based on velocity norm)
        - angle of attack over time
    """

    # Reduce data array
    data_reduced = data[:, ::10]
    time_reduced = time_steps[::10]

    # -------------- Prepare data --------------
    h = (data_reduced[1] - c.R_EARTH) / 1000.       # altitude h; [km]
    s = data_reduced[0] / 1000.                     # downtrack s; [km]

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
            
            # Debug output for first few iterations
            if i < 3:
                print(f"\nIteration {i}:")
                print(f"  Time: {time_reduced[i]:.2f} s")
                print(f"  Altitude: {(current_radius - c.R_EARTH)/1000:.2f} km")
                print(f"  Velocity: {data_reduced[2][i]:.2f} m/s")
                print(f"  Dynamic pressure q: {q[i]:.2f} Pa")
                print(f"  Mass: {current_mass:.2f} kg")
                print(f"  Drag accel: {drag_accel:.6f} m/s^2")
                print(f"  Gravity accel component: {grav_accel * np.sin(current_theta):.6f} m/s^2")

            grav_loss.append(grav_accel * np.sin(current_theta) * (time_reduced[i] - time_reduced[i-1]) + grav_loss[i-1] if i > 0 else 0.0)
            drag_loss.append(drag_accel * (time_reduced[i] - time_reduced[i-1]) + drag_loss[i-1] if i > 0 else 0.0)

            time_loss.append(time_reduced[i])


    # -------------- Plotting --------------
    fig1, axs1 = plt.subplots(2, 4, figsize=(15, 15))

    # Position plot: r over s
    axs1[0, 0].plot(s, h)
    axs1[0, 0].set_xlabel('downtrack s [km]')
    axs1[0, 0].set_ylabel('altitude h [km]')
    axs1[0, 0].set_title('Trajectory of Rocket')
    axs1[0, 0].grid()

    # Position plot: downtrack over time
    axs1[0, 1].plot(time_reduced, s)
    axs1[0, 1].set_xlabel('time [s]')
    axs1[0, 1].set_ylabel('downtrack s [km]')
    axs1[0, 1].set_title('Downtrack over Time')
    axs1[0, 1].grid()

    # Position plot: y over time
    axs1[0, 2].plot(time_reduced, h)
    axs1[0, 2].set_xlabel('time [s]')
    axs1[0, 2].set_ylabel('altitude h [km]')
    axs1[0, 2].set_title('Altitude over Time')
    axs1[0, 2].grid()

    # Velocity plot
    axs1[0, 3].plot(time_reduced, data_reduced[2])
    axs1[0, 3].set_xlabel('time [s]')
    axs1[0, 3].set_ylabel('v [m/s]')
    axs1[0, 3].set_title('Velocity Norm over Time')
    axs1[0, 3].grid()

    # Flight path angle plot
    axs1[1, 0].plot(time_reduced, np.rad2deg(data_reduced[3]))
    axs1[1, 0].set_xlabel('time [s]')
    axs1[1, 0].set_ylabel('gamma [rad]')
    axs1[1, 0].set_title('Flight Path Angle over Time')
    axs1[1, 0].grid()

    # Mass plot
    axs1[1, 1].plot(time_reduced, data_reduced[4])
    axs1[1, 1].set_xlabel('time [s]')
    axs1[1, 1].set_ylabel('mass [kg]')
    axs1[1, 1].set_title('Mass of Rocket over Time')
    axs1[1, 1].grid()

    # Dynamic Pressure plot
    axs1[1, 2].plot(time_reduced, q)
    axs1[1, 2].set_xlabel('time [s]')
    axs1[1, 2].set_ylabel('q [Pa]')
    axs1[1, 2].set_title('Dynamic Pressure over Time')
    axs1[1, 2].grid()

    # Angle of Attack plot
    axs1[1, 3].plot(time_reduced, np.rad2deg(angle_of_attacks))
    axs1[1, 3].set_xlabel('time [s]')
    axs1[1, 3].set_ylabel('angle of attack [deg]')
    axs1[1, 3].set_title('Angle of Attack over Time')
    axs1[1, 3].grid()


    if ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL is not None:
        # Plot the gravity loss, the drag loss and the total loss in one plot
        fig2, axs2 = plt.subplots(figsize=(10, 5))
        axs2.plot(time_loss, grav_loss, label="Gravity Loss", color="blue")
        axs2.plot(time_loss, drag_loss, label="Drag Loss", color="orange")
        axs2.plot(time_loss, np.array(grav_loss) + np.array(drag_loss), label="Total Loss", color="red")
        axs2.set_xlabel('time [s]')
        axs2.set_ylabel('loss [m/s]')
        axs2.set_title('Losses over Time')
        axs2.legend()
        axs2.grid()
        
        print("\nLosses:")
        print("\t* Gravity loss:\t\t\t\t\t", grav_loss[-1], "m/s")
        print("\t* Drag loss:\t\t\t\t\t", drag_loss[-1], "m/s")
        print("\t* Total loss:\t\t\t\t\t", grav_loss[-1] + drag_loss[-1], "m/s")
        print("\n\n")

    plt.tight_layout()
    plt.show()
    
    
    
def plot_trajectory_xy(data):
    """
    Plots the rocket trajectory in x-y coordinates with Earth shown as a blue disk.
    
    Inputs:
        - data: array of data points with the following structure:
            * data[0]: downtrack s; [m]
            * data[1]: current radius r from Earth's center; [m]
    """
    # Reduce data_reduced array
    data_reduced = data[:, ::10]

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
    ax.legend()

    # Set limits to make sure Earth is fully visible
    ax.set_xlim(min(x) - 1200, max(x) + 1200)  # Adjust margins around trajectory
    ax.set_ylim(min(y) - 1200, max(y) + 1200)
    ax.set_aspect('equal')  # Keep aspect ratio realistic

    # plt.savefig("rocket_trajectory.jpg", dpi=1000, bbox_inches="tight", pad_inches=0)
    plt.show()
