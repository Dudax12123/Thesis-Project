import numpy as np
import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu
from Auxiliary import gravity as grav
from Input_File import simulation_parameters as sim_params

_C_D_ASCENT = 0.3
_A_REF = 10.52


def plot_trajectory_losses_over_time(time_steps, data, thrust_data, time_thrust,
                                     alpha_data, alpha_time_data,
                                     save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    q = psu.compute_dynamic_pressure(channels['v'], channels['alt'])

    # Truncate at SECO — losses only meaningful during powered ascent
    idx = psu.cutoff_index(time_steps, psu.event_times().get('seco'))
    t = time_steps[:idx]

    thrust_interp = psu.interpolate_to_time(time_thrust, thrust_data, t)
    alpha_interp = psu.interpolate_to_time(alpha_time_data, alpha_data, t)

    grav_loss = np.zeros(len(t))
    drag_loss = np.zeros(len(t))
    steering_loss = np.zeros(len(t))

    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        r_val = channels['r'][i]
        gamma = channels['gamma'][i]
        m = channels['m'][i]

        g = grav.gravitational_acceleration(r_val)
        drag_accel = (q[i] * _C_D_ASCENT * _A_REF) / m if sim_params.INCLUDE_DRAG else 0.0
        thrust_accel = thrust_interp[i] / m
        steer_accel = thrust_accel * (1.0 - np.cos(alpha_interp[i]))

        grav_loss[i] = grav_loss[i - 1] + g * np.sin(gamma) * dt
        drag_loss[i] = drag_loss[i - 1] + drag_accel * dt
        steering_loss[i] = steering_loss[i - 1] + steer_accel * dt

    total_loss = grav_loss + drag_loss + steering_loss

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, grav_loss, linewidth=2.0, color='tab:blue', label='Gravity Loss')
    ax.plot(t, drag_loss, linewidth=2.0, color='tab:orange', label='Drag Loss')
    ax.plot(t, steering_loss, linewidth=2.0, color='tab:green', label='Steering Loss')
    ax.plot(t, total_loss, linewidth=2.5, color='tab:red', label='Total Loss')
    ax.set_title('Trajectory Losses Over Time (Powered Ascent)')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('\u0394v Loss [m/s]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
