import numpy as np
import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu
from Auxiliary import losses as loss_mod
from Input_File import simulation_parameters as sim_params


def plot_trajectory_losses_over_time(time_steps, data, thrust_data, time_thrust,
                                     alpha_data, alpha_time_data,
                                     save_path=None, show=False):
    channels = psu.extract_state_channels(data)

    # Truncate at SECO — losses only meaningful during powered ascent
    events = psu.event_times()
    idx = psu.cutoff_index(time_steps, events.get('seco'))
    t = time_steps[:idx]

    thrust_interp = psu.interpolate_to_time(time_thrust, thrust_data, t)
    alpha_interp = psu.interpolate_to_time(alpha_time_data, alpha_data, t)

    # The integrals themselves live in Auxiliary/losses.py so that this plot and
    # the results harness cannot drift apart, and so that the drag coefficient
    # and reference area come from rocket_specs rather than being restated here.
    hist = loss_mod.loss_histories(
        t,
        channels['alt'][:idx],
        channels['v'][:idx],
        channels['gamma'][:idx],
        channels['m'][:idx],
        thrust_interp,
        alpha_interp,
        t_meco=events.get('meco'),
        include_drag=sim_params.INCLUDE_DRAG,
        thrust_mode=sim_params.THRUST_1_MODE,
    )

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, hist['gravity'], linewidth=2.0, color='tab:blue', label='Gravity Loss')
    ax.plot(t, hist['drag'], linewidth=2.0, color='tab:orange', label='Drag Loss')
    ax.plot(t, hist['steering'], linewidth=2.0, color='tab:green', label='Steering Loss')
    if hist['pressure_applicable']:
        ax.plot(t, hist['pressure'], linewidth=2.0, color='tab:purple', label='Pressure Loss')
    ax.plot(t, hist['total'], linewidth=2.5, color='tab:red', label='Total Loss')
    ax.set_title('Trajectory Losses Over Time (Powered Ascent)')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Δv Loss [m/s]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
