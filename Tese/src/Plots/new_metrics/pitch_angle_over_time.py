import matplotlib.pyplot as plt
import numpy as np

from Plots import plot_state_utils as psu


def plot_pitch_angle_over_time(time_steps, data, alpha_data, alpha_time_data,
                               save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    alpha_interp = psu.interpolate_to_time(alpha_time_data, alpha_data, time_steps)
    theta = channels['gamma'] + alpha_interp

    t, d = psu.reduce_data(time_steps, np.vstack([theta]), reduction_factor=5)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, np.rad2deg(d[0]), linewidth=2.0, label='Pitch Angle')
    ax.axhline(0.0,  color='k',    linestyle='--', alpha=0.4, label='0° (horizontal)')
    ax.axhline(90.0, color='grey', linestyle='--', alpha=0.4, label='90° (vertical)')
    ax.set_title('Pitch Angle Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Pitch Angle [deg]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
