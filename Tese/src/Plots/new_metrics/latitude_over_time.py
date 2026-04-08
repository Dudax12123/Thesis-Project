import matplotlib.pyplot as plt
import numpy as np

from Plots import plot_state_utils as psu


def plot_latitude_over_time(time_steps, data, save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    if channels['lat'] is None:
        return None

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(time_steps, np.rad2deg(channels['lat']), linewidth=2.0, color='tab:green', label='Latitude')
    ax.set_title('Geocentric Latitude Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Latitude [deg]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
