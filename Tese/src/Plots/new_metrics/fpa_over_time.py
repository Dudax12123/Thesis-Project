import matplotlib.pyplot as plt
import numpy as np

from Plots import plot_state_utils as psu


def plot_fpa_over_time(time_steps, data, save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    t, d = psu.reduce_data(time_steps, np.vstack([channels['gamma']]), reduction_factor=5)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, np.rad2deg(d[0]), linewidth=2.0, label='Flight Path Angle')
    ax.axhline(0.0, color='k', linestyle='--', alpha=0.4)
    ax.set_title('Flight Path Angle Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('FPA [deg]')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
