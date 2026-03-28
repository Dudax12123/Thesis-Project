import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_altitude_over_time(time_steps, data, save_path=None, show=False):
    channels = psu.extract_state_channels(data)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(time_steps, channels['alt_km'], linewidth=2.0, color='tab:blue', label='Altitude')
    ax.set_title('Altitude Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Altitude [km]')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
