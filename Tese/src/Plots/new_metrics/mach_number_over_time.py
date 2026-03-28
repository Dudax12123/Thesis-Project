import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_mach_number_over_time(time_steps, data, save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    mach = psu.compute_mach(channels['v'], channels['alt'])

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(time_steps, mach, linewidth=2.0, color='tab:brown', label='Mach Number')
    ax.set_title('Mach Number Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Mach [-]')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
