import matplotlib.pyplot as plt
import numpy as np

from Plots import plot_state_utils as psu


def plot_cross_heading_counter_force_over_time(time_thrust, counter_force_data, save_path=None, show=False):
    t, f = psu.reduce_data(time_thrust, counter_force_data.reshape(1, -1), reduction_factor=5)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, f[0] / 1e3, linewidth=2.0, label='Cross-Heading Counter Force')
    ax.set_title('Cross-Heading Counter Force Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Force [kN]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
