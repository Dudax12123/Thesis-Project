import matplotlib.pyplot as plt
import numpy as np

from Plots import plot_state_utils as psu


def plot_steering_angle_over_time(alpha_data, alpha_time_data, save_path=None, show=False):
    t, a = psu.prepare_monotonic_series(alpha_time_data, alpha_data)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, np.rad2deg(a), linewidth=2.0, label='Steering Angle (α)')
    ax.axhline(0.0, color='k', linestyle='--', alpha=0.4)
    ax.set_title('Steering Angle Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Steering Angle [deg]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
