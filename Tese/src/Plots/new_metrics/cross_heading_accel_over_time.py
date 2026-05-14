import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_cross_heading_accel_over_time(time_thrust, cross_heading_accel_data,
                                       save_path=None, show=False):
    t, a = psu.reduce_data(time_thrust, cross_heading_accel_data.reshape(1, -1),
                           reduction_factor=5)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, a[0], linewidth=2.0, color='purple',
            label='Cross-Heading Accel.')
    ax.set_title('Cross-Heading Pseudo-Force Acceleration Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Acceleration [m/s²]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
