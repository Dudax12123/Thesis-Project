import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_pseudo_forces_over_time(time_steps, time_thrust,
                                 coriolis_mag_data, centrifugal_mag_data,
                                 save_path=None, show=False):
    """Plot Coriolis and centrifugal acceleration magnitudes."""
    cor_interp = psu.interpolate_to_time(time_thrust, coriolis_mag_data, time_steps)
    cent_interp = psu.interpolate_to_time(time_thrust, centrifugal_mag_data, time_steps)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(time_steps, cor_interp, linewidth=1.5, label='Coriolis')
    ax.plot(time_steps, cent_interp, linewidth=1.5, label='Centrifugal')
    ax.set_title('Pseudo-Force Acceleration Magnitudes')
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
