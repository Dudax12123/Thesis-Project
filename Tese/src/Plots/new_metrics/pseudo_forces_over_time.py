import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_pseudo_forces_over_time(time_steps, time_thrust,
                                 coriolis_mag_data, centrifugal_mag_data,
                                 save_path=None, show=False):
    """Plot Coriolis and centrifugal acceleration magnitudes."""
    cor_interp = psu.interpolate_to_time(time_thrust, coriolis_mag_data, time_steps)
    cent_interp = psu.interpolate_to_time(time_thrust, centrifugal_mag_data, time_steps)

    # Truncate at SECO — pseudo-forces are zero after ECI transition
    idx = psu.cutoff_index(time_steps, psu.event_times().get('seco'))
    t_plot = time_steps[:idx]
    cor_plot = cor_interp[:idx]
    cent_plot = cent_interp[:idx]

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t_plot, cor_plot, linewidth=1.5, label='Coriolis')
    ax.plot(t_plot, cent_plot, linewidth=1.5, label='Centrifugal')
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
