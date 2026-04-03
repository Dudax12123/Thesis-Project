import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_dynamic_pressure_over_time(time_steps, data, save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    q = psu.compute_dynamic_pressure(channels['v'], channels['alt'])

    # Truncate at SECO — dynamic pressure is negligible in vacuum coast
    idx = psu.cutoff_index(time_steps, psu.event_times().get('seco'))
    t_plot = time_steps[:idx]
    q_plot = q[:idx]

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t_plot, q_plot / 1000.0, linewidth=2.0, color='tab:purple', label='Dynamic Pressure')
    ax.set_title('Dynamic Pressure Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('q [kPa]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
