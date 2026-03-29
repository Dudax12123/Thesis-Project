import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_propellant_mass_over_time(time_steps, data, save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    m_prop = psu.compute_propellant_mass(channels['m'], time_steps=time_steps)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(time_steps, m_prop, linewidth=2.0, color='tab:green', label='Propellant Mass')
    ax.set_title('Propellant Mass Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Propellant Mass [kg]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
