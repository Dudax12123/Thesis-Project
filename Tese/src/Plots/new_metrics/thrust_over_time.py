import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_thrust_over_time(time_steps, thrust_data, time_thrust, save_path=None, show=False):
    thrust_interp = psu.interpolate_to_time(time_thrust, thrust_data, time_steps) / 1000.0

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(time_steps, thrust_interp, linewidth=2.0, color='tab:red', label='Thrust')
    ax.set_title('Thrust Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Thrust [kN]')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
