import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu


def plot_rocket_accelerations_over_time(time_steps, data, thrust_data, time_thrust, save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    acc = psu.compute_acceleration_components(time_steps, channels, thrust_data=thrust_data, time_thrust=time_thrust)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(time_steps, acc['total_accel'], linewidth=2.0, label='Total dv/dt')
    ax.plot(time_steps, acc['thrust_accel'], linewidth=1.8, label='Thrust accel')
    ax.plot(time_steps, -acc['drag_accel'], linewidth=1.8, label='-Drag accel')
    ax.plot(time_steps, -acc['grav_along'], linewidth=1.8, label='-Gravity along-path')
    ax.set_title('Rocket Accelerations Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Acceleration [m/s^2]')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
