import numpy as np
import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu
from Auxiliary import rocket_specs as r
from Auxiliary import constants as c


def plot_mass_flow_rate_over_time(time_steps, thrust_data, time_thrust,
                                  save_path=None, show=False):
    thrust_interp = psu.interpolate_to_time(time_thrust, thrust_data, time_steps)

    # Determine Isp per time step from thrust level:
    #   Stage 1 thrust ~ 7.6 MN  →  ISP_1
    #   Stage 2 thrust ~ 934 kN  →  ISP_2
    #   Coast (thrust ≈ 0)       →  ISP_2 (mass flow is zero anyway)
    threshold = (r.F_THRUST_1 + r.F_THRUST_2) / 2.0
    isp = np.where(thrust_interp > threshold, r.ISP_1_SL, r.ISP_2)

    mdot = thrust_interp / (isp * c.G_0)

    # Truncate at SECO
    idx = psu.cutoff_index(time_steps, psu.event_times().get('seco'))
    t_plot = time_steps[:idx]
    mdot_plot = mdot[:idx]

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t_plot, mdot_plot, linewidth=2.0, color='tab:cyan', label='Mass Flow Rate')
    ax.set_title('Nozzle Mass Flow Rate Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Mass Flow Rate [kg/s]')
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
