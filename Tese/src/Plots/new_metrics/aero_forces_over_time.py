import matplotlib.pyplot as plt

from Plots import plot_state_utils as psu
from Input_File import simulation_parameters as sim_params

# Hard-coded ascent aerodynamic coefficients — rocket_specs.C_D is zeroed
# during the coast phase, so we cannot read it at plot time.
_C_D_ASCENT = 0.3
_C_L_ASCENT = 0.1
_A_REF = 10.52  # cross-sectional area [m^2]


def plot_aero_forces_over_time(time_steps, data, save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    q = psu.compute_dynamic_pressure(channels['v'], channels['alt'])

    F_D = q * _C_D_ASCENT * _A_REF
    F_L = q * _C_L_ASCENT * _A_REF if sim_params.INCLUDE_LIFT else q * 0.0

    # Truncate at SECO — aero forces are negligible in vacuum coast
    idx = psu.cutoff_index(time_steps, psu.event_times().get('seco'))
    t_plot = time_steps[:idx]
    F_D_plot = F_D[:idx]
    F_L_plot = F_L[:idx]

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t_plot, F_D_plot / 1000.0, linewidth=2.0, color='tab:red', label='Drag')
    if sim_params.INCLUDE_LIFT:
        ax.plot(t_plot, F_L_plot / 1000.0, linewidth=2.0, color='tab:blue', label='Lift')
    ax.set_title('Aerodynamic Forces Over Time')
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
