import matplotlib.pyplot as plt
import numpy as np

from Plots import plot_state_utils as psu


def plot_theta_cmd_over_time(theta_cmd_data, theta_cmd_time_data,
                              guidance_mode=None, save_path=None, show=False):
    t, d = psu.prepare_monotonic_series(theta_cmd_time_data, theta_cmd_data)

    label = "θ_cmd (CPR)" if guidance_mode == "cpr" else "γ_cmd (CFPAR)"
    ylabel = "Commanded Pitch [deg]" if guidance_mode == "cpr" else "Commanded FPA [deg]"
    title = "Commanded Pitch Angle Over Time" if guidance_mode == "cpr" \
            else "Commanded Flight-Path Angle Over Time"

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, np.rad2deg(d), linewidth=2.0, label=label)
    ax.axhline(0.0,  color='k',    linestyle='--', alpha=0.4, label='0° (horizontal)')
    ax.axhline(90.0, color='grey', linestyle='--', alpha=0.4, label='90° (vertical)')
    ax.set_title(title)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
