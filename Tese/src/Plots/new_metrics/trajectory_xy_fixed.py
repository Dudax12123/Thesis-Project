import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from Auxiliary import constants as c
from Plots import plot_state_utils as psu


def plot_trajectory_xy_fixed(time_steps, data, save_path=None, show=False):
    channels = psu.extract_state_channels(data)
    s = channels['s']
    r = channels['r']
    alt_km = channels['alt_km']
    s_km = channels['s_km']

    theta = s / c.R_EARTH
    x_km = (r * np.sin(theta)) / 1000.0
    y_km = (r * np.cos(theta)) / 1000.0

    fig, axs = plt.subplots(1, 2, figsize=(15, 6))

    axs[0].plot(s_km, alt_km, linewidth=2.0, label='Trajectory (altitude-downrange)')
    axs[0].set_title('Trajectory Profile')
    axs[0].set_xlabel('Downrange [km]')
    axs[0].set_ylabel('Altitude [km]')
    axs[0].grid(True, alpha=0.3)

    events = psu.event_times()
    for key, marker, color in [("guidance_start", '^', 'cyan'), ("meco", 'o', 'orange'), ("seco", 's', 'black')]:
        t_evt = events[key]
        if t_evt is None:
            continue
        idx = int(np.argmin(np.abs(np.asarray(time_steps) - t_evt)))
        axs[0].scatter(s_km[idx], alt_km[idx], marker=marker, color=color, s=70, zorder=5)

    earth_r_km = c.R_EARTH / 1000.0
    circle = plt.Circle((0.0, 0.0), earth_r_km, color='lightgray', fill=False, linestyle='--', linewidth=1.0)
    axs[1].add_patch(circle)
    axs[1].plot(x_km, y_km, linewidth=2.0, label='Trajectory (XY)')
    axs[1].set_title('Trajectory in XY Plane')
    axs[1].set_xlabel('X [km]')
    axs[1].set_ylabel('Y [km]')
    axs[1].axis('equal')
    axs[1].grid(True, alpha=0.3)
    axs[1].legend()

    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    if show:
        plt.show(block=False)
    return fig
