import matplotlib.pyplot as plt
import numpy as np

from Plots import plot_state_utils as psu


def plot_apollo_tgo_over_time(tgo_time_data, tgo_data, freeze_threshold=None,
                               save_path=None, show=False):
    """Plot Apollo time-to-go estimate over mission elapsed time.

    Parameters
    ----------
    tgo_time_data : array-like
        Mission elapsed time stamps for each t_go sample [s]
    tgo_data : array-like
        Time-to-go estimates [s]
    freeze_threshold : float, optional
        APOLLO_FREEZE_THRESHOLD value; drawn as a reference line if provided.
    save_path : Path or str, optional
        File path to save the figure.
    show : bool
        If True, call plt.show(block=False).
    """
    t, tgo = psu.prepare_monotonic_series(tgo_time_data, tgo_data)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, tgo, color="steelblue", linewidth=2.0, label="Time-to-go")

    if freeze_threshold is not None:
        ax.axhline(
            freeze_threshold,
            color="tomato", linestyle="--", linewidth=1.2,
            label=f"Freeze threshold ({freeze_threshold:.0f} s)",
        )

    ax.set_title("Apollo Guidance — Time-to-Go Estimate over Time")
    ax.set_xlabel("Mission elapsed time [s]")
    ax.set_ylabel("Time-to-go $t_{go}$ [s]")
    ax.set_xlim(left=t[0] if len(t) > 0 else 0)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    psu.add_event_markers(ax)
    ax.legend()
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
    if show:
        plt.show(block=False)
    return fig
