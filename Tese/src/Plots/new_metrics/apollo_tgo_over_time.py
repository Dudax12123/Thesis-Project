import matplotlib.pyplot as plt
import numpy as np

from Plots import plot_state_utils as psu


def _break_across_coasts(t, tgo, thrust_time, thrust):
    """Insert a NaN between consecutive t_go samples with an engine-off sample
    strictly between them, so the line is not drawn across a coast. t_go is only
    logged while a burn is guided, so without this the last sample of one burn is
    joined to the first of the next by a straight ramp through the coast."""
    tt, ff = psu.prepare_monotonic_series(thrust_time, thrust)
    if len(t) < 2 or len(tt) == 0:
        return t, tgo
    n_off = np.concatenate([[0], np.cumsum(np.asarray(ff, dtype=float) <= 0.0)])
    lo = np.searchsorted(tt, t[:-1], side="right")
    hi = np.searchsorted(tt, t[1:], side="left")
    gaps = np.nonzero(n_off[hi] - n_off[lo] > 0)[0]
    if len(gaps) == 0:
        return t, tgo
    t = np.insert(np.asarray(t, dtype=float), gaps + 1, np.nan)
    tgo = np.insert(np.asarray(tgo, dtype=float), gaps + 1, np.nan)
    return t, tgo


def plot_apollo_tgo_over_time(tgo_time_data, tgo_data, freeze_threshold=None,
                               save_path=None, show=False, thrust_time=None, thrust=None):
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
    thrust_time, thrust : array-like, optional
        The thrust channel. When given, the line is broken across every coast
        instead of being drawn straight through it.
    """
    t, tgo = psu.prepare_monotonic_series(tgo_time_data, tgo_data)
    x_left = t[0] if len(t) > 0 else 0
    if thrust_time is not None and thrust is not None:
        t, tgo = _break_across_coasts(t, tgo, thrust_time, thrust)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, tgo, color="steelblue", linewidth=2.0, label="Time-to-go")
    # A burn guided for a single cycle leaves one sample, which a line cannot show.
    fin = np.isfinite(np.asarray(tgo, dtype=float))
    lone = fin & ~np.r_[False, fin[:-1]] & ~np.r_[fin[1:], False]
    if lone.any():
        ax.plot(np.asarray(t)[lone], np.asarray(tgo)[lone], "o", color="steelblue",
                markersize=5)

    if freeze_threshold is not None:
        ax.axhline(
            freeze_threshold,
            color="tomato", linestyle="--", linewidth=1.2,
            label=f"Freeze threshold ({freeze_threshold:.0f} s)",
        )

    ax.set_title("Apollo Guidance — Time-to-Go Estimate over Time")
    ax.set_xlabel("Mission elapsed time [s]")
    ax.set_ylabel("Time-to-go $t_{go}$ [s]")
    ax.set_xlim(left=x_left)
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
