import matplotlib.pyplot as plt
import numpy as np

# compute_direct_objective's value for a trajectory that crashed or never flew
_FAILED = 1e19


def plot_direct_grid_landscape(gamma_p, J, t_burn_pct=None, gamma_best=None,
                               zoom_halfwidth_rad=0.004, save_path=None, show=False):
    """The objective the direct grid optimiser searched, J against gamma_p.

    The grid-search counterpart of the PSO convergence plot: where a swarm leaves a
    best-so-far curve, the grid leaves the whole landscape. Left, the full grid on a
    log scale, with the kicks that crashed or burned out shaded; right, a linear zoom
    around the optimum, which is where the roughness Brent has to cope with shows.
    ``t_burn_pct`` (nested search only: the best burn at each gamma_p) is drawn on a
    second axis in the zoom.

    Parameters
    ----------
    gamma_p    : array-like  Grid of pitch-manoeuvre angles [rad].
    J          : array-like  Objective at each grid point.
    t_burn_pct : array-like or None  Inner optimum burn [% of T_MAX_2] per grid point.
    gamma_best : float or None  The optimiser's final gamma_p (after Brent) [rad].
    """
    g = np.asarray(gamma_p, dtype=float)
    J = np.asarray(J, dtype=float)
    ok = J < _FAILED
    deg = np.degrees(g)

    fig, (ax_full, ax_zoom) = plt.subplots(1, 2, figsize=(14, 6))

    ax_full.semilogy(deg[ok], J[ok], linewidth=1.5, label="J (flies)")
    if (~ok).any():
        bad = deg[~ok]
        ax_full.axvspan(bad.min(), bad.max(), color="0.85", zorder=0,
                        label=f"crash / burnout ({(~ok).sum()} of {len(g)} points)")
    i_best = int(np.argmin(J))
    x_star = np.degrees(gamma_best) if gamma_best is not None else deg[i_best]
    ax_full.axvline(x_star, color="C3", linestyle="--", linewidth=1.2,
                    label=f"optimum {x_star:.4f} deg")
    ax_full.set_title("Direct grid search: objective over the whole grid")
    ax_full.set_xlabel(r"$\gamma_p$  [deg]")
    ax_full.set_ylabel("J  (log scale)")
    ax_full.grid(True, which="both", alpha=0.3)
    ax_full.legend()

    centre = gamma_best if gamma_best is not None else g[i_best]
    near = ok & (np.abs(g - centre) <= zoom_halfwidth_rad)
    ax_zoom.plot(deg[near], J[near], marker="o", markersize=2.5, linewidth=1.0,
                 label="J (grid points)")
    ax_zoom.axvline(x_star, color="C3", linestyle="--", linewidth=1.2, label="optimum")
    ax_zoom.set_title(rf"Around the optimum ($\pm${np.degrees(zoom_halfwidth_rad):.2f} deg)")
    ax_zoom.set_xlabel(r"$\gamma_p$  [deg]")
    ax_zoom.set_ylabel("J")
    ax_zoom.grid(True, alpha=0.3)
    handles, labels = ax_zoom.get_legend_handles_labels()
    if t_burn_pct is not None:
        tb = np.asarray(t_burn_pct, dtype=float)
        ax_tb = ax_zoom.twinx()
        ax_tb.plot(deg[near], tb[near], color="C2", linewidth=1.0, alpha=0.8,
                   label="best burn at this kick")
        ax_tb.set_ylabel(r"$t_{burn}$  [% of $T_{max,2}$]")
        h2, l2 = ax_tb.get_legend_handles_labels()
        handles, labels = handles + h2, labels + l2
    ax_zoom.legend(handles, labels)
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
    if show:
        plt.show(block=False)
    return fig
