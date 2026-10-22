"""Figures of the Extended Abstract, drawn from the same results set as Chapter 6.

The abstract is a two-column article at 10 pt, the thesis body size, so these
figures keep every label at 10 pt; they differ from the chapter's in what they
show, never in type size. Each writes an ``ea_*.png`` beside the thesis
figures, which it leaves untouched (float review X-0, 2026-10-22).

    python -m Plots.results_figures.make_all --abstract --root <results set>
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from . import _panels as pn
from . import _style as st
from . import sec63_peg as sec63

# One column of the abstract: 160 mm text width less the 10 pt column gap, halved.
COLUMN_IN = 3.08
FS = 10.0


def _skip(what, missing):
    print("  skip %-34s missing: %s" % (what, ", ".join(missing)))
    return None


def ea_reference_profile(cases):
    """The atmospheric reference against time, in one column (review F-2, second pass).

    The chapter's panel (a): altitude, flight-path angle, thrust acceleration,
    dynamic pressure and Mach on five parallel y-axes over one compressed time
    axis. To fit one column at 10 pt the y-axes carry tick values only, each
    spine in its quantity's colour, and the quantities are named in a colour
    legend below the plot. The coast is shaded, MECO and both cut-offs dotted,
    and the waypoint marked on the altitude.
    """
    from Auxiliary import constants as const

    if "pmp_baseline" not in cases:
        return _skip("ea reference profile", ["pmp_baseline"])
    case = cases["pmp_baseline"]
    end = case.insertion_index()
    t = case.time[:end]
    alt_exit = float((case.manifest.get("config") or {}).get("ALT_NO_ATMOSPHERE", 65e3)) / 1e3
    crossed = np.where(case.alt_km[:end] >= alt_exit)[0]
    in_atm = t <= (t[crossed[0]] if crossed.size else t[-1])
    series = {
        "alt": case.alt_km[:end],
        "gamma": case.gamma_deg[:end],
        "accel": case.thrust[:end] / case.mass[:end] / const.G_0,
        "q": np.where(in_atm, case.q[:end] / 1e3, np.nan),
        "mach": np.where(in_atm, case.mach[:end], np.nan),
    }
    spec = {k: (c, lim) for k, _l, c, lim in sec63.PROFILE_AXES}
    # Every axis starts at zero on the same baseline, and the top ticks of
    # altitude, flight-path angle and dynamic pressure sit at one height.
    spec["gamma"] = (spec["gamma"][0], (0.0, 90.0 * 540.0 / 500.0))
    spec["q"] = (spec["q"][0], (0.0, 60.0 * 540.0 / 500.0))
    names = {"alt": "Altitude [km]", "gamma": r"Flight-path angle $\gamma$ [deg]",
             "accel": r"Thrust accel. $T/(m g_0)$ [g]", "q": "Dynamic pressure [kPa]",
             "mach": "Mach"}
    # Two ticks per axis, its zero and its top, so the axes can sit close.
    ticks = {"alt": [0, 500], "gamma": [0, 90], "accel": [0, 6],
             "q": [0, 60], "mach": [0, 12]}
    # Spine positions in axes fractions: two on the left, three on the right.
    spines = {"gamma": ("left", -0.135), "accel": ("right", 1.0),
              "q": ("right", 1.075), "mach": ("right", 1.18)}

    fig = plt.figure(figsize=(COLUMN_IN, 3.62))
    ax = fig.add_axes([0.150, 0.475, 0.670, 0.51])
    hosts = {"alt": ax}
    for key in ("gamma", "accel", "q", "mach"):
        side, off = spines[key]
        tw = ax.twinx()
        tw.spines["right" if side == "left" else "left"].set_visible(False)
        tw.spines["top"].set_visible(False)
        tw.spines[side].set_visible(True)
        tw.spines[side].set_position(("axes", off))
        tw.yaxis.set_ticks_position(side)
        hosts[key] = tw
    legend = []
    for key, host in hosts.items():
        colour, lim = spec[key]
        tt, yy = st.thin(t, series[key])
        host.plot(tt, yy, color=colour, linewidth=1.15)
        host.set_ylim(*lim)
        host.set_yticks(ticks[key])
        host.tick_params(axis="y", colors=colour, labelcolor=colour, labelsize=FS, pad=1.0, length=2)
        side = "left" if key in ("alt", "gamma") else "right"
        host.spines[side].set_color(colour)
        host.spines[side].set_linewidth(1.2)
        legend.append(Line2D([], [], color=colour, linewidth=1.6, label=names[key]))
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    burns = pn.burn_intervals(case, stage2_only=True)
    coasts = case.coast_intervals()
    for s0, s1 in coasts:
        ax.axvspan(s0, s1, color=st.FAINT, alpha=0.45, linewidth=0)
    events = [(case.t_meco, "MECO")] + [(b[1], "SECO %d" % (i + 1)) for i, b in enumerate(burns)]
    for t_evt, text in events:
        if t_evt is None:
            continue
        ax.axvline(t_evt, color=st.INK, linestyle=":", linewidth=0.8)
        ax.annotate(text, xy=(t_evt, 1.0), xycoords=("data", "axes fraction"),
                    xytext=(-2, -3), textcoords="offset points", fontsize=FS,
                    color=st.INK, rotation=90, ha="right", va="top")
    wp = pn.waypoint(case)
    if wp is not None:
        pn.mark_waypoint(ax, wp, label="Coast-start waypoint")

    t_end = float(t[-1])
    c0, c1 = max(coasts, key=lambda s: s[1] - s[0])
    cut0, cut1 = c0 + sec63._COAST_MARGIN_S[0], c1 - sec63._COAST_MARGIN_S[1]
    ax.set_xlim(0.0, t_end + 10.0)
    ax.set_xscale("function", functions=pn.compressed_time(cut0, cut1, sec63._COAST_WIDTH_S))
    xt = list(np.arange(0.0, cut0, 200.0))
    xt.append(max(np.round(t_end / 10.0) * 10.0, np.ceil(cut1 / 10.0) * 10.0))
    ax.set_xticks(xt)
    ax.set_xticklabels(["%.0f" % v for v in xt])
    ax.tick_params(axis="x", labelsize=FS)
    ax.set_xlabel("Time [s]", fontsize=FS, labelpad=2)
    for cut in (cut0, cut1):
        ax.annotate("//", xy=(cut, 0.0), xycoords=("data", "axes fraction"),
                    ha="center", va="center", fontsize=FS, color=st.INK,
                    bbox={"boxstyle": "square,pad=0.05", "fc": "white", "ec": "none"})
    ax.grid(True, color=st.FAINT, linewidth=0.5, alpha=0.8)
    ax.set_axisbelow(True)

    legend += [Patch(color=st.FAINT, alpha=0.45, label="Coast (%.0f s)" % (c1 - c0)),
               ax.get_legend_handles_labels()[0][-1]]
    fig.legend(handles=legend, loc="lower center", ncol=1, fontsize=FS, frameon=False,
               bbox_to_anchor=(0.5, -0.01), handlelength=1.6, labelspacing=0.25,
               borderaxespad=0.1)
    return st.save(fig, "ea_reference_profile.png")


FIGURES = [ea_reference_profile]
