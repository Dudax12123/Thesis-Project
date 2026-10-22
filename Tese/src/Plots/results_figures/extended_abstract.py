"""Figures of the Extended Abstract, drawn from the same results set as Chapter 6.

The abstract is a two-column article at 10 pt, the thesis body size, so these
figures keep every label at 10 pt; they differ from the chapter's in what they
show, never in type size. Each writes an ``ea_*.png`` beside the thesis
figures, which it leaves untouched (float review X-0, 2026-10-22).

    python -m Plots.results_figures.make_all --abstract --root <results set>
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
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
    """The atmospheric reference against time, in one column (review F-2).

    The chapter draws it as panel (a) of a three-panel figure with five parallel
    y-axes. Five axes do not fit one column at 10 pt, so the same five
    quantities are split over three stacked panels on one compressed time axis,
    two axes at most each: altitude and flight-path angle; thrust acceleration;
    dynamic pressure and Mach. The coast is shaded and MECO and both cut-offs are
    dotted in every panel; the waypoint is marked on the altitude.
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
    labels = {"alt": "Altitude [km]", "gamma": r"$\gamma$ [deg]",
              "accel": r"$T/(m g_0)$ [g]", "q": r"$q$ [kPa]", "mach": "Mach"}
    rows = (("alt", "gamma"), ("accel", None), ("q", "mach"))

    fig, panels = plt.subplots(3, 1, sharex=True, figsize=(COLUMN_IN, 4.35),
                               gridspec_kw={"height_ratios": (1.45, 0.9, 0.9)})
    fig.subplots_adjust(left=0.20, right=0.81, top=0.985, bottom=0.17, hspace=0.12)

    burns = pn.burn_intervals(case, stage2_only=True)
    coasts = case.coast_intervals()
    events = [(case.t_meco, "MECO")] + [(b[1], "SECO %d" % (i + 1)) for i, b in enumerate(burns)]
    t_end = float(t[-1])
    c0, c1 = max(coasts, key=lambda s: s[1] - s[0])
    cut0, cut1 = c0 + sec63._COAST_MARGIN_S[0], c1 - sec63._COAST_MARGIN_S[1]

    for i, (ax, (left, right)) in enumerate(zip(panels, rows)):
        for key, host in ((left, ax), (right, ax.twinx() if right else None)):
            if key is None:
                continue
            colour, lim = spec[key]
            tt, yy = st.thin(t, series[key])
            host.plot(tt, yy, color=colour, linewidth=1.15)
            host.set_ylim(*lim)
            host.set_ylabel(labels[key], color=colour, fontsize=FS)
            host.tick_params(axis="y", colors=colour, labelcolor=st.INK, labelsize=FS)
            side = "left" if host is ax else "right"
            host.spines[side].set_color(colour)
            host.spines["top"].set_visible(False)
            if host is ax:
                host.spines["right"].set_visible(False)
        if left == "alt":
            ax.set_yticks([0, 250, 500])
        if left == "accel":
            ax.set_yticks([0, 3, 6])
        for s0, s1 in coasts:
            ax.axvspan(s0, s1, color=st.FAINT, alpha=0.45, linewidth=0,
                       label="Coast" if i == 0 else None)
        for t_evt, text in events:
            if t_evt is None:
                continue
            ax.axvline(t_evt, color=st.INK, linestyle=":", linewidth=0.8)
            if i == 0:
                ax.annotate(text, xy=(t_evt, 1.0), xycoords=("data", "axes fraction"),
                            xytext=(-2, -3), textcoords="offset points", fontsize=FS,
                            color=st.INK, rotation=90, ha="right", va="top")
        ax.grid(True, color=st.FAINT, linewidth=0.5, alpha=0.8)
        ax.set_axisbelow(True)

    top = panels[0]
    wp = pn.waypoint(case)
    if wp is not None:
        pn.mark_waypoint(top, wp, label="Coast-start waypoint")

    bottom = panels[-1]
    bottom.set_xlim(0.0, t_end + 10.0)
    bottom.set_xscale("function", functions=pn.compressed_time(cut0, cut1, sec63._COAST_WIDTH_S))
    ticks = list(np.arange(0.0, cut0, 200.0))
    ticks.append(max(np.round(t_end / 10.0) * 10.0, np.ceil(cut1 / 10.0) * 10.0))
    bottom.set_xticks(ticks)
    bottom.set_xticklabels(["%.0f" % v for v in ticks])
    bottom.tick_params(axis="x", labelsize=FS)
    bottom.set_xlabel("Time [s]", fontsize=FS)
    for ax in panels:
        for cut in (cut0, cut1):
            ax.annotate("//", xy=(cut, 0.0), xycoords=("data", "axes fraction"),
                        ha="center", va="center", fontsize=FS, color=st.INK,
                        bbox={"boxstyle": "square,pad=0.05", "fc": "white", "ec": "none"})
    panels[2].annotate("coast\n%.0f s" % (c1 - c0), xy=(0.5 * (cut0 + cut1), 0.5),
                       xycoords=("data", "axes fraction"), fontsize=FS, color=st.GREY,
                       ha="center", va="center")

    handles, names = top.get_legend_handles_labels()
    fig.legend(handles, names, loc="lower center", ncol=2, fontsize=FS, frameon=False,
               bbox_to_anchor=(0.5, 0.0), handlelength=1.4, columnspacing=1.0)
    return st.save(fig, "ea_reference_profile.png")


FIGURES = [ea_reference_profile]
