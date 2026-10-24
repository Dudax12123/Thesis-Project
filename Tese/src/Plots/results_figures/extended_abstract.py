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
    dynamic pressure and the second-stage pitch on five parallel y-axes over one
    compressed time axis. The pitch replaces the chapter's Mach (review Q-1,
    2026-10-23) and is drawn over the second-stage burns only, where it differs
    from gamma; the linear-tangent law fitted to the first burn (sec63._tangent_fit,
    as in fig:reference_tangent_fit) is dashed over it. To fit one column at 10 pt the y-axes carry tick values only, each
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
    burns = pn.burn_intervals(case, stage2_only=True)
    in_burn = np.zeros(t.size, dtype=bool)
    for b0, b1 in burns:
        in_burn |= (t >= b0) & (t <= b1)
    series = {
        "alt": case.alt_km[:end],
        "gamma": case.gamma_deg[:end],
        "accel": case.thrust[:end] / case.mass[:end] / const.G_0,
        "q": np.where(in_atm, case.q[:end] / 1e3, np.nan),
        "theta": np.where(in_burn, case.theta_deg[:end], np.nan),
    }
    spec = {k: (c, lim) for k, _l, c, lim in sec63.PROFILE_AXES}
    spec["theta"] = (sec63.OVERLAY_COLOURS["mach"], (0.0, 20.0 * 540.0 / 500.0))
    # Every axis starts at zero on the same baseline, and the top ticks of
    # altitude, flight-path angle and dynamic pressure sit at one height.
    spec["gamma"] = (spec["gamma"][0], (0.0, 90.0 * 540.0 / 500.0))
    spec["q"] = (spec["q"][0], (0.0, 60.0 * 540.0 / 500.0))
    names = {"alt": "Altitude [km]", "gamma": r"Flight-path angle $\gamma$ [deg]",
             "accel": r"Thrust accel. $T/(m g_0)$ [g]", "q": "Dynamic pressure [kPa]",
             "theta": r"Pitch $\theta$, second stage [deg]"}
    # Two ticks per axis, its zero and its top, so the axes can sit close.
    ticks = {"alt": [0, 500], "gamma": [0, 90], "accel": [0, 6],
             "q": [0, 60], "theta": [0, 20]}
    # Spine positions in axes fractions: two on the left, three on the right.
    spines = {"gamma": ("left", -0.135), "accel": ("right", 1.0),
              "q": ("right", 1.075), "theta": ("right", 1.18)}

    fig = plt.figure(figsize=(COLUMN_IN, 3.80))
    ax = fig.add_axes([0.150, 0.500, 0.670, 0.485])
    hosts = {"alt": ax}
    for key in ("gamma", "accel", "q", "theta"):
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
    long_burns = [b for b in burns if b[1] - b[0] >= sec63.MIN_FIT_S]
    tf, fitted, rms = sec63._tangent_fit(case, *long_burns[0])
    hosts["theta"].plot(tf, fitted, color=st.INK, linestyle="--", linewidth=1.0)
    legend.append(Line2D([], [], color=st.INK, linestyle="--", linewidth=1.0,
                         label=r"Linear tangent, fitted"))
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

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


def ea_segmented(cases):
    """The two segmented cases against the reference, altitude only, in one
    column (review F-3).

    The chapter's figure adds the angle of attack as panel (b); the abstract
    quotes no value from it, so only panel (a) is drawn: altitude against time,
    on a linear time axis, the waypoint and each hand-off marked.
    """
    from . import _data
    from . import sec67_capabilities as sec67

    names = ("pmp_baseline", "show_seg_fixed_alt", "show_seg_opt_alt")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("ea segmented", missing)
    ref = cases["pmp_baseline"]
    fixed, opt = cases["show_seg_fixed_alt"], cases["show_seg_opt_alt"]
    segmented_entries = [(opt, st.ACCENT, "-", st.case_label("show_seg_opt_alt")),
                         (fixed, st.ENV_COLORS["baseline"], "-",
                          st.case_label("show_seg_fixed_alt"))]
    entries = [(ref, st.INK, ":", "Reference (indirect PMP)")] + segmented_entries

    with plt.rc_context({"axes.labelsize": FS, "xtick.labelsize": FS,
                         "ytick.labelsize": FS, "axes.titlesize": FS}):
        fig, ax = plt.subplots(figsize=(COLUMN_IN, 3.3))
        pn.altitude_panel(ax, entries, shade_coasts=False)
        for curve in ax.get_lines():
            curve.set_linewidth(1.0)
        for case, colour, _style, _label in segmented_entries:
            point = sec67._handoff(case)
            if point is None:
                continue
            ax.plot(*point, linestyle="none", marker="D", markersize=4.2, color=colour,
                    zorder=5)
            ax.annotate("%.0f km" % point[1], xy=point, xytext=(5, -2),
                        textcoords="offset points", fontsize=FS, color=st.INK,
                        ha="left", va="top")
        ax.plot([], [], linestyle="none", marker="D", markersize=4.2,
                markerfacecolor="none", markeredgecolor=st.INK, label="Hand-off")
        wp = pn.waypoint(ref)
        if wp is not None:
            pn.mark_waypoint(ax, wp, label="Coast-start waypoint")
        # Linear time axis, the coast drawn at its true length (review F-3, edited).
        ax.set_xlim(0.0, float(ref.time[ref.insertion_index() - 1]) + 10.0)
        st.tidy(ax, legend=False)
        fig.subplots_adjust(left=0.18, right=0.97, top=0.98, bottom=0.44)
        handles, labels = ax.get_legend_handles_labels()
        fig.legend(handles, labels, loc="lower center", ncol=1, fontsize=FS,
                   frameon=False, bbox_to_anchor=(0.5, 0.0), labelspacing=0.25)
        return st.save(fig, "ea_segmented.png")


def ea_gravity_turn(cases):
    """The gravity turn's architecture and rotation studies, in one column
    (review P-1, edited 2026-10-24: the engine-model panel is left out).

    (a) Altitude under the coast-parameter search and the apogee check, each
    coast shaded in its flight's colour, as the chapter's fig:gt_architecture.
    (b) Altitude with and without the rotation of the Earth, the chapter's
    fig:gt_rotation (a), in its colours. Both panels share one time axis.
    """
    from . import _data
    from . import sec62_gravity_turn as sec62

    names = ("gt_baseline", "gt_apogee", "gt_norot")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("ea gravity turn", missing)
    base = cases["gt_baseline"]
    panels = (
        ("(a)", [(base, st.ENV_COLORS["baseline"], "Coast parameter", True),
                 (cases["gt_apogee"], sec62.APOGEE_COLOUR, "Apogee check", True)]),
        ("(b)", [(base, sec62.ROTATING_COLOUR, "Rotating Earth", False),
                 (cases["gt_norot"], sec62.NON_ROTATING_COLOUR, "Non-rotating Earth", False)]),
    )
    with plt.rc_context({"axes.labelsize": FS, "xtick.labelsize": FS,
                         "ytick.labelsize": FS, "axes.titlesize": FS}):
        fig, axes = plt.subplots(2, 1, figsize=(COLUMN_IN, 4.1), sharex=True)
        for ax, (title, entries) in zip(axes, panels):
            for case, colour, label, shade in entries:
                end = case.insertion_index()
                t, alt_km = st.thin(case.time[:end], case.alt_km[:end])
                ax.plot(t, alt_km, color=colour, linewidth=1.15, label=label)
                if shade:
                    for t0, t1 in case.coast_intervals():
                        ax.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
            ax.set_ylabel("Altitude [km]", labelpad=2)
            ax.set_yticks([0, 250, 500])
            ax.set_title(title, loc="left", color=st.INK, fontweight="bold", fontsize=FS, pad=2)
            st.tidy(ax, legend_loc="lower right",
                    legend_kw={"fontsize": FS, "handlelength": 1.4, "borderaxespad": 0.2,
                               "labelspacing": 0.25})
        axes[-1].set_xlabel("Time [s]", labelpad=2)
        axes[-1].set_xlim(0.0, None)
        fig.subplots_adjust(left=0.19, right=0.97, top=0.95, bottom=0.11, hspace=0.32)
        return st.save(fig, "ea_gravity_turn.png")


def ea_peg(cases):
    """Powered explicit guidance against the reference, in one column (review
    P-2, 2026-10-24): both PEG studies on the chapter's fig:peg_architectures.

    (a) Altitude against time, the reference's long coast compressed and shaded;
    (b) the commanded angle of attack. The reference is dotted black, PEG under
    the coast-parameter search red and by direct insertion blue, as in the
    chapter, and the drag-free PEG flight, the chapter's fig:peg_atmosphere,
    joins them in its environment colour. The panels are stacked, the event
    lines left out, and one legend runs under both.
    """
    from . import _data

    names = ("pmp_baseline", "peg_baseline", "peg_direct", "peg_vacuum")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("ea peg", missing)
    ref = cases["pmp_baseline"]
    entries = [
        (ref, st.INK, ":", "Reference (indirect PMP)"),
        (cases["peg_baseline"], st.ACCENT, "-", "PEG, coast-parameter search"),
        (cases["peg_direct"], st.ENV_COLORS["baseline"], "-", "PEG, direct insertion"),
        (cases["peg_vacuum"], st.ENV_COLORS["no_atmosphere"], "-", "PEG, no atmosphere"),
    ]
    with plt.rc_context({"axes.labelsize": FS, "xtick.labelsize": FS,
                         "ytick.labelsize": FS, "axes.titlesize": FS}):
        fig, (ax_a, ax_b) = plt.subplots(2, 1, figsize=(COLUMN_IN, 5.0))
        pn.altitude_panel(ax_a, entries, shade_coasts=False)
        for t0, t1 in ref.coast_intervals():
            ax_a.axvspan(t0, t1, color=st.INK, alpha=0.12, linewidth=0)
        ax_a.fill_between([], [], color=st.INK, alpha=0.12, linewidth=0,
                          label="Coast (reference)")
        pn.compress_long_coast(ax_a, ref, float(ref.time[ref.insertion_index() - 1]) + 10.0,
                               tick_step=200.0, label_y=0.12, fontsize=FS, label_box=True)
        ax_a.set_yticks([0, 250, 500])

        pn.alpha_panel(ax_b, entries, fontsize=FS, notes=False)
        for t0, t1 in ref.coast_intervals():
            ax_b.axvspan(t0, t1, color=st.INK, alpha=0.12, linewidth=0)
        ax_b.set_ylabel(r"Angle of attack $\alpha$ [deg]")
        for ax, letter in ((ax_a, "a"), (ax_b, "b")):
            for curve in ax.get_lines():
                curve.set_linewidth(1.1)
            ax.xaxis.labelpad = 2
            ax.yaxis.labelpad = 2
            st.panel_tag(ax, letter)
            st.tidy(ax, legend=False)
        fig.subplots_adjust(left=0.19, right=0.97, top=0.96, bottom=0.27, hspace=0.42)
        handles, labels = ax_a.get_legend_handles_labels()
        fig.legend(handles, labels, loc="lower center", ncol=1, fontsize=FS,
                   frameon=False, bbox_to_anchor=(0.5, 0.0), labelspacing=0.25,
                   handlelength=1.6)
        return st.save(fig, "ea_peg.png")


def ea_vacuum_coast_bound(cases):
    """The drag-free reference at the 2000 s and 3500 s coast bounds, altitude
    against downrange, in one column (review S-1, 2026-10-24): the chapter's
    fig:vacuum_coast_bound with its legend moved below the axes, the downrange
    in thousands of kilometres, and the lowest-point label between the curves.
    """
    from . import _data

    names = ("pmp_vacuum", "pmp_vacuum_c3500")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("ea vacuum coast bound", missing)
    fig, ax = plt.subplots(figsize=(COLUMN_IN, 2.55))
    styles = (("pmp_vacuum", "Coast bound 2000 s (reported)", st.REFERENCE),
              ("pmp_vacuum_c3500", "Coast bound 3500 s", st.ACCENT))
    for name, label, colour in styles:
        case = cases[name]
        _t, s_km, h_km = pn.to_insertion(case, case.downrange_km, case.alt_km)
        ax.plot(s_km / 1e3, h_km, color=colour, linewidth=1.15, label=label)

    case = cases["pmp_vacuum_c3500"]
    t, s_km, h_km = pn.to_insertion(case, case.downrange_km, case.alt_km)
    after = t > case.t_meco
    i = int(np.argmin(np.where(after, h_km, np.inf)))
    ax.plot(s_km[i] / 1e3, h_km[i], "o", color=st.ACCENT, markersize=4.0, zorder=4)
    ax.annotate("lowest point\n%.1f km" % h_km[i], xy=(s_km[i] / 1e3, h_km[i]),
                xytext=(0.215, 0.165), textcoords="axes fraction",
                fontsize=FS, color=st.ACCENT, ha="left", va="bottom", linespacing=1.0,
                arrowprops={"arrowstyle": "-", "color": st.ACCENT,
                            "linewidth": 0.7, "shrinkA": 1, "shrinkB": 3})
    target = case.row.get("target_alt_km")
    if target is not None:
        ax.axhline(target, color=st.GREY, linestyle="--", linewidth=0.8,
                   label="Target orbit, %.0f km" % target)
    ax.axhline(0.0, color=st.INK, linewidth=0.8)
    ax.set_xlabel(r"Downrange [$10^3$ km]", fontsize=FS, labelpad=2)
    ax.set_ylabel("Altitude [km]", fontsize=FS, labelpad=2)
    ax.set_yticks([0, 250, 500])
    ax.tick_params(labelsize=FS)
    st.tidy(ax, legend=False)
    fig.subplots_adjust(left=0.18, right=0.97, top=0.97, bottom=0.44)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=1, fontsize=FS, frameon=False,
               bbox_to_anchor=(0.5, 0.0), labelspacing=0.25, handlelength=1.6)
    return st.save(fig, "ea_vacuum_coast_bound.png")


FIGURES = [ea_reference_profile, ea_vacuum_coast_bound, ea_gravity_turn, ea_peg,
           ea_segmented]
