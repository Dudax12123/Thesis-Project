"""Figures of the Extended Abstract, drawn from the same results set as Chapter 6.

The abstract is a two-column article at 10 pt, the thesis body size, so these
figures keep every label at 10 pt; they differ from the chapter's in what they
show, never in type size. Each writes an ``ea_*.png`` beside the thesis
figures, which it leaves untouched (float review X-0, 2026-10-22).

    python -m Plots.results_figures.make_all --abstract --root <results set>
"""

import os
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


def save_pieces(fig, name, pieces):
    """Save a multi-panel figure as one PNG per piece, each a figure of its own
    in the abstract with its own caption and label (reviews AC-1, AD-1).

    Each piece is the union of its artists' tight boxes, cut across the full
    width of the figure, so every piece keeps the figure's width and its text
    prints at the same size whichever column it lands in. name holds one %s,
    filled with the piece's name: ea_peg_%s.png gives ea_peg_altitude.png, ..."""
    from matplotlib.transforms import Bbox

    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    full = fig.get_tightbbox(renderer)
    to_inches = fig.dpi_scale_trans.inverted()
    paths = []
    for suffix, artists in pieces:
        boxes = [a.get_tightbbox(renderer) for a in artists]
        union = Bbox.union([b for b in boxes if b is not None]).transformed(to_inches)
        box = Bbox.from_extents(full.x0, union.y0, full.x1, union.y1).padded(0.03)
        path = os.path.join(st.OUT_DIR, name % suffix)
        os.makedirs(st.OUT_DIR, exist_ok=True)
        fig.savefig(path, bbox_inches=box)
        print("  wrote %s" % path)
        paths.append(path)
    plt.close(fig)
    return paths


def ea_reference_profile(cases):
    """The atmospheric reference in one column: (a) the profile, (b) the steering
    of its first second-stage burn (review F-2, second pass; Q-1, 2026-10-23;
    T-1, 2026-10-24).

    (a) Altitude, flight-path angle, thrust acceleration and dynamic pressure on
    four parallel y-axes over one compressed time axis. To fit one column at
    10 pt the y-axes carry tick values only, each spine in its quantity's
    colour, and the quantities are named in a colour legend below the plots.
    The coast is shaded, MECO and both cut-offs dotted, and the waypoint
    marked on the altitude.
    (b) The steering angles over the first second-stage burn, where the law is
    fitted: the angle of attack, the pitch, and the linear-tangent law fitted to
    the pitch (sec63._tangent_fit, as in fig:reference_tangent_fit) dashed.
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
    series = {
        "alt": case.alt_km[:end],
        "gamma": case.gamma_deg[:end],
        "accel": case.thrust[:end] / case.mass[:end] / const.G_0,
        "q": np.where(in_atm, case.q[:end] / 1e3, np.nan),
    }
    spec = {k: (c, lim) for k, _l, c, lim in sec63.PROFILE_AXES}
    # Every axis starts at zero on the same baseline, and the top ticks of
    # altitude, flight-path angle and dynamic pressure sit at one height.
    spec["gamma"] = (spec["gamma"][0], (0.0, 90.0 * 540.0 / 500.0))
    spec["q"] = (spec["q"][0], (0.0, 60.0 * 540.0 / 500.0))
    names = {"alt": "Altitude [km]", "gamma": r"Flight-path angle $\gamma$ [deg]",
             "accel": r"Thrust accel. $T/(m g_0)$ [g]", "q": "Dynamic pressure [kPa]"}
    # Two ticks per axis, its zero and its top, so the axes can sit close.
    ticks = {"alt": [0, 500], "gamma": [0, 90], "accel": [0, 6], "q": [0, 60]}
    # Spine positions in axes fractions: two on the left, two on the right.
    spines = {"gamma": ("left", -0.135), "accel": ("right", 1.0), "q": ("right", 1.085)}

    # Axes boxes in inches from the foot: legend of (b), (b), legend of (a), (a).
    # Each panel carries its own legend, so the two can be printed apart.
    height = 5.40
    fig = plt.figure(figsize=(COLUMN_IN, height))
    ax = fig.add_axes([0.150, 3.30 / height, 0.700, 1.86 / height])
    ax_b = fig.add_axes([0.150, 1.05 / height, 0.795, 1.03 / height])
    hosts = {"alt": ax}
    for key in ("gamma", "accel", "q"):
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

    # (b) The steering of the first second-stage burn, where the law is fitted.
    long_burns = [b for b in burns if b[1] - b[0] >= sec63.MIN_FIT_S]
    b0, b1 = long_burns[0]
    sel = (case.time >= b0) & (case.time <= b1)
    tb = case.time[sel]
    keep = np.ones(tb.size, dtype=bool)
    keep[1:] = np.diff(tb) > 0.0
    tb = tb[keep]
    # Colours of their own, so neither is read as a quantity of panel (a).
    steer = (("theta", r"Pitch $\theta$", sec63.OVERLAY_COLOURS["mach"]),
             ("alpha", r"Angle of attack $\alpha$", sec63.OVERLAY_COLOURS["thrust"]))
    for key, label, colour in steer:
        values = (case.theta_deg if key == "theta" else case.alpha_deg)[sel][keep]
        ax_b.plot(*st.thin(tb, values), color=colour, linewidth=1.15, label=label)
    tf, fitted, _rms = sec63._tangent_fit(case, b0, b1)
    ax_b.plot(tf, fitted, color=st.INK, linestyle="--", linewidth=1.0,
              label="Linear tangent fit")
    ax_b.set_xlim(b0 - 5.0, b1 + 5.0)
    ax_b.set_ylim(0.0, 16.0)
    ax_b.set_yticks([0, 5, 10, 15])
    ax_b.set_xlabel("Time [s]", fontsize=FS, labelpad=2)
    ax_b.set_ylabel("Angle [deg]", fontsize=FS, labelpad=2)
    ax_b.tick_params(labelsize=FS)
    st.tidy(ax_b, legend=False)

    legend += [Patch(color=st.FAINT, alpha=0.45, label="Coast (%.0f s)" % (c1 - c0)),
               ax.get_legend_handles_labels()[0][-1]]
    style = dict(loc="upper center", ncol=2, fontsize=FS, frameon=False,
                 bbox_transform=fig.transFigure, handlelength=1.0, handletextpad=0.35,
                 columnspacing=0.6, labelspacing=0.2, borderaxespad=0.1, borderpad=0.1)
    # Column-major fill: the three long names in the first column, the three
    # short ones in the second, so no row pairs two long names.
    by_label = {h.get_label(): h for h in legend}
    order = [names["gamma"], names["accel"], names["q"], names["alt"]]
    order += [h.get_label() for h in legend if h.get_label() not in order]
    legend = [by_label[k] for k in order]
    leg_a = fig.legend(handles=legend, bbox_to_anchor=(0.486, 2.84 / height), **style)
    leg_b = fig.legend(handles=ax_b.get_legend_handles_labels()[0],
                       bbox_to_anchor=(0.486, 0.59 / height), **style)
    return save_pieces(fig, "ea_reference_%s.png",
                       [("profile", list(hosts.values()) + [leg_a]),
                        ("steering", [ax_b, leg_b])])


def ea_segmented(cases):
    """The two segmented cases against the reference, altitude only, in one
    column (review F-3).

    The chapter's figure adds the angle of attack as panel (b); the abstract
    quotes no value from it, so only panel (a) is drawn: altitude against time,
    on a linear time axis, the waypoint and each hand-off marked.
    Review AG-1 (2026-10-24): 2.55 in tall, short labels, the legend in two
    columns, to save space.
    """
    from . import _data
    from . import sec67_capabilities as sec67

    names = ("pmp_baseline", "show_seg_fixed_alt", "show_seg_opt_alt")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("ea segmented", missing)
    ref = cases["pmp_baseline"]
    fixed, opt = cases["show_seg_fixed_alt"], cases["show_seg_opt_alt"]
    segmented_entries = [(opt, st.ACCENT, "-", "Optimized hand-off"),
                         (fixed, st.ENV_COLORS["baseline"], "-", "Fixed hand-off")]
    entries = [(ref, st.INK, ":", "Reference")] + segmented_entries

    with plt.rc_context({"axes.labelsize": FS, "xtick.labelsize": FS,
                         "ytick.labelsize": FS, "axes.titlesize": FS}):
        fig, ax = plt.subplots(figsize=(COLUMN_IN, 2.55))
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
            pn.mark_waypoint(ax, wp, label="Waypoint")
        # Linear time axis, the coast drawn at its true length (review F-3, edited).
        ax.set_xlim(0.0, float(ref.time[ref.insertion_index() - 1]) + 10.0)
        ax.set_yticks([0, 250, 500])
        st.tidy(ax, legend=False)
        fig.subplots_adjust(left=0.18, right=0.97, top=0.975, bottom=0.425)
        handles, labels = ax.get_legend_handles_labels()
        fig.legend(handles, labels, loc="lower center", ncol=2, fontsize=FS,
                   frameon=False, bbox_to_anchor=(0.5, 0.0), labelspacing=0.25,
                   columnspacing=1.0, handlelength=1.6)
        return st.save(fig, "ea_segmented.png")


def ea_gravity_turn(cases):
    """The gravity turn's architecture and rotation studies, in one column
    (review P-1, edited 2026-10-24: the engine-model panel is left out, and
    the pseudo-force panel of fig:gt_rotation joins as (c)).

    (a) Altitude under the coast-parameter search and the apogee check, each
    coast shaded in its flight's colour, as the chapter's fig:gt_architecture.
    (b) Altitude with and without the rotation of the Earth, the chapter's
    fig:gt_rotation (a), in its colours. (a) and (b) share one time axis.
    (c) The pseudo-forces along the rotating flight up to the end of its first
    second-stage burn, fig:gt_rotation (b): the Coriolis and centrifugal
    magnitudes and the cross-plane component, with the lateral force that would
    cancel it on the right axis. Its legend runs under the panel.
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
        # Axes boxes in inches from the foot: legend of (c), (c), (b), (a). Every
        # panel labels its own time axis, so each can be printed apart.
        height = 6.60
        fig = plt.figure(figsize=(COLUMN_IN, height))
        box = lambda y, h, w=0.78: [0.19, y / height, w, h / height]
        ax_a = fig.add_axes(box(4.90, 1.50))
        ax_b = fig.add_axes(box(2.75, 1.50))
        ax_c = fig.add_axes(box(0.80, 1.30, 0.64))
        for ax, (title, entries) in zip((ax_a, ax_b), panels):
            for case, colour, label, shade in entries:
                end = case.insertion_index()
                t, alt_km = st.thin(case.time[:end], case.alt_km[:end])
                ax.plot(t, alt_km, color=colour, linewidth=1.15, label=label)
                if shade:
                    for t0, t1 in case.coast_intervals():
                        ax.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
            ax.set_ylabel("Altitude [km]", labelpad=2)
            ax.set_yticks([0, 250, 500])
            st.tidy(ax, legend_loc="lower right",
                    legend_kw={"fontsize": FS, "handlelength": 1.4, "borderaxespad": 0.2,
                               "labelspacing": 0.25})
        for axis in (ax_a, ax_b):
            axis.tick_params(labelbottom=True)
            axis.set_xlabel("Time [s]", labelpad=2)
        ax_a.set_xlim(0.0, None)
        ax_b.set_xlim(0.0, None)

        forces = base.pseudo_forces_at_azimuth()
        t_end = base.t_coast_start or float(base.time[base.insertion_index() - 1])
        sel = base.time <= t_end
        t, cor, cen, cross, mass = st.thin(base.time[sel], *(f[sel] for f in forces),
                                           base.mass[sel])
        ax_c.plot(t, cor, color=sec62.NON_ROTATING_COLOUR, linewidth=1.15, label="Coriolis")
        ax_c.plot(t, cen, color=sec62.ROTATING_COLOUR, linewidth=1.15, label="Centrifugal")
        ax_c.plot(t, cross, color=st.INK, linewidth=1.15, label="Cross-plane")
        ax_f = ax_c.twinx()
        ax_f.plot(t, mass * cross / 1e3, color=st.VIOLET, linewidth=1.15,
                  label="Lateral force (right)")
        ax_f.set_ylim(0.0, 24.0)
        ax_f.set_yticks([0, 10, 20])
        ax_f.set_ylabel("Force [kN]", labelpad=2)
        ax_f.spines["top"].set_visible(False)
        if base.t_meco is not None:
            ax_c.axvline(base.t_meco, color=st.GREY, linestyle=":", linewidth=0.8)
            # Level, in the headroom above every curve.
            ax_c.annotate("MECO", xy=(base.t_meco, 0.99), xycoords=("data", "axes fraction"),
                          xytext=(3, 0), textcoords="offset points", fontsize=FS,
                          color=st.INK, ha="left", va="top")
        ax_c.set_xlim(0.0, t_end)
        ax_c.set_ylim(0.0, 1.2)
        ax_c.set_yticks([0, 0.5, 1.0])
        ax_c.set_xlabel("Time [s]", labelpad=2)
        ax_c.set_ylabel(r"Accel. [m/s$^2$]", labelpad=2)
        st.tidy(ax_c, legend=False)
        handles, labels = [], []
        for ax in (ax_c, ax_f):
            h, l = ax.get_legend_handles_labels()
            handles += h
            labels += l
        leg_c = fig.legend(handles, labels, loc="lower center", ncol=2, fontsize=FS, frameon=False,
                   bbox_to_anchor=(0.5, 0.0), handlelength=1.2, handletextpad=0.4,
                   columnspacing=0.8, labelspacing=0.2, borderaxespad=0.1, borderpad=0.1)
        return save_pieces(fig, "ea_gravity_turn_%s.png",
                           [("architecture", [ax_a]), ("rotation", [ax_b]),
                            ("pseudo_forces", [ax_c, ax_f, leg_c])])


def ea_peg(cases):
    """Powered explicit guidance against the reference, in one column (review
    P-2, 2026-10-24; atmosphere panels U-1, 2026-10-24).

    (a) Altitude against time, the reference's long coast compressed and
    shaded: the reference dotted black, PEG under the coast-parameter search
    red, by direct insertion blue and without the atmosphere in its environment
    colour, the legend under the axes in two columns, its labels without
    "PEG,", which the caption names (review AG-3, 2026-10-24).
    (b) and (c) are the chapter's fig:peg_atmosphere in this column: the
    cumulative gravity (solid) and drag (dashed) losses of PEG with and without
    the atmosphere, in the colours of (a), and the dynamic pressure of the
    flight with the atmosphere up to separation, beside the flown Intelsat 35e
    ascent (sec63.THROTTLED_FLIGHT) dotted with its throttle-down shaded.
    """
    from . import _data
    from . import validation_falcon9 as vf

    names = ("pmp_baseline", "peg_baseline", "peg_direct", "peg_vacuum")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("ea peg", missing)
    ref = cases["pmp_baseline"]
    base, vac = cases["peg_baseline"], cases["peg_vacuum"]
    entries = [
        (ref, st.INK, ":", "Reference"),
        (base, st.ACCENT, "-", "Coast parameter"),
        (cases["peg_direct"], st.ENV_COLORS["baseline"], "-", "Direct insertion"),
        (vac, st.ENV_COLORS["no_atmosphere"], "-", "No atmosphere"),
    ]
    with plt.rc_context({"axes.labelsize": FS, "xtick.labelsize": FS,
                         "ytick.labelsize": FS, "axes.titlesize": FS}):
        # Axes boxes in inches from the foot: (c), (b), the legend of (a), (a).
        height = 6.55
        fig = plt.figure(figsize=(COLUMN_IN, height))
        box = lambda y, h: [0.19, y / height, 0.78, h / height]
        ax = fig.add_axes(box(4.83, 1.60))
        ax_b = fig.add_axes(box(2.12, 1.10))
        ax_c = fig.add_axes(box(0.40, 1.10))

        pn.altitude_panel(ax, entries, shade_coasts=False)
        for t0, t1 in ref.coast_intervals():
            ax.axvspan(t0, t1, color=st.INK, alpha=0.12, linewidth=0)
        ax.fill_between([], [], color=st.INK, alpha=0.12, linewidth=0,
                        label="Coast")
        pn.compress_long_coast(ax, ref, float(ref.time[ref.insertion_index() - 1]) + 10.0,
                               tick_step=200.0, label_y=0.12, fontsize=FS, label_box=True)
        ax.set_yticks([0, 250, 500])
        for curve in ax.get_lines():
            curve.set_linewidth(1.1)
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles, labels, loc="upper center", ncol=2, fontsize=FS,
                  frameon=False, bbox_to_anchor=(0.42, -0.25), labelspacing=0.25,
                  columnspacing=1.0,
                  handlelength=1.6, borderpad=0.1)

        for case, colour in ((base, st.ACCENT), (vac, st.ENV_COLORS["no_atmosphere"])):
            hist = case.loss_histories()
            t = case.time[:case.cutoff_index()]
            t, grav, drag = st.thin(t, hist["gravity"], hist["drag"])
            ax_b.plot(t, grav, color=colour, linewidth=1.1)
            if np.any(drag > 1e-9):
                ax_b.plot(t, drag, color=colour, linestyle="--", linewidth=1.1)
        # (b) names its own colours, so it reads without (a) beside it.
        ax_b.legend(handles=[Line2D([], [], color=st.INK, linewidth=1.1, label="Gravity"),
                             Line2D([], [], color=st.INK, linestyle="--", linewidth=1.1,
                                    label="Drag"),
                             Line2D([], [], color=st.ACCENT, linewidth=1.1, label="Atmosphere"),
                             Line2D([], [], color=st.ENV_COLORS["no_atmosphere"], linewidth=1.1,
                                    label="No atmosphere")],
                    loc="lower right", bbox_to_anchor=(1.0, 0.06), ncol=2, fontsize=FS,
                    frameon=False, handlelength=1.0,
                    handletextpad=0.35, columnspacing=0.6, labelspacing=0.25, borderpad=0.1)
        ax_b.set_ylim(0.0, 2200.0)
        ax_b.set_yticks([0, 1000, 2000])
        ax_b.set_xlabel("Time [s]", fontsize=FS, labelpad=2)
        ax_b.set_ylabel("Loss [m/s]", fontsize=FS, labelpad=2)

        # The separation is the planned 8 s after MECO.
        t_end = base.t_meco + 8.0
        keep = base.time <= t_end
        ax_c.plot(*st.thin(base.time[keep], np.asarray(base.q)[keep] / 1e3),
                  color=st.ACCENT, linewidth=1.1, label="PEG")
        flown = [f for f in vf.load_flights() if f.name == sec63.THROTTLED_FLIGHT]
        if flown:
            f = flown[0]
            t_f, q_f = vf._q(f.time, f.v, f.alt_km)
            sel = t_f <= t_end
            ax_c.plot(t_f[sel], q_f[sel], color=st.INK, linestyle=":", linewidth=1.1,
                      label=f.label)
            t0, t1 = f.event("throttle_down_start"), f.event("throttle_down_end")
            if np.isfinite(t0) and np.isfinite(t1):
                ax_c.axvspan(t0, t1, color=st.INK, alpha=0.12, linewidth=0,
                             label="Throttle-down")
        ax_c.set_xlim(0.0, t_end)
        ax_c.set_ylim(0.0, 70.0)
        ax_c.set_yticks([0, 20, 40, 60])
        ax_c.set_xlabel("Time [s]", fontsize=FS, labelpad=2)
        ax_c.set_ylabel("q [kPa]", fontsize=FS, labelpad=2)
        ax_c.legend(loc="upper right", fontsize=FS, frameon=False, handlelength=1.2,
                    labelspacing=0.25, borderpad=0.1)

        for axis in (ax, ax_b, ax_c):
            axis.xaxis.labelpad = 2
            axis.yaxis.labelpad = 2
            st.tidy(axis, legend=False)
        return save_pieces(fig, "ea_peg_%s.png", [("altitude", [ax]), ("losses", [ax_b]),
                                                   ("dynamic_pressure", [ax_c])])


def ea_vacuum_coast_bound(cases):
    """The drag-free reference at the 2000 s and 3500 s coast bounds, altitude
    against downrange, in one column (review S-1, 2026-10-24): the chapter's
    fig:vacuum_coast_bound with its legend moved below the axes, the downrange
    in thousands of kilometres, and the lowest-point label between the curves.
    Review AG-2 (2026-10-24): one legend row, short labels, the target line
    unlabelled, to save space.
    """
    from . import _data

    names = ("pmp_vacuum", "pmp_vacuum_c3500")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("ea vacuum coast bound", missing)
    fig, ax = plt.subplots(figsize=(COLUMN_IN, 2.10))
    styles = (("pmp_vacuum", "2000 s bound", st.REFERENCE),
              ("pmp_vacuum_c3500", "3500 s bound", st.ACCENT))
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
        ax.axhline(target, color=st.GREY, linestyle="--", linewidth=0.8)
    ax.axhline(0.0, color=st.INK, linewidth=0.8)
    ax.set_xlabel(r"Downrange [$10^3$ km]", fontsize=FS, labelpad=2)
    ax.set_ylabel("Altitude [km]", fontsize=FS, labelpad=2)
    ax.set_yticks([0, 250, 500])
    ax.tick_params(labelsize=FS)
    st.tidy(ax, legend=False)
    fig.subplots_adjust(left=0.18, right=0.97, top=0.965, bottom=0.36)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=2, fontsize=FS, frameon=False,
               columnspacing=1.2,
               bbox_to_anchor=(0.5, 0.0), labelspacing=0.25, handlelength=1.6)
    return st.save(fig, "ea_vacuum_coast_bound.png")


FIGURES = [ea_reference_profile, ea_vacuum_coast_bound, ea_gravity_turn, ea_peg,
           ea_segmented]
