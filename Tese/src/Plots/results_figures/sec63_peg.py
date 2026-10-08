"""Sections 6.1, 6.3 and 6.4 figures -- the reference and powered explicit guidance.

Since 2026-09-29 this law carries two of the secondary factors: direct
insertion and the atmosphere. gt_direct and peg_vacuum_norot are archived but
not reported, which retired the two-law direct contrast and the two-step
environment ladder.

The outline review of 2026-10-04 removed two figures: the reference card now
carries what results_reference_trajectory.png showed (N6-02), and the
direct-insertion flight is drawn in the waypoint figure instead of in a figure
of its own, results_peg_vs_reference.png (N6-04).

Outputs
-------
results_reference_profiles.png    fig:reference_card (the figure that replaces the card)
results_reference_tangent_fit.png fig:reference_tangent_fit
results_vacuum_coast_bound.png    fig:vacuum_coast_bound
results_peg_architectures.png     fig:peg_architectures
results_peg_atmosphere.png        fig:peg_atmosphere

On request only: results_reference_card.png.
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
from . import _panels as pn
from . import _style as st
from . import run_card

# A burn shorter than this is not fitted with the linear-tangent form: the
# reference's final burn lasts a fraction of a second, and two parameters
# fitted to a handful of samples say nothing about the steering law.
MIN_FIT_S = 5.0


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def reference_card(cases):
    """The references as a run card, the format of every later trajectory figure.

    The chapter opens on the reference, so this is where the reader first meets
    the four panels. The drawing is run_card.py's, shared with the gravity-turn
    baseline card and with main.py's interactive card.

    The drag-free reference is drawn faint behind the atmospheric one, panel (b)
    marks the coast-start waypoint that Sections 6.4 to 6.6 aim other laws at,
    and panel (c) carries the linear-tangent form fitted to each Stage-2 burn
    long enough to fit, because the question Chapter 4 raises about this
    trajectory is whether the optimal steering has the signature the analytical
    treatment predicts. Stage 1 is not fitted: it is the fixed gravity turn every
    case shares, flown before any costate exists.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("reference card", missing)
    ref = cases["pmp_baseline"]
    fits = [_tangent_fit(ref, t0, t1)
            for t0, t1 in pn.burn_intervals(ref, stage2_only=True)
            if t1 - t0 >= MIN_FIT_S]
    return run_card.draw(ref, "results_reference_card.png",
                         background=cases.get("pmp_vacuum"),
                         background_label="Reference, no atmosphere",
                         waypoint=pn.waypoint(ref), pitch_fits=fits)


def peg_architectures(cases):
    """PEG aimed at the orbit under the coast-parameter and the direct-insertion
    architectures, with the reference: (a) altitude against time, each case's
    coasts shaded, (b) the commanded angle of attack.

    The waypoint-aimed flight of this law is Section 6.4's own figure
    (sec67_capabilities.segmented), not drawn here (red-note review S3.1).
    """
    names = ("pmp_baseline", "peg_baseline", "peg_direct")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("peg architectures", missing)
    entries = [
        (cases["pmp_baseline"], st.ENV_COLORS["reference"], "-", "Reference (indirect PMP)"),
        (cases["peg_baseline"], st.ENV_COLORS["baseline"], "-", "PEG, coast-parameter search"),
        (cases["peg_direct"], st.SECOND_ARCH, "-.", "PEG, direct insertion"),
    ]
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    pn.altitude_panel(ax_a, entries)
    pn.mark_arc_events(ax_a, entries, label_top=True)
    ref = cases["pmp_baseline"]
    pn.compress_long_coast(ax_a, ref, float(ref.time[ref.insertion_index() - 1]) + 10.0,
                           label_y=0.15)
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend=False)

    pn.alpha_panel(ax_b, entries)
    pn.mark_arc_events(ax_b, entries)
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)
    pn.figure_legend(fig, ax_a)
    return st.save(fig, "results_peg_architectures.png")


def peg_atmosphere(cases):
    """PEG at the baseline against the drag-free run: (a) altitude against
    downrange, (b) the cumulative gravity and drag losses.

    The two runs differ on two counts and not one: ``INCLUDE_DRAG=False`` is the
    master no-atmosphere switch, so the vacuum case also drops the fairing and
    flies vacuum thrust and vacuum Isp. Panel (b) shows both consequences at
    once -- the drag integral vanishes, and the gravity loss changes because the
    trajectory it is flown along has changed.
    """
    missing = _data.missing_from(cases, "peg_baseline", "peg_vacuum")
    if missing:
        return _skip("peg atmosphere", missing)
    base, vac = cases["peg_baseline"], cases["peg_vacuum"]
    flights = ((base, st.ENV_COLORS["baseline"], "PEG, with atmosphere"),
               (vac, st.ENV_COLORS["no_atmosphere"], "PEG, no atmosphere"))

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, colour, label in flights:
        _t, s_km, alt_km = pn.to_insertion(case, case.downrange_km, case.alt_km)
        ax_a.plot(*st.thin(s_km, alt_km), color=colour, label=label)
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right", legend_kw={"fontsize": 6.3})

    for case, colour, _label in flights:
        hist = case.loss_histories()
        t = case.time[:case.cutoff_index()]
        t, grav, drag = st.thin(t, hist["gravity"], hist["drag"])
        ax_b.plot(t, grav, color=colour)
        if np.any(drag > 1e-9):
            ax_b.plot(t, drag, color=colour, linestyle="--")
    ax_b.plot([], [], color=st.GREY, label="Gravity")
    ax_b.plot([], [], color=st.GREY, linestyle="--", label="Drag")
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Cumulative loss [m/s]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend_loc="upper left")

    fig.tight_layout()
    return st.save(fig, "results_peg_atmosphere.png")


def reference_tangent_fit(cases):
    """How close the reference's first second-stage burn is to the linear-tangent
    law (red-note review S1.4): (a) the pitch flown and the law fitted to it,
    tan(theta) = a + b t by least squares (_tangent_fit), (b) their difference,
    with its RMS.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("reference tangent fit", missing)
    ref = cases["pmp_baseline"]
    burns = [b for b in pn.burn_intervals(ref, stage2_only=True) if b[1] - b[0] >= MIN_FIT_S]
    if not burns:
        return _skip("reference tangent fit", ["a second-stage burn"])
    t0, t1 = burns[0]
    t, fitted, rms = _tangent_fit(ref, t0, t1)
    sel = (ref.time >= t0) & (ref.time <= t1)
    keep = np.ones(int(sel.sum()), dtype=bool)
    keep[1:] = np.diff(ref.time[sel]) > 0.0
    flown = ref.theta_deg[sel][keep]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(6.3, 2.2))
    ax_a.plot(t, flown, color=st.ENV_COLORS["reference"], label="Reference, flown")
    ax_a.plot(t, fitted, color=st.INK, linestyle="--", linewidth=1.0,
              label=r"Linear tangent, $\tan\theta = a + b\,t$")
    ax_a.set_xlabel("Time [s]")
    ax_a.set_ylabel(r"Pitch $\theta$ [deg]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="upper right", legend_kw={"fontsize": 6.3})

    ax_b.plot(t, flown - fitted, color=st.ENV_COLORS["reference"])
    ax_b.axhline(0.0, color=st.FAINT, linewidth=0.8)
    for sign in (1.0, -1.0):
        ax_b.axhline(sign * rms, color=st.GREY, linestyle=":", linewidth=0.8)
    ax_b.annotate(r"RMS $%.2f^\circ$ (dotted)" % rms, xy=(0.97, 0.06),
                  xycoords="axes fraction", ha="right", va="bottom", fontsize=6.5,
                  color=st.INK)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Flown $-$ fitted [deg]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)
    fig.tight_layout()
    return st.save(fig, "results_reference_tangent_fit.png")


def _tangent_fit(case, t0, t1):
    """The linear-tangent law fitted to the pitch of one burn.

    Chapter 4's form, tan(theta) = a + b*t, with theta from the local
    horizontal; least squares on tan(theta) over the burn's samples. Returns
    the time axis, the fitted pitch [deg] and its RMS departure from the flown
    pitch [deg].
    """
    sel = (case.time >= t0) & (case.time <= t1)
    t = case.time[sel]
    theta = case.theta_deg[sel]
    keep = np.ones(t.size, dtype=bool)
    keep[1:] = np.diff(t) > 0.0
    t, theta = t[keep], theta[keep]
    design = np.vstack([np.ones_like(t), t]).T
    coef, *_ = np.linalg.lstsq(design, np.tan(np.deg2rad(theta)), rcond=None)
    fitted = np.rad2deg(np.arctan(design @ coef))
    return t, fitted, float(np.sqrt(np.mean((fitted - theta) ** 2)))


def reference_timeline(cases, filename="reference_timeline.png"):
    """The reference's control and loads against time, one quantity per panel.

    Steering angle, thrust, propellant remaining, dynamic pressure and Mach
    share one time axis and nothing else: each panel keeps its own scale, so no
    quantity is read off another's axis. Dynamic pressure and Mach stop at the
    atmosphere exit (ALT_NO_ATMOSPHERE, 65 km), above which neither describes
    the flow. The time axis ends shortly after the coast begins: the coast runs
    another ~1450 s and the final burn lasts a fraction of a second, so the
    axis would otherwise hold all the flight's activity in its first fifth. The
    coast and the final burn are stated in the figure instead.

    Not part of the chapter's FIGURES list; drawn on request.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("reference timeline", missing)
    ref = cases["pmp_baseline"]
    colour = st.REFERENCE

    t_coast = ref.t_coast_start
    t_hi = (t_coast + 55.0) if t_coast is not None else float(ref.time[-1])
    view = ref.time <= t_hi
    t = ref.time[view]

    alt_exit_km = float((ref.manifest.get("config") or {}).get("ALT_NO_ATMOSPHERE", 65e3)) / 1e3
    crossed = np.where(ref.alt_km >= alt_exit_km)[0]
    t_exit = float(ref.time[crossed[0]]) if crossed.size else None
    in_atm = t <= (t_exit if t_exit is not None else t[-1])

    q, mach = ref.q[view], ref.mach[view]
    i_qmax = int(np.nanargmax(np.where(in_atm, q, -np.inf)))
    t_kick = (ref.row.get("t_kick_start")
              or (float(ref._z["t_kick_start"]) if "t_kick_start" in ref._z.files else None))
    burns = pn.burn_intervals(ref)
    t_ignition = burns[1][0] if len(burns) > 1 else None

    fig, axes = plt.subplots(5, 1, figsize=(6.3, 7.4), sharex=True,
                             gridspec_kw={"hspace": 0.18})
    ax_al, ax_th, ax_pr, ax_q, ax_m = axes

    ax_al.plot(t, ref.alpha_deg[view], color=colour)
    ax_al.axhline(0.0, color=st.FAINT, linewidth=0.8)
    ax_al.set_ylabel("Steering angle\n" + r"$\alpha$ [deg]")

    ax_th.plot(t, ref.thrust[view] / 1e6, color=colour)
    ax_th.set_ylabel("Thrust\n[MN]")

    prop_t = ref.prop_kg[view] / 1e3
    ax_pr.plot(t, prop_t, color=colour)
    ax_pr.set_ylabel("Propellant\nremaining [t]")

    ax_q.plot(t[in_atm], q[in_atm] / 1e3, color=colour)
    ax_q.set_ylabel("Dynamic\npressure [kPa]")
    ax_q.annotate(r"max $q$ = %.1f kPa at %.0f s" % (q[i_qmax] / 1e3, t[i_qmax]),
                  xy=(t[i_qmax], q[i_qmax] / 1e3), xytext=(8, -4),
                  textcoords="offset points", fontsize=6.8, color=st.INK, va="top")

    ax_m.plot(t[in_atm], mach[in_atm], color=colour)
    ax_m.set_ylabel("Mach\n[-]")
    ax_m.set_xlabel("Time [s]")
    ax_m.set_xlim(0.0, t_hi)

    # Arc structure, the same in every panel: the staging interval from MECO to
    # Stage-2 ignition, and the coast from where it begins to the axis end.
    for ax in axes:
        if ref.t_meco is not None and t_ignition is not None:
            ax.axvspan(ref.t_meco, t_ignition, color=st.FAINT, alpha=0.6, linewidth=0)
        if t_coast is not None:
            ax.axvspan(t_coast, t_hi, color=colour, alpha=0.10, linewidth=0)
        for t_evt in (t_kick, float(t[i_qmax]), t_exit):
            if t_evt is not None:
                ax.axvline(t_evt, color=st.GREY, linestyle=":", linewidth=0.8)
        st.tidy(ax, legend=False)

    # Event names once, along the top of the first panel. The atmosphere exit,
    # 3.5 s after MECO, is named in the Mach panel instead, where its label
    # does not land on MECO's.
    top = ax_al.get_ylim()[1]
    events = [(t_kick, "kick", False), (float(t[i_qmax]), "max q", False),
              (ref.t_meco, "MECO", True), (t_ignition, "S2 ignition", False)]
    for t_evt, text, right in events:
        if t_evt is None:
            continue
        ax_al.annotate(text, xy=(t_evt, top), xytext=(-2 if right else 2, -2),
                       textcoords="offset points", fontsize=6.3, color=st.INK,
                       rotation=90, ha="right" if right else "left", va="top")

    if t_coast is not None:
        final = burns[-1] if len(burns) > 2 else None
        text = "coast to\n%.0f s" % (ref.t_seco or ref.time[-1])
        if final is not None:
            text += "\nfinal burn\n%.3f s" % (final[1] - final[0])
        ax_al.annotate(text, xy=(t_coast + 3.0, 0.08), xycoords=("data", "axes fraction"),
                       fontsize=6.3, color=st.INK, va="bottom")
        ax_pr.annotate("%.1f t at\ninsertion" % (ref.prop_kg[-1] / 1e3),
                       xy=(t_coast + 3.0, prop_t[-1]), xytext=(0, 5),
                       textcoords="offset points", fontsize=6.3, color=st.INK,
                       va="bottom")
    if t_ignition is not None:
        s2 = (t > t_ignition + 1.0) & (t < (t_coast or t_hi))
        if np.any(s2):
            f2 = float(np.median(ref.thrust[view][s2])) / 1e3
            ax_th.annotate("Stage 2: %.0f kN" % f2, xy=(0.5 * (t_ignition + (t_coast or t_hi)),
                                                       f2 / 1e3),
                           xytext=(0, 4), textcoords="offset points", fontsize=6.3,
                           color=st.INK, ha="center", va="bottom")
    if t_exit is not None:
        ax_m.annotate("atmosphere exit,\n%.0f km at %.1f s" % (alt_exit_km, t_exit),
                      xy=(t_exit, 0.5), xycoords=("data", "axes fraction"),
                      xytext=(3, 0), textcoords="offset points", fontsize=6.3,
                      color=st.GREY, va="center")

    for ax, letter in zip(axes, "abcde"):
        ax.annotate("(%s)" % letter, xy=(-0.13, 1.0), xycoords="axes fraction",
                    fontsize=8, fontweight="bold", color=st.INK, va="top")
    fig.align_ylabels(axes)
    return st.save(fig, filename)


# One hue per curve of the overlay below. Every curve can cross every other, so
# the set is checked on all pairs (scripts/validate_palette.js --pairs all,
# light mode). Five come from the dataviz reference palette (yellow one step
# darker for contrast on white); no six of its eight pass on all pairs, so the
# acceleration takes a purple from outside it, the only candidate tried that
# clears every floor against the other five. Two warnings remain: thrust
# against dynamic pressure under deuteranopia (dE 6.2, in the band legal with
# secondary encoding -- the axis labels and the legend), and yellow at 2.99:1
# against the surface, relieved the same way.
OVERLAY_COLOURS = {
    "gamma": "#2a78d6", "thrust": "#c98500", "accel": "#b13ab8",
    "prop": "#008300", "mach": "#4a3aa7", "q": "#e34948",
}

# Text sizes [pt] for the overlay, larger than the chapter's house style: the
# figure is shown on a screen or a slide rather than printed at text width.
OVERLAY_FONT = {"tick": 11.5, "label": 13.0, "note": 11.0, "legend": 11.5}


def reference_timeline_overlay(cases, filename="reference_timeline_overlay.png"):
    """The reference's flight-path angle and loads on one plot, one y-axis each.

    Drawn on request as a single overlay with parallel y-axes, which the
    chapter's figures avoid: flight-path angle, thrust, acceleration,
    propellant, dynamic pressure and Mach each get their own axis, coloured as
    their curve. The time axis, the event marks and the atmosphere cut-off for
    q and Mach are those of :func:`reference_timeline`. The acceleration is the
    thrust acceleration T/(m g0): drag is not subtracted, which matters only
    around max q.

    Not part of the chapter's FIGURES list; drawn on request.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("reference timeline overlay", missing)
    from Auxiliary import constants as const

    ref = cases["pmp_baseline"]
    t_coast = ref.t_coast_start
    t_hi = (t_coast + 55.0) if t_coast is not None else float(ref.time[-1])
    view = ref.time <= t_hi
    t = ref.time[view]

    alt_exit_km = float((ref.manifest.get("config") or {}).get("ALT_NO_ATMOSPHERE", 65e3)) / 1e3
    crossed = np.where(ref.alt_km >= alt_exit_km)[0]
    t_exit = float(ref.time[crossed[0]]) if crossed.size else None
    in_atm = t <= (t_exit if t_exit is not None else t[-1])
    q, mach = ref.q[view], ref.mach[view]
    i_qmax = int(np.nanargmax(np.where(in_atm, q, -np.inf)))
    t_kick = float(ref._z["t_kick_start"]) if "t_kick_start" in ref._z.files else None
    burns = pn.burn_intervals(ref)
    t_ignition = burns[1][0] if len(burns) > 1 else None
    accel_g = ref.thrust[view] / ref.mass[view] / const.G_0

    col = OVERLAY_COLOURS
    c_ga, c_f, c_acc, c_pr, c_m, c_q = (col[k] for k in
                                        ("gamma", "thrust", "accel", "prop", "mach", "q"))
    fs = OVERLAY_FONT
    fig, host = plt.subplots(figsize=(12.0, 7.0))
    fig.subplots_adjust(left=0.21, right=0.79, top=0.89, bottom=0.21)

    def _axis(side, offset, colour, label, ylim):
        ax = host.twinx()
        ax.spines["right" if side == "left" else "left"].set_visible(False)
        ax.spines["top"].set_visible(False)
        spine = ax.spines[side]
        spine.set_visible(True)
        spine.set_position(("axes", offset))
        spine.set_color(colour)
        spine.set_linewidth(1.4)
        ax.yaxis.set_ticks_position(side)
        ax.yaxis.set_label_position(side)
        ax.tick_params(axis="y", colors=colour, labelcolor=st.INK, width=1.0,
                       labelsize=fs["tick"])
        ax.set_ylabel(label, color=colour, fontweight="bold", fontsize=fs["label"])
        ax.set_ylim(*ylim)
        return ax

    # Host: the flight-path angle, its spine coloured like every other axis.
    host.plot(t, ref.gamma_deg[view], color=c_ga, linewidth=1.6,
              label=r"Flight-path angle $\gamma$ [deg]")
    host.set_ylim(0.0, 95.0)
    host.spines["left"].set_color(c_ga)
    host.spines["left"].set_linewidth(1.4)
    host.tick_params(axis="y", colors=c_ga, labelcolor=st.INK, width=1.0)
    host.tick_params(axis="both", labelsize=fs["tick"])
    host.set_ylabel(r"Flight-path angle $\gamma$ [deg]", color=c_ga, fontweight="bold",
                    fontsize=fs["label"])
    host.set_xlabel("Time [s]", fontsize=fs["label"])
    host.set_xlim(0.0, t_hi)

    ax_f = _axis("left", -0.095, c_f, "Thrust [MN]", (0.0, 9.0))
    ax_f.plot(t, ref.thrust[view] / 1e6, color=c_f, linewidth=1.6, label="Thrust [MN]")
    ax_a = _axis("left", -0.19, c_acc, r"Acceleration $T/(m g_0)$ [g]", (0.0, 7.5))
    ax_a.plot(t, accel_g, color=c_acc, linewidth=1.6,
              label=r"Thrust acceleration $T/(m g_0)$ [g]")
    ax_p = _axis("right", 1.0, c_pr, "Propellant remaining [t]", (0.0, 520.0))
    ax_p.plot(t, ref.prop_kg[view] / 1e3, color=c_pr, linewidth=1.6,
              label="Propellant remaining [t]")
    # The atmosphere exit, 3.5 s after MECO, is named in these two legend
    # entries rather than with a third label in the crowded staging interval.
    ax_q = _axis("right", 1.095, c_q, "Dynamic pressure [kPa]", (0.0, 60.0))
    ax_q.plot(t[in_atm], q[in_atm] / 1e3, color=c_q, linewidth=1.6,
              label="Dynamic pressure [kPa], to %.0f km" % alt_exit_km)
    ax_m = _axis("right", 1.19, c_m, "Mach [-]", (0.0, 13.0))
    ax_m.plot(t[in_atm], mach[in_atm], color=c_m, linewidth=1.6,
              label="Mach [-], to %.0f km" % alt_exit_km)

    # Arc structure and events, once, on the host.
    if ref.t_meco is not None and t_ignition is not None:
        host.axvspan(ref.t_meco, t_ignition, color=st.FAINT, alpha=0.6, linewidth=0)
    if t_coast is not None:
        host.axvspan(t_coast, t_hi, color=st.FAINT, alpha=0.35, linewidth=0)
    for t_evt in (t_kick, float(t[i_qmax])):
        if t_evt is not None:
            host.axvline(t_evt, color=st.GREY, linestyle=":", linewidth=0.8)
    events = [(t_kick, "kick", False), (float(t[i_qmax]), "max q", False),
              (ref.t_meco, "MECO", True), (t_ignition, "S2 ignition", False)]
    for t_evt, text, right in events:
        if t_evt is None:
            continue
        host.annotate(text, xy=(t_evt, 1.0), xycoords=("data", "axes fraction"),
                      xytext=(-2 if right else 2, 2), textcoords="offset points",
                      fontsize=fs["note"], color=st.INK, rotation=90,
                      ha="right" if right else "left", va="bottom")
    if t_coast is not None:
        final = burns[-1] if len(burns) > 2 else None
        text = "coast to\n%.0f s" % (ref.t_seco or ref.time[-1])
        if final is not None:
            text += "\n\nfinal burn\n%.3f s" % (final[1] - final[0])
        text += "\n\n%.1f t left" % (ref.prop_kg[-1] / 1e3)
        host.annotate(text, xy=(t_coast + 3.0, 0.40), xycoords=("data", "axes fraction"),
                      fontsize=fs["note"], color=st.INK, va="bottom")

    host.grid(True, color=st.FAINT, linewidth=0.5, alpha=0.8)
    host.set_axisbelow(True)
    host.spines["top"].set_visible(False)
    host.spines["right"].set_visible(False)

    handles, labels = [], []
    for ax in (host, ax_f, ax_a, ax_p, ax_q, ax_m):
        h, l = ax.get_legend_handles_labels()
        handles += h
        labels += l
    fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=fs["legend"],
               bbox_to_anchor=(0.5, 0.01), frameon=False)
    return st.save(fig, filename)


# The quantities of the reference profiles, in legend order, with their colours
# and fixed limits: the three references share every axis, so a difference
# between panels is a difference between flights, not between scales.
PROFILE_AXES = [
    ("alt", "Altitude [km]", st.INK, (0.0, 540.0)),
    ("gamma", r"Flight-path angle $\gamma$ [deg]", OVERLAY_COLOURS["gamma"], (-3.0, 93.0)),
    ("accel", r"Thrust accel. $T/(m g_0)$ [g]", OVERLAY_COLOURS["accel"], (0.0, 7.5)),
    ("q", "Dynamic pressure [kPa]", OVERLAY_COLOURS["q"], (0.0, 60.0)),
    ("mach", "Mach [-]", OVERLAY_COLOURS["mach"], (0.0, 13.0)),
]

# Spine positions of the parallel y-axes, in axes fractions of the host.
_PROFILE_SPINES = {"gamma": ("left", -0.16), "accel": ("right", 1.0),
                   "q": ("right", 1.13), "mach": ("right", 1.26)}

# What the compressed time axis keeps at full scale on either side of a coast,
# and how wide [s of axis] the coast itself is drawn.
_COAST_MARGIN_S = (40.0, 25.0)
_COAST_WIDTH_S = 70.0


_compressed_time = pn.compressed_time


def _profile_panel(ax, case, title, waypoint=None):
    """One reference against time on five parallel y-axes. Returns the axes by
    quantity, for the shared legend."""
    from Auxiliary import constants as const

    end = case.insertion_index()
    t = case.time[:end]
    series = {
        "alt": case.alt_km[:end],
        "gamma": case.gamma_deg[:end],
        "accel": case.thrust[:end] / case.mass[:end] / const.G_0,
    }
    # Dynamic pressure and Mach up to the atmosphere exit, above which neither
    # describes a flow; a drag-free reference flew neither, so it gets neither.
    if case.row.get("include_drag") is not False:
        alt_exit = float((case.manifest.get("config") or {})
                         .get("ALT_NO_ATMOSPHERE", 65e3)) / 1e3
        crossed = np.where(case.alt_km[:end] >= alt_exit)[0]
        in_atm = t <= (t[crossed[0]] if crossed.size else t[-1])
        series["q"] = np.where(in_atm, case.q[:end] / 1e3, np.nan)
        series["mach"] = np.where(in_atm, case.mach[:end], np.nan)

    axes = {"alt": ax}
    for key, label, colour, ylim in PROFILE_AXES:
        if key not in series:
            continue
        if key != "alt":
            side, offset = _PROFILE_SPINES[key]
            twin = ax.twinx()
            twin.spines["right" if side == "left" else "left"].set_visible(False)
            twin.spines["top"].set_visible(False)
            twin.spines[side].set_visible(True)
            twin.spines[side].set_position(("axes", offset))
            twin.yaxis.set_ticks_position(side)
            twin.yaxis.set_label_position(side)
            axes[key] = twin
        target = axes[key]
        target.spines["left" if key in ("alt", "gamma") else "right"].set_color(colour)
        target.tick_params(axis="y", colors=colour, labelcolor=st.INK, labelsize=6.8)
        target.set_ylabel(label, color=colour, fontsize=7.4)
        target.set_ylim(*ylim)
        tt, yy = st.thin(t, series[key])
        target.plot(tt, yy, color=colour, linewidth=1.15, label=label)

    # Arc structure: the coast shaded, dotted lines at MECO and at the two
    # second-stage cut-offs.
    burns = pn.burn_intervals(case, stage2_only=True)
    coasts = case.coast_intervals()
    for c0, c1 in coasts:
        ax.axvspan(c0, c1, color=st.FAINT, alpha=0.45, linewidth=0, label="Coast")
    events = [(case.t_meco, "MECO")] + [(b[1], "SECO %d" % (i + 1))
                                        for i, b in enumerate(burns)]
    for t_evt, text in events:
        if t_evt is None:
            continue
        ax.axvline(t_evt, color=st.INK, linestyle=":", linewidth=0.8)
        ax.annotate(text, xy=(t_evt, 1.0), xycoords=("data", "axes fraction"),
                    xytext=(-2, -3), textcoords="offset points", fontsize=6.3,
                    color=st.INK, rotation=90, ha="right", va="top")
    if waypoint is not None:
        pn.mark_waypoint(ax, waypoint)

    # The coast is most of the flight and nothing happens in it, so it is drawn
    # compressed between two breaks, its length printed in it.
    t_end = float(t[-1])
    ax.set_xlim(0.0, t_end + 10.0)
    if coasts:
        c0, c1 = max(coasts, key=lambda s: s[1] - s[0])
        cut0, cut1 = c0 + _COAST_MARGIN_S[0], c1 - _COAST_MARGIN_S[1]
        if cut1 - cut0 > _COAST_WIDTH_S:
            ax.set_xscale("function", functions=_compressed_time(cut0, cut1,
                                                                 _COAST_WIDTH_S))
            # One tick after the coast: the final segment is ~40 s of axis, too
            # narrow for two labels.
            ticks = list(np.arange(0.0, cut0, 100.0))
            ticks.append(max(np.round(t_end / 10.0) * 10.0, np.ceil(cut1 / 10.0) * 10.0))
            ax.set_xticks(ticks)
            ax.set_xticklabels(["%.0f" % v for v in ticks])
            for cut in (cut0, cut1):
                ax.annotate("//", xy=(cut, 0.0), xycoords=("data", "axes fraction"),
                            ha="center", va="center", fontsize=8, color=st.INK,
                            bbox={"boxstyle": "square,pad=0.05", "fc": "white",
                                  "ec": "none"})
            # The middle of the coast maps to the middle of its compressed band.
            ax.annotate("coast\n%.0f s" % (c1 - c0), xy=(0.5 * (cut0 + cut1), 0.2),
                        xycoords=("data", "axes fraction"), fontsize=6.5,
                        color=st.GREY, ha="center", va="center")
    ax.set_xlabel("Time [s]")
    ax.set_title(title, loc="left", fontsize=8.5, color=st.INK, fontweight="bold")
    ax.grid(True, color=st.FAINT, linewidth=0.5, alpha=0.8)
    ax.set_axisbelow(True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    return axes


def reference_profiles(cases):
    """The three references against time, one panel each, every quantity on its
    own y-axis (walkthrough S1-F1, 2026-10-06).

    Altitude, flight-path angle, thrust acceleration, dynamic pressure and Mach
    share the time axis; MECO and the second-stage cut-offs are dotted, the coast
    is shaded and drawn compressed. The atmospheric reference carries the
    coast-start waypoint. The linear-tangent fit of the reference's steering,
    which the card drew, is quoted in the text instead.
    """
    names = ("pmp_baseline", "pmp_vacuum", "pmp_norot")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("reference profiles", missing)
    titles = ("(a) Reference", "(b) Reference, no atmosphere",
              "(c) Reference, non-rotating Earth")

    fig, panels = plt.subplots(3, 1, figsize=(6.3, 8.2))
    fig.subplots_adjust(left=0.17, right=0.70, top=0.97, bottom=0.12, hspace=0.42)
    first = None
    for ax, name, title in zip(panels, names, titles):
        wp = pn.waypoint(cases[name]) if name == "pmp_baseline" else None
        axes = _profile_panel(ax, cases[name], title, waypoint=wp)
        first = first or axes

    # One legend for the figure, from the first panel, which carries every entry.
    entries = {}
    for ax in first.values():
        for h, l in zip(*ax.get_legend_handles_labels()):
            entries.setdefault(l, h)
    fig.legend(list(entries.values()), list(entries.keys()), loc="lower center",
               ncol=3, fontsize=6.8, frameon=False, bbox_to_anchor=(0.5, 0.0))
    return st.save(fig, "results_reference_profiles.png")


def vacuum_coast_bound(cases):
    """The drag-free reference with its coast bounded at 2000 s and at 3500 s,
    altitude against downrange (Section 6.1, user decision 2026-10-07).

    Without drag nothing opposes an early turn to the horizontal, so with the
    wider bound the optimum burns low and coasts from a lowest point a few
    kilometres above the surface; that point is marked. Each curve stops at its
    own insertion.
    """
    names = ("pmp_vacuum", "pmp_vacuum_c3500")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("vacuum coast bound", missing)

    fig, ax = plt.subplots(figsize=(6.3, 2.7))
    styles = (("pmp_vacuum", "Coast bound 2000 s (reported)", st.REFERENCE),
              ("pmp_vacuum_c3500", "Coast bound 3500 s", st.ACCENT))
    for name, label, colour in styles:
        case = cases[name]
        _t, s_km, h_km = pn.to_insertion(case, case.downrange_km, case.alt_km)
        ax.plot(s_km, h_km, color=colour, label=label)

    case = cases["pmp_vacuum_c3500"]
    t, s_km, h_km = pn.to_insertion(case, case.downrange_km, case.alt_km)
    after = t > case.t_meco
    i = int(np.argmin(np.where(after, h_km, np.inf)))
    ax.plot(s_km[i], h_km[i], "o", color=st.ACCENT, markersize=4.5, zorder=4)
    ax.annotate("lowest point %.1f km" % h_km[i], xy=(s_km[i], h_km[i]),
                xytext=(8, 10), textcoords="offset points", fontsize=7,
                color=st.ACCENT)

    target = case.row.get("target_alt_km")
    if target is not None:
        ax.axhline(target, color=st.GREY, linestyle="--", linewidth=0.8,
                   label="Target orbit, %.0f km" % target)
    ax.axhline(0.0, color=st.INK, linewidth=0.8)
    ax.set_xlabel("Downrange [km]")
    ax.set_ylabel("Altitude [km]")
    st.tidy(ax, legend=True, legend_loc="center right")
    return st.save(fig, "results_vacuum_coast_bound.png")


# Since 2026-10-06 (walkthrough) the chapter draws the references as profiles
# rather than as a card (S1-F1); reference_card is drawn on request. Since
# 2026-10-08 (red-note review S3.1) Section 6.3 draws its two architectures and
# PEG with and without the atmosphere; the waypoint flight moved to Section 6.4.
FIGURES = [reference_profiles, reference_tangent_fit, vacuum_coast_bound,
           peg_architectures, peg_atmosphere]
