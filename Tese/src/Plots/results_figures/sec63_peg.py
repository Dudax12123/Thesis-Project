"""Sections 6.1, 6.3 and 6.4 figures -- the reference and powered explicit guidance.

Since 2026-09-29 this law carries two of the secondary factors: direct
insertion (drawn with the coast-parameter flight against the reference) and the
atmosphere. gt_direct and peg_vacuum_norot are archived but not reported, which
retired the two-law direct contrast and the two-step environment ladder.

Outputs
-------
results_reference_card.png        fig:reference_card
results_reference_trajectory.png  fig:reference_trajectory
results_peg_vs_reference.png      fig:peg_vs_reference
results_peg_atmosphere.png        fig:peg_atmosphere
results_peg_waypoint.png          fig:peg_waypoint
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
    """The reference as a run card, the format of every later trajectory figure.

    The chapter opens on the reference, so this is where the reader first meets
    the four panels. The drawing is run_card.py's, shared with the gravity-turn
    baseline card and with main.py's interactive card.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("reference card", missing)
    return run_card.draw(cases["pmp_baseline"], "results_reference_card.png")


def peg_vs_reference(cases):
    """PEG under the coast-parameter and direct-insertion architectures, and the
    reference.

    Panel (a) is the arc structure: the reference coasts for most of its
    flight, and the question is what the two PEG flights do instead. Panel (b)
    is limited to the steered part of each flight; the reference's final burn,
    at the end of its coast, is a fraction of a second (see _panels.alpha_panel
    for the clipping).
    """
    names = ("pmp_baseline", "peg_baseline", "peg_direct")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("peg vs reference", missing)

    entries = [
        (cases["pmp_baseline"], st.REFERENCE, "-", "Reference (indirect PMP)"),
        (cases["peg_baseline"], st.BASELINE, "-",
         "PEG, %s" % st.arch_label("pso_coast")),
        (cases["peg_direct"], st.VARIANT, "--", "PEG, %s" % st.arch_label("direct")),
    ]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    pn.altitude_panel(ax_a, entries)
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right")

    pn.alpha_panel(ax_b, entries)
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_peg_vs_reference.png")


def peg_waypoint(cases):
    """One law, two targets for its first burn: PEG aimed at the orbit under
    the coast-parameter architecture, and at the reference's coast-start
    waypoint, with the reference."""
    return pn.waypoint_figure(cases, "peg_baseline", "show_ref_track", "PEG",
                              "results_peg_waypoint.png")


def peg_atmosphere(cases):
    """PEG at the baseline against the drag-free run.

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

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, colour, label in ((base, st.BASELINE, "With atmosphere"),
                                (vac, st.VARIANT, "No atmosphere")):
        _t, s_km, alt_km = pn.to_insertion(case, case.downrange_km, case.alt_km)
        s_km, alt_km = st.thin(s_km, alt_km)
        ax_a.plot(s_km, alt_km, color=colour, label=label)
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right")

    for case, colour, tag in ((base, st.BASELINE, "atm."), (vac, st.VARIANT, "vac.")):
        hist = case.loss_histories()
        t = case.time[:case.cutoff_index()]
        t, grav, drag = st.thin(t, hist["gravity"], hist["drag"])
        ax_b.plot(t, grav, color=colour, label="Gravity (%s)" % tag)
        ax_b.plot(t, drag, color=colour, linestyle="--",
                  label="Drag (%s)" % tag)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Cumulative loss [m/s]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend_loc="upper left")

    fig.tight_layout()
    return st.save(fig, "results_peg_atmosphere.png")


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


def reference_trajectory(cases):
    """The atmospheric and drag-free references, and how linear-tangent their
    steering is.

    Panel (a) marks the atmospheric reference's coast-start waypoint, which
    Sections 6.4 to 6.6 aim other laws at. Panel (b) shows the pitch, because
    the question Chapter 4 raises about this trajectory is whether the optimal
    steering carries the linear-tangent signature the analytical treatment
    predicts; the form is fitted to each Stage-2 burn long enough to fit.
    Stage 1 is not fitted: it is the fixed gravity turn every case shares, flown
    before any costate exists.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("reference trajectory", missing)

    ref = cases["pmp_baseline"]
    entries = [(ref, st.REFERENCE, "-", "Reference")]
    if "pmp_vacuum" in cases:
        entries.append((cases["pmp_vacuum"], st.REFERENCE, "--",
                        "Reference, no atmosphere"))

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    pn.altitude_panel(ax_a, entries)
    wp = pn.waypoint(ref)
    if wp is not None:
        pn.mark_waypoint(ax_a, wp)
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right")

    t_hi = 1.12 * max(pn.steering_end(case) for case, *_ in entries)
    fits, too_short = [], []
    for case, colour, style, label in entries:
        t, th = st.thin(case.time, case.theta_deg)
        ax_b.plot(t, th, color=colour, linestyle=style, label=label)
        for t0, t1 in pn.burn_intervals(case, stage2_only=True):
            if t1 - t0 < MIN_FIT_S:
                too_short.append(t1 - t0)
            else:
                fits.append(_tangent_fit(case, t0, t1))
    # Drawn after both references so the legend lists the flown curves first.
    for i, (t_fit, th_fit, _err) in enumerate(fits):
        ax_b.plot(t_fit, th_fit, color=st.INK, linestyle=":", linewidth=1.1,
                  label=None if i else "Linear-tangent fit")
    ax_b.set_xlim(0.0, t_hi)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Pitch $\theta$ [deg]")
    notes = []
    if fits:
        notes.append("fit RMS: " + ", ".join("%.2f" % f[2] for f in fits)
                     + r"$^\circ$")
    if too_short:
        notes.append("final burns (%s s) too short to fit"
                     % ", ".join("%.2f" % d for d in too_short))
    for i, text in enumerate(notes):
        ax_b.annotate(text, xy=(0.97, 0.62 - 0.08 * i), xycoords="axes fraction",
                      fontsize=6.5, color=st.GREY, ha="right", va="top")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend_loc="upper right")

    fig.tight_layout()
    return st.save(fig, "results_reference_trajectory.png")


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


FIGURES = [reference_card, reference_trajectory, peg_vs_reference, peg_atmosphere,
           peg_waypoint]
