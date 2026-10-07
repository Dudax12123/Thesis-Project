"""Panels shared by the Chapter 6 trajectory comparisons.

Several figures draw the same pair: altitude against time up to insertion, and
the commanded angle of attack over the steered part of the ascent. They differ
only in which cases they overlay and in what they mark on top -- the
reference's coast-start waypoint, a segmented schedule's hand-off -- so the
pair is drawn here once and each figure passes its cases and its marks.

An *entry* is ``(case, colour, linestyle, label)``, in drawing order.
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
from . import _style as st

# A coast longer than this ends the steered part of a flight for the alpha
# panel. The reference and the cases aimed at its waypoint coast for ~1450 s
# and then burn for a fraction of a second, so a time axis that runs to their
# insertion squeezes all of the steering into its first fifth. A short coast
# between two real burns -- the Apollo flight's 37 s -- is kept inside.
LONG_COAST_S = 300.0


def to_insertion(case, *channels):
    """The time axis and *channels* up to insertion, dropping the orbit after it.

    Every pso_coast archive carries some 1000 s of flight after insertion. On a
    time axis that ends where the reference inserts, that tail would read as
    flight still in progress.
    """
    end = case.insertion_index()
    return (case.time[:end],) + tuple(np.asarray(ch)[:end] for ch in channels)


def burn_intervals(case, stage2_only=False, floor_frac=0.01):
    """Powered spans of the ascent up to SECO, as [(t_start, t_end), ...].

    Read from the thrust trace, like Case.coast_intervals, because every
    architecture records thrust and not every one records its arc times. The
    apogee check's impulsive circularisation is not in the thrust trace and is
    not returned; Case.t_insertion gives its instant.
    """
    end = case.cutoff_index()
    time, thrust = case.time[:end], case.thrust[:end]
    if not len(thrust):
        return []
    on = thrust > floor_frac * float(np.max(thrust))
    spans, start = [], None
    for i, powered in enumerate(on):
        if powered and start is None:
            start = time[i]
        elif not powered and start is not None:
            spans.append((float(start), float(time[i - 1])))
            start = None
    if start is not None:
        spans.append((float(start), float(time[-1])))
    spans = [(a, b) for a, b in spans if b > a]
    if stage2_only and case.t_meco is not None:
        spans = [(a, b) for a, b in spans if a > case.t_meco + 0.5]
    return spans


def steering_end(case):
    """When the steered part of *case* ends: its first long coast, else SECO."""
    for t0, t1 in case.coast_intervals():
        if t1 - t0 > LONG_COAST_S:
            return t0
    return case.t_seco if case.t_seco is not None else float(case.time[-1])


def waypoint(ref):
    """The reference's state where its first Stage-2 burn ends and its coast begins.

    This is the coast-start waypoint that the reference-tracking and segmented
    cases aim their first burn at: those archives store the same state as
    ``arc1_target`` (radius, speed, flight-path angle). It is read off the
    reference trajectory so a figure without those cases can still mark it.
    Returns None when the reference records no coast.
    """
    if ref is None or ref.t_coast_start is None:
        return None
    i = min(int(np.searchsorted(ref.time, ref.t_coast_start)), len(ref.time) - 1)
    return {"t": float(ref.time[i]), "alt_km": float(ref.alt_km[i]),
            "v_kms": float(ref.v[i]) / 1e3, "gamma_deg": float(ref.gamma_deg[i])}


def mark_waypoint(ax, wp):
    """The waypoint as an open circle on an altitude-against-time panel.

    Its state goes in the legend entry rather than beside the marker: the
    marker sits where the trajectories bend into their coast, and a label there
    lands on the curves or on the legend.
    """
    ax.plot(wp["t"], wp["alt_km"], linestyle="none", marker="o", markersize=5.0,
            markerfacecolor="white", markeredgecolor=st.INK,
            markeredgewidth=1.1, zorder=5,
            label="Coast-start waypoint\n(%.0f km, %.2f km/s, "
                  % (wp["alt_km"], wp["v_kms"])
                  + r"$\gamma$ %.1f$^\circ$)" % wp["gamma_deg"])


def altitude_panel(ax, entries, shade_coasts=True):
    """Altitude against time up to insertion, each case's coasts in its colour."""
    for case, colour, style, label in entries:
        t, alt_km = to_insertion(case, case.alt_km)
        t, alt_km = st.thin(t, alt_km)
        ax.plot(t, alt_km, color=colour, linestyle=style, label=label)
        if shade_coasts:
            for t0, t1 in case.coast_intervals():
                ax.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Altitude [km]")


def alpha_panel(ax, entries, t_hi=None):
    """The commanded angle of attack over the steered part of the ascent.

    The x-range ends a little after the latest :func:`steering_end` of the
    entries. The y-range covers the bulk of each flight's steering (2nd-98th
    percentile of its steered samples), so a last-seconds swing of +-85 deg
    does not flatten a +-8 deg reference; the clipped peaks are printed rather
    than hidden.
    """
    if t_hi is None:
        t_hi = 1.06 * max(steering_end(case) for case, *_ in entries)

    shown = []
    lo, hi = 0.0, 0.0
    for case, _colour, _style, _label in entries:
        t, al = to_insertion(case, case.alpha_deg)
        al = al[t <= t_hi]
        shown.append(al)
        steered = al[np.abs(al) > 1e-6]
        if steered.size:
            lo = min(lo, float(np.nanpercentile(steered, 2.0)))
            hi = max(hi, float(np.nanpercentile(steered, 98.0)))
    pad = 0.12 * max(hi - lo, 1.0)
    lo, hi = lo - pad, hi + pad

    def _clipped(top):
        out = []
        for (_case, colour, _style, label), al in zip(entries, shown):
            if al.size:
                peaks = [v for v in (float(np.nanmin(al)), float(np.nanmax(al)))
                         if v < lo or v > top]
                if peaks:
                    out.append((colour, label, peaks))
        return out

    # Headroom for the clipped-peak notes, which sit along the top of the
    # panel: without it they print over a flight that steers up to the limit.
    # Recounted after the extension, since the headroom can bring a peak in.
    clipped = _clipped(hi)
    if clipped:
        hi += 0.09 * len(clipped) * (hi - lo) / (1.0 - 0.09 * len(clipped))
        clipped = _clipped(hi)

    for case, colour, style, label in entries:
        t, al_full = to_insertion(case, case.alpha_deg)
        ax.plot(t, al_full, color=colour, linestyle=style, label=label)
    ax.axhline(0.0, color=st.FAINT, linewidth=0.8)
    ax.set_ylim(lo, hi)
    ax.set_xlim(0.0, t_hi)
    for i, (colour, label, peaks) in enumerate(clipped):
        ax.annotate("%s: peaks %s" % (label, ", ".join("%+.0f" % v for v in peaks))
                    + r"$^\circ$, clipped",
                    xy=(0.02, 0.95 - 0.07 * i), xycoords="axes fraction",
                    fontsize=6.3, color=colour, va="top")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Angle of attack $\alpha$ [deg]")
    return len(clipped)


def mark_instant(ax, t, text, colour=st.INK):
    """A dotted vertical line on a time panel, named along its foot."""
    ax.axvline(t, color=colour, linestyle=":", linewidth=0.8)
    ax.annotate(text, xy=(t, 0.03), xycoords=("data", "axes fraction"),
                xytext=(-2, 0), textcoords="offset points", fontsize=6.3,
                color=colour, rotation=90, ha="right", va="bottom")


def mark_arc_events(ax, entries, label_top=False):
    """Dotted lines at main-engine cut-off and at every second-stage cut-off.

    MECO is the same instant for every case, so it is drawn once and named --
    along the panel's foot, or its top where the curves pass through the foot;
    each case's cut-offs are drawn in its own colour and named once, by a legend
    entry, since four sets of rotated labels would land on one another. Lines
    past the panel's x-range are clipped by the axes.
    """
    t_meco = next((case.t_meco for case, *_ in entries if case.t_meco), None)
    if t_meco is not None and label_top:
        ax.axvline(t_meco, color=st.INK, linestyle=":", linewidth=0.8)
        ax.annotate("MECO", xy=(t_meco, 0.97), xycoords=("data", "axes fraction"),
                    xytext=(-2, 0), textcoords="offset points", fontsize=6.3,
                    color=st.INK, rotation=90, ha="right", va="top")
    elif t_meco is not None:
        mark_instant(ax, t_meco, "MECO")
    for case, colour, _style, _label in entries:
        for _t0, t1 in burn_intervals(case, stage2_only=True):
            ax.axvline(t1, color=colour, linestyle=":", linewidth=0.8, alpha=0.9)
    ax.plot([], [], color=st.GREY, linestyle=":", linewidth=0.8,
            label="SECO (case colour)")


def waypoint_figure(cases, orbit_name, waypoint_name, law, filename, extra=()):
    """One law with its first burn aimed at the orbit and at the waypoint.

    The pso_coast flight aims its first burn at the final orbit across a coast
    it is never told about; the waypoint flight (segmented or reference-tracking)
    aims the same law at the reference's coast-start state. The reference is drawn with both, and
    the waypoint is marked in both panels. *extra* holds further
    ``(case_name, colour, linestyle, label)`` entries, drawn after the
    orbit-aimed flight.

    Both panels carry the arc structure: each case's coasts shaded in its
    colour, and dotted lines at MECO and at every cut-off (walkthrough S3-F1,
    2026-10-06).
    """
    names = ("pmp_baseline", orbit_name, waypoint_name) + tuple(e[0] for e in extra)
    missing = _data.missing_from(cases, *names)
    if missing:
        print("  [skip] %s -- missing %s" % (filename, ", ".join(missing)))
        return None

    ref = cases["pmp_baseline"]
    entries = [
        (ref, st.REFERENCE, "-", "Reference (indirect PMP)"),
        (cases[orbit_name], st.BASELINE, "-", "%s, first burn to the orbit" % law),
    ]
    entries += [(cases[name], colour, style, label)
                for name, colour, style, label in extra]
    entries.append(
        (cases[waypoint_name], st.VARIANT, "--", "%s, first burn to the waypoint" % law))
    wp = waypoint(ref)

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    altitude_panel(ax_a, entries)
    if wp is not None:
        mark_waypoint(ax_a, wp)
    mark_arc_events(ax_a, entries, label_top=True)
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right", legend_kw={"fontsize": 6.3})

    alpha_panel(ax_b, entries)
    for t0, t1 in [s for case, *_ in entries for s in case.coast_intervals()]:
        ax_b.axvspan(t0, t1, color=st.FAINT, alpha=0.35, linewidth=0)
    mark_arc_events(ax_b, entries)
    if wp is not None:
        mark_instant(ax_b, wp["t"], "waypoint")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, filename)
