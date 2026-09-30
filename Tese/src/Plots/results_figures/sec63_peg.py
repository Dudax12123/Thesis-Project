"""Sections 6.1 and 6.3 figures -- the reference and powered explicit guidance.

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
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
from . import _style as st
from . import run_card


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def _to_insertion(case, *channels):
    """The time axis and *channels* up to insertion, dropping the orbit after it.

    Every archive carries some 1000 s of flight after insertion. On a time axis
    that ends where the reference inserts, that tail would read as flight still
    in progress.
    """
    end = case.insertion_index()
    return (case.time[:end],) + tuple(np.asarray(ch)[:end] for ch in channels)


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
    is limited to the first burn, which is where the steering differs; the
    reference's final burn, at the end of its coast, is a fraction of a second.
    The y-range covers the bulk of each flight's steering (2nd-98th percentile
    of its steered samples), so the coast-parameter flight's last-seconds swing
    of some +-85 deg does not flatten the reference's +-8 deg; the clipped
    peaks are printed rather than hidden.
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
    for case, colour, style, label in entries:
        t, alt_km = _to_insertion(case, case.alt_km)
        t, alt_km = st.thin(t, alt_km)
        ax_a.plot(t, alt_km, color=colour, linestyle=style, label=label)
        for t0, t1 in case.coast_intervals():
            ax_a.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
    ax_a.set_xlabel("Time [s]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right")

    ref = cases["pmp_baseline"]
    peg_ends = [c.t_seco for c, _col, _s, _l in entries[1:] if c.t_seco is not None]
    first_burn_end = [t for t in peg_ends + [ref.t_coast_start] if t is not None]
    t_hi = 1.06 * max(first_burn_end) if first_burn_end else None

    shown = []
    lo, hi = 0.0, 0.0
    for case, _colour, _style, _label in entries:
        t, al = _to_insertion(case, case.alpha_deg)
        al = al[t <= (t_hi if t_hi is not None else t[-1])]
        shown.append(al)
        steered = al[np.abs(al) > 1e-6]
        if steered.size:
            lo = min(lo, float(np.nanpercentile(steered, 2.0)))
            hi = max(hi, float(np.nanpercentile(steered, 98.0)))
    pad = 0.12 * max(hi - lo, 1.0)
    lo, hi = lo - pad, hi + pad

    clipped = []
    for (case, colour, style, label), al in zip(entries, shown):
        t, al_full = _to_insertion(case, case.alpha_deg)
        ax_b.plot(t, al_full, color=colour, linestyle=style, label=label)
        peaks = [v for v in (float(np.nanmin(al)), float(np.nanmax(al)))
                 if v < lo or v > hi]
        if peaks:
            clipped.append((colour, label, peaks))
    ax_b.axhline(0.0, color=st.FAINT, linewidth=0.8)
    ax_b.set_ylim(lo, hi)
    if t_hi is not None:
        ax_b.set_xlim(0.0, t_hi)
    for i, (colour, label, peaks) in enumerate(clipped):
        ax_b.annotate("%s: peaks %s" % (label, ", ".join("%+.0f" % v for v in peaks))
                      + r"$^\circ$, clipped",
                      xy=(0.02, 0.95 - 0.07 * i), xycoords="axes fraction",
                      fontsize=6.3, color=colour, va="top")
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Angle of attack $\alpha$ [deg]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_peg_vs_reference.png")


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
        _t, s_km, alt_km = _to_insertion(case, case.downrange_km, case.alt_km)
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


def reference_trajectory(cases):
    """The indirect optimum, with the flyable baselines behind it.

    The two flyable laws are drawn faint rather than omitted so the yardstick is
    seen against what it is a yardstick for. Panel (b) shows the pitch, because
    the question Chapter 4 raises about this trajectory is whether the optimal
    steering carries the near-linear-tangent signature the analytical treatment
    predicts.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("reference trajectory", missing)

    primary = [(cases["pmp_baseline"], st.REFERENCE, "-", "Indirect PMP")]
    if "pmp_vacuum" in cases:
        primary.append((cases["pmp_vacuum"], st.REFERENCE, "--",
                        "Indirect PMP, no atmosphere"))
    # The two background traces share the faint grey; the linestyle is what
    # distinguishes them, since two identical faint entries in the legend
    # cannot be matched to their curves.
    context = [(case, st.FAINT, st.context_style(i), st.law_label(case.law))
               for i, case in enumerate(
                   cases[n] for n in ("gt_baseline", "peg_baseline")
                   if n in cases)]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, colour, style, label in context + primary:
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax_a.plot(s_km, alt_km, color=colour, linestyle=style, label=label,
                  linewidth=1.0 if colour == st.FAINT else 1.3)
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    for case, colour, style, label in context + primary:
        t, th = st.thin(case.time, case.theta_deg)
        ax_b.plot(t, th, color=colour, linestyle=style, label=label,
                  linewidth=1.0 if colour == st.FAINT else 1.3)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Pitch $\theta$ [deg]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_reference_trajectory.png")


FIGURES = [reference_card, reference_trajectory, peg_vs_reference, peg_atmosphere]
