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


FIGURES = [reference_card, reference_trajectory, peg_vs_reference, peg_atmosphere,
           peg_waypoint]
