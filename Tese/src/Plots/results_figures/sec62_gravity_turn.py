"""Section 6.2 figures -- the gravity turn along its four secondary axes.

Five figures. The first is the reporting template of Section 6.1, shown in full
once here and cited by every later section rather than repeated; the other four
are each the baseline overlaid with the single case that differs from it.

Outputs
-------
results_gt_baseline_card.png   F6.1
results_gt_architecture.png    F6.2
results_gt_atmosphere.png      F6.3
results_gt_rotation.png        F6.4
results_gt_engine.png          F6.5
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
from . import _style as st
from . import run_card


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def baseline_card(cases):
    """F6.1 -- the reporting template, in full, once.

    The drawing lives in run_card.py because main.py draws the same card for an
    interactive run under PLOT_SUITE = "new"; keeping one implementation means
    the figure the thesis prints and the figure seen while working cannot drift
    apart.
    """
    missing = _data.missing_from(cases, "gt_baseline")
    if missing:
        return _skip("F6.1 baseline card", missing)
    return run_card.draw(cases["gt_baseline"], "results_gt_baseline_card.png")


def _overlay_trajectory(ax, entries):
    """Altitude against downrange for several cases, in the shared house style."""
    for case, colour, label, style in entries:
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax.plot(s_km, alt_km, color=colour, label=label, linestyle=style)
    ax.set_xlabel("Downrange [km]")
    ax.set_ylabel("Altitude [km]")


def _derivative(time, values):
    """d(values)/d(time), tolerant of the repeated timestamps at arc joins.

    An archived time axis is the concatenation of separately propagated arcs, so
    the last sample of one arc and the first of the next share a timestamp --
    six or seven of them in a typical ascent, at the kick, at staging, at the
    coast boundaries and at SECO. np.gradient divides by the local step, so each
    duplicate becomes a division by zero and a NaN in the middle of an otherwise
    good curve. Dropping the repeats costs one sample each and leaves the
    derivative finite everywhere.
    """
    time = np.asarray(time, dtype=float)
    values = np.asarray(values, dtype=float)
    keep = np.ones(time.size, dtype=bool)
    keep[1:] = np.diff(time) > 0.0
    return time[keep], np.gradient(values[keep], time[keep])


def architecture(cases):
    """F6.2 -- the same law under three optimization architectures.

    The direct run is expected to finish suborbital and is drawn, annotated
    with its periapsis, rather than suppressed: paired with ``peg_direct`` in
    Section 6.3 it is the chapter's argument for why coast arcs exist.
    """
    names = ("gt_baseline", "gt_apogee", "gt_direct")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("F6.2 architecture", missing)

    colours = (st.BASELINE, st.VARIANT2, st.VARIANT)
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)

    entries = []
    for name, colour in zip(names, colours):
        case = cases[name]
        label = st.arch_label(case.architecture)
        if not case.reached_orbit:
            label += " (suborbital)"
        entries.append((case, colour, label, "-"))
    _overlay_trajectory(ax_a, entries)

    failed = [c for c in (cases[n] for n in names) if not c.reached_orbit]
    for case in failed:
        peri = case.row.get("periapsis_km")
        if peri is not None:
            ax_a.annotate("periapsis %.0f km" % peri,
                          xy=(case.downrange_km[-1], case.alt_km[-1]),
                          xytext=(-4, -12), textcoords="offset points",
                          fontsize=7, color=st.VARIANT, ha="right")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    # (b) the same three in time, with each one's arc structure shaded. The
    # single continuous burn of the direct architecture reads here as the
    # absence of a coast, which is the whole of the explanation.
    for name, colour in zip(names, colours):
        case = cases[name]
        t, alt_km = st.thin(case.time, case.alt_km)
        ax_b.plot(t, alt_km, color=colour, label=st.arch_label(case.architecture))
        for t0, t1 in case.coast_intervals():
            ax_b.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Altitude [km]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_gt_architecture.png")


def atmosphere(cases):
    """F6.3 -- baseline against the drag-free run.

    The two runs differ on two counts and not one: ``INCLUDE_DRAG=False`` is the
    master no-atmosphere switch, so the vacuum case also drops the fairing and
    flies vacuum thrust and vacuum Isp. Panel (b) shows both consequences at
    once -- the drag integral vanishes, and the gravity loss changes because the
    trajectory it is flown along has changed.
    """
    missing = _data.missing_from(cases, "gt_baseline", "gt_vacuum")
    if missing:
        return _skip("F6.3 atmosphere", missing)
    base, vac = cases["gt_baseline"], cases["gt_vacuum"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    _overlay_trajectory(ax_a, [
        (base, st.BASELINE, "With atmosphere", "-"),
        (vac, st.VARIANT, "No atmosphere", "-"),
    ])
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

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
    return st.save(fig, "results_gt_atmosphere.png")


def rotation(cases):
    """F6.4 -- baseline against the non-rotating Earth.

    Panel (b) is deliberately a null result for one of the two cases: the
    pseudo-force channels are recomputed under the same gate the equations of
    motion use, so the non-rotating run is identically zero rather than small.
    That is the evidence the switch did what it claims.
    """
    missing = _data.missing_from(cases, "gt_baseline", "gt_norot")
    if missing:
        return _skip("F6.4 rotation", missing)
    base, norot = cases["gt_baseline"], cases["gt_norot"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    _overlay_trajectory(ax_a, [
        (base, st.BASELINE, "Rotating Earth", "-"),
        (norot, st.VARIANT, "Non-rotating", "-"),
    ])
    st.panel_tag(ax_a, "a")
    # The legend is pinned rather than left on loc="best". Matplotlib scores the
    # artists in the axes and knows nothing about an inset_axes child, so "best"
    # picked the same lower-right corner as the latitude inset and drew the
    # legend straight through it.
    st.tidy(ax_a, legend_loc="center")

    lat = base.latitude_deg
    if lat is not None:
        ax_lat = ax_a.inset_axes([0.58, 0.08, 0.38, 0.26])
        t, lat_t = st.thin(base.time, lat)
        ax_lat.plot(t, lat_t, color=st.BASELINE, linewidth=1.0)
        ax_lat.set_title("Latitude [deg]", fontsize=6.5, pad=2)
        ax_lat.tick_params(labelsize=6)
        for side in ("top", "right"):
            ax_lat.spines[side].set_visible(False)

    for case, colour, tag in ((base, st.BASELINE, "rot."),
                              (norot, st.VARIANT, "non-rot.")):
        cor, cen = case.coriolis, case.centrifugal
        if cor is None or cen is None:
            continue
        t, cor, cen = st.thin(case.time, cor, cen)
        ax_b.plot(t, cor, color=colour, label="Coriolis (%s)" % tag)
        ax_b.plot(t, cen, color=colour, linestyle="--",
                  label="Centrifugal (%s)" % tag)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Pseudo-force accel. [m/s$^2$]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b)

    fig.tight_layout()
    return st.save(fig, "results_gt_rotation.png")


def engine(cases):
    """F6.5 -- the pressure-dependent nozzle against a constant sea-level one.

    This is the one figure in which thrust and mass flow earn separate curves.
    Everywhere else mass flow is the thrust trace rescaled by a constant, since
    Isp is fixed per stage; under the pressure model Isp varies with altitude,
    so the two curves genuinely differ, and that difference is the case.
    """
    missing = _data.missing_from(cases, "gt_baseline", "gt_sea_level_engine")
    if missing:
        return _skip("F6.5 engine model", missing)
    base, sl = cases["gt_baseline"], cases["gt_sea_level_engine"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)

    ax_mdot = ax_a.twinx()
    ax_mdot.set_ylabel("Mass flow [kg/s]  (dotted)")
    ax_mdot.spines["top"].set_visible(False)
    for case, colour in ((base, st.BASELINE), (sl, st.VARIANT)):
        label = "%s nozzle" % case.row.get("thrust_1_mode", "?")
        end = case.t_meco if case.t_meco else case.time[-1]
        sel = case.time <= end
        t, thr = st.thin(case.time[sel], case.thrust[sel])
        ax_a.plot(t, thr / 1e3, color=colour, label=label)
        # Mass flow from the mass trace itself, so it reflects the Isp actually
        # flown rather than a nominal value.
        t_d, mdot = _derivative(case.time[sel], case.mass[sel])
        t2, mdot = st.thin(t_d, -mdot)
        ax_mdot.plot(t2, mdot, color=colour, linestyle=":", linewidth=1.0)
    ax_a.set_xlabel("Time [s]")
    ax_a.set_ylabel("Stage-1 thrust [kN]  (solid)")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right")

    for case, colour in ((base, st.BASELINE), (sl, st.VARIANT)):
        t, m = st.thin(case.time, case.mass)
        ax_b.plot(t, m / 1e3, color=colour,
                  label="%s" % case.row.get("thrust_1_mode", "?"))
        if case.t_meco is not None:
            ax_b.axvline(case.t_meco, color=colour, linestyle=":", linewidth=0.9)
            ax_b.annotate("MECO %.0f s" % case.t_meco,
                          xy=(case.t_meco, 1.0), xycoords=("data", "axes fraction"),
                          xytext=(2, -9), textcoords="offset points",
                          fontsize=6.5, color=colour, rotation=90, va="top")
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Total mass [t]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b)

    fig.tight_layout()
    return st.save(fig, "results_gt_engine.png")


FIGURES = [baseline_card, architecture, atmosphere, rotation, engine]
