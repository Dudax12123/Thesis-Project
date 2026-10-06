"""Section 6.2 figures -- the gravity turn along its three secondary axes.

Two figures. The first is the baseline flight as a run card; the second draws
each secondary axis as the baseline overlaid with the single case that differs
from it. The axes were three figures of their own until the outline review of
2026-10-04 (N6-03) merged them into three rows of two panels; the walkthrough
of 2026-10-06 (S2-F3) kept the four panels the text reads and dropped the two
that repeated another (the architecture in downrange, the rotating and
non-rotating trajectories, which overlap). The atmosphere is not among the axes:
it is varied on powered explicit guidance, and gt_vacuum and gt_direct are
archived but not reported.

Outputs
-------
results_gt_baseline_card.png   fig:gt_baseline_card
results_gt_axes.png            fig:gt_axes
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
    # The reference faint behind it, when the batch has it: the card is read
    # against Section 6.1's, and the overlay saves the reader the page-turn.
    return run_card.draw(cases["gt_baseline"], "results_gt_baseline_card.png",
                         background=cases.get("pmp_baseline"),
                         background_label="Reference (indirect PMP)")


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


def _mass_flow(time, mass, step_factor=10.0):
    """Mass flow -dm/dt of a burn, and the times of any discrete mass drops in it.

    The fairing leaves the vehicle as a ~1.9 t step taken inside one integrator
    sample (~1 ms), which a derivative across it reads as a flow of ~10^6 kg/s --
    enough to set the axis and flatten the real ~2.7 t/s curve onto zero. Any
    sample-to-sample drop faster than *step_factor* times the median burn rate
    is such a step: the trace is split there and each piece differentiated on
    its own, so the step is reported as an event rather than as flow.
    """
    time = np.asarray(time, dtype=float)
    mass = np.asarray(mass, dtype=float)
    keep = np.ones(time.size, dtype=bool)
    keep[1:] = np.diff(time) > 0.0
    time, mass = time[keep], mass[keep]

    rate = -np.diff(mass) / np.diff(time)
    typical = np.median(rate)
    cuts = np.where(rate > step_factor * typical)[0] if typical > 0.0 else []

    t_out, mdot_out, t_steps = [], [], []
    start = 0
    for end in list(cuts) + [time.size - 1]:
        if end - start >= 1:
            t_piece, dm = _derivative(time[start:end + 1], mass[start:end + 1])
            t_out.append(t_piece)
            mdot_out.append(-dm)
        if end < time.size - 1:
            t_steps.append(time[end])
        start = end + 1
    return np.concatenate(t_out), np.concatenate(mdot_out), t_steps


def _architecture_panel(cases, ax, tag):
    """The same law under the coast-parameter and the apogee-check architectures,
    in time, with each one's arc structure shaded: a guided final burn after the
    swarm's coast against a coast to apogee closed by an impulsive
    circularisation.

    The direct-insertion architecture is compared on powered explicit guidance
    instead (sec63_peg.peg_waypoint); gt_direct is archived, not reported.
    """
    for name, colour in (("gt_baseline", st.BASELINE), ("gt_apogee", st.VARIANT2)):
        case = cases[name]
        end = case.insertion_index()
        t, alt_km = st.thin(case.time[:end], case.alt_km[:end])
        ax.plot(t, alt_km, color=colour, label=st.arch_label(case.architecture))
        for t0, t1 in case.coast_intervals():
            ax.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Altitude [km]")
    st.panel_tag(ax, tag)
    st.tidy(ax, legend_loc="lower right")


def _rotation_panel(cases, ax, tag):
    """The pseudo-force accelerations the baseline flies, up to its insertion.

    The non-rotating run is not drawn: its channels are recomputed under the
    same gate the equations of motion use, so they are identically zero, and the
    caption says so rather than spending two legend entries on flat lines.
    """
    base = cases["gt_baseline"]
    cor, cen = base.coriolis, base.centrifugal
    if cor is not None and cen is not None:
        end = base.insertion_index()
        t, cor, cen = st.thin(base.time[:end], cor[:end], cen[:end])
        ax.plot(t, cor, color=st.BASELINE, label="Coriolis")
        ax.plot(t, cen, color=st.BASELINE, linestyle="--", label="Centrifugal")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Pseudo-force accel. [m/s$^2$]")
    st.panel_tag(ax, tag)
    st.tidy(ax)


def _engine_panels(cases, ax_a, ax_b, tags):
    """The pressure-dependent nozzle against a constant sea-level one.

    This is the one figure in which thrust and mass flow earn separate curves.
    Everywhere else mass flow is the thrust trace rescaled by a constant, since
    Isp is fixed per stage; under the pressure model Isp varies with altitude,
    so the two curves genuinely differ, and that difference is the case.
    Returns the end-of-trace labels of the mass panel, for st.dodge_labels once
    the whole figure is laid out.
    """
    base, sl = cases["gt_baseline"], cases["gt_sea_level_engine"]

    ax_mdot = ax_a.twinx()
    ax_mdot.set_ylabel("Mass flow [kg/s]  (dotted)")
    ax_mdot.spines["top"].set_visible(False)
    thrust_peak, mdot_shown, jettisons = 0.0, [], []
    for case, colour in ((base, st.BASELINE), (sl, st.VARIANT)):
        label = st.nozzle_label(case.row.get("thrust_1_mode", "?"))
        end = case.t_meco if case.t_meco else case.time[-1]
        sel = case.time <= end
        t, thr = st.thin(case.time[sel], case.thrust[sel])
        ax_a.plot(t, thr / 1e3, color=colour, label=label)
        thrust_peak = max(thrust_peak, float(np.max(thr)) / 1e3)
        # Mass flow from the mass trace itself, so it reflects the Isp actually
        # flown rather than a nominal value.
        t_d, mdot, t_steps = _mass_flow(case.time[sel], case.mass[sel])
        t2, mdot = st.thin(t_d, mdot)
        ax_mdot.plot(t2, mdot, color=colour, linestyle=":", linewidth=1.0)
        mdot_shown.append(mdot)
        jettisons.extend((t_s, colour) for t_s in t_steps)

    # The two quantities share the panel in separate bands: thrust along the
    # top, mass flow along the bottom, the legend in the empty middle. The mass
    # flow differs between the nozzles by ~1.6 %, so its axis is zoomed to its
    # own range -- on an axis from zero the difference the figure exists to show
    # would be a pixel.
    ax_a.set_ylim(0.0, 1.10 * thrust_peak)
    lo = min(float(np.min(m)) for m in mdot_shown)
    hi = max(float(np.max(m)) for m in mdot_shown)
    span = max(hi - lo, 1.0)
    ax_mdot.set_ylim(lo - 0.25 * span, hi + 2.6 * span)
    ax_mdot.ticklabel_format(axis="y", style="plain", useOffset=False)

    for i, (t_s, colour) in enumerate(sorted(jettisons)):
        ax_a.axvline(t_s, color=colour, linestyle="--", linewidth=0.7, alpha=0.7)
        if i == 0:
            ax_a.annotate("Fairing jettison", xy=(t_s, 0.62),
                          xycoords=("data", "axes fraction"), xytext=(-2, 0),
                          textcoords="offset points", fontsize=6.5, color=st.GREY,
                          rotation=90, ha="right", va="center")

    ax_a.set_xlabel("Time [s]")
    ax_a.set_ylabel("Stage-1 thrust [kN]  (solid)")
    st.panel_tag(ax_a, tags[0])
    st.tidy(ax_a, legend_loc="center left")

    # The two cut-offs are under 2 s apart, so their labels are stacked beside
    # the thrust cut-off rather than rotated onto each other, to the tenth of a
    # second the shift is measured in.
    for row, (case, colour) in enumerate(((base, st.BASELINE), (sl, st.VARIANT))):
        if case.t_meco is not None:
            ax_a.annotate("MECO %.1f s" % case.t_meco,
                          xy=(case.t_meco, 0.40 - 0.08 * row),
                          xycoords=("data", "axes fraction"), xytext=(-3, 0),
                          textcoords="offset points", fontsize=6.5, color=colour,
                          ha="right", va="top")

    # The two nozzle models are compared on what reaches orbit, and that
    # difference is a couple of tonnes against a 517 t launch mass -- invisible
    # on an axis scaled to the launch mass. This panel therefore draws the mass
    # from MECO to each flight's insertion, where the whole of the difference is.
    end_labels = []
    for case, colour in ((base, st.BASELINE), (sl, st.VARIANT)):
        start = int(np.searchsorted(case.time, case.t_meco or 0.0))
        tail = slice(start, case.insertion_index())
        t_tail, m_tail = st.thin(case.time[tail], case.mass[tail])
        ax_b.plot(t_tail, m_tail / 1e3, color=colour)
        end_labels.append(ax_b.annotate(
            "%.1f t" % (m_tail[-1] / 1e3),
            xy=(t_tail[-1], m_tail[-1] / 1e3), xytext=(3, 0),
            textcoords="offset points", fontsize=6.5, color=colour))
    ax_b.margins(x=0.14)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Mass after MECO [t]")
    st.panel_tag(ax_b, tags[1])
    # No legend: it would repeat panel (c)'s in the same two colours.
    st.tidy(ax_b, legend=False)
    return end_labels


def secondary_axes(cases):
    """The gravity turn along its three secondary axes, in four panels.

    (a) the architecture, (b) the pseudo-forces of the rotating Earth, (c) and
    (d) the Stage-1 engine model; each the baseline against the one case that
    differs from it, every curve stopping at its own insertion.
    """
    names = ("gt_baseline", "gt_apogee", "gt_sea_level_engine")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("gt secondary axes", missing)

    fig, axes = plt.subplots(2, 2, figsize=st.WIDE_4)
    _architecture_panel(cases, axes[0][0], "a")
    _rotation_panel(cases, axes[0][1], "b")
    end_labels = _engine_panels(cases, *axes[1], tags="cd")

    fig.tight_layout()
    st.dodge_labels(fig, end_labels)
    return st.save(fig, "results_gt_axes.png")


FIGURES = [baseline_card, secondary_axes]
