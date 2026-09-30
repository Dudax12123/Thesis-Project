"""Section 6.2 figures -- the gravity turn along its three secondary axes.

Four figures. The first is the baseline flight as a run card; the other three
are each the baseline overlaid with the single case that differs from it. The
atmosphere is not among them: since 2026-09-29 it is varied on powered explicit
guidance (sec63_peg.peg_atmosphere), and gt_vacuum and gt_direct are archived
but not reported.

Outputs
-------
results_gt_baseline_card.png   fig:gt_baseline_card
results_gt_architecture.png    fig:gt_architecture
results_gt_rotation.png        fig:gt_rotation
results_gt_engine.png          fig:gt_engine
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


def _overlay_trajectory(ax, entries, to_insertion=False):
    """Altitude against downrange for several cases, in the shared house style.

    *to_insertion* drops the orbit each archive carries after insertion, for a
    figure whose cases insert at very different times.
    """
    for case, colour, label, style in entries:
        end = case.insertion_index() if to_insertion else len(case.time)
        s_km, alt_km = st.thin(case.downrange_km[:end], case.alt_km[:end])
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


def architecture(cases):
    """The same law under the coast-parameter and the apogee-check architectures.

    The direct-insertion architecture is compared on powered explicit guidance
    instead (sec63_peg.peg_vs_reference); gt_direct is archived, not reported.
    """
    names = ("gt_baseline", "gt_apogee")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("gt architecture", missing)

    colours = (st.BASELINE, st.VARIANT2)
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)

    _overlay_trajectory(ax_a, [
        (cases[name], colour, st.arch_label(cases[name].architecture), "-")
        for name, colour in zip(names, colours)], to_insertion=True)
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    # (b) the same two in time, with each one's arc structure shaded: a guided
    # final burn after the swarm's coast against a coast to apogee closed by an
    # impulsive circularisation.
    for name, colour in zip(names, colours):
        case = cases[name]
        end = case.insertion_index()
        t, alt_km = st.thin(case.time[:end], case.alt_km[:end])
        ax_b.plot(t, alt_km, color=colour, label=st.arch_label(case.architecture))
        for t0, t1 in case.coast_intervals():
            ax_b.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Altitude [km]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_gt_architecture.png")


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
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="center left")

    # The two nozzle models are compared on what reaches orbit, and that
    # difference is a couple of tonnes against a 517 t launch mass -- invisible
    # on an axis scaled to the launch mass. The inset zooms the post-MECO tail,
    # where the whole of the difference is.
    # The inset starts at half width so the MECO labels beside the cut-off
    # lines stay clear of its tick labels.
    ax_zoom = ax_b.inset_axes([0.50, 0.38, 0.47, 0.56])
    end_labels = []
    meco_row = 0
    for case, colour in ((base, st.BASELINE), (sl, st.VARIANT)):
        t, m = st.thin(case.time, case.mass)
        ax_b.plot(t, m / 1e3, color=colour,
                  label=st.nozzle_label(case.row.get("thrust_1_mode", "?")))
        if case.t_meco is not None:
            ax_b.axvline(case.t_meco, color=colour, linestyle=":", linewidth=0.9)
            # The two cut-offs are under 2 s apart on a ~1800 s axis, so their
            # lines coincide and rotated labels would print on top of each
            # other. They are stacked beside the lines instead, to the tenth of
            # a second the shift is measured in.
            ax_b.annotate("MECO %.1f s" % case.t_meco,
                          xy=(case.t_meco, 0.97 - 0.08 * meco_row),
                          xycoords=("data", "axes fraction"), xytext=(4, 0),
                          textcoords="offset points", fontsize=6.5, color=colour,
                          ha="left", va="top")
            meco_row += 1

        tail = case.time >= (case.t_meco or 0.0)
        t_tail, m_tail = st.thin(case.time[tail], case.mass[tail])
        ax_zoom.plot(t_tail, m_tail / 1e3, color=colour, linewidth=1.0)
        end_labels.append(ax_zoom.annotate(
            "%.1f t" % (m_tail[-1] / 1e3),
            xy=(t_tail[-1], m_tail[-1] / 1e3), xytext=(3, 0),
            textcoords="offset points", fontsize=6.5, color=colour))

    ax_zoom.set_title("After MECO", fontsize=6.5, pad=2)
    ax_zoom.tick_params(labelsize=6)
    ax_zoom.margins(x=0.22)
    for side in ("top", "right"):
        ax_zoom.spines[side].set_visible(False)

    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Total mass [t]")
    st.panel_tag(ax_b, "b")
    # No legend here: it would repeat panel (a)'s in the same two colours, and
    # the only corner with room for it is the one the inset occupies.
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    st.dodge_labels(fig, end_labels)
    return st.save(fig, "results_gt_engine.png")


FIGURES = [baseline_card, architecture, rotation, engine]
