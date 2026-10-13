"""Section 6.2 figures -- the gravity turn along its three secondary axes.

The baseline flight as a run card, then one figure per secondary axis, each the
baseline against the single case that differs from it (red-note review S2.2,
2026-10-08: the 2x2 figure of the three axes is split so that each subsection
presents its own). The atmosphere is not among the axes: it is varied on powered
explicit guidance, and gt_vacuum and gt_direct are archived but not reported.

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
from . import _panels as pn
from . import _style as st
from . import run_card


# Every label of a figure redrawn for the 10 pt rule (figure review,
# 2026-10-12): drawn at 6.3 in and placed at \textwidth, so this is close to
# its printed size.
FS = 10.0

# Section 6.2.3 (figure review, 2026-10-12): the rotating flight orange, the
# non-rotating one blue.
ROTATING_COLOUR = st.ENV_COLORS["no_rotation"]
NON_ROTATING_COLOUR = st.ENV_COLORS["baseline"]

# The apogee check in Section 6.2.2, in red (figure review, 2026-10-12).
APOGEE_COLOUR = st.ACCENT


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


def gt_architecture(cases):
    """The same law under the coast-parameter and the apogee-check architectures:
    altitude against time, each architecture's coast shaded -- a guided final
    burn after the swarm's coast, against a coast to apogee closed by an
    impulsive circularization. The direct-insertion architecture is compared on
    powered explicit guidance instead.

    Since the figure review of 2026-10-12 the propellant panel is gone (Table
    6.3 gives the propellant remaining), the apogee check is drawn solid in its
    own colour rather than dashed ink, the coast shading is in the legend, and
    every label is drawn at FS.
    """
    names = ("gt_baseline", "gt_apogee")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("gt architecture", missing)
    entries = [(cases["gt_baseline"], st.ENV_COLORS["baseline"]),
               (cases["gt_apogee"], APOGEE_COLOUR)]

    fig, ax = plt.subplots(figsize=(6.3, 2.8))
    for case, colour in entries:
        end = case.insertion_index()
        name = st.arch_label(case.architecture)
        t, alt_km = st.thin(case.time[:end], case.alt_km[:end])
        ax.plot(t, alt_km, color=colour, label=name)
        for i, (t0, t1) in enumerate(case.coast_intervals()):
            ax.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0,
                       label=None if i else "Coast (%s)" % name)
    ax.set_xlabel("Time [s]", fontsize=FS)
    ax.set_ylabel("Altitude [km]", fontsize=FS)
    ax.tick_params(labelsize=FS)
    st.tidy(ax, legend_loc="lower right", legend_kw={"fontsize": FS})
    fig.tight_layout()
    return st.save(fig, "results_gt_architecture.png")


def gt_rotation(cases):
    """The gravity turn with and without the rotation of the Earth.

    (a) Altitude against time for both. (b) The pseudo-forces along the baseline
    on the launch azimuth (_data.Case.pseudo_forces_at_azimuth): the Coriolis and
    centrifugal magnitudes and the cross-plane component the planar model omits,
    with the lateral force m|a_h,perp| an actuator would supply to cancel it on the
    right axis (red-note review S2.2: the lateral load Chapter 2 defines). Panel
    (b) stops where the first second-stage burn ends: no actuator acts in a
    coast, where the cross-plane term only describes the ground turning beneath
    a Keplerian arc. The non-rotating flight has no pseudo-forces to draw.

    Figure review, 2026-10-12: the rotating flight is orange and the
    non-rotating one blue; every force is a continuous line, in its own colour;
    the in-plane terms say so; every label is drawn at FS, and the legend of
    (b) runs under both panels.
    """
    names = ("gt_baseline", "gt_norot")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("gt rotation", missing)
    base, norot = cases["gt_baseline"], cases["gt_norot"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(6.3, 3.4),
                                     gridspec_kw={"width_ratios": [1.0, 1.15]})
    for case, colour, label in ((base, ROTATING_COLOUR, "Rotating Earth"),
                                (norot, NON_ROTATING_COLOUR,
                                 "Non-rotating Earth")):
        end = case.insertion_index()
        t, alt_km = st.thin(case.time[:end], case.alt_km[:end])
        ax_a.plot(t, alt_km, color=colour, label=label)
    ax_a.set_xlabel("Time [s]", fontsize=FS)
    ax_a.set_ylabel("Altitude [km]", fontsize=FS)
    ax_a.tick_params(labelsize=FS)
    ax_a.set_title("(a)", loc="left", color=st.INK, fontweight="bold", fontsize=FS)
    st.tidy(ax_a, legend_loc="lower right", legend_kw={"fontsize": FS})

    forces = base.pseudo_forces_at_azimuth()
    if forces is not None:
        t_end = base.t_coast_start or float(base.time[base.insertion_index() - 1])
        sel = base.time <= t_end
        t, cor, cen, cross, mass = st.thin(base.time[sel], *(f[sel] for f in forces),
                                           base.mass[sel])
        ax_b.plot(t, cor, color=NON_ROTATING_COLOUR, label="Coriolis (in plane)")
        ax_b.plot(t, cen, color=ROTATING_COLOUR, label="Centrifugal (in plane)")
        ax_b.plot(t, cross, color=st.INK, label=r"Cross-plane $|a_{h,\perp}|$")
        ax_f = ax_b.twinx()
        ax_f.plot(t, mass * cross / 1e3, color=st.VIOLET,
                  label=r"Lateral force $m|a_{h,\perp}|$ (right)")
        ax_f.set_ylabel(r"Lateral force $m|a_{h,\perp}|$ [kN]", fontsize=FS)
        ax_f.tick_params(labelsize=FS)
        ax_f.set_ylim(0.0, None)
        ax_f.spines["top"].set_visible(False)
        if base.t_meco is not None:
            ax_b.axvline(base.t_meco, color=st.GREY, linestyle=":", linewidth=0.8)
            # Right of the line, in the gap between the Coriolis curve and the
            # lateral force: at 10 pt, left of it lands on the lateral force.
            ax_b.annotate("MECO", xy=(base.t_meco, 0.64), xycoords=("data", "axes fraction"),
                          xytext=(3, 0), textcoords="offset points", fontsize=FS,
                          color=st.INK, rotation=90, ha="left", va="top")
        ax_b.set_xlim(0.0, t_end)
        handles, texts = [], []
        for ax in (ax_b, ax_f):
            h, l = ax.get_legend_handles_labels()
            handles += h
            texts += l
    ax_b.set_ylim(0.0, None)
    ax_b.set_xlabel("Time [s]", fontsize=FS)
    ax_b.set_ylabel(r"Acceleration [m/s$^2$]", fontsize=FS)
    ax_b.tick_params(labelsize=FS)
    ax_b.set_title("(b)", loc="left", color=st.INK, fontweight="bold", fontsize=FS)
    st.tidy(ax_b, legend=False)
    fig.tight_layout(rect=(0.0, 0.15, 1.0, 1.0))
    if forces is not None:
        # Under both panels: every corner of (b) holds a curve, and at 10 pt
        # the four entries need the full width.
        fig.legend(handles, texts, loc="lower center", bbox_to_anchor=(0.5, 0.0),
                   ncol=2, fontsize=FS, frameon=False)
    return st.save(fig, "results_gt_rotation.png")


def gt_engine(cases):
    """The pressure-dependent nozzle against a constant sea-level one, over the
    first stage: (a) thrust, (b) specific impulse, (c) mass flow, with each
    flight's main-engine cut-off dotted.

    The mass flow is read from the mass trace, so it is the one actually flown,
    and the specific impulse is the thrust over it; the fairing jettison, a
    discrete drop, is excluded from both (_mass_flow).

    Every label is drawn at FS (figure review, 2026-10-12); at that size the
    legend runs under the three panels.
    """
    names = ("gt_baseline", "gt_sea_level_engine")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("gt engine", missing)
    from Auxiliary import constants as const

    fig, axes = plt.subplots(1, 3, figsize=(6.3, 3.0))
    for case, colour in ((cases["gt_baseline"], st.ENV_COLORS["baseline"]),
                         (cases["gt_sea_level_engine"], st.ENV_COLORS["sea_level"])):
        label = {"pressure": "Pressure-dependent", "sea_level": "Sea level, constant"}.get(
            case.row.get("thrust_1_mode"), st.nozzle_label(case.row.get("thrust_1_mode", "?")))
        t_end = case.t_meco if case.t_meco else float(case.time[-1])
        sel = case.time <= t_end
        t_d, mdot, _steps = _mass_flow(case.time[sel], case.mass[sel])
        thrust = np.interp(t_d, case.time[sel], case.thrust[sel])
        # The cut-off itself: the last samples' thrust has fallen while the
        # mass trace's difference has not, which would read as an Isp of zero.
        burning = (mdot > 0.5 * np.median(mdot)) & (thrust > 0.5 * np.max(thrust))
        t_d, mdot, thrust = t_d[burning], mdot[burning], thrust[burning]
        isp = thrust / (mdot * const.G_0)
        for ax, values in zip(axes, (thrust / 1e6, isp, mdot)):
            ax.plot(*st.thin(t_d, values), color=colour, label=label)
            ax.axvline(t_end, color=colour, linestyle=":", linewidth=0.8)
    for ax, ylabel, tag in zip(axes, ("Thrust [MN]", "Specific impulse [s]",
                                      "Mass flow [kg/s]"), "abc"):
        ax.set_xlabel("Time [s]", fontsize=FS)
        ax.set_ylabel(ylabel, fontsize=FS)
        ax.tick_params(labelsize=FS)
        ax.set_title("(%s)" % tag, loc="left", color=st.INK, fontweight="bold",
                     fontsize=FS)
        st.tidy(ax, legend=False)
    axes[2].ticklabel_format(axis="y", style="plain", useOffset=False)
    axes[0].plot([], [], color=st.GREY, linestyle=":", linewidth=0.8, label="MECO")
    fig.tight_layout(rect=(0.0, 0.1, 1.0, 1.0))
    fig.legend(*axes[0].get_legend_handles_labels(), loc="lower center",
               bbox_to_anchor=(0.5, 0.0), ncol=3, fontsize=FS, frameon=False)
    return st.save(fig, "results_gt_engine.png")


FIGURES = [baseline_card, gt_architecture, gt_rotation, gt_engine]


