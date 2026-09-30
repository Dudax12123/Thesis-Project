"""Sections 6.5, 6.6 and 6.7 figures -- the remaining laws, the segmented
schedule, and the cost of the search.

Outputs
-------
results_showcase_laws.png      fig:showcase_laws
results_apollo_waypoint.png    fig:apollo_waypoint
results_segmented_handoff.png  fig:segmented_handoff
results_solve_cost.png         fig:solve_cost
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
from . import _panels as pn
from . import _style as st

# The remaining laws at the baseline, in the chapter's order: the open-loop
# laws, then Apollo. peg_new and apollo flying the reference's plan have their
# own figures (sec63_peg.peg_waypoint, apollo_waypoint below).
SHOWCASE = ["show_cpr", "show_linear_tangent", "show_bilinear_tangent",
            "show_exp_shooting", "show_apollo"]

# One representative case per architecture for the cost panel. gt_apogee and
# show_ref_track run no swarm and have no convergence curve to draw; their bars
# are there because that is itself part of the cost comparison.
COST_CASES = ["gt_baseline", "peg_direct", "pmp_baseline", "show_seg_opt_alt",
              "gt_apogee", "show_ref_track"]

# The coast strips along the foot of the showcase trajectory panel, in axes
# fractions: the first strip's bottom, each strip's height, and the pitch.
_STRIP_Y0, _STRIP_H, _STRIP_STEP = 0.03, 0.026, 0.040


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def showcase_laws(cases):
    """The remaining laws at the baseline, with the reference faint for scale.

    Panel (a) is altitude against time up to insertion. Each law's coasts are a
    strip in its colour along the foot of the panel rather than a span shaded
    over the curves: five overlapping translucent spans mix into colours no law
    has. Panel (b) is small multiples rather than an overlay because alpha is
    what distinguishes these laws from one another, and five alpha traces on
    shared axes would be a solid block. Each small panel keeps the same limits,
    and carries the reference's alpha faint behind the law's.
    """
    present = [n for n in SHOWCASE if n in cases]
    if not present:
        return _skip("showcase laws", SHOWCASE)
    ref = cases.get("pmp_baseline")
    # Green is left out: across the chapter it is the reference's colour, and
    # the reference is in this figure too (faint).
    palette = [c for c in st.VARIANT_CYCLE if c != st.REFERENCE]
    colours = {n: palette[i % len(palette)] for i, n in enumerate(present)}

    # Wider than the standard text-width figure: the trajectory legend sits
    # outside the axes, and the grid keeps its own width regardless.
    n_rows = -(-len(SHOWCASE) // 3)
    fig_h = 5.6 + 1.62 * (n_rows - 2)
    fig = plt.figure(figsize=(7.4, fig_h))
    grid = fig.add_gridspec(1 + n_rows, 3, height_ratios=[1.45] + [1.0] * n_rows,
                            hspace=0.62, wspace=0.35, right=0.80)
    ax_traj = fig.add_subplot(grid[0, :])

    strips = ([(ref, st.FAINT)] if ref is not None else []) + \
             [(cases[n], colours[n]) for n in present]
    if ref is not None:
        t, alt_km = st.thin(*pn.to_insertion(ref, ref.alt_km))
        ax_traj.plot(t, alt_km, color=st.FAINT, linewidth=1.2,
                     label="Reference (indirect PMP)", zorder=1)
    for name in present:
        case = cases[name]
        t, alt_km = st.thin(*pn.to_insertion(case, case.alt_km))
        ax_traj.plot(t, alt_km, color=colours[name], label=st.case_label(name),
                     zorder=2)
    for k, (case, colour) in enumerate(strips):
        y0 = _STRIP_Y0 + _STRIP_STEP * k
        for t0, t1 in case.coast_intervals():
            ax_traj.axvspan(t0, t1, ymin=y0, ymax=y0 + _STRIP_H, color=colour,
                            linewidth=0)
    ax_traj.annotate("coasts", xy=(0.995, _STRIP_Y0 + _STRIP_STEP * len(strips) / 2),
                     xycoords="axes fraction", fontsize=6.5, color=st.GREY,
                     ha="right", va="center")
    ax_traj.set_xlabel("Time [s]")
    ax_traj.set_ylabel("Altitude [km]")
    st.panel_tag(ax_traj, "a")
    st.tidy(ax_traj, legend=False)
    # Outside the axes: six entries over a trajectory panel cover the curves
    # they are labelling whichever corner they are put in.
    ax_traj.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), ncol=1,
                   fontsize=6.8)

    # Shared limits, so the small panels compare rather than merely coexist.
    # Taken from a percentile rather than the extremes: one law transients to
    # about -140 deg for a few seconds, and letting that set the range flattens
    # the others into a band a few pixels high. The clip is printed on the
    # panels it affects rather than hidden.
    alphas = {n: pn.to_insertion(cases[n], cases[n].alpha_deg) for n in present}
    stacked = np.concatenate([al for _t, al in alphas.values()])
    alpha_lo = float(np.nanpercentile(stacked, 0.5))
    alpha_hi = float(np.nanpercentile(stacked, 99.5))
    pad = 0.10 * max(alpha_hi - alpha_lo, 1.0)
    alpha_lo, alpha_hi = alpha_lo - pad, alpha_hi + pad
    t_hi = max(float(t[-1]) for t, _al in alphas.values())
    ref_alpha = pn.to_insertion(ref, ref.alpha_deg) if ref is not None else None

    first_small = None
    for i, name in enumerate(SHOWCASE):
        ax = fig.add_subplot(grid[1 + i // 3, i % 3])
        if first_small is None:
            first_small = ax
        if name not in cases:
            ax.axis("off")
            continue
        if ref_alpha is not None:
            ax.plot(*st.thin(*ref_alpha), color=st.FAINT, linewidth=0.9)
        t, al = alphas[name]
        ax.plot(*st.thin(t, al), color=colours[name], linewidth=1.0)
        ax.axhline(0.0, color=st.FAINT, linewidth=0.7)
        ax.set_xlim(0, t_hi)
        ax.set_ylim(alpha_lo, alpha_hi)
        # Say so when the shared range does not hold this law's full excursion,
        # rather than letting the curve run off the top or bottom silently.
        clipped = []
        if float(np.nanmin(al)) < alpha_lo:
            clipped.append("%.0f" % float(np.nanmin(al)))
        if float(np.nanmax(al)) > alpha_hi:
            clipped.append("%.0f" % float(np.nanmax(al)))
        if clipped:
            ax.annotate("peaks " + ", ".join(clipped) + r"$^\circ$",
                        xy=(0.97, 0.90), xycoords="axes fraction",
                        fontsize=6, color=st.GREY, ha="right")
        ax.set_title(st.case_label(name), fontsize=7.5, pad=3)
        ax.tick_params(labelsize=6.5)
        if i % 3 == 0:
            ax.set_ylabel(r"$\alpha$ [deg]", fontsize=7.5)
        # Label time on every panel with nothing beneath it, including the
        # panel above an empty last slot.
        if i + 3 >= len(SHOWCASE):
            ax.set_xlabel("Time [s]", fontsize=7.5)
        st.tidy(ax, legend=False)
    # The unused slots of the last row stay empty.
    for j in range(len(SHOWCASE), 3 * n_rows):
        fig.add_subplot(grid[1 + j // 3, j % 3]).axis("off")

    # 0.43 in above the first small panel: where the tag sat in the fixed
    # 5.6 in layout this replaced.
    fig.text(0.055, first_small.get_position().y1 + 0.4288 / fig_h, "(b)",
             fontsize=9, fontweight="bold", color=st.INK)
    return st.save(fig, "results_showcase_laws.png")


def apollo_waypoint(cases):
    """The Apollo law with its first burn aimed at the orbit (coast-parameter
    architecture) and at the coast-start waypoint, with the reference -- the
    format of sec63_peg.peg_waypoint."""
    return pn.waypoint_figure(cases, "show_apollo", "show_ref_track_apollo",
                              "Apollo", "results_apollo_waypoint.png")


def _handoff(case):
    """(time, altitude [km]) where a segmented flight hands over to its next law.

    The schedule stores the activation altitude; the instant is where the
    trajectory first reaches it.
    """
    schedule = case.segment_schedule or []
    if len(schedule) < 2:
        return None
    alt_km = schedule[1][1] / 1e3
    above = np.where(case.alt_km >= alt_km)[0]
    if not len(above):
        return None
    return float(case.time[above[0]]), alt_km


def segmented_handoff(cases):
    """Who chooses the hand-off altitude, and what it is worth.

    One law combination, twice: flown at the altitude Chapter 4's atmospheric
    and exoatmospheric division suggests, and with that altitude appended to
    the decision vector. The reference and PEG aimed alone at the same waypoint
    are drawn with them: the segmented schedule differs from the latter only in
    who steers before the hand-off.
    """
    names = ("show_seg_fixed_alt", "show_seg_opt_alt")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("segmented hand-off", missing)

    fixed, opt = cases["show_seg_fixed_alt"], cases["show_seg_opt_alt"]
    ref = cases.get("pmp_baseline")
    entries = []
    if ref is not None:
        entries.append((ref, st.REFERENCE, "-", "Reference (indirect PMP)"))
    if "show_ref_track" in cases:
        entries.append((cases["show_ref_track"], st.VARIANT2, "-.",
                        "PEG alone, first burn to the waypoint"))
    segmented = [(fixed, st.BASELINE, "-", "Segmented, hand-off fixed"),
                 (opt, st.VARIANT, "--", "Segmented, hand-off optimised")]
    entries += segmented

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    pn.altitude_panel(ax_a, entries, shade_coasts=False)
    # The hand-off as a point on each segmented curve, not a line across the
    # panel: a horizontal line at 66 or 120 km runs through the legend.
    for i, (case, colour, _style, _label) in enumerate(segmented):
        point = _handoff(case)
        if point is None:
            continue
        ax_a.plot(*point, linestyle="none", marker="D", markersize=4.2,
                  color=colour, zorder=5,
                  label="Hand-off" if i == 0 else None)
        ax_a.annotate("%.0f km" % point[1], xy=point, xytext=(-5, 3),
                      textcoords="offset points", fontsize=6.5, color=colour,
                      ha="right", va="bottom")
    wp = pn.waypoint(ref)
    if wp is not None:
        pn.mark_waypoint(ax_a, wp)
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right", legend_kw={"fontsize": 6.3})

    n_notes = pn.alpha_panel(ax_b, entries)
    for case, colour, _style, _label in segmented:
        point = _handoff(case)
        if point is not None:
            pn.mark_instant(ax_b, point[0], "hand-off", colour)
    if wp is not None:
        pn.mark_instant(ax_b, wp["t"], "waypoint")
    schedule = opt.segment_schedule or []
    if schedule:
        # Top left, under any clipped-peak notes: every flight is still on the
        # gravity turn there, commanding alpha = 0.
        ax_b.annotate("Schedule: " + r" $\rightarrow$ ".join(
                          st.law_label(law) for law, _a in schedule),
                      xy=(0.02, 0.95 - 0.07 * n_notes), xycoords="axes fraction",
                      fontsize=6.3, color=st.INK, va="top")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_segmented_handoff.png")


def _cost_note(case):
    """What one architecture's bar in the cost panel measures, in words.

    Returns (wall clock [min] or None when the matrix did not time the search,
    note). The indirect-PMP row is re-flown from a stored extremal, so its
    archived wall clock (about a second) and evaluation count (the batch's
    budget, not the extremal's) describe the re-flight; drawing them as a bar
    would present the cheapest step as the cost of the most expensive search.
    """
    wall = (case.row.get("wall_clock_s") or 0.0) / 60.0
    n_eval = case.row.get("n_evaluations") or 0
    config = case.manifest.get("config") or {}
    if case.extremal_budget is not None:
        particles, generations = case.extremal_budget
        return None, (r"not timed: extremal re-flown (%d$\times$%d swarm + polish, "
                      "offline)" % (particles, generations))
    shown = "%.0f min" % wall if wall >= 1.0 else "%.0f s" % (wall * 60.0)
    if case.architecture == "direct" and config.get("DIRECT_OPTIMIZER") == "grid_brent":
        return wall, "%s  (%d flights: kick grid + Brent, no swarm)" % (shown, n_eval)
    if case.architecture == "apogee_check":
        return wall, "%s  (kick grid, no swarm)" % shown
    if case.architecture == "reference_track":
        return wall, "%s  (no search)" % shown
    if n_eval:
        return wall, "%s  (%d evals)" % (shown, n_eval)
    return wall, "%s  (no swarm)" % shown


def solve_cost(cases):
    """What each architecture costs, and whether its swarm converged.

    Panel (a), the convergence curves, answers only whether a poor result came
    from the law or from a search that was stopped too early -- a failure mode
    this work has met, where an apparently incapable configuration proved
    merely under-converged. It ranks nothing. Only the coast-parameter and
    segmented architectures run a swarm inside the matrix. Panel (b) is the
    wall clock of one representative solve per architecture.
    """
    present = [n for n in COST_CASES if n in cases]
    if not present:
        return _skip("solve cost", COST_CASES)
    palette = [c for c in st.VARIANT_CYCLE if c != st.REFERENCE]
    colours = {n: (st.REFERENCE if cases[n].architecture == "indirect_pmp"
                   else palette[i % len(palette)])
               for i, n in enumerate(present)}

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)

    drawn = 0
    for name in present:
        case = cases[name]
        history = case.pso_history
        if history is None:
            continue
        gens, gbest = history
        arch, detail = st.arch_label(case.architecture), st.case_label(name)
        # "Segmented (hand-off optimised)", not "Segmented (Segmented, ...)".
        if detail.lower().startswith(arch.lower()):
            detail = detail[len(arch):].lstrip(", ")
        ax_a.plot(gens, gbest, color=colours[name], label="%s (%s)" % (arch, detail))
        drawn += 1
    if drawn:
        ax_a.set_yscale("log")
    ax_a.set_xlabel("Generation")
    ax_a.set_ylabel(r"Best objective $J'$")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_kw={"fontsize": 6.5})

    rows = [(st.arch_label(cases[n].architecture), colours[n]) + _cost_note(cases[n])
            for n in present]
    walls = [w for _l, _c, w, _n in rows if w is not None]
    x_max = max(walls + [1.0]) * 1.55
    for i, (_label, colour, wall, note) in enumerate(rows):
        if wall is not None:
            ax_b.barh(i, wall, height=0.6, color=colour, edgecolor="white",
                      linewidth=0.5)
        ax_b.annotate(note, xy=(wall or 0.0, i), xytext=(4, 0),
                      textcoords="offset points", fontsize=6.3, va="center",
                      color=st.INK if wall is not None else st.GREY,
                      style="normal" if wall is not None else "italic")
    ax_b.set_yticks(np.arange(len(rows)))
    ax_b.set_yticklabels([r[0] for r in rows])
    ax_b.invert_yaxis()
    ax_b.set_xlabel("Wall clock [min]")
    ax_b.set_xlim(0, x_max)
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_solve_cost.png")


FIGURES = [showcase_laws, apollo_waypoint, segmented_handoff, solve_cost]
