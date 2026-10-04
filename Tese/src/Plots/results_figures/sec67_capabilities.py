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

# The open-loop laws at the baseline, in the chapter's order. show_apollo is
# drawn once, in apollo_waypoint below, beside the Apollo law flying the
# reference's plan; peg_new's pair is sec63_peg.peg_waypoint.
SHOWCASE = ["show_cpr", "show_linear_tangent", "show_bilinear_tangent",
            "show_exp_shooting"]

# One representative case per architecture: the convergence curve in panel (a)
# of the cost figure, and the row order and colour of panel (b), which draws
# every reported case of each architecture. gt_apogee and show_ref_track run no
# swarm and have no convergence curve to draw.
COST_CASES = ["gt_baseline", "peg_direct", "pmp_baseline", "show_seg_opt_alt",
              "gt_apogee", "show_ref_track"]

# What the searches of each architecture are, for panel (b)'s notes. {evals} is
# the evaluation count(s) of the cases drawn.
_COST_NOTES = {
    "pso_coast": "swarm, {evals} evaluations each",
    "segmented": "swarm, {evals} evaluations each",
    "direct": "kick grid + Brent, {evals} flights",
    "apogee_check": "kick grid, {evals} flights",
    "indirect_pmp": "offline: swarm, {evals} evals, + refinement",
    "reference_track": "no search, one flight each",
}

# The coast strips along the foot of the showcase trajectory panel, in axes
# fractions: the first strip's bottom, each strip's height, and the pitch.
_STRIP_Y0, _STRIP_H, _STRIP_STEP = 0.03, 0.026, 0.040


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def showcase_laws(cases):
    """The open-loop laws at the baseline, with the reference faint for scale.

    Panel (a) is altitude against time up to insertion. Each law's coasts are a
    strip in its colour along the foot of the panel rather than a span shaded
    over the curves: overlapping translucent spans mix into colours no law has.
    Panel (b) is small multiples rather than an overlay because alpha is what
    distinguishes these laws from one another, and their alpha traces on shared
    axes would be a solid block. Each small panel keeps the same limits, and
    carries the reference's alpha faint behind the law's. Up to four laws sit
    on a two-column grid, more on three.
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
    n_cols = 2 if len(SHOWCASE) <= 4 else 3
    n_rows = -(-len(SHOWCASE) // n_cols)
    fig_h = 5.6 + 1.62 * (n_rows - 2)
    fig = plt.figure(figsize=(7.4, fig_h))
    grid = fig.add_gridspec(1 + n_rows, n_cols, height_ratios=[1.45] + [1.0] * n_rows,
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
    # Outside the axes: five entries over a trajectory panel cover the curves
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
        ax = fig.add_subplot(grid[1 + i // n_cols, i % n_cols])
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
        if i % n_cols == 0:
            ax.set_ylabel(r"$\alpha$ [deg]", fontsize=7.5)
        # Label time on every panel with nothing beneath it, including the
        # panel above an empty last slot.
        if i + n_cols >= len(SHOWCASE):
            ax.set_xlabel("Time [s]", fontsize=7.5)
        st.tidy(ax, legend=False)
    # The unused slots of the last row stay empty.
    for j in range(len(SHOWCASE), n_cols * n_rows):
        fig.add_subplot(grid[1 + j // n_cols, j % n_cols]).axis("off")

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


def _search_cost(case):
    """(wall clock [s], evaluations) of the search that produced this case, or
    None when it was not recorded.

    A stored extremal is re-flown in about a second, so its own wall clock and
    evaluation count describe the re-flight. Its archive records the offline
    swarm and refinement it came from (search_*), and that is the cost. Every
    other case's search ran inside the batch and is the run itself.
    """
    row = case.row
    if row.get("search_wall_clock_s") is not None:
        return float(row["search_wall_clock_s"]), row.get("search_n_evaluations")
    if case.extremal_budget is not None:
        return None                    # an archive written before search_* existed
    return float(row.get("wall_clock_s") or 0.0), row.get("n_evaluations")


def _duration(seconds, unit_of=None):
    """Seconds as the unit that suits ``unit_of`` (default: the value itself)."""
    ref = seconds if unit_of is None else unit_of
    if ref < 90.0:
        return "%.0f s" % seconds, "s"
    if ref < 5400.0:
        return "%.0f min" % (seconds / 60.0), "min"
    return "%.1f h" % (seconds / 3600.0), "h"


def _duration_range(lo, hi):
    if np.isclose(lo, hi, rtol=0.05):
        return _duration(hi)[0]
    shown_hi, unit = _duration(hi)
    shown_lo = _duration(lo, unit_of=hi)[0].rsplit(" ", 1)[0]
    return "%s–%s" % (shown_lo, shown_hi)


def _cost_colours(cases):
    """One colour per architecture of COST_CASES, shared by both panels: the
    reference's for the indirect row, the variant cycle in order for the rest."""
    palette = iter([c for c in st.VARIANT_CYCLE if c != st.REFERENCE] * 2)
    return {n: (st.REFERENCE if cases[n].architecture == "indirect_pmp" else next(palette))
            for n in COST_CASES if n in cases}


def _cost_rows(cases):
    """Panel (b)'s rows: (architecture, colour, [wall clock s], note), one per
    architecture of COST_CASES, each over every reported case it flew."""
    colours = _cost_colours(cases)
    rows = []
    for rep in (n for n in COST_CASES if n in cases):
        arch = cases[rep].architecture
        colour = colours[rep]
        members = [cases[n] for n in _data.REPORTED_CASES
                   if n in cases and cases[n].architecture == arch]
        costs = [_search_cost(c) for c in members]
        timed = [c for c in costs if c is not None]
        if not timed:
            rows.append((st.arch_label(arch), colour, [],
                         "not recorded (%d case%s)" % (len(members),
                                                       "" if len(members) == 1 else "s")))
            continue
        walls = [w for w, _n in timed]
        evals = sorted({int(n) for _w, n in timed if n})
        note = _COST_NOTES.get(arch, "{evals} evaluations").format(
            evals=" / ".join("{:,}".format(n).replace(",", r"$\,$") for n in evals) or "?")
        count = "" if len(timed) == 1 else "%d cases; " % len(timed)
        rows.append((st.arch_label(arch), colour, walls,
                     "%s   (%s%s)" % (_duration_range(min(walls), max(walls)), count, note)))
    return rows


def solve_cost(cases):
    """What each architecture costs, and whether its swarm converged.

    Panel (a), the convergence curves, answers only whether a poor result came
    from the law or from a search that was stopped too early -- a failure mode
    this work has met, where an apparently incapable configuration proved
    merely under-converged. It ranks nothing. Only the coast-parameter and
    segmented architectures run a swarm inside the matrix. Panel (b) is the
    wall clock of every reported case, grouped by architecture, on a log axis:
    one law's search takes 1.7 h and another's 15.5 h under the same
    architecture, and a single representative would hide which. The indirect
    row is the offline swarm and refinement its stored extremal came from.
    """
    present = [n for n in COST_CASES if n in cases]
    if not present:
        return _skip("solve cost", COST_CASES)
    colours = _cost_colours(cases)

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2,
                                     gridspec_kw={"width_ratios": [1.0, 1.7]})

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
    st.tidy(ax_a, legend_loc="upper right", legend_kw={"fontsize": 6.5})

    # (b) every case's search, in hours on a log axis: the range of each
    # architecture as a line, each case as a dot, the note above the line.
    rows = _cost_rows(cases)
    all_h = [w / 3600.0 for _l, _c, walls, _n in rows for w in walls]
    x_lo = 10 ** np.floor(np.log10(min(all_h + [1.0])))
    for i, (_label, colour, walls, note) in enumerate(rows):
        hours = [w / 3600.0 for w in walls]
        if hours:
            ax_b.plot([min(hours), max(hours)], [i, i], color=colour, linewidth=5,
                      alpha=0.35, solid_capstyle="round")
            ax_b.plot(hours, [i] * len(hours), "o", color=colour, markersize=4)
        # From the left edge, inside the axes: anchored at the dots, the long
        # notes of the slow rows ran off the figure and squeezed both panels.
        ax_b.annotate(note, xy=(0.02, i), xycoords=("axes fraction", "data"),
                      xytext=(0, 5), textcoords="offset points", fontsize=6.3,
                      va="bottom",
                      color=st.INK if hours else st.GREY,
                      style="normal" if hours else "italic")
    ax_b.set_xscale("log")
    ax_b.set_xlim(x_lo, 10 ** np.ceil(np.log10(max(all_h + [1.0])) + 0.3))
    ax_b.set_yticks(np.arange(len(rows)))
    ax_b.set_yticklabels([r[0] for r in rows])
    ax_b.set_ylim(len(rows) - 0.5, -0.8)
    ax_b.set_xlabel("Wall clock of the search [h]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_solve_cost.png")


FIGURES = [showcase_laws, apollo_waypoint, segmented_handoff, solve_cost]
