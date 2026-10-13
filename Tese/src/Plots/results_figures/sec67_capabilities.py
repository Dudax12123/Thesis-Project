"""Sections 6.5, 6.6 and 6.7 figures -- the remaining laws, the segmented
schedule, and the cost of the search.

Outputs
-------
results_showcase_laws.png      fig:showcase_laws
results_apollo_waypoint.png    fig:apollo_waypoint
results_segmented.png          fig:segmented
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

# The small multiples of the angle of attack add the Apollo law, so that
# Section 6.5.2 has a figure of its own law to point to (red-note review S5.2);
# the altitude panel keeps the open-loop laws only.
SHOWCASE_ALPHA = SHOWCASE + ["show_apollo"]

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


# Every label of a figure redrawn for the 10 pt rule (figure review,
# 2026-10-13): drawn at 6.3 in and placed at \textwidth.
FS = 10.0


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def showcase_laws(cases):
    """The open-loop laws at the baseline, with the reference faint for scale.

    Panel (a) is altitude against time up to insertion. Each law's coasts are a
    strip in its colour along the foot of the panel rather than a span shaded
    over the curves: overlapping translucent spans mix into colours no law has.
    Panel (b) is the commanded angle of attack of the four laws and Apollo on
    one set of axes, the reference faint behind them (figure review,
    2026-10-13: the five small multiples joined into one panel). One legend,
    under both panels, names every curve; every label is drawn at FS.
    """
    present = [n for n in SHOWCASE if n in cases]
    if not present:
        return _skip("showcase laws", SHOWCASE)
    with plt.rc_context({"axes.labelsize": FS, "xtick.labelsize": FS,
                         "ytick.labelsize": FS, "axes.titlesize": FS}):
        return _draw_showcase_laws(cases, present)


def _draw_showcase_laws(cases, present):
    """showcase_laws's drawing, run inside its font sizes."""
    ref = cases.get("pmp_baseline")
    # Green is left out: across the chapter it is the reference's colour, and
    # the reference is in this figure too (faint).
    palette = [c for c in st.VARIANT_CYCLE if c != st.REFERENCE]
    present_alpha = [n for n in SHOWCASE_ALPHA if n in cases]
    colours = {n: palette[i % len(palette)] for i, n in enumerate(present_alpha)}

    fig, (ax_traj, ax_al) = plt.subplots(2, 1, figsize=(6.3, 6.0),
                                         gridspec_kw={"height_ratios": [1.2, 1.0]})

    strips = ([(ref, st.FAINT)] if ref is not None else []) +              [(cases[n], colours[n]) for n in present]
    if ref is not None:
        t, alt_km = st.thin(*pn.to_insertion(ref, ref.alt_km))
        ax_traj.plot(t, alt_km, color=st.FAINT, linewidth=1.2, zorder=1)
    for name in present:
        case = cases[name]
        t, alt_km = st.thin(*pn.to_insertion(case, case.alt_km))
        ax_traj.plot(t, alt_km, color=colours[name], zorder=2)
    for k, (case, colour) in enumerate(strips):
        y0 = _STRIP_Y0 + _STRIP_STEP * k
        for t0, t1 in case.coast_intervals():
            ax_traj.axvspan(t0, t1, ymin=y0, ymax=y0 + _STRIP_H, color=colour,
                            linewidth=0)
    ax_traj.annotate("coasts", xy=(0.995, _STRIP_Y0 + _STRIP_STEP * len(strips) / 2),
                     xycoords="axes fraction", fontsize=FS, color=st.GREY,
                     ha="right", va="center")
    ax_traj.set_xlabel("Time [s]")
    ax_traj.set_ylabel("Altitude [km]")
    st.panel_tag(ax_traj, "a")
    st.tidy(ax_traj, legend=False)

    # The full range of every law, so nothing is clipped and no note is needed
    # (figure review, 2026-10-13: the clipped-peak note removed); Apollo's
    # transient after ignition sets the top.
    alphas = {n: pn.to_insertion(cases[n], cases[n].alpha_deg) for n in present_alpha}
    stacked = np.concatenate([al for _t, al in alphas.values()])
    alpha_lo = min(float(np.nanmin(stacked)), 0.0)
    alpha_hi = max(float(np.nanmax(stacked)), 0.0)
    pad = 0.06 * max(alpha_hi - alpha_lo, 1.0)
    alpha_lo, alpha_hi = alpha_lo - pad, alpha_hi + pad
    # The time axis ends just after the last steering ends: what follows is
    # coast and zero command, and on one shared panel it would take half the
    # width.
    t_hi = 1.06 * max(pn.steering_end(cases[n]) for n in present_alpha)

    if ref is not None:
        ax_al.plot(*st.thin(*pn.to_insertion(ref, ref.alpha_deg)), color=st.FAINT,
                   linewidth=1.2, label="Reference (indirect PMP)", zorder=1)
    for name in present_alpha:
        t, al = alphas[name]
        ax_al.plot(*st.thin(t, al), color=colours[name], linewidth=1.1,
                   label=st.case_label(name), zorder=2)
    ax_al.axhline(0.0, color=st.FAINT, linewidth=0.7)
    ax_al.set_xlim(0, t_hi)
    ax_al.set_ylim(alpha_lo, alpha_hi)
    # MECO in both panels, dotted grey and named (figure review, 2026-10-13);
    # the same instant for every law.
    t_meco = next((cases[n].t_meco for n in present_alpha if cases[n].t_meco), None)
    if t_meco is not None:
        for ax in (ax_traj, ax_al):
            pn.mark_instant(ax, t_meco, "MECO", st.GREY, fontsize=FS, top=True)
    ax_al.set_xlabel("Time [s]")
    ax_al.set_ylabel(r"Angle of attack $\alpha$ [deg]")
    st.panel_tag(ax_al, "b")
    st.tidy(ax_al, legend=False)
    # One legend for both panels, from (b), which carries every curve.
    pn.figure_legend(fig, ax_al, ncol=3, fontsize=FS)
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


def segmented(cases):
    """PEG with its first burn aimed at the coast-start waypoint, in the two
    segmented cases, against the reference (red-note review S4.1, 2026-10-08:
    the waypoint panels of the former PEG figure and the hand-off figure merged
    into one).

    (a) Altitude against time, the reference's long coast drawn compressed, with
    the waypoint and each case's hand-off marked.
    (b) The commanded angle of attack of the reference and the two segmented
    cases, with the hand-offs and the waypoint instant marked.

    Figure review, 2026-10-13: PEG aimed at the orbit is no longer drawn; the
    reference is dotted black, the optimised hand-off red and the fixed one
    blue, both solid; every label is drawn at FS.
    """
    names = ("pmp_baseline", "show_seg_fixed_alt", "show_seg_opt_alt")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("segmented", missing)
    with plt.rc_context({"axes.labelsize": FS, "xtick.labelsize": FS,
                         "ytick.labelsize": FS, "axes.titlesize": FS}):
        return _draw_segmented(cases)


def _draw_segmented(cases):
    """segmented's drawing, run inside its font sizes."""
    ref = cases["pmp_baseline"]
    fixed, opt = cases["show_seg_fixed_alt"], cases["show_seg_opt_alt"]
    segmented_entries = [(opt, st.ACCENT, "-", st.case_label("show_seg_opt_alt")),
                         (fixed, st.ENV_COLORS["baseline"], "-",
                          st.case_label("show_seg_fixed_alt"))]
    entries = [(ref, st.INK, ":", "Reference (indirect PMP)")]
    entries += segmented_entries

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(6.3, 3.6))
    pn.altitude_panel(ax_a, entries, shade_coasts=False)
    # thinner than the house weight: the three curves run on top of each
    # other for most of the flight (figure review, 2026-10-13)
    for curve in ax_a.get_lines():
        curve.set_linewidth(1.0)
    for case, colour, _style, _label in segmented_entries:
        point = _handoff(case)
        if point is None:
            continue
        ax_a.plot(*point, linestyle="none", marker="D", markersize=4.2, color=colour,
                  zorder=5)
        ax_a.annotate("%.0f km" % point[1], xy=point, xytext=(5, -2),
                      textcoords="offset points", fontsize=FS, color=st.INK,
                      ha="left", va="top")
    # one legend entry for both hand-offs: the symbol alone, unfilled
    ax_a.plot([], [], linestyle="none", marker="D", markersize=4.2,
              markerfacecolor="none", markeredgecolor=st.INK, label="Hand-off")
    wp = pn.waypoint(ref)
    if wp is not None:
        # its state is in Section 6.1's table, not the legend (review, 2026-10-13)
        pn.mark_waypoint(ax_a, wp, label="Coast-start waypoint")
    pn.compress_long_coast(ax_a, ref, float(ref.time[ref.insertion_index() - 1]) + 10.0,
                           label_y=0.15, fontsize=FS, label_box=True)
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend=False)

    pn.alpha_panel(ax_b, entries, fontsize=FS)
    # Named from the top, right of each hand-off and left of the waypoint:
    # along the foot the names land on the curves at 10 pt.
    for case, colour, _style, _label in segmented_entries:
        point = _handoff(case)
        if point is not None:
            pn.mark_instant(ax_b, point[0], "hand-off", colour, fontsize=FS,
                            top=True, right=True)
    if wp is not None:
        pn.mark_instant(ax_b, wp["t"], "waypoint", fontsize=FS, top=True)
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)
    pn.figure_legend(fig, ax_a, ncol=2, fontsize=FS)
    return st.save(fig, "results_segmented.png")


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


# apollo_waypoint needs show_ref_track_apollo, out of the matrix since
# 2026-10-07; it is drawn on request only.
FIGURES = [showcase_laws, segmented, solve_cost]
