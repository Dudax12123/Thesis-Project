"""Section 6.7 figures -- the loss budget, the ranking and the arc structure.

Section 6.7 owns no runs. It re-reads the cases already reported, which is why
the chapter is shorter than the matrix is wide.

Outputs
-------
results_loss_budget.png            fig:loss_budget
results_loss_accumulation.png      fig:loss_accumulation
results_law_ranking.png            fig:law_ranking
results_accuracy_vs_propellant.png fig:accuracy_vs_propellant
results_arc_structure.png          fig:arc_structure
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from . import _data
from . import _panels as pn
from . import _style as st

# The cases the budget is drawn for: the gravity turn, PEG and the reference at
# the baseline, and the drag-free counterparts of the last two (gt_vacuum is
# archived but not reported). Not every case in the matrix -- the full budget
# goes to the chapter's table.
BUDGET_CASES = [
    ("gt_baseline", "Gravity turn"),
    ("peg_baseline", "PEG"),
    ("peg_vacuum", "PEG, vac."),
    ("pmp_baseline", "Indirect PMP"),
    ("pmp_vacuum", "Indirect PMP, vac."),
]

# The settings that make up a case's environment, for matching it to the
# reference flown in the same one.
_ENVIRONMENT_KEYS = ("include_drag", "earth_rotation", "thrust_1_mode")

# A Stage-2 burn shorter than this is drawn as a tick in the arc-structure
# chart: on a 3000 s axis a 0.2 s bar is not a pixel wide.
SHORT_BURN_S = 5.0


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def loss_budget(cases):
    """F6.10 -- the delta-v budget as stacked bars, one per principal case.

    The rotational gain is drawn to the left of zero because it is a credit
    rather than a loss, and the achieved increment is marked so that the
    identity closing the budget is visible rather than asserted. A case flown
    under a constant nozzle model, or with no atmosphere, has no pressure
    segment at all: the loss is undefined there, and an absent segment says so
    where a zero-height one would read as a measurement of nothing.
    """
    present = [(n, label) for n, label in BUDGET_CASES if n in cases]
    if not present:
        return _skip("F6.10 loss budget", [n for n, _ in BUDGET_CASES])

    components = ("gravity", "drag", "steering", "pressure")
    fig, ax = plt.subplots(figsize=st.bar_size(len(present)))

    # Legend entries are claimed by the first bar that actually carries the
    # component, not by the first bar outright: the gravity turn commands
    # alpha = 0 and so has no steering loss, and gating on row zero would drop
    # steering from the legend for every case below it.
    labelled = set()

    def _claim(key, text):
        if key in labelled:
            return None
        labelled.add(key)
        return text

    y_pos = np.arange(len(present))
    for row_i, (name, label) in enumerate(present):
        budget = cases[name].budget()
        left = 0.0
        for comp in components:
            value = budget.get("dv_" + comp)
            if value is None or abs(value) < 1e-9:
                continue
            ax.barh(row_i, value, left=left, height=0.6,
                    color=st.LOSS_COLORS[comp], edgecolor="white", linewidth=0.5,
                    label=_claim(comp, comp.capitalize()))
            left += value

        gain = budget.get("dv_gain")
        if gain:
            ax.barh(row_i, -gain, left=0.0, height=0.6, color=st.GREEN,
                    edgecolor="white", linewidth=0.5, alpha=0.75,
                    label=_claim("gain", "Launch-site gain"))

        ax.annotate("%.0f m/s" % left, xy=(left, row_i), xytext=(4, 0),
                    textcoords="offset points", fontsize=7, va="center",
                    color=st.INK)

    ax.set_yticks(y_pos)
    ax.set_yticklabels([label for _n, label in present])
    ax.invert_yaxis()
    ax.axvline(0.0, color=st.INK, linewidth=0.8)
    ax.set_xlabel(r"$\Delta V$ [m/s]   (losses right of zero, gain left)")
    # Below the axes in one row, not inside them: the bars span the full width
    # at every row, so there is no interior corner a legend can occupy without
    # covering a segment or one of the total labels.
    st.tidy(ax, legend_loc="upper center",
            legend_kw={"bbox_to_anchor": (0.5, -0.18), "ncol": 5,
                       "columnspacing": 1.4, "handlelength": 1.4})

    fig.tight_layout()
    return st.save(fig, "results_loss_budget.png")


def loss_accumulation(cases):
    """F6.11 -- where in the flight each loss is actually incurred.

    The scalar budget says how much; this says when, which is what distinguishes
    two laws that spend the same total differently. Drag is confined to the
    first minute or so of flight while the gravity loss accrues throughout, and
    that asymmetry is the reason the two are traded against each other rather
    than minimised separately.
    """
    names = [n for n in ("gt_baseline", "peg_baseline") if n in cases]
    if not names:
        return _skip("F6.11 loss accumulation", ["gt_baseline", "peg_baseline"])

    fig, ax = plt.subplots(figsize=st.WIDE_1)
    styles = {"gt_baseline": "-", "peg_baseline": "--"}

    for name in names:
        case = cases[name]
        hist = case.loss_histories()
        t_full = case.time[:case.cutoff_index()]
        for comp in ("gravity", "drag", "steering", "pressure"):
            series = hist.get(comp)
            if series is None or not np.any(np.abs(series) > 1e-9):
                continue
            t, series = st.thin(t_full, series)
            ax.plot(t, series, color=st.LOSS_COLORS[comp],
                    linestyle=styles[name],
                    label="%s (%s)" % (comp.capitalize(), st.law_label(case.law)))
        st.add_events(ax, case, coast=False, seco=False)

    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Cumulative $\Delta V$ loss [m/s]")
    st.tidy(ax, legend_loc="upper left")
    fig.tight_layout()
    return st.save(fig, "results_loss_accumulation.png")


def _orbit_spread_km(case):
    """Apoapsis minus periapsis at insertion [km], or None without elements.

    For a target that is circular by definition the gap is the whole miss;
    accuracy_vs_propellant plots the same quantity.
    """
    peri, apo = case.row.get("periapsis_km"), case.row.get("apoapsis_km")
    if peri is None or apo is None:
        return None
    return abs(apo - peri)


def _is_reference(case):
    return case.architecture == "indirect_pmp"


def _reference_for(case, cases):
    """The reference flown in *case*'s environment, and whether it truly matches.

    Drag-free cases are measured against pmp_vacuum and everything else against
    pmp_baseline. That is an exact match only when the reference also shares
    the case's Earth rotation and Stage-1 nozzle model. No reference was flown
    for the non-rotating or the sea-level-nozzle environment, so those two
    cases get no difference at all (decision 2026-09-30).
    """
    name = "pmp_vacuum" if case.row.get("include_drag") is False else "pmp_baseline"
    ref = cases.get(name)
    if ref is None:
        return None, False
    same = all(case.row.get(k) == ref.row.get(k) for k in _ENVIRONMENT_KEYS)
    return ref, same


def _baseline_environment(case, cases):
    """Whether *case* was flown in the baseline reference's environment."""
    base = cases.get("pmp_baseline")
    return base is not None and all(case.row.get(k) == base.row.get(k)
                                    for k in _ENVIRONMENT_KEYS)


def _with_propellant(cases):
    return [n for n in _data.REPORTED_CASES
            if n in cases and cases[n].row.get("prop_remaining_kg") is not None]


def law_ranking(cases):
    """Propellant remaining at insertion for every reported case, each against
    the reference of its own environment.

    Each bar prints the propellant, its difference from that reference where
    one was flown, and the apoapsis-periapsis spread the propellant was bought
    with: the swarm's
    objective trades the two, and a propellant bar alone does not show it.
    Cases that miss the target orbit are drawn hatched and grey below the rest
    and are not ranked: propellant unspent by a vehicle that failed to arrive
    is not a saving.
    """
    present = _with_propellant(cases)
    if not present:
        return _skip("law ranking", _data.REPORTED_CASES)

    records = []
    for name in present:
        case = cases[name]
        ref, same = _reference_for(case, cases)
        records.append((name, case, case.row["prop_remaining_kg"] / 1e3,
                        case.reached_orbit, _orbit_spread_km(case), ref, same))
    valid = sorted([r for r in records if r[3]], key=lambda r: r[2])
    invalid = sorted([r for r in records if not r[3]], key=lambda r: r[2])
    records = invalid + valid

    fig, ax = plt.subplots(figsize=st.bar_size(len(records), row_height=0.30))
    unmatched = False
    for i, (name, case, prop_t, ok, spread, ref, same) in enumerate(records):
        colour = (st.REFERENCE if _is_reference(case)
                  else st.BASELINE if ok else st.FAILED)
        ax.barh(i, prop_t, height=0.66, color=colour,
                hatch=None if ok else "//", edgecolor="white", linewidth=0.5)
        # No difference is printed where no reference was flown in the case's
        # own environment: against another environment's reference it would be
        # mostly the environment (rotation, nozzle), not the guidance.
        text = "%.2f t" % prop_t
        if ref is not None and not _is_reference(case) and same:
            text += r"    $\Delta$ %+.2f t" % (prop_t - ref.row["prop_remaining_kg"] / 1e3)
        if spread is not None and not _is_reference(case):
            text += "    %.1f km" % spread
        if not _is_reference(case) and not same:
            text += r"  $^\dagger$"
            unmatched = True
        # Inside the bar's end: outside, the longer label runs across the
        # reference lines that sit just past the longest bars.
        ax.annotate(text, xy=(prop_t, i), xytext=(-4, 0),
                    textcoords="offset points", fontsize=6.5, va="center",
                    ha="right", color="white" if ok else st.INK)

    for name, style, text in (("pmp_baseline", "--", "Reference"),
                              ("pmp_vacuum", ":", "Reference, no atmosphere")):
        ref = cases.get(name)
        if ref is not None and ref.row.get("prop_remaining_kg") is not None:
            ref_t = ref.row["prop_remaining_kg"] / 1e3
            ax.axvline(ref_t, color=st.REFERENCE, linestyle=style, linewidth=1.1,
                       zorder=0.8, label="%s (%.2f t)" % (text, ref_t))

    ax.set_yticks(np.arange(len(records)))
    ax.set_yticklabels([st.case_label(r[0]) for r in records], fontsize=7)
    ax.set_xlabel("Propellant remaining at insertion [t]")
    ax.set_xlim(0.0, 1.04 * max(r[2] for r in records))
    st.tidy(ax, legend_loc="upper center",
            legend_kw={"bbox_to_anchor": (0.5, -0.07), "ncol": 2,
                       "fontsize": 6.8})

    # Under the legend, not inside the axes: the bars start at zero on every
    # row, so any note placed inside sits on top of one of them.
    footnotes = [r"$\Delta$: against the reference flown in the same environment;"
                 " km: apoapsis-periapsis spread at insertion"]
    if unmatched:
        footnotes.append(r"$^\dagger$ no reference was flown for this environment "
                         r"(non-rotating Earth, sea-level nozzle), so no $\Delta$")
    if invalid:
        footnotes.append("hatched: target orbit not reached, not ranked")
    for i, text in enumerate(footnotes):
        ax.annotate(text, xy=(0.0, -0.115 - 0.028 * i), xycoords="axes fraction",
                    fontsize=6.3, color=st.GREY, va="top")
    fig.tight_layout()
    return st.save(fig, "results_law_ranking.png")


def accuracy_vs_propellant(cases):
    """The trade, stated directly rather than inferred from two tables.

    Insertion accuracy on one axis and propellant on the other separates the
    cases that buy accuracy with propellant from those that give up both, which
    a ranking on either quantity alone cannot show. Every reported case is
    drawn; those flown in another environment than the baseline reference's
    are hollow, because their propellant is not comparable with the rest.
    """
    present = [n for n in _with_propellant(cases)
               if _orbit_spread_km(cases[n]) is not None]
    if not present:
        return _skip("accuracy vs propellant", _data.REPORTED_CASES)

    fig, ax = plt.subplots(figsize=st.TALL_1)
    labels, any_failed, any_other_env = [], False, False
    for name in present:
        case = cases[name]
        prop_t = case.row["prop_remaining_kg"] / 1e3
        spread = _orbit_spread_km(case)
        colour = st.REFERENCE if _is_reference(case) else st.BASELINE
        same_env = _baseline_environment(case, cases)
        any_other_env = any_other_env or not same_env
        marker = "*" if _is_reference(case) else ("o" if case.reached_orbit else "X")
        ax.scatter(spread, prop_t, s=70 if marker == "*" else 30, marker=marker,
                   facecolor=colour if same_env else "white", edgecolor=colour,
                   linewidth=1.0, zorder=5 if marker == "*" else 3)
        # Left of the point for the largest misses, which sit at the right
        # edge; a white backing keeps the reference line from striking through.
        right_edge = spread > 5.0
        labels.append(ax.annotate(st.case_label(name), xy=(spread, prop_t),
                                  xytext=(-6 if right_edge else 6, -2),
                                  textcoords="offset points",
                                  ha="right" if right_edge else "left",
                                  fontsize=6.3, color=st.INK, zorder=4,
                                  bbox={"facecolor": "white", "edgecolor": "none",
                                        "pad": 0.4, "alpha": 0.85}))
        any_failed = any_failed or not case.reached_orbit

    base = cases.get("pmp_baseline")
    if base is not None:
        ax.axhline(base.row["prop_remaining_kg"] / 1e3, color=st.REFERENCE,
                   linestyle="--", linewidth=0.9, zorder=1)

    # Symlog, linear below 0.1 km: half the cases insert within a few hundred
    # metres of circular and the others miss by up to 20 km, and a linear axis
    # would put the first half on top of one another at zero.
    ax.set_xscale("symlog", linthresh=0.1, linscale=0.6)
    ax.set_xlim(-0.004, 60.0)
    ax.set_xticks([0.0, 0.1, 1.0, 10.0])
    ax.set_xticklabels(["0", "0.1", "1", "10"])
    ax.set_xlabel("Apoapsis-periapsis spread at insertion [km]   (lower is better)")
    ax.set_ylabel("Propellant remaining [t]")

    footnotes = ["star: reference; dashed: the baseline reference's propellant"]
    if any_other_env:
        footnotes.append("hollow: flown in another environment (no atmosphere, "
                         "non-rotating Earth or sea-level nozzle)")
    if any_failed:
        footnotes.append("X: target orbit not reached")
    ax.annotate("\n".join(footnotes), xy=(0.0, -0.16), xycoords="axes fraction",
                fontsize=6.3, color=st.GREY, va="top")
    st.tidy(ax, legend=False)
    fig.tight_layout()
    # Placed last: the de-collider measures rendered extents, so it has to run
    # after every artist that moves them.
    # Downward: the cluster near zero spread sits under the reference line,
    # and pushed up its labels land on the points above them.
    st.dodge_labels(fig, labels, downward=True)
    return st.save(fig, "results_accuracy_vs_propellant.png")


def arc_structure(cases):
    """The arc structure of every case on one time axis, ordered by propellant.

    The first stage, each second-stage burn and each coast as a bar, read from
    the thrust trace (_panels.burn_intervals, Case.coast_intervals) so every
    architecture is drawn by the same rule. The gap after the first stage is
    the planned separation delay. A second-stage burn too short to show as a
    bar is a tick, and the apogee check's impulsive circularisation, which the
    thrust trace does not hold, is a diamond at the instant Case.t_insertion
    finds. Ordered by the propellant remaining at insertion, most at the top.
    """
    present = _with_propellant(cases)
    if not present:
        return _skip("arc structure", _data.REPORTED_CASES)
    order = sorted(present, key=lambda n: cases[n].row["prop_remaining_kg"],
                   reverse=True)

    fig, ax = plt.subplots(figsize=st.bar_size(len(order), row_height=0.27))
    half = 0.28
    t_max = 0.0
    for row, name in enumerate(order):
        case = cases[name]
        for t0, t1 in pn.burn_intervals(case):
            stage1 = case.t_meco is not None and t0 < case.t_meco - 0.5
            if not stage1 and t1 - t0 < SHORT_BURN_S:
                ax.plot(t1, row, marker="|", markersize=8, markeredgewidth=1.6,
                        color=st.THRUST, zorder=3)
            else:
                ax.broken_barh([(t0, t1 - t0)], (row - half, 2 * half),
                               facecolors=st.THRUST, alpha=0.45 if stage1 else 1.0,
                               linewidth=0, zorder=2)
        for t0, t1 in case.coast_intervals():
            ax.broken_barh([(t0, t1 - t0)], (row - half, 2 * half),
                           facecolors=st.FAINT, linewidth=0, zorder=1)
        t_end = case.t_insertion if case.t_insertion is not None else case.time[-1]
        if case.architecture == "apogee_check":
            ax.plot(t_end, row, marker="D", markersize=4.2, color=st.AMBER, zorder=4)
        ax.annotate("%.2f t" % (case.row["prop_remaining_kg"] / 1e3),
                    xy=(t_end, row), xytext=(7, 0), textcoords="offset points",
                    fontsize=6.3, va="center", color=st.INK)
        t_max = max(t_max, t_end)

    ax.set_yticks(np.arange(len(order)))
    ax.set_yticklabels([st.case_label(n) for n in order], fontsize=7)
    ax.set_ylim(len(order) - 0.5, -0.5)
    ax.set_xlim(0.0, 1.12 * t_max)
    ax.set_xlabel("Time [s]   (propellant remaining at insertion printed after each)")
    handles = [
        Patch(facecolor=st.THRUST, alpha=0.45, label="First-stage burn"),
        Patch(facecolor=st.THRUST, label="Second-stage burn"),
        Patch(facecolor=st.FAINT, label="Coast"),
        Line2D([], [], color=st.THRUST, marker="|", markersize=8,
               markeredgewidth=1.6, linestyle="none",
               label="Burn under %.0f s" % SHORT_BURN_S),
        Line2D([], [], color=st.AMBER, marker="D", markersize=4.2,
               linestyle="none", label="Impulsive circularisation"),
    ]
    st.tidy(ax, legend=False)
    ax.grid(False, axis="y")
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.07),
              ncol=3, fontsize=6.8)
    fig.tight_layout()
    return st.save(fig, "results_arc_structure.png")


FIGURES = [loss_budget, loss_accumulation, law_ranking, accuracy_vs_propellant,
           arc_structure]
