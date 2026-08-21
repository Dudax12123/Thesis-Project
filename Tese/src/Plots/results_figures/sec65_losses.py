"""Sections 6.5 and 6.6 figures -- the loss budget and the ranking.

Neither section owns any runs. Both re-read the cases already reported, which
is why the chapter is shorter than the matrix is wide.

Outputs
-------
results_loss_budget.png            F6.10
results_loss_accumulation.png      F6.11
results_law_ranking.png            F6.12
results_accuracy_vs_propellant.png F6.13
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
from . import _style as st

# The cases the budget is drawn for: the three laws at the baseline, and the
# drag-free counterpart of each that has one. Not every case in the matrix --
# the full budget goes to the appendix table.
BUDGET_CASES = [
    ("gt_baseline", "Gravity turn"),
    ("gt_vacuum", "Gravity turn, vac."),
    ("peg_baseline", "PEG"),
    ("peg_vacuum", "PEG, vac."),
    ("pmp_baseline", "Indirect PMP"),
    ("pmp_vacuum", "Indirect PMP, vac."),
]

# Every law flown at the baseline architecture, for the ranking.
RANKING_CASES = [
    "gt_baseline", "peg_baseline", "show_cpr", "show_linear_tangent",
    "show_bilinear_tangent", "show_apollo", "show_peg", "show_exp_shooting",
]


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
    ax.set_xlabel(r"$\Delta v$ [m/s]   (losses right of zero, gain left)")
    st.tidy(ax, legend_loc="lower right")

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
    ax.set_ylabel(r"Cumulative $\Delta v$ loss [m/s]")
    st.tidy(ax, legend_loc="upper left")
    fig.tight_layout()
    return st.save(fig, "results_loss_accumulation.png")


def law_ranking(cases):
    """F6.12 -- the summary figure: propellant remaining, every law, one bar.

    Cases that miss the target orbit are drawn hatched and grey and are not
    ranked against the rest. Propellant unspent by a vehicle that failed to
    arrive is not a saving, and sorting it alongside the successes would put the
    worst results at the top.
    """
    present = [n for n in RANKING_CASES if n in cases]
    if not present:
        return _skip("F6.12 law ranking", RANKING_CASES)

    records = []
    for name in present:
        case = cases[name]
        prop = case.row.get("prop_remaining_kg")
        if prop is None:
            continue
        records.append((st.law_label(case.law), prop / 1e3, case.reached_orbit))

    valid = sorted([r for r in records if r[2]], key=lambda r: r[1])
    invalid = sorted([r for r in records if not r[2]], key=lambda r: r[1])
    records = invalid + valid
    if not records:
        return _skip("F6.12 law ranking", ["any case with a propellant figure"])

    fig, ax = plt.subplots(figsize=st.bar_size(len(records)))
    for i, (label, prop_t, ok) in enumerate(records):
        ax.barh(i, prop_t, height=0.62,
                color=st.BASELINE if ok else st.FAILED,
                hatch=None if ok else "//",
                edgecolor="white", linewidth=0.5)
        ax.annotate("%.2f t" % prop_t, xy=(prop_t, i), xytext=(4, 0),
                    textcoords="offset points", fontsize=7, va="center",
                    color=st.INK if ok else st.FAILED)

    ref = cases.get("pmp_baseline")
    if ref is not None and ref.row.get("prop_remaining_kg") is not None:
        ref_t = ref.row["prop_remaining_kg"] / 1e3
        ax.axvline(ref_t, color=st.REFERENCE, linestyle="--", linewidth=1.1,
                   label="Indirect PMP reference (%.2f t)" % ref_t)

    ax.set_yticks(np.arange(len(records)))
    ax.set_yticklabels([r[0] for r in records])
    ax.set_xlabel("Propellant remaining at insertion [t]")
    n_invalid = len(invalid)
    if n_invalid:
        ax.annotate("hatched: target orbit not reached, not ranked",
                    xy=(0.02, 0.02), xycoords="axes fraction", fontsize=7,
                    color=st.FAILED)
    st.tidy(ax, legend_loc="lower right")
    fig.tight_layout()
    return st.save(fig, "results_law_ranking.png")


def accuracy_vs_propellant(cases):
    """F6.13 -- the trade, stated directly rather than inferred from two tables.

    Insertion accuracy on one axis and propellant on the other separates the
    laws that buy accuracy with propellant from those that give up both, which
    a ranking on either quantity alone cannot show.
    """
    present = [n for n in RANKING_CASES if n in cases]
    if not present:
        return _skip("F6.13 accuracy vs propellant", RANKING_CASES)

    target = None
    ref = cases.get("pmp_baseline")
    if ref is not None:
        target = ref.row.get("insertion_alt_km")

    fig, ax = plt.subplots(figsize=st.WIDE_1)
    plotted = 0
    for i, name in enumerate(present):
        case = cases[name]
        prop = case.row.get("prop_remaining_kg")
        peri = case.row.get("periapsis_km")
        apo = case.row.get("apoapsis_km")
        if prop is None or peri is None or apo is None:
            continue
        # Accuracy as the spread between apoapsis and periapsis: for a target
        # that is circular by definition, that gap is the whole of the miss.
        spread = abs(apo - peri)
        colour = st.VARIANT_CYCLE[i % len(st.VARIANT_CYCLE)]
        ax.scatter(spread, prop / 1e3, s=34, color=colour,
                   edgecolor="white", linewidth=0.6, zorder=3,
                   marker="o" if case.reached_orbit else "X")
        ax.annotate(st.law_label(case.law), xy=(spread, prop / 1e3),
                    xytext=(5, 4), textcoords="offset points", fontsize=7,
                    color=st.INK)
        plotted += 1

    if not plotted:
        return _skip("F6.13 accuracy vs propellant", ["any case with orbit elements"])

    if ref is not None and ref.row.get("prop_remaining_kg") is not None:
        ax.axhline(ref.row["prop_remaining_kg"] / 1e3, color=st.REFERENCE,
                   linestyle="--", linewidth=1.1, label="Indirect PMP reference")

    ax.set_xscale("symlog", linthresh=1.0)
    ax.set_xlabel("Apoapsis-periapsis spread at insertion [km]   (lower is better)")
    ax.set_ylabel("Propellant remaining [t]")
    ax.annotate("X: target orbit not reached", xy=(0.02, 0.04),
                xycoords="axes fraction", fontsize=7, color=st.FAILED)
    st.tidy(ax, legend_loc="lower left")
    fig.tight_layout()
    return st.save(fig, "results_accuracy_vs_propellant.png")


FIGURES = [loss_budget, loss_accumulation, law_ranking, accuracy_vs_propellant]
