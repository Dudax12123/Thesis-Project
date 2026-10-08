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
from matplotlib.text import Text

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
    """F6.11 -- where in the reference's flight each term of the budget accrues
    (red-note review S6.7, 2026-10-08: the reference only, every component).

    The four losses, the residual (the in-plane centrifugal work, dotted) and the
    launch-site gain below zero, as the credit it is: omega r cos(phi0) at the
    current radius, which at insertion is the budget's dv_gain. The residual is
    the budget identity along the flight, ideal - losses - (v - v0), which at
    insertion is the row's residual. The coast is drawn compressed; MECO, the
    start of the coast and insertion are dotted.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("F6.11 loss accumulation", missing)
    from Auxiliary import constants as const

    ref = cases["pmp_baseline"]
    hist = ref.loss_histories()
    end = ref.cutoff_index()
    t_full = ref.time[:end]
    fig, ax = plt.subplots(figsize=st.WIDE_1)
    for comp in ("gravity", "drag", "steering", "pressure"):
        series = hist.get(comp)
        if series is None or not np.any(np.abs(series) > 1e-9):
            continue
        ax.plot(*st.thin(t_full, series), color=st.LOSS_COLORS[comp],
                label=comp.capitalize())
    v = ref.v[:end]
    residual = hist["ideal"] - hist["total"] - (v - v[0])
    ax.plot(*st.thin(t_full, residual), color=st.INK, linestyle=":",
            label="Residual (in-plane centrifugal work)")
    if ref.budget().get("dv_gain"):
        lat = np.deg2rad((ref.manifest.get("config") or {}).get("LAUNCH_LATITUDE", 28.5))
        gain = const.OMEGA_EARTH * ref.data[1, :end] * np.cos(lat)
        ax.plot(*st.thin(t_full, -gain), color=st.ENV_COLORS["reference"], linestyle="--",
                label=r"Launch-site gain $\omega r \cos\phi_0$ (credit)")
    ax.axhline(0.0, color=st.INK, linewidth=0.7)

    burns = pn.burn_intervals(ref, stage2_only=True)
    events = [(ref.t_meco, "MECO")]
    if burns:
        events.append((burns[0][1], "coast start"))
    events.append((ref.t_insertion, "insertion"))
    for t_evt, text in events:
        if t_evt is None:
            continue
        ax.axvline(t_evt, color=st.GREY, linestyle=":", linewidth=0.8)
        ax.annotate(text, xy=(t_evt, 0.97), xycoords=("data", "axes fraction"),
                    xytext=(-2, 0), textcoords="offset points", fontsize=6.3,
                    color=st.INK, rotation=90, ha="right", va="top")
    pn.compress_long_coast(ax, ref, float(t_full[-1]) + 10.0, label_y=0.55)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Cumulative $\Delta V$ [m/s]")
    st.tidy(ax, legend=False)
    pn.figure_legend(fig, ax, ncol=3)
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

    Drag-free cases are measured against pmp_vacuum, non-rotating ones against
    pmp_norot and everything else against pmp_baseline. That is an exact match
    only when the reference also shares the case's remaining settings. No
    reference was flown for the sea-level-nozzle environment (decision
    2026-10-04, D1) or for a drag-free non-rotating one, so those cases get no
    difference at all (decision 2026-09-30).
    """
    if case.row.get("include_drag") is False:
        name = "pmp_vacuum"
    elif case.row.get("earth_rotation") is False:
        name = "pmp_norot"
    else:
        name = "pmp_baseline"
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


# The environment blocks of the multi-case figures, in reading order, each with
# the reference flown in it (red-note review S6.3/S6.4, 2026-10-08). No reference
# was flown for the sea-level nozzle.
ENV_BLOCKS = [("baseline", "pmp_baseline"), ("no_atmosphere", "pmp_vacuum"),
              ("no_rotation", "pmp_norot"), ("sea_level", None)]


def environment_blocks(cases):
    """[(environment, [case names])]: the reported cases grouped by environment,
    each block its reference first and the rest by propellant, most first."""
    present = _with_propellant(cases)
    blocks = []
    for env, ref in ENV_BLOCKS:
        members = sorted((n for n in present if st.environment(cases[n].row) == env
                          and not _is_reference(cases[n])),
                         key=lambda n: -cases[n].row["prop_remaining_kg"])
        rows = ([ref] if ref in present else []) + members
        if rows:
            blocks.append((env, rows))
    return blocks


def _block_rows(blocks, gap=0.55, heading=0.9):
    """y position of every case and of every block heading, top to bottom."""
    y, rows, headings = 0.0, [], []
    for env, names in blocks:
        headings.append((env, y))
        y += heading
        for name in names:
            rows.append((name, y))
            y += 1.0
        y += gap
    return rows, headings, y - gap


def _block_axes(ax, cases, rows, headings, bold_refs=True):
    """Case labels on the y axis (references bold), block headings above each."""
    ax.set_yticks([y for _n, y in rows])
    ax.set_yticklabels([st.case_label(n) for n, _y in rows], fontsize=7)
    for tick, (name, _y) in zip(ax.get_yticklabels(), rows):
        if bold_refs and _is_reference(cases[name]):
            tick.set_fontweight("bold")
    for env, y in headings:
        ax.annotate(st.ENV_LABELS[env], xy=(0.0, y + 0.15),
                    xycoords=("axes fraction", "data"), xytext=(2, 0),
                    textcoords="offset points", fontsize=7.2, fontweight="bold",
                    color=st.INK, va="center", ha="left")


def law_ranking(cases):
    """Propellant remaining at insertion for every reported case, grouped by the
    environment it was flown in (red-note review S6.3, 2026-10-08).

    Each block opens with the reference flown in its environment, in green with a
    bold label, and the cases follow in their environment's colour, by
    propellant. The grouping puts each case beside its own reference, so the
    shortfall is read off the bars rather than printed; only the propellant is.
    """
    blocks = environment_blocks(cases)
    if not blocks:
        return _skip("law ranking", _data.REPORTED_CASES)
    rows, headings, y_end = _block_rows(blocks)

    fig, ax = plt.subplots(figsize=(6.3, 0.9 + 0.235 * (y_end + 1.0)))
    for name, y in rows:
        case = cases[name]
        prop_t = case.row["prop_remaining_kg"] / 1e3
        colour = st.ENV_COLORS["reference" if _is_reference(case)
                               else st.environment(case.row)]
        ax.barh(y, prop_t, height=0.7, color=colour, edgecolor="white", linewidth=0.5,
                hatch=None if case.reached_orbit else "//")
        light = colour == st.ENV_COLORS["no_rotation"]
        ax.annotate("%.2f t" % prop_t, xy=(prop_t, y), xytext=(-4, 0),
                    textcoords="offset points", fontsize=6.5, va="center", ha="right",
                    color=st.INK if light else "white",
                    fontweight="bold" if _is_reference(case) else "normal")
    _block_axes(ax, cases, rows, headings)
    ax.set_ylim(y_end + 0.6, -0.6)
    ax.set_xlim(0.0, 1.03 * max(cases[n].row["prop_remaining_kg"] for n, _y in rows) / 1e3)
    ax.set_xlabel("Propellant remaining at insertion [t]")
    st.tidy(ax, legend=False)
    ax.grid(False, axis="y")
    fig.tight_layout()
    return st.save(fig, "results_law_ranking.png")


def accuracy_vs_propellant(cases):
    """The trade, stated directly rather than inferred from two tables.

    Insertion accuracy on one axis and propellant on the other separates the
    cases that buy accuracy with propellant from those that give up both, which
    a ranking on either quantity alone cannot show. Every reported case is
    drawn, coloured by the environment it was flown in, the references as green
    stars (red-note review X4/S6.1, 2026-10-08).
    """
    present = [n for n in _with_propellant(cases)
               if _orbit_spread_km(cases[n]) is not None]
    if not present:
        return _skip("accuracy vs propellant", _data.REPORTED_CASES)

    fig, ax = plt.subplots(figsize=st.TALL_1)
    labels, points = [], []
    for name in present:
        case = cases[name]
        prop_t = case.row["prop_remaining_kg"] / 1e3
        spread = _orbit_spread_km(case)
        ref = _is_reference(case)
        colour = st.ENV_COLORS["reference" if ref else st.environment(case.row)]
        marker = "*" if ref else ("o" if case.reached_orbit else "X")
        ax.scatter(spread, prop_t, s=90 if ref else 32, marker=marker, facecolor=colour,
                   edgecolor="white", linewidth=0.6, zorder=5 if ref else 3)
        points.append((spread, prop_t))
        right_edge = spread > 5.0
        labels.append(ax.annotate(st.case_label(name), xy=(spread, prop_t),
                                  xytext=(-7 if right_edge else 7, -2),
                                  textcoords="offset points",
                                  ha="right" if right_edge else "left",
                                  fontsize=6.3, color=st.INK, zorder=4,
                                  fontweight="bold" if ref else "normal",
                                  bbox={"facecolor": "white", "edgecolor": "none",
                                        "pad": 0.4, "alpha": 0.85},
                                  arrowprops={"arrowstyle": "-", "color": st.GREY,
                                              "linewidth": 0.5, "shrinkA": 0,
                                              "shrinkB": 3}))

    # Symlog, linear below 0.1 km: half the cases insert within a few hundred
    # metres of circular and the others miss by up to 20 km, and a linear axis
    # would put the first half on top of one another at zero.
    ax.set_xscale("symlog", linthresh=0.1, linscale=0.6)
    ax.set_xlim(-0.004, 60.0)
    ax.set_xticks([0.0, 0.1, 1.0, 10.0])
    ax.set_xticklabels(["0", "0.1", "1", "10"])
    ax.set_xlabel("Apoapsis-periapsis spread at insertion [km]   (lower is better)")
    ax.set_ylabel("Propellant remaining [t]")

    envs = []
    for name in present:
        env = st.environment(cases[name].row)
        if env not in envs and not _is_reference(cases[name]):
            envs.append(env)
    handles = [Line2D([], [], marker="*", markersize=9, linestyle="none",
                      markerfacecolor=st.ENV_COLORS["reference"], markeredgecolor="white",
                      label="Reference")]
    handles += [Line2D([], [], marker="o", markersize=5.5, linestyle="none",
                       markerfacecolor=st.ENV_COLORS[e], markeredgecolor="white",
                       label=st.ENV_LABELS[e]) for e in envs]
    st.tidy(ax, legend=False)
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.17), ncol=5,
              fontsize=6.8, handletextpad=0.3, columnspacing=1.2)
    fig.tight_layout()
    # Placed last: the placement measures rendered extents, so it has to run
    # after every artist that moves them.
    _place_labels(fig, ax, labels, points)
    return st.save(fig, "results_accuracy_vs_propellant.png")


# Offsets [pt] tried, in order, for a point label: right of the point, then
# above and below it on the right, then left of it, then further right.
_LABEL_OFFSETS = [(7, -2, "left"), (-7, -2, "right"), (7, 6, "left"), (7, -10, "left"),
                  (24, -2, "left"), (24, 8, "left"), (24, -12, "left"), (7, 13, "left"),
                  (7, -17, "left"), (40, -2, "left"), (40, 10, "left"), (40, -14, "left")]


def _place_labels(fig, ax, labels, points, pad=2.0):
    """Give every point label the first offset that keeps it inside the axes and
    clear of every marker and of every label already placed.

    A point's label can collide with its neighbours' labels and with their
    markers, and from the axis at the left edge; only the rendered extents show
    which, and per-case offsets fixed by hand would rot the next time a case is
    re-flown. Labels are placed from the top down; a label that finds no free
    offset keeps the first.
    """
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    frame = ax.get_window_extent(renderer=renderer)
    pixels = [ax.transData.transform(p) for p in points]
    order = sorted(range(len(labels)), key=lambda k: -points[k][1])
    placed = []

    def clear(box, k):
        if box.x0 < frame.x0 or box.x1 > frame.x1 or box.y0 < frame.y0 or box.y1 > frame.y1:
            return False
        for j, (x, y) in enumerate(pixels):
            if j != k and box.x0 - pad < x < box.x1 + pad and box.y0 - pad < y < box.y1 + pad:
                return False
        return not any(box.x0 < o.x1 + pad and box.x1 > o.x0 - pad
                       and box.y0 < o.y1 + pad and box.y1 > o.y0 - pad for o in placed)

    def text_box(ann):
        # The text alone: an annotation's own extent includes its leader line.
        ann.update_positions(renderer)
        return Text.get_window_extent(ann, renderer)

    for k in order:
        ann = labels[k]
        chosen = None
        for dx, dy, ha in _LABEL_OFFSETS:
            ann.set_position((dx, dy))
            ann.set_ha(ha)
            box = text_box(ann)
            if clear(box, k):
                chosen = box
                break
        if chosen is None:
            dx, dy, ha = _LABEL_OFFSETS[0]
            ann.set_position((dx, dy))
            ann.set_ha(ha)
            chosen = text_box(ann)
        placed.append(chosen)


# The phases of the arc-structure chart. The environment is given by the block
# headings there, not by colour (red-note review S6.4: the bars carry the phase).
PHASE_COLORS = {"stage1": "#eb6834", "stage2": "#2f4b7c", "coast": st.FAINT}


def arc_structure(cases):
    """The arc structure of every case on one time axis, grouped by environment
    as in the ranking, each block its reference first (red-note review S6.4).

    The first stage, the second-stage burns -- before and after the coast alike
    -- and the coasts, read from the thrust trace (_panels.burn_intervals,
    Case.coast_intervals) so every architecture is drawn by the same rule. A burn
    too short to show at this scale is drawn at a minimum width, in the same
    colour. The gap after the first stage is the planned separation delay. The
    apogee check's impulsive circularization, which the thrust trace does not
    hold, is a diamond at the instant Case.t_insertion finds.
    """
    blocks = environment_blocks(cases)
    if not blocks:
        return _skip("arc structure", _data.REPORTED_CASES)
    rows, headings, y_end = _block_rows(blocks)
    t_max = max((cases[n].t_insertion or float(cases[n].time[-1])) for n, _y in rows)
    min_w = 0.006 * t_max

    fig, ax = plt.subplots(figsize=(6.3, 0.9 + 0.235 * (y_end + 1.0)))
    half = 0.30
    for name, y in rows:
        case = cases[name]
        for t0, t1 in pn.burn_intervals(case):
            stage1 = case.t_meco is not None and t0 < case.t_meco - 0.5
            ax.broken_barh([(t0, max(t1 - t0, min_w))], (y - half, 2 * half),
                           facecolors=PHASE_COLORS["stage1" if stage1 else "stage2"],
                           linewidth=0, zorder=2)
        for t0, t1 in case.coast_intervals():
            ax.broken_barh([(t0, t1 - t0)], (y - half, 2 * half),
                           facecolors=PHASE_COLORS["coast"], linewidth=0, zorder=1)
        t_end = case.t_insertion if case.t_insertion is not None else case.time[-1]
        if case.architecture == "apogee_check":
            ax.plot(t_end, y, marker="D", markersize=4.2, color=st.INK, zorder=4)
    _block_axes(ax, cases, rows, headings)
    ax.set_ylim(y_end + 0.6, -0.6)
    ax.set_xlim(0.0, 1.02 * t_max)
    ax.set_xlabel("Time [s]")
    handles = [
        Patch(facecolor=PHASE_COLORS["stage1"], label="First-stage burn"),
        Patch(facecolor=PHASE_COLORS["stage2"], label="Second-stage burn"),
        Patch(facecolor=PHASE_COLORS["coast"], label="Coast"),
        Line2D([], [], color=st.INK, marker="D", markersize=4.2,
               linestyle="none", label="Impulsive circularization"),
    ]
    st.tidy(ax, legend=False)
    ax.grid(False, axis="y")
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.09),
              ncol=4, fontsize=6.8)
    fig.tight_layout()
    return st.save(fig, "results_arc_structure.png")


FIGURES = [loss_budget, loss_accumulation, law_ranking, accuracy_vs_propellant,
           arc_structure]
