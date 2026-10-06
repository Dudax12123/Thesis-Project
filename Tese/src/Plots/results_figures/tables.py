"""Build every Chapter 6 table from the results-matrix output.

The tables' counterpart of ``make_all``. Each table is written as one LaTeX
``tabular`` -- column heads, units and rows -- to ``<TAB_OUT>/<name>.tex``. The
thesis keeps the caption, the label and the type size, and draws the body with
``\\resultstable{<name>}``, which inputs the file when it exists and a
placeholder when it does not. A number in a table therefore cannot disagree with
the figure beside it -- both are read from the same archives by the same loader
-- and a re-flown case reaches the chapter by re-running this script, not by
editing numbers by hand.

Usage
-----
Write drafts to the scratch preview directory::

    python -m Plots.results_figures.tables

Write straight into the thesis repository::

    TAB_OUT=".../Thesis_Overleaf/Tables" python -m Plots.results_figures.tables

Options: ``--root`` to read a different results directory, ``--only`` to filter
by table name. A table whose cases are missing is skipped with a note.

Conventions, shared by every table: propellant in tonnes to 10 kg, with the
shortfall in kilograms beside it; the shortfall is taken against the reference
of the case's own environment (``sec65_losses._reference_for``), and is a dash
where no such reference was flown; burns and coasts are those of the second
stage; a dash marks a quantity that does not exist for the case, never a zero.
"""

import argparse
import math
import os
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent.parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import numpy as np

from Auxiliary import rocket_specs
from Plots.results_figures import _data
from Plots.results_figures import _panels
from Plots.results_figures.sec65_losses import _reference_for
from Plots.results_figures.sec67_capabilities import _search_cost

OUT_DIR = os.environ.get(
    "TAB_OUT",
    os.path.join(str(_SRC), "Output_Plots", "chapter_tables"),
)

DASH = "---"

# Propellant left at the swarm point each stored extremal was refined from. The
# re-flown extremals' archives do not carry it; the swarm archives do
# (Output/pmp_budget_750x1500/{pmp_baseline,pmp_vacuum,pmp_norot}/seed_3/,
# untracked), and Output/pmp_refine/pmp_swarm_polish_{b750half,vacuum_b750half_s3,
# norot_b750half_s3}.log print it as the refinement's "start 0".
SWARM_POINT_KG = {
    "pmp_baseline": 20432.367052514797,
    "pmp_vacuum": 21977.460563307217,
    "pmp_norot": 17638.87219156476,
}

# The thesis's names for the four open-loop laws of Section 6.5.1, which differ
# from the figure legends' in one place: Chapter 4 calls exp_shooting the
# exponential pitch law. show_apollo is tabulated once, in apollo_waypoint.
SHOWCASE = [
    ("show_cpr", "Constant pitch rate"),
    ("show_linear_tangent", "Linear tangent"),
    ("show_bilinear_tangent", "Bilinear tangent"),
    ("show_exp_shooting", "Exponential pitch"),
]

ARCH_SHORT = {
    "indirect_pmp": "indirect",
    "pso_coast": "coast",
    "apogee_check": "apogee check",
    "direct": "direct",
    "reference_track": "waypoint",
    "segmented": "segmented",
}

COST_ROWS = [
    ("apogee_check", "Apogee check (grid)"),
    ("pso_coast", "Coast parameter (swarm)"),
    ("direct", "Direct insertion (law-terminated, grid)"),
    ("indirect_pmp", "Indirect (swarm and refinement)"),
    ("reference_track", "Coast-start waypoint (no search)"),
    ("segmented", "Segmented (swarm)"),
]


# --- number formatting -----------------------------------------------------
def _grouped(text):
    """Thin-space thousands groups in the integer part, as the chapter writes
    $250\\,000$."""
    sign = text[0] if text[0] in "+-" else ""
    whole, dot, frac = text[len(sign):].partition(".")
    groups = []
    while len(whole) > 3:
        groups.insert(0, whole[-3:])
        whole = whole[:-3]
    return sign + r"\,".join([whole] + groups) + dot + frac


def num(value, digits=1, signed=False):
    """A number in math mode, so that a minus sign is a minus sign."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return DASH
    text = "%.*f" % (digits, value)
    if float(text) == 0.0:
        text = text.lstrip("-")
    elif signed and value > 0:
        text = "+" + text
    return "$%s$" % _grouped(text)


def tonnes(kg):
    return DASH if kg is None else num(kg / 1e3, 2)


def seconds(value):
    """A duration: to the millisecond under a second (the reference's final
    burn lasts 25 ms), to a tenth otherwise."""
    if value is None:
        return DASH
    return num(value, 3 if value < 1.0 else 1)


def hours(wall_s):
    if wall_s is None:
        return DASH
    h = wall_s / 3600.0
    if h < 0.001:
        return "$<0.001$"
    return num(h, 3 if h < 1.0 else 1)


def count(n):
    return DASH if n is None else num(float(n), 0)


def case_cell(name):
    return r"\texttt{%s}" % name.replace("_", r"\_")


# --- derived quantities ----------------------------------------------------
def prop(case):
    return case.row.get("prop_remaining_kg")


def shortfall(case, cases):
    """Reference propellant minus the case's [kg], against the reference of the
    case's own environment; None for a reference, or where none was flown."""
    if case.architecture == "indirect_pmp":
        return None
    ref, same = _reference_for(case, cases)
    if ref is None or not same:
        return None
    return prop(ref) - prop(case)


def arcs(case):
    """(first burn, coast, final burn) of the second stage [s], None where the
    architecture flies no such arc.

    Read from the thrust trace, like the arc-structure figure, rather than from
    Case.coast_intervals, whose 5 s floor would drop the half-second coast the
    swarm leaves PEG. The apogee check's coast runs from its cut-off to the
    impulsive circularisation at apogee, which is not a burn in the trace.
    """
    burns = _panels.burn_intervals(case, stage2_only=True)
    if not burns:
        return None, None, None
    first = burns[0][1] - burns[0][0]
    if case.architecture == "apogee_check":
        return first, case.t_insertion - case.t_seco, None
    if len(burns) < 2:
        return first, None, None
    coast = sum(b[0] - a[1] for a, b in zip(burns, burns[1:]))
    return first, coast, burns[-1][1] - burns[-1][0]


def hand_off_km(case):
    """Where the segmented schedule hands over to its second law [km]."""
    schedule = case.segment_schedule
    if not schedule or len(schedule) < 2:
        return None
    return schedule[1][1] / 1e3


def gamma_p_deg(case):
    g = case.gamma_p
    return None if g is None else math.degrees(g)


def search(case):
    """(wall clock [s], evaluations) of the search behind the case. The
    reference-tracking cases search nothing and fly once, which the archive
    leaves blank."""
    cost = _search_cost(case)
    if cost is None:
        return None, None
    wall, evals = cost
    if not evals and case.architecture == "reference_track":
        evals = 1
    return wall, evals


def tail_pct(case):
    """The share of the swarm's improvement made in its last quarter [%]."""
    row = case.row
    frac = row.get("search_tail_improvement_frac")
    if frac is None:
        frac = row.get("pso_tail_improvement_frac")
    return None if frac is None else 100.0 * frac


# --- layout ----------------------------------------------------------------
def tabular(spec, heads, units, rows, rule_under_heads=None):
    """One booktabs tabular. A row of None is an \\addlinespace."""
    lines = [r"\begin{tabular}{%s}" % spec, r"  \toprule",
             "  " + " & ".join(heads) + r" \\"]
    if rule_under_heads:
        lines.append("  " + rule_under_heads)
    lines += ["  " + " & ".join(units) + r" \\", r"  \midrule"]
    for row in rows:
        lines.append(r"  \addlinespace" if row is None
                     else "  " + " & ".join(row) + r" \\")
    lines += [r"  \bottomrule", r"\end{tabular}"]
    return "\n".join(lines) + "\n"


def bold(*heads):
    return [r"\textbf{%s}" % h if h else "" for h in heads]


def _missing(cases, names, table):
    absent = [n for n in names if n not in cases]
    if absent:
        print("  skip %-20s missing %s" % (table, ", ".join(absent)))
    return bool(absent)


# --- the tables --------------------------------------------------------------
def reference_results(cases):
    names = ["pmp_baseline", "pmp_vacuum", "pmp_norot"]
    if _missing(cases, names, "reference_results"):
        return None
    rows = []
    for n in names:
        k = cases[n]
        first, coast, final = arcs(k)
        rows.append([case_cell(n), tonnes(prop(k)), seconds(first), seconds(coast),
                     seconds(final), num(k.row["apoapsis_km"]),
                     num(k.row["periapsis_km"]), tonnes(SWARM_POINT_KG.get(n))])
    return tabular(
        "l r r r r r r r",
        bold("Case", "Prop. left", "First burn", "Coast", "Final burn", "$h_a$",
             "$h_p$", "Swarm point"),
        ["", "[t]", "[s]", "[s]", "[s]", "[km]", "[km]", "[t]"], rows)


def gt_results(cases):
    names = ["gt_baseline", "gt_apogee", "gt_norot", "gt_sea_level_engine"]
    if _missing(cases, names, "gt_results"):
        return None
    base = prop(cases["gt_baseline"])
    rows = []
    for n in names:
        k = cases[n]
        vs = None if n == "gt_baseline" else prop(k) - base
        rows.append([case_cell(n), tonnes(prop(k)), num(vs, 0, signed=True),
                     num(shortfall(k, cases), 0), num(k.row["apoapsis_km"]),
                     num(k.row["periapsis_km"]), seconds(arcs(k)[1])])
    return tabular(
        "l r r r r r r",
        bold("Case", "Prop. left", "vs baseline", "Shortfall", "$h_a$", "$h_p$", "Coast"),
        ["", "[t]", "[kg]", "[kg]", "[km]", "[km]", "[s]"], rows)


def _law_row(name, cases, label=None):
    """Prop., shortfall, coast, steering loss, h_a, h_p."""
    k = cases[name]
    return [label or case_cell(name), tonnes(prop(k)), num(shortfall(k, cases), 0),
            seconds(arcs(k)[1]), num(k.row["dv_steering"]),
            num(k.row["apoapsis_km"]), num(k.row["periapsis_km"])]


def peg_results(cases):
    names = ["gt_baseline", "peg_baseline", "peg_direct", "peg_vacuum"]
    if _missing(cases, names, "peg_results"):
        return None
    rows = [_law_row("gt_baseline", cases), None] + [_law_row(n, cases)
                                                     for n in names[1:]]
    return tabular(
        "l r r r r r r",
        bold("Case", "Prop. left", "Shortfall", "Coast", "Steering loss", "$h_a$",
             "$h_p$"),
        ["", "[t]", "[kg]", "[s]", "[m/s]", "[km]", "[km]"], rows)


def _waypoint_table(cases, orbit, waypoint, table):
    names = [orbit, waypoint, "pmp_baseline"]
    if _missing(cases, names, table):
        return None
    rows = []
    for n in names:
        k = cases[n]
        prop_, short, coast, steer, ha, hp = _law_row(n, cases)[1:]
        miss = k.waypoint_miss()
        dh, dv, dg = (DASH,) * 3 if miss is None else (
            num(miss[0], 0, signed=True), num(miss[1], 2, signed=True),
            num(miss[2], 3, signed=True))
        rows.append([case_cell(n), prop_, short, coast, dh, dv, dg, steer, ha, hp])
    return tabular(
        "l r r r r r r r r r",
        bold("Case", "Prop. left", "Shortfall", "Coast")
        + [r"\multicolumn{3}{c}{\textbf{Waypoint miss}}"]
        + bold("Steering loss", "$h_a$", "$h_p$"),
        ["", "[t]", "[kg]", "[s]", r"$\Delta h$ [m]", r"$\Delta v$ [m/s]",
         r"$\Delta\gamma$ [deg]", "[m/s]", "[km]", "[km]"],
        rows, rule_under_heads=r"\cmidrule(lr){5-7}")


def peg_waypoint(cases):
    return _waypoint_table(cases, "peg_baseline", "show_ref_track", "peg_waypoint")


def apollo_waypoint(cases):
    return _waypoint_table(cases, "show_apollo", "show_ref_track_apollo",
                           "apollo_waypoint")


def showcase_laws(cases):
    names = [n for n, _label in SHOWCASE]
    if _missing(cases, names, "showcase_laws"):
        return None
    rows = [_law_row(n, cases, label) + [hours(search(cases[n])[0])]
            for n, label in SHOWCASE]
    return tabular(
        "l r r r r r r r",
        bold("Guidance law", "Prop. left", "Shortfall", "Coast", "Steering loss",
             "$h_a$", "$h_p$", "Solve"),
        ["", "[t]", "[kg]", "[s]", "[m/s]", "[km]", "[km]", "[h]"], rows)


def segmented_results(cases):
    # peg_baseline is left out: tab:peg_results and tab:peg_waypoint carry it.
    names = ["show_seg_fixed_alt", "show_seg_opt_alt", "show_ref_track"]
    if _missing(cases, names, "segmented_results"):
        return None
    rows = []
    for n in names:
        k = cases[n]
        wall = None if k.architecture == "reference_track" else search(k)[0]
        rows.append([case_cell(n), tonnes(prop(k)), num(shortfall(k, cases), 0),
                     num(hand_off_km(k)), num(gamma_p_deg(k), 2), seconds(arcs(k)[1]),
                     num(k.row["apoapsis_km"]), num(k.row["periapsis_km"]), hours(wall)])
    return tabular(
        "l r r r r r r r r",
        bold("Case", "Prop. left", "Shortfall", "Hand-off", r"$\gamma_p$", "Coast",
             "$h_a$", "$h_p$", "Solve"),
        ["", "[t]", "[kg]", "[km]", "[deg]", "[s]", "[km]", "[km]", "[h]"], rows)


def full_results(cases):
    """What the section tables leave out, for every case: the insertion state,
    the kick, the arc durations, the event times and the cost of the search.
    The delta-v budget is tab:loss_budget, in full."""
    names = [n for n in _data.REPORTED_CASES if n in cases]
    rows = []
    for n in names:
        k = cases[n]
        first, coast, final = arcs(k)
        wall, evals = search(k)
        rows.append([case_cell(n), ARCH_SHORT.get(k.architecture, k.architecture),
                     num(prop(k), 0), num(k.row["insertion_alt_km"]),
                     num(k.row["insertion_v_ms"]), num(k.row["insertion_fpa_deg"], 3),
                     num(k.row["apoapsis_km"]), num(k.row["periapsis_km"]),
                     num(gamma_p_deg(k), 2), seconds(first), seconds(coast),
                     seconds(final), num(k.t_meco), num(k.t_insertion),
                     count(evals), hours(wall)])
    return tabular(
        "l l " + " ".join(["r"] * 14),
        bold("Case", "Architecture", "Prop.", "$h$", "$v$", r"$\gamma$", "$h_a$",
             "$h_p$", r"$\gamma_p$", "First burn", "Coast", "Final burn",
             r"$t_{MECO}$", r"$t_{ins}$", "Evaluations", "Wall clock"),
        ["", "", "[kg]", "[km]", "[m/s]", "[deg]", "[km]", "[km]", "[deg]", "[s]",
         "[s]", "[s]", "[s]", "[s]", "[-]", "[h]"], rows)


def loss_budget(cases):
    names = [n for n in _data.REPORTED_CASES if n in cases]
    rows = []
    for n in names:
        r = cases[n].row
        pressure = r["dv_pressure"] if r.get("pressure_applicable") else None
        rows.append([case_cell(n), num(r["dv_ideal"]), num(r["dv_gravity"]),
                     num(r["dv_drag"]), num(r["dv_steering"]), num(pressure),
                     num(r["dv_gain"]), num(r["residual"])])
    return tabular(
        "l r r r r r r r",
        bold("Case", r"$\Delta V_{\mathrm{ideal}}$", "Gravity", "Drag", "Steering", "Pressure",
             "Gain", "Residual"),
        ["", "[m/s]", "[m/s]", "[m/s]", "[m/s]", "[m/s]", "[m/s]", "[m/s]"], rows)


def _pct_range(values):
    """Tail improvements [%] as a range, to two significant figures of the
    largest; a swarm that stopped improving altogether reads below 1e-5 %."""
    values = [v for v in values if v is not None]
    if not values:
        return DASH
    hi = max(values)
    if hi < 1e-5:
        return "$<10^{-5}$"
    digits = max(2, 1 - int(math.floor(math.log10(hi))))
    lo = num(min(values), digits)
    return lo if len(values) == 1 or lo == num(hi, digits) else "%s--%s" % (
        lo, num(hi, digits))


def architecture_cost(cases):
    """One row per architecture, over every reported case it flew. Where its
    cases share one budget the wall clock is a range; where each ran its own
    (the extremals) the values are listed per case."""
    rows = []
    for arch, label in COST_ROWS:
        members = [cases[n] for n in _data.REPORTED_CASES
                   if n in cases and cases[n].architecture == arch]
        if not members:
            continue
        costs = [search(k) for k in members]
        walls = [w for w, _e in costs if w is not None]
        evals = [e for _w, e in costs if e]
        if len(set(evals)) > 1:
            evals_cell = " / ".join(count(e) for e in evals)
            wall_cell = " / ".join(hours(w) for w in walls)
        else:
            evals_cell = count(evals[0]) if evals else DASH
            lo, hi = hours(min(walls)), hours(max(walls))
            wall_cell = lo if lo == hi else "%s--%s" % (lo, hi)
        rows.append([label, evals_cell, wall_cell,
                     _pct_range([tail_pct(k) for k in members])])
    return tabular(
        "l r r r",
        bold("Architecture", "Evaluations", "Wall clock", "Tail improvement"),
        ["", "[-]", "[h]", r"[\%]"], rows)


def drag_after_separation(case):
    """Drag loss the budget accumulates after stage separation [m/s].

    The swarm architectures fly that interval without drag, but the budget's
    drag integral runs to the last cut-off, so this much of each case's drag
    term was never flown."""
    hist = case.loss_histories()
    t = case.time[:case.cutoff_index()]
    t_sep = case.t_meco + rocket_specs.TIME_First_STAGE_SEPARATION
    i = int(np.searchsorted(t, t_sep))
    drag = hist["drag"]
    return float(drag[-1] - drag[i]) if i < len(drag) else 0.0


def caption_values(cases):
    """Numbers the Chapter 6 captions quote, as LaTeX macros rather than a table.

    The thesis preamble inputs Tables/caption_values.tex, so a caption follows a
    re-flown case exactly as a table body does:
    \\circDvApogee is the apogee check's impulsive circularisation [m/s], and
    \\dragAfterSeparation the largest drag the budget books after separation
    on an architecture that flies without it there [m/s]."""
    if _missing(cases, ["gt_apogee"], "caption_values"):
        return None
    circ = cases["gt_apogee"].row["circularisation_dv"]
    post = [drag_after_separation(k) for k in cases.values()
            if k.architecture != "apogee_check" and k.row.get("include_drag", True)]
    return ("\\newcommand{\\circDvApogee}{%s}\n"
            "\\newcommand{\\dragAfterSeparation}{%s}\n"
            % (_grouped("%.1f" % circ), _grouped("%.1f" % max(post))))


TABLES = [
    reference_results, gt_results, peg_results, peg_waypoint, showcase_laws,
    apollo_waypoint, segmented_results, full_results, loss_budget,
    architecture_cost, caption_values,
]

_HEADER = ("% Generated by Tese/src/Plots/results_figures/tables.py from the results\n"
           "% matrix. Do not edit by hand: re-run the script.\n")


def write(name, body):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name + ".tex")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(_HEADER + body)
    return path


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", help="results directory "
                                       "(default: Output/results_matrix)")
    parser.add_argument("--only", help="substring filter over table names")
    args = parser.parse_args()

    cases = _data.load_many(_data.REPORTED_CASES, root=args.root)
    _data.check_one_rotation_model(cases)
    print("=" * 70)
    print("CHAPTER 6 TABLES -- %d of %d cases available"
          % (len(cases), len(_data.REPORTED_CASES)))
    print("output: %s" % OUT_DIR)
    print("=" * 70)

    written, skipped = 0, 0
    for table in TABLES:
        if args.only is not None and args.only not in table.__name__:
            continue
        body = table(cases)
        if body is None:
            skipped += 1
            continue
        print("  wrote %s" % write(table.__name__, body))
        written += 1

    print("=" * 70)
    print("%d table(s) written, %d skipped" % (written, skipped))
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
