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
import re
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent.parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import numpy as np

from Auxiliary import rocket_specs
from Plots.results_figures import _data
from Plots.results_figures import _panels
from Plots.results_figures import _style
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
def num(value, digits=1, signed=False):
    """A number through siunitx's \\num, which sets the minus sign and the
    thousands groups; the preamble's retain-explicit-plus keeps a '+'."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return DASH
    text = "%.*f" % (digits, value)
    if float(text) == 0.0:
        text = text.lstrip("-")
    elif signed and value > 0:
        text = "+" + text
    return r"\num{%s}" % text


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
        return r"\num{<0.001}"
    return num(h, 3 if h < 1.0 else 1)


def count(n):
    return DASH if n is None else num(float(n), 0)


def case_cell(name):
    return r"\texttt{%s}" % name.replace("_", r"\_")


def label_cell(name):
    """The case's readable name, as the figures print it (red-note review S3.3,
    2026-10-08); the section tables use it, the appendix and budget tables the id."""
    return _style.case_label(name)


def orbit(case):
    """Apoapsis x periapsis [km], one cell."""
    ha, hp = case.row.get("apoapsis_km"), case.row.get("periapsis_km")
    if ha is None or hp is None:
        return DASH
    return r"\numproduct{%.1f x %.1f}" % (ha, hp)


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


def coast_start(case):
    """When the second stage's first burn ends [s], or None for a single burn."""
    burns = _panels.burn_intervals(case, stage2_only=True)
    return burns[0][1] if len(burns) >= 2 else None


def altitude_at(case, t):
    """Altitude [km] on the archived trajectory at time *t*, None for no time."""
    if t is None:
        return None
    return float(np.interp(t, case.time, case.alt_km))


def steering_split(case):
    """Steering loss [m/s] before and after the second stage's longest coast.

    The loss integral covers the powered arc only, so the split is its value
    where that coast starts and the remainder. With no coast long enough to
    separate two burns (Case.coast_intervals' 5 s floor) the remainder is None.
    """
    hist = case.loss_histories()["steering"]
    t = case.time[:case.cutoff_index()]
    total = float(hist[-1])
    spans = case.coast_intervals()
    if not spans:
        return total, None
    t0, _t1 = max(spans, key=lambda s: s[1] - s[0])
    before = float(np.interp(t0, t, hist))
    return before, total - before


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


def refined_law(case):
    """True for a guidance-law case reported at its refined point: the swarm's best
    flight refined as the references are (search_refine_*), the coast-parameter
    gravity turn since 2026-10-07. The references refine too, but are no law."""
    return (case.architecture != "indirect_pmp"
            and case.row.get("search_refine_wall_clock_s") is not None)


def refinement(case):
    """(wall clock [s], flights) of a refined law case's refinement alone."""
    return float(case.row["search_refine_wall_clock_s"]), int(case.row.get("n_evaluations") or 0)


def swarm_search(case):
    """(wall clock [s], evaluations) of the swarm behind the case, without the
    refinement a refined law case adds (caption_values gives that)."""
    wall, evals = search(case)
    if refined_law(case):
        r_wall, r_flights = refinement(case)
        wall, evals = wall - r_wall, evals - r_flights
    return wall, evals


def tail_pct(case):
    """The share of the swarm's improvement made in its last quarter [%]."""
    row = case.row
    frac = row.get("search_tail_improvement_frac")
    if frac is None:
        frac = row.get("pso_tail_improvement_frac")
    return None if frac is None else 100.0 * frac


# --- layout ----------------------------------------------------------------
# The header units, as siunitx unit macros; "[-]" (dimensionless) stays as is.
SI_UNITS = {
    "t": r"\tonne", "kg": r"\kg", "km": r"\km", "m": r"\m", "s": r"\s",
    "h": r"\hour", "m/s": r"\m\per\s", "km/s": r"\km\per\s", "deg": r"\degree",
    "kPa": r"\kilo\pascal", r"\%": r"\percent",
}


def si_unit_head(head):
    """'[m/s]' -> '[\\unit{\\m\\per\\s}]', wherever the bracket sits in the head."""
    def sub(match):
        unit = SI_UNITS.get(match.group(1))
        return match.group(0) if unit is None else r"[\unit{%s}]" % unit
    return re.sub(r"\[([^\]]+)\]", sub, head)


def tabular(spec, heads, units, rows, rule_under_heads=None):
    """One booktabs tabular. A row of None is an \\addlinespace."""
    lines = [r"\begin{tabular}{%s}" % spec, r"  \toprule",
             "  " + " & ".join(heads) + r" \\"]
    if rule_under_heads:
        lines.append("  " + rule_under_heads)
    lines += ["  " + " & ".join(si_unit_head(u) for u in units) + r" \\",
              r"  \midrule"]
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
    """The three references, arc by arc: where each arc begins and how long it
    lasts. Every reference inserts on the circular target orbit, so the altitudes
    worth a column are those at main-engine cut-off and at the coast start
    (walkthrough S1-T1 and S1-F2, 2026-10-06), not h_a and h_p."""
    names = ["pmp_baseline", "pmp_vacuum", "pmp_norot"]
    if _missing(cases, names, "reference_results"):
        return None
    rows = []
    for n in names:
        k = cases[n]
        first, coast, final = arcs(k)
        rows.append([REFERENCE_LABELS[n], tonnes(prop(k)), num(altitude_at(k, k.t_meco)),
                     seconds(first), num(altitude_at(k, coast_start(k))),
                     seconds(coast), seconds(final), tonnes(SWARM_POINT_KG.get(n))])
    return tabular(
        "l r r r r r r r",
        bold("Environment", "Prop. left", r"$h_{\mathrm{MECO}}$", "First burn",
             r"$h_{\mathrm{coast}}$", "Coast", "Final burn", "Swarm point"),
        ["", "[t]", "[km]", "[s]", "[km]", "[s]", "[s]", "[t]"], rows)


# The references by the environment each was flown in, the row label of the
# tables that list the references alone.
REFERENCE_LABELS = {"pmp_baseline": "Baseline", "pmp_vacuum": "No atmosphere",
                    "pmp_norot": "Non-rotating Earth"}

# The seeds every reference was swarmed from (750x1500), and where the swarm
# point and its refinement at the 2000 s coast bound of each are kept inside the
# results set: <root>/_pmp_seeds_750x1500/<case>/seed_<n>/{swarm,refined}/<case>.json
# (red-note review S1.3, option B, 2026-10-08). Seed 3 is the one reported.
REFERENCE_SEEDS = (1, 2, 3, 4, 42)
REPORTED_SEED = 3
SEEDS_DIR = "_pmp_seeds_750x1500"
ROOT = None                      # the results root, set by main()


def _seed_prop(case_name, seed, stage):
    """Propellant left [kg] at a reference's swarm point or refined extremal for
    one seed, or None when that archive is not in the results set."""
    import json

    root = Path(ROOT) if ROOT is not None else _data.DEFAULT_ROOT
    path = root / SEEDS_DIR / case_name / ("seed_%d" % seed) / stage / (case_name + ".json")
    if not path.exists():
        return None
    with open(path, encoding="utf-8") as fh:
        return json.load(fh).get("prop_remaining_kg")


def reference_seeds(cases):
    """Each reference's swarm point and refined extremal for every seed it was
    swarmed from, propellant left in tonnes; the reported seed is marked."""
    names = ["pmp_baseline", "pmp_vacuum", "pmp_norot"]
    rows = []
    for n in names:
        swarm = [tonnes(_seed_prop(n, sd, "swarm")) for sd in REFERENCE_SEEDS]
        refined = [tonnes(_seed_prop(n, sd, "refined")) for sd in REFERENCE_SEEDS]
        if all(c == DASH for c in swarm + refined):
            print("  skip %-20s no seed archives under %s" % ("reference_seeds", SEEDS_DIR))
            return None
        rows.append([REFERENCE_LABELS[n], "swarm point"] + swarm)
        rows.append(["", "refined"] + refined)
        if n != names[-1]:
            rows.append(None)
    heads = ["Seed %d%s" % (sd, "$^{*}$" if sd == REPORTED_SEED else "")
             for sd in REFERENCE_SEEDS]
    return tabular(
        "l l " + " ".join(["r"] * len(REFERENCE_SEEDS)),
        bold("Environment", "", *heads),
        ["", ""] + ["[t]"] * len(REFERENCE_SEEDS), rows)


def gt_results(cases):
    names = ["gt_baseline", "gt_apogee", "gt_norot", "gt_sea_level_engine"]
    if _missing(cases, names, "gt_results"):
        return None
    base = prop(cases["gt_baseline"])
    rows = []
    for n in names:
        k = cases[n]
        vs = None if n == "gt_baseline" else prop(k) - base
        rows.append([label_cell(n), tonnes(prop(k)), num(vs, 0, signed=True),
                     num(shortfall(k, cases), 0), orbit(k), seconds(arcs(k)[1])])
    return tabular(
        "l r r r r r",
        bold("Case", "Prop. left", "vs baseline", "Shortfall", "Orbit", "Coast"),
        ["", "[t]", "[kg]", "[kg]", "[km]", "[s]"], rows)


def _law_row(name, cases, label=None, steering=True):
    """Prop., shortfall, coast, [steering loss,] orbit."""
    k = cases[name]
    row = [label or label_cell(name), tonnes(prop(k)), num(shortfall(k, cases), 0),
           seconds(arcs(k)[1])]
    if steering:
        row.append(num(k.row["dv_steering"]))
    return row + [orbit(k)]


def peg_results(cases):
    names = ["gt_baseline", "peg_baseline", "peg_direct", "peg_vacuum"]
    if _missing(cases, names, "peg_results"):
        return None
    # No steering column (red-note review S3.3): the steering losses are
    # tab:loss_budget's.
    rows = [_law_row("gt_baseline", cases, steering=False), None] + [
        _law_row(n, cases, steering=False) for n in names[1:]]
    return tabular(
        "l r r r r",
        bold("Case", "Prop. left", "Shortfall", "Coast", "Orbit"),
        ["", "[t]", "[kg]", "[s]", "[km]"], rows)


def _waypoint_table(cases, orbit_name, waypoints, table):
    """The waypoint miss of each case aimed at the waypoint; the steering losses,
    split at the coast until 2026-10-08, are tab:loss_budget's and the text's
    (red-note review S3.3: the table ran off the page)."""
    names = [orbit_name, *waypoints, "pmp_baseline"]
    if _missing(cases, names, table):
        return None
    rows = []
    for n in names:
        k = cases[n]
        label = {"peg_baseline": "PEG, first burn to the orbit",
                 "pmp_baseline": "Reference"}.get(n, label_cell(n))
        prop_, short, coast, orbit_cell = _law_row(n, cases, steering=False)[1:]
        miss = k.waypoint_miss()
        dh, dv, dg = (DASH,) * 3 if miss is None else (
            num(miss[0], 0, signed=True), num(miss[1], 2, signed=True),
            num(miss[2], 3, signed=True))
        rows.append([label, prop_, short, coast, dh, dv, dg, orbit_cell])
    return tabular(
        "l r r r r r r r",
        bold("Case", "Prop. left", "Shortfall", "Coast")
        + [r"\multicolumn{3}{c}{\textbf{Waypoint miss}}"] + bold("Orbit"),
        ["", "[t]", "[kg]", "[s]", r"$\Delta h$ [m]", r"$\Delta v$ [m/s]",
         r"$\Delta\gamma$ [deg]", "[km]"],
        rows, rule_under_heads=r"\cmidrule(lr){5-7}")


def peg_waypoint(cases):
    # The two segmented flights are PEG with its first burn aimed at the
    # waypoint; the reference-tracking case left the matrix on 2026-10-07.
    return _waypoint_table(cases, "peg_baseline",
                           ["show_seg_fixed_alt", "show_seg_opt_alt"], "peg_waypoint")


def apollo_waypoint(cases):
    return _waypoint_table(cases, "show_apollo", ["show_ref_track_apollo"],
                           "apollo_waypoint")


def showcase_laws(cases):
    names = [n for n, _label in SHOWCASE]
    if _missing(cases, names, "showcase_laws"):
        return None
    rows = [_law_row(n, cases, label) + [hours(search(cases[n])[0])]
            for n, label in SHOWCASE]
    # The gravity turn above, as in peg_results, and the Apollo law below: both
    # fly the same architecture, and the section ranks the open-loop laws
    # between them (walkthrough S5-F2, 2026-10-07).
    if "gt_baseline" in cases:
        rows = [_law_row("gt_baseline", cases, "Gravity turn")
                + [hours(search(cases["gt_baseline"])[0])], None] + rows
    if "show_apollo" in cases:
        rows += [None, _law_row("show_apollo", cases, "Apollo")
                 + [hours(search(cases["show_apollo"])[0])]]
    return tabular(
        "l r r r r r r",
        bold("Guidance law", "Prop. left", "Shortfall", "Coast", "Steering loss",
             "Orbit", "Solve"),
        ["", "[t]", "[kg]", "[s]", "[m/s]", "[km]", "[h]"], rows)


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


def cross_plane_burns(case):
    """The cross-plane pseudo-acceleration integrated over the burns [m/s], on the
    launch azimuth (_data.Case.pseudo_forces_at_azimuth), or None without rotation.

    A diagnostic outside the budget (red-note review S6.6, option B): the term
    does no work in the plane, so it enters no loss, and over the coasts it only
    describes the ground frame turning beneath a Keplerian arc, so the coasts are
    left out. It is the speed the term would impart across the plane if nothing
    cancelled it during the burns.
    """
    forces = case.pseudo_forces_at_azimuth()
    if forces is None:
        return None
    cross = forces[2]
    total = 0.0
    for t0, t1 in _panels.burn_intervals(case):
        sel = (case.time >= t0) & (case.time <= t1)
        t, a = case.time[sel], cross[sel]
        keep = np.ones(t.size, dtype=bool)
        keep[1:] = np.diff(t) > 0.0
        if keep.sum() > 1:
            total += float(np.trapezoid(a[keep], t[keep]))
    return total


def loss_budget(cases):
    names = [n for n in _data.REPORTED_CASES if n in cases]
    rows = []
    for n in names:
        r = cases[n].row
        pressure = r["dv_pressure"] if r.get("pressure_applicable") else None
        rows.append([case_cell(n), num(r["dv_ideal"]), num(r["dv_gravity"]),
                     num(r["dv_drag"]), num(r["dv_steering"]), num(pressure),
                     num(r["dv_gain"]), num(r["residual"]),
                     num(cross_plane_burns(cases[n]))])
    # The cross-plane column is set apart from the closed budget it is not part of.
    return tabular(
        "l r r r r r r r @{\hspace{1.4em}} r",
        bold("Case", r"$\Delta V_{\mathrm{ideal}}$", "Gravity", "Drag", "Steering", "Pressure",
             "Gain", "Residual", "Cross-plane"),
        ["", "[m/s]", "[m/s]", "[m/s]", "[m/s]", "[m/s]", "[m/s]", "[m/s]", "[m/s]"], rows)


def _pct_range(values):
    """Tail improvements [%] as a range, to two significant figures of the
    largest; a swarm that stopped improving altogether reads below 1e-5 %."""
    values = [v for v in values if v is not None]
    if not values:
        return DASH
    hi = max(values)
    if hi < 1e-5:
        return r"\num{<e-5}"
    digits = max(2, 1 - int(math.floor(math.log10(hi))))
    lo = num(min(values), digits)
    return lo if len(values) == 1 or lo == num(hi, digits) else "%s--%s" % (
        lo, num(hi, digits))


def architecture_cost(cases):
    """One row per architecture, over every reported case it flew. Where its
    cases share one budget the wall clock is a range; where each ran its own
    (the extremals) the values are listed per case. A refined law case counts
    its swarm only; caption_values gives the refinement's cost."""
    rows = []
    for arch, label in COST_ROWS:
        members = [cases[n] for n in _data.REPORTED_CASES
                   if n in cases and cases[n].architecture == arch]
        if not members:
            continue
        costs = [swarm_search(k) for k in members]
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
    on an architecture that flies without it there [m/s]; \\refineFlights and
    \\refineWall the refinement of the refined law cases, flights and seconds,
    as a range over them (architecture_cost counts their swarms only)."""
    if _missing(cases, ["gt_apogee"], "caption_values"):
        return None
    circ = cases["gt_apogee"].row["circularisation_dv"]
    post = [drag_after_separation(k) for k in cases.values()
            if k.architecture != "apogee_check" and k.row.get("include_drag", True)]
    text = ("\\newcommand{\\circDvApogee}{%s}\n"
            "\\newcommand{\\dragAfterSeparation}{%s}\n"
            % ("%.1f" % circ, "%.1f" % max(post)))
    refined = [refinement(k) for k in cases.values() if refined_law(k)]
    if refined:
        def span(lo, hi):
            return lo if lo == hi else "%s--%s" % (lo, hi)
        flights = [n for _w, n in refined]
        walls = [w for w, _n in refined]
        text += ("\\newcommand{\\refineFlights}{%s}\n"
                 "\\newcommand{\\refineWall}{%s}\n"
                 % (span("%d" % min(flights), "%d" % max(flights)),
                    span("%.0f" % min(walls), "%.0f" % max(walls))))
    return text


# --- results validation (Section 6.8) -----------------------------------------
def _validation_inputs(cases):
    from Plots.results_figures import validation_falcon9 as val
    names = [n for n, _c in val.SIM_CASES]
    if _missing(cases, names, "validation"):
        return None
    flights = val.load_flights()
    if not flights:
        print("  skip validation           no flight data")
        return None
    rows = val.summary_rows(cases, flights)
    return val, names, flights, rows[:len(flights)], rows[len(flights):]


# The PMP reference flown to the Nov 2019 Starlink mission (280 km circular, 53 deg,
# 15.6 t payload; branch validation-starlink, never merged), refined with the coast
# bound at 2500 s, first pass only (user decision 2026-10-12). The coast grows with
# the bound, so the row's coast is the bound's, not an interior optimum.
STARLINK_RUN = _SRC / "Output" / "validation_starlink_20261012" / "polish" / "c2500"
STARLINK_LABEL = "Reference, Starlink mission"


def _starlink_case():
    """The Starlink-mission reference archive, or None if it is absent."""
    holders = sorted(p.parent for p in STARLINK_RUN.glob("*/pmp_baseline.npz"))
    if len(holders) != 1:
        print("  skip Starlink-mission row  %d archives under %s" % (len(holders), STARLINK_RUN))
        return None
    return _data.load("pmp_baseline", root=holders[0])


def _timeline_row(label, k, r, val):
    _first, coast, final = arcs(k)
    return [label, num(r["t_maxq"], 0), num(r["t_meco"], 0), num(r["t_ses1"], 0),
            num(val.fairing_time(k), 0), num(r["t_burn_end"], 0),
            num(coast, 0 if coast is not None and coast >= 10.0 else 1), num(final, 1)]


def _mission_row(label, k):
    config, vehicle = k.manifest["config"], k.manifest["vehicle"]
    _first, coast, _final = arcs(k)
    payload = vehicle["M_PAYLOAD"] / 1e3
    return [label,
            r"\qty{%.0f}{\km} circular" % (config["TARGET_ORBITAL_ALTITUDE"] / 1e3),
            num(config["TARGET_ORBIT_INCLINATION"], 1),
            num(payload, 0 if payload == int(payload) else 1),
            num(1 if coast is None else 2, 0), "Expended"]


def validation_timeline(cases):
    """Event times: the SpaceX user's guide sample LEO mission and the Nov 2019
    Starlink flight (planned), the Feb 2018 Paz flight (launch report, a direct
    insertion: dashes for coast and final burn) and the reference (user decision
    2026-10-12: cited values only). The cases' max-Q is the peak of their
    dynamic pressure."""
    inputs = _validation_inputs(cases)
    if inputs is None:
        return None
    val, names, _flights, _flight_rows, case_rows = inputs
    rows = []
    for label, g in [(r"Sample LEO mission~\cite{SpaceX_FalconUsersGuide_2025}",
                      val.GUIDE_LEO),
                     (r"Starlink, Nov.~2019~\cite{SpaceX_StarlinkPressKit_2019}",
                      val.STARLINK_2019),
                     (None, None),
                     (r"Paz, Feb.~2018~\cite{NSF_Paz_2018}", val.PAZ_2018)]:
        if label is None:
            rows.append(None)
            continue
        two_burns = "ses2" in g
        rows.append([label, num(g["maxq"], 0), num(g["meco"], 0), num(g["ses1"], 0),
                     num(g["fairing"], 0), num(g["seco1"], 0),
                     num(g["ses2"] - g["seco1"], 0) if two_burns else DASH,
                     num(g["seco2"] - g["ses2"], 0) if two_burns else DASH])
    rows.append(None)
    for n, r in zip(names, case_rows):
        rows.append(_timeline_row(label_cell(n), cases[n], r, val))
    starlink = _starlink_case()
    if starlink is not None:
        r = val.summary_rows({"pmp_baseline": starlink}, [])[0]
        rows.append(_timeline_row(STARLINK_LABEL, starlink, r, val))
    return tabular(
        "l r r r r r r r",
        bold("Source", "Max-Q", "MECO", "SES-1", "Fairing", "SECO-1", "Coast",
             "Final burn"),
        ["", "[s]", "[s]", "[s]", "[s]", "[s]", "[s]", "[s]"], rows)


# Mission parameters of the validation table's flight data, cited values only
# (user decision 2026-10-12); None where no source states the value. Columns:
# orbit, inclination [deg], payload [t], second-stage burns, first stage.
#   Guide: Falcon User's Guide (May 2025) Table 10-4 states no orbit, payload or
#     recovery; its timeline has two second-stage burns.
#   Starlink, 11 Nov 2019: deployment at 280 km (press kit), 53 deg (Spaceflight
#     Now), 60 x 260 kg = 15.6 t and the drone-ship landing (NASASpaceFlight).
#   Paz, 22 Feb 2018: ~514 km circular at 97.44 deg, 1,341 kg + 2 x 400 kg,
#     one burn (direct insertion), booster not recovered (NASASpaceFlight).
VALIDATION_MISSIONS = [
    (r"Sample LEO mission~\cite{SpaceX_FalconUsersGuide_2025}",
     None, None, None, 2, None),
    (r"Starlink, Nov.~2019~\cite{SpaceX_StarlinkPressKit_2019}",
     r"\qty{280}{\km} (deployment)", r"\num{53}", 15.6, 2, "Landed"),
    (r"Paz, Feb.~2018~\cite{NSF_Paz_2018}",
     r"\qty{514}{\km} circular", r"\num{97.44}", 2.1, 1, "Expended"),
]


def validation_missions(cases):
    """Mission parameters: the guide's sample LEO mission, the Nov 2019 Starlink
    flight and Paz (VALIDATION_MISSIONS), against the reference's target
    orbit, payload and arcs as its manifest records them."""
    inputs = _validation_inputs(cases)
    if inputs is None:
        return None
    _val, names, _flights, _flight_rows, _case_rows = inputs

    def cell(value, digits):
        return DASH if value is None else (value if isinstance(value, str)
                                           else num(value, digits))

    rows = [[label, cell(o, 0), cell(i, 0), cell(p, 1), cell(b, 0), cell(f, 0)]
            for label, o, i, p, b, f in VALIDATION_MISSIONS]
    rows.append(None)
    for n in names:
        rows.append(_mission_row(label_cell(n), cases[n]))
    starlink = _starlink_case()
    if starlink is not None:
        rows.append(_mission_row(STARLINK_LABEL, starlink))
    return tabular(
        "l l r r r l",
        bold("Source", "Orbit", "Inclination", "Payload", "Stage-2 burns",
             "First stage"),
        ["", "", "[deg]", "[t]", "", ""], rows)


def validation_states(cases):
    """Peak dynamic pressure and the state at MECO, flights against cases.
    Out of the thesis since 2026-10-12 (its values are mostly derived); built
    on request only."""
    inputs = _validation_inputs(cases)
    if inputs is None:
        return None
    val, names, flights, flight_rows, case_rows = inputs
    rows = []
    labels = [f.label for f in flights] + [None] + [label_cell(n) for n in names]
    for label, r in zip(labels, flight_rows + [None] + case_rows):
        if r is None:
            rows.append(None)
            continue
        rows.append([label, num(r["maxq"]), num(r["h_meco"]), num(r["v_meco"] / 1e3, 2),
                     num(r["g_meco"]), num(r["s_meco"], 0)])
    return tabular(
        "l r r r r r",
        bold("Source", "Max-Q", "$h$ at MECO", "$v$ at MECO", r"$\gamma$ at MECO",
             "$s$ at MECO"),
        ["", "[kPa]", "[km]", "[km/s]", "[deg]", "[km]"], rows)


# apollo_waypoint and segmented_results need the reference-tracking cases, out
# of the matrix since 2026-10-07; they are built on request only.
# --- Extended Abstract -------------------------------------------------------
# One table of every reported case replaces the abstract's Tables 4-8 (float
# review C-1, 2026-10-22): grouped by environment and, within the baseline,
# ordered by propellant left, as the ranking figure is. Written as ea_*.tex.
EA_GROUPS = [
    ("Baseline", ["pmp_baseline", "gt_apogee", "gt_baseline", "show_seg_opt_alt",
                  "show_seg_fixed_alt", "show_bilinear_tangent", "show_cpr",
                  "show_linear_tangent", "show_exp_shooting", "show_apollo",
                  "peg_direct", "peg_baseline"]),
    # The atmosphere is varied on the reference alone (review AT-4, 2026-10-25).
    ("No atmosphere", ["pmp_vacuum"]),
    ("Non-rotating Earth", ["pmp_norot", "gt_norot"]),
    ("Sea-level engine", ["gt_sea_level_engine"]),
]
# The shortest names the abstract's own text and its table of cases make
# unambiguous, so the table fits one column at 10 pt (review 2026-10-26): the
# apogee check is flown by the gravity turn alone, the segmented hand-off is
# optimized or fixed at 120 km, and the open-loop laws go by their adjective.
EA_LABELS = {"pmp_baseline": "Reference", "gt_apogee": "Apogee check",
             "gt_baseline": "Gravity turn", "show_seg_opt_alt": "Segmented",
             "show_seg_fixed_alt": "Seg., 120 km", "show_bilinear_tangent": "Bilinear",
             "show_cpr": "Pitch rate", "show_linear_tangent": "Linear",
             "show_exp_shooting": "Exponential", "peg_direct": "PEG, direct",
             "pmp_vacuum": "Reference", "pmp_norot": "Reference", "gt_norot": "Gravity turn",
             "gt_sea_level_engine": "Gravity turn"}


def ea_all_cases(cases):
    """Every reported case: propellant left, staging, coast and the two losses
    that account for the ranking (Extended Abstract, review C-1).

    One column at 10 pt (review 2026-10-26): the baseline block comes first with
    no heading, each other environment is an italic heading spanning the row, so
    its name costs no width, and the outer padding is dropped; the abstract sets
    \tabcolsep to 2 pt around it. No orbit column: the text gives the one case
    that misses the target."""
    names = [n for _g, members in EA_GROUPS for n in members]
    if _missing(cases, names, "ea_all_cases"):
        return None
    rows = []
    for g, (group, members) in enumerate(EA_GROUPS):
        if g:
            rows.append([r"\multicolumn{6}{@{}l}{\emph{%s}}" % group])
        for n in members:
            k = cases[n]
            r = k.row
            # No t_MECO column: the first stage burns to depletion at a fixed
            # flow in every case, so it is the same in every row (the text says so).
            rows.append([EA_LABELS.get(n, label_cell(n)), tonnes(prop(k)),
                         num(altitude_at(k, k.t_meco)), seconds(arcs(k)[1]),
                         num(r["dv_gravity"], 0), num(r["dv_steering"], 0)])
    # One line of heads above the units; the losses take the symbols of the
    # abstract's loss equation.
    return tabular(
        "@{}l r r r r r@{}",
        bold("Case", "Prop.", r"$h_{\mathrm{MECO}}$", "Coast",
             r"$\Delta V_{\mathrm{grav}}$", r"$\Delta V_{\mathrm{steer}}$"),
        ["", "[t]", "[km]", "[s]", "[m/s]", "[m/s]"], rows)


EA_COST_ROWS = [
    ("apogee_check", "Apogee check (grid)"),
    ("direct", "Direct insertion (grid)"),
    ("pso_coast", "Coast parameter (swarm)"),
    ("segmented", "Segmented (swarm)"),
    ("indirect_pmp", "Reference (swarm, refined)"),
]


def ea_architecture_cost(cases):
    """Search cost per case of each architecture: flights and wall clock, as a
    range over its cases (Extended Abstract, review AA-1, 2026-10-24). The thesis's
    tail-improvement column is left to the text, and the labels are shortened
    to fit one column. A refined law case counts its swarm only, as in
    architecture_cost; the text gives the refinement's cost."""
    rows = []
    for arch, label in EA_COST_ROWS:
        members = [cases[n] for n in _data.REPORTED_CASES
                   if n in cases and cases[n].architecture == arch]
        if not members:
            continue
        costs = [swarm_search(k) for k in members]
        walls = [w for w, _e in costs if w is not None]
        evals = sorted(set(e for _w, e in costs if e))
        evals_cell = DASH if not evals else (
            count(evals[0]) if len(evals) == 1
            else "%s--%s" % (count(evals[0]), count(evals[-1])))
        lo, hi = hours(min(walls)), hours(max(walls))
        rows.append([label, evals_cell, lo if lo == hi else "%s--%s" % (lo, hi)])
    return tabular(
        "@{}l r r@{}",
        bold("Architecture", "Flights", "Wall clock"),
        ["", "[-]", "[h]"], rows)


def ea_validation_timeline(cases):
    """The thesis's event-time table for the abstract's validation subsection
    (Extended Abstract, review AA-2, 2026-10-24), without the Paz row and the
    space that follows it, which the abstract does not discuss, and without the
    500 km reference's row (user, 2026-10-24)."""
    rows = validation_timeline(cases).split("\n")
    paz = [i for i, r in enumerate(rows) if r.lstrip().startswith("Paz,")]
    assert len(paz) == 1, "Paz row not found exactly once"
    i = paz[0]
    drop = {i, i + 1} if rows[i + 1].strip() == r"\addlinespace" else {i}
    rows = [r for k, r in enumerate(rows) if k not in drop]
    # Only the reference flown to the Starlink mission is kept (user, 2026-10-24):
    # its coast bound, 2500 s, lies above the flight's 2161 s coast, whereas the
    # 500 km reference's 2000 s bound lies below it.
    ref = [k for k, r in enumerate(rows) if r.lstrip().startswith("Reference (indirect PMP)")]
    assert len(ref) == 1, "500 km reference row not found exactly once"
    del rows[ref[0]]
    # Every column is in seconds, which the caption states once, so the units
    # row goes (user, 2026-10-24).
    h = next(k for k, r in enumerate(rows) if r.lstrip().startswith(r"\textbf{Source}"))
    assert set(c.strip() for c in rows[h + 1].rstrip().rstrip("\\").split("&")) == {"", r"[\unit{\s}]"}
    del rows[h + 1]
    # Transposed to fit one column of the abstract (review AK-1, 2026-10-24):
    # events down the side, the three sources across under short names that
    # the caption explains, with the citations moved there.
    cells = [[c.strip() for c in r.strip().rstrip("\\").strip().split("&")]
             for r in rows if "&" in r]
    events = [c[len(r"\textbf{"):-1] for c in cells[0][1:]]
    sources = cells[1:]
    assert [s[0].split("~")[0].split(",")[0] for s in sources] == ["Sample LEO mission", "Starlink", "Reference"]
    lines = [r"\begin{tabular}{@{}l r r r@{}}", r"  \toprule",
             r"  \textbf{Event} & \textbf{Guide} & \textbf{Starlink} & \textbf{Reference} \\",
             r"  \midrule"]
    lines += ["  %s & %s \\\\" % (e, " & ".join(s[i + 1] for s in sources))
              for i, e in enumerate(events)]
    lines += [r"  \bottomrule", r"\end{tabular}"]
    return "\n".join(lines)


EA_TABLES = [ea_all_cases, ea_architecture_cost, ea_validation_timeline]

TABLES = [
    reference_results, reference_seeds, gt_results, peg_results, peg_waypoint,
    showcase_laws, full_results, loss_budget, architecture_cost, caption_values,
    validation_missions, validation_timeline,
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
    parser.add_argument("--abstract", action="store_true",
                        help="write the Extended Abstract tables (ea_*.tex) instead")
    args = parser.parse_args()
    global ROOT
    ROOT = args.root

    cases = _data.load_many(_data.REPORTED_CASES, root=args.root)
    _data.check_one_rotation_model(cases)
    print("=" * 70)
    print("CHAPTER 6 TABLES -- %d of %d cases available"
          % (len(cases), len(_data.REPORTED_CASES)))
    print("output: %s" % OUT_DIR)
    print("=" * 70)

    written, skipped = 0, 0
    for table in (EA_TABLES if args.abstract else TABLES):
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
