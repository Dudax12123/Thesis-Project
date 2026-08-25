"""Overlay any N archived runs, and say what actually differs between them.

The Chapter 6 figures cannot do this. Every one of them names its cases --
``sec62_gravity_turn.py`` asks for "gt_baseline" and "gt_vacuum" by name, and
there are ten more like it -- because each is a fixed comparison the chapter
makes. That is right for the thesis and useless for the question "how does the
run I just flew compare with the one from last week", which is the question a
working simulator is asked most often.

So this is the generic half: pick archives by name, get the standard overlays,
and get a table of the configuration keys on which the runs disagree. The table
is the part that matters. Two trajectories that differ are easy to draw and hard
to explain, and the explanation is almost always a setting somebody changed and
did not write down -- which is exactly what the manifest was added to record.

The per-figure functions in sec62/63/65/67 are untouched and keep working.
"""

import csv
from datetime import datetime
from pathlib import Path

import numpy as np

from . import store

def _style():
    from Plots.results_figures import _style as st
    return st


# The manifest sections compared, in the order they are reported. "constants"
# is included because a changed R_EARTH would silently invalidate every
# comparison drawn above it.
DIFF_SECTIONS = ("config", "vehicle", "constants")

# The scalar results shown side by side under the configuration diff. Kept short
# on purpose: this is the "did they end up in the same place" summary, not the
# results table, which is the .json row and can be read in full from there.
RESULT_KEYS = [
    'architecture', 'guidance_mode', 'crashed', 'J_prime',
    'insertion_alt_km', 'insertion_v_ms', 'insertion_fpa_deg',
    'eccentricity', 'periapsis_km', 'apoapsis_km', 'prop_remaining_kg',
    'dv_ideal', 'dv_gravity', 'dv_drag', 'dv_steering', 'dv_pressure',
    'dv_losses', 'dv_gain', 'dv_achieved', 'residual',
    't_meco', 't_seco', 'n_evaluations', 'wall_clock_s',
]


def load_all(names, root=None, sim_params=None):
    """Resolve and load every named archive, in the order given."""
    cases = []
    for name in names:
        cases.append(store.load_run(name, root=root, sim_params=sim_params))
    return cases


def default_labels(cases):
    """Short legend labels: the law, disambiguated by architecture then by id."""
    st = _style()
    laws = [st.law_label(case.law) or case.name for case in cases]
    if len(set(laws)) == len(laws):
        return laws
    pairs = ["%s / %s" % (law, st.arch_label(case.architecture))
             for law, case in zip(laws, cases)]
    if len(set(pairs)) == len(pairs):
        return pairs
    return [case.name for case in cases]


def _colours(n):
    st = _style()
    cycle = [st.BASELINE] + list(st.VARIANT_CYCLE)
    return [cycle[i % len(cycle)] for i in range(n)]


# =========================================================================
#  The overlays
# =========================================================================

def fig_trajectory(cases, labels, colours):
    """Where the runs went: in space, and in time with each one's arc structure."""
    import matplotlib.pyplot as plt
    st = _style()

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, label, colour in zip(cases, labels, colours):
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        tag = label if case.reached_orbit else label + " (suborbital)"
        ax_a.plot(s_km, alt_km, color=colour, label=tag)
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    # Coast arcs are shaded per case in that case's colour rather than marked
    # once: which architecture put a coast where is usually the whole difference.
    for case, label, colour in zip(cases, labels, colours):
        t, alt_km = st.thin(case.time, case.alt_km)
        ax_b.plot(t, alt_km, color=colour, label=label)
        for t0, t1 in case.coast_intervals():
            ax_b.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Altitude [km]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "compare_trajectory.png")


def fig_angles(cases, labels, colours):
    """The three angles and the speed, which between them are the steering story."""
    import matplotlib.pyplot as plt
    st = _style()

    fig, axes = plt.subplots(2, 2, figsize=st.WIDE_4)
    (ax_g, ax_t), (ax_a, ax_v) = axes
    panels = (
        (ax_g, lambda c: c.gamma_deg, "Flight-path angle γ [deg]", "a"),
        (ax_t, lambda c: c.theta_deg, "Pitch θ [deg]", "b"),
        (ax_a, lambda c: c.alpha_deg, "Angle of attack α [deg]", "c"),
        (ax_v, lambda c: c.v / 1e3, "Speed [km/s]", "d"),
    )
    for ax, channel, ylabel, tag in panels:
        for case, label, colour in zip(cases, labels, colours):
            t, y = st.thin(case.time, channel(case))
            ax.plot(t, y, color=colour, label=label)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(ylabel)
        st.panel_tag(ax, tag)
        st.tidy(ax, legend=(tag == "a"))
    fig.tight_layout()
    return st.save(fig, "compare_angles.png")


def fig_losses(cases, labels, colours):
    """Where the delta-v went: accumulated along the arc, and as a total budget."""
    import matplotlib.pyplot as plt
    st = _style()

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, label, colour in zip(cases, labels, colours):
        hist = case.loss_histories()
        t = case.time[:case.cutoff_index()]
        t, grav, drag = st.thin(t, hist["gravity"], hist["drag"])
        ax_a.plot(t, grav, color=colour, label="Gravity (%s)" % label)
        ax_a.plot(t, drag, color=colour, linestyle="--", label="Drag (%s)" % label)
    ax_a.set_xlabel("Time [s]")
    ax_a.set_ylabel("Cumulative loss [m/s]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="upper left")

    # Stacked budget bars. A component the run does not define -- the pressure
    # deficit under a constant-thrust engine model -- is absent from the row and
    # is left out of the stack rather than drawn as a zero, which would read as
    # "measured, and negligible".
    components = ["gravity", "drag", "steering", "pressure"]
    x = np.arange(len(cases), dtype=float)
    bottom = np.zeros(len(cases))
    for comp in components:
        values = np.array([float(case.budget().get("dv_" + comp) or 0.0)
                           for case in cases])
        if not np.any(values):
            continue
        ax_b.bar(x, values, bottom=bottom, width=0.6,
                 color=st.LOSS_COLORS[comp], label=comp.capitalize())
        bottom += values
    ax_b.set_xticks(x)
    ax_b.set_xticklabels(labels, rotation=20, ha="right")
    ax_b.set_ylabel("Loss [m/s]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b)

    fig.tight_layout()
    return st.save(fig, "compare_losses.png")


def fig_propellant(cases, labels, colours):
    """What each run spent, over time and at insertion."""
    import matplotlib.pyplot as plt
    st = _style()

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, label, colour in zip(cases, labels, colours):
        t, prop = st.thin(case.time, case.prop_kg)
        ax_a.plot(t, prop / 1e3, color=colour, label=label)
    ax_a.set_xlabel("Time [s]")
    ax_a.set_ylabel("Propellant remaining [t]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    # Unspent propellant on a vehicle that never arrived is not a saving, so a
    # suborbital run is drawn hatched instead of being ranked alongside.
    x = np.arange(len(cases), dtype=float)
    values = [float(case.row.get("prop_remaining_kg") or 0.0) for case in cases]
    for xi, value, case, colour in zip(x, values, cases, colours):
        ax_b.bar([xi], [value], width=0.6, color=colour,
                 hatch=None if case.reached_orbit else "//",
                 edgecolor=st.INK, linewidth=0.4)
        if not case.reached_orbit:
            ax_b.annotate("did not reach orbit", xy=(xi, value),
                          xytext=(0, 4), textcoords="offset points",
                          ha="center", fontsize=6.5, color=st.GREY)
    ax_b.set_xticks(x)
    ax_b.set_xticklabels(labels, rotation=20, ha="right")
    ax_b.set_ylabel("Stage-2 propellant remaining [kg]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "compare_propellant.png")


def fig_convergence(cases, labels, colours):
    """Whether each swarm converged or merely ran out of generations.

    Runs with no swarm -- apogee_check -- are omitted rather than drawn flat: a
    horizontal line at the final objective would look like instant convergence.
    """
    import matplotlib.pyplot as plt
    st = _style()

    entries = [(case, label, colour)
               for case, label, colour in zip(cases, labels, colours)
               if case.pso_history is not None]
    if not entries:
        return None

    fig, ax = plt.subplots(figsize=st.WIDE_1)
    for case, label, colour in entries:
        gens, gbest = case.pso_history
        ax.plot(gens, gbest, color=colour, label=label)
    ax.set_xlabel("Generation")
    ax.set_ylabel("Best objective J'")
    if all(np.all(case.pso_history[1] > 0) for case, _l, _c in entries):
        ax.set_yscale("log")
    st.tidy(ax)
    fig.tight_layout()
    return st.save(fig, "compare_convergence.png")


FIGURES = [fig_trajectory, fig_angles, fig_losses, fig_propellant,
           fig_convergence]


# =========================================================================
#  The manifest diff -- why the curves differ
# =========================================================================

ABSENT = "(absent)"


def manifest_diff(cases):
    """``(compared, rows, unrecorded)`` -- the keys the runs disagree on.

    Only runs that carry a manifest are compared. A run without one is named in
    ``unrecorded`` rather than being given a column of ABSENT for every key: it
    did not disagree about 129 settings, it simply predates the manifest, and a
    table saying otherwise would be worse than no table.
    """
    compared = [case for case in cases if case.manifest]
    unrecorded = [case.name for case in cases if not case.manifest]

    rows = []
    for section in DIFF_SECTIONS:
        blocks = [case.manifest.get(section) or {} for case in compared]
        if not blocks:
            continue
        keys = set()
        for block in blocks:
            keys.update(block)
        for key in sorted(keys):
            values = [block.get(key, ABSENT) for block in blocks]
            if all(value == values[0] for value in values[1:]):
                continue
            rows.append((section, key, values))
    return compared, rows, unrecorded


def result_table(cases):
    """``[(key, [value per case]), ...]`` for the scalars worth seeing side by side."""
    rows = []
    for key in RESULT_KEYS:
        values = [case.row.get(key) for case in cases]
        if all(value is None for value in values):
            continue
        rows.append((key, values))
    return rows


def _fmt(value):
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "True" if value else "False"
    if isinstance(value, float):
        return "%.6g" % value
    text = str(value)
    return text if len(text) <= 60 else text[:57] + "..."


def _md_table(header, rows):
    """A GitHub-flavoured markdown table from a header and rows of cells."""
    lines = ["| " + " | ".join(header) + " |",
             "|" + "|".join(["---"] * len(header)) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(_fmt(cell) for cell in row) + " |")
    return lines


def write_diff(cases, labels, out_dir):
    """Write compare_manifest.md and .csv, and return the two paths."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    compared, rows, unrecorded = manifest_diff(cases)
    compared_labels = [labels[cases.index(case)] for case in compared]

    lines = ["# Run comparison", ""]
    lines.append("Archives compared:")
    lines.append("")
    for case, label in zip(cases, labels):
        man = case.manifest or {}
        git = (man.get('git') or {}).get('describe')
        lines.append("- **%s** -- `%s`%s%s" % (
            label, case.name,
            "  (%s)" % man['created_local'] if man.get('created_local') else "",
            "  git %s" % git if git else ""))
    lines.append("")

    lines.append("## Configuration differences")
    lines.append("")
    if len(compared) < 2:
        lines.append("Fewer than two of these archives carry a manifest, so "
                     "there is nothing to diff. Archives written before "
                     "manifests existed cannot say what they were flown under.")
    elif not rows:
        lines.append("None. Every recorded setting, vehicle constant and "
                     "physical constant agrees across these runs, so any "
                     "difference in the trajectories comes from the solver "
                     "itself -- a different PSO seed, or a swarm that landed "
                     "somewhere else.")
    else:
        lines.extend(_md_table(["Section", "Setting"] + compared_labels,
                               [[section, key] + values
                                for section, key, values in rows]))
    if unrecorded:
        lines.append("")
        lines.append("Not compared (no manifest recorded): %s"
                     % ", ".join("`%s`" % n for n in unrecorded))
    lines.append("")

    lines.append("## Results")
    lines.append("")
    results = result_table(cases)
    lines.extend(_md_table(["Quantity"] + list(labels),
                           [[key] + values for key, values in results]))
    lines.append("")

    md_path = out_dir / "compare_manifest.md"
    with open(md_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))

    csv_path = out_dir / "compare_manifest.csv"
    with open(csv_path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["kind", "section", "key"] + list(labels))
        for section, key, values in rows:
            padded = [values[compared.index(case)] if case in compared else ABSENT
                      for case in cases]
            writer.writerow(["config_diff", section, key] + padded)
        for key, values in results:
            writer.writerow(["result", "", key] + values)

    return md_path, csv_path, rows, unrecorded, compared


# =========================================================================
#  Driver
# =========================================================================

def _out_dir_for(labels, base=None, sim_params=None):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if len(labels) == 2:
        tail = "%s_vs_%s" % (_slug(labels[0]), _slug(labels[1]))
    else:
        tail = "%d_runs" % len(labels)
    base = (Path(base) if base is not None
            else store.plots_root(sim_params) / "comparisons")
    return base / ("%s_%s" % (stamp, tail))


def _slug(text):
    keep = "".join(ch if (ch.isalnum() or ch in "-_") else "_" for ch in str(text))
    return keep.strip("_")[:32] or "run"


def compare(names, root=None, out_dir=None, labels=None, cards=False,
            sim_params=None, verbose=True):
    """Overlay N archives and diff their manifests. Returns the output directory.

    *names* accept anything ``store.resolve`` accepts -- a run id, a unique
    prefix of one, ``<dir>::<stem>`` or a path -- so a run flown five minutes ago
    and a results-matrix case from the batch can be named in the same call.
    """
    if len(names) < 2:
        raise SystemExit("compare needs at least two archives; got %d" % len(names))

    cases = load_all(names, root=root, sim_params=sim_params)
    labels = list(labels) if labels else default_labels(cases)
    if len(labels) != len(cases):
        raise SystemExit("%d labels for %d archives" % (len(labels), len(cases)))

    st = _style()
    out_dir = (Path(out_dir) if out_dir is not None
               else _out_dir_for(labels, sim_params=sim_params))
    out_dir.mkdir(parents=True, exist_ok=True)
    st.OUT_DIR = str(out_dir)
    st.use_thesis_style()

    colours = _colours(len(cases))
    written = []
    for figure in FIGURES:
        path = figure(cases, labels, colours)
        if path is None:
            if verbose:
                print("  [skip] %s -- nothing to draw" % figure.__name__)
        else:
            written.append(path)

    if cards:
        from Plots.results_figures import run_card
        for case, label in zip(cases, labels):
            written.append(run_card.draw(case, "card_%s.png" % _slug(case.name),
                                         title="%s  |  %s" % (label, case.name)))

    md_path, csv_path, rows, unrecorded, compared = write_diff(cases, labels, out_dir)
    written += [md_path, csv_path]

    if verbose:
        print("  wrote %s" % md_path)
        print("  wrote %s" % csv_path)
        print("")
        if len(compared) < 2:
            print("  configuration diff: not possible -- fewer than two of these "
                  "archives carry a manifest.")
        elif not rows:
            print("  configuration diff: nothing differs. Same settings, same "
                  "vehicle, same constants.")
        else:
            print("  configuration diff -- %d setting(s) differ:" % len(rows))
            for section, key, values in rows:
                print("    %-9s %-34s %s"
                      % (section, key, "  |  ".join(_fmt(v) for v in values)))
        if unrecorded:
            print("  no manifest recorded for: %s" % ", ".join(unrecorded))
        print("")
        print("  %d file(s) in %s" % (len(written), out_dir))
    return out_dir
