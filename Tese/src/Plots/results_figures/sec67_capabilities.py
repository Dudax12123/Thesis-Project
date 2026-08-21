"""Section 6.7 figures -- breadth at reduced depth, and the cost of the search.

Outputs
-------
results_showcase_laws.png      F6.14
results_segmented_handoff.png  F6.15
results_solve_cost.png         F6.16
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
from . import _style as st

SHOWCASE = ["show_cpr", "show_linear_tangent", "show_bilinear_tangent",
            "show_apollo", "show_peg", "show_exp_shooting"]

# One representative case per architecture for the convergence panel. The
# apogee_check case is deliberately absent: it runs no swarm and has no
# convergence history to show, which is itself part of the cost comparison.
COST_CASES = ["gt_baseline", "peg_direct", "pmp_baseline", "show_seg_opt_alt",
              "gt_apogee"]


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def showcase_laws(cases):
    """F6.14 -- the six remaining laws, at the baseline, in one figure.

    Panel (b) is small multiples rather than an overlay because alpha is what
    distinguishes these laws from one another, and six alpha traces on shared
    axes would be a solid block. Each panel keeps the same axis limits so the
    shapes remain comparable.
    """
    present = [n for n in SHOWCASE if n in cases]
    if not present:
        return _skip("F6.14 showcase laws", SHOWCASE)

    # Wider than the standard text-width figure: the trajectory legend sits
    # outside the axes, and the grid keeps its own width regardless.
    fig = plt.figure(figsize=(7.4, 5.6))
    grid = fig.add_gridspec(3, 3, height_ratios=[1.45, 1.0, 1.0], hspace=0.62,
                            wspace=0.35, right=0.80)
    ax_traj = fig.add_subplot(grid[0, :])

    for name in ("gt_baseline", "peg_baseline"):
        if name in cases:
            case = cases[name]
            s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
            ax_traj.plot(s_km, alt_km, color=st.FAINT, linewidth=1.0,
                         label=st.law_label(case.law), zorder=1)

    for i, name in enumerate(present):
        case = cases[name]
        colour = st.VARIANT_CYCLE[i % len(st.VARIANT_CYCLE)]
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax_traj.plot(s_km, alt_km, color=colour, label=st.law_label(case.law),
                     zorder=2)
    ax_traj.set_xlabel("Downrange [km]")
    ax_traj.set_ylabel("Altitude [km]")
    st.panel_tag(ax_traj, "a")
    st.tidy(ax_traj, legend=False)
    # Outside the axes: eight entries over a trajectory panel cover the curves
    # they are labelling whichever corner they are put in.
    ax_traj.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), ncol=1,
                   fontsize=6.8)

    # Shared limits, so the six small panels compare rather than merely coexist.
    # Taken from a percentile rather than the extremes: one law transients to
    # about -140 deg for a few seconds, and letting that set the range flattens
    # the other five into a band a few pixels high. The clip is drawn as a
    # marker on the panels it affects rather than hidden.
    stacked = np.concatenate([np.asarray(cases[n].alpha_deg) for n in present])
    alpha_lo = float(np.nanpercentile(stacked, 0.5))
    alpha_hi = float(np.nanpercentile(stacked, 99.5))
    pad = 0.10 * max(alpha_hi - alpha_lo, 1.0)
    alpha_lo, alpha_hi = alpha_lo - pad, alpha_hi + pad
    t_hi = max(float(cases[n].time[-1]) for n in present)

    for i, name in enumerate(SHOWCASE):
        ax = fig.add_subplot(grid[1 + i // 3, i % 3])
        if name not in cases:
            ax.axis("off")
            continue
        case = cases[name]
        colour = st.VARIANT_CYCLE[present.index(name) % len(st.VARIANT_CYCLE)]
        t, al = st.thin(case.time, case.alpha_deg)
        ax.plot(t, al, color=colour, linewidth=1.0)
        ax.axhline(0.0, color=st.FAINT, linewidth=0.7)
        ax.set_xlim(0, t_hi)
        ax.set_ylim(alpha_lo, alpha_hi)
        # Say so when the shared range does not hold this law's full excursion,
        # rather than letting the curve run off the top or bottom silently.
        full_lo = float(np.nanmin(case.alpha_deg))
        full_hi = float(np.nanmax(case.alpha_deg))
        clipped = []
        if full_lo < alpha_lo:
            clipped.append("%.0f" % full_lo)
        if full_hi > alpha_hi:
            clipped.append("%.0f" % full_hi)
        if clipped:
            ax.annotate("peaks " + ", ".join(clipped) + r"$^\circ$",
                        xy=(0.97, 0.90), xycoords="axes fraction",
                        fontsize=6, color=st.GREY, ha="right")
        ax.set_title(st.law_label(case.law), fontsize=7.5, pad=3)
        ax.tick_params(labelsize=6.5)
        if i % 3 == 0:
            ax.set_ylabel(r"$\alpha$ [deg]", fontsize=7.5)
        if i // 3 == 1:
            ax.set_xlabel("Time [s]", fontsize=7.5)
        st.tidy(ax, legend=False)

    fig.text(0.055, 0.615, "(b)", fontsize=9, fontweight="bold", color=st.INK)
    return st.save(fig, "results_showcase_laws.png")


def segmented_handoff(cases):
    """F6.15 -- who chooses the hand-off altitude, and what it is worth.

    One law combination, twice: flown at the altitude Chapter 4's atmospheric
    and exoatmospheric division suggests, and flown with that altitude appended
    to the decision vector. The comparison is whether the optimiser agrees.
    """
    names = ("show_seg_fixed_alt", "show_seg_opt_alt")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("F6.15 segmented hand-off", missing)

    fixed, opt = cases["show_seg_fixed_alt"], cases["show_seg_opt_alt"]
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)

    for name in ("gt_baseline", "peg_baseline"):
        if name in cases:
            case = cases[name]
            s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
            ax_a.plot(s_km, alt_km, color=st.FAINT, linewidth=1.0,
                      label="%s alone" % st.law_label(case.law), zorder=1)

    for case, colour, label in ((fixed, st.BASELINE, "Hand-off fixed"),
                                (opt, st.VARIANT, "Hand-off optimised")):
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax_a.plot(s_km, alt_km, color=colour, label=label, zorder=2)
        schedule = case.segment_schedule or []
        for _law, alt_m in schedule[1:]:
            ax_a.axhline(alt_m / 1e3, color=colour, linestyle=":", linewidth=0.9)
            ax_a.annotate("%.0f km" % (alt_m / 1e3), xy=(1.0, alt_m / 1e3),
                          xycoords=("axes fraction", "data"), xytext=(-4, 3),
                          textcoords="offset points", fontsize=6.5,
                          color=colour, ha="right")
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    for case, colour, label in ((fixed, st.BASELINE, "Hand-off fixed"),
                                (opt, st.VARIANT, "Hand-off optimised")):
        t, al = st.thin(case.time, case.alpha_deg)
        ax_b.plot(t, al, color=colour, label=label)
    ax_b.axhline(0.0, color=st.FAINT, linewidth=0.8)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Angle of attack $\alpha$ [deg]")
    schedule = opt.segment_schedule or []
    if schedule:
        ax_b.annotate("Schedule: " + " -> ".join(st.law_label(l) for l, _a in schedule),
                      xy=(0.02, 0.94), xycoords="axes fraction", fontsize=6.8,
                      color=st.INK, va="top")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b)

    fig.tight_layout()
    return st.save(fig, "results_segmented_handoff.png")


def solve_cost(cases):
    """F6.16 -- what each architecture costs, and whether it converged.

    The convergence curves are the diagnostic tier of Section 6.1 gathered into
    one place. They answer only the question of whether a poor result came from
    the law or from a search that was stopped too early -- a failure mode this
    work has met, where an apparently incapable configuration proved merely
    under-converged. They rank nothing.
    """
    present = [n for n in COST_CASES if n in cases]
    if not present:
        return _skip("F6.16 solve cost", COST_CASES)

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)

    drawn = 0
    for i, name in enumerate(present):
        case = cases[name]
        history = case.pso_history
        if history is None:
            continue
        gens, gbest = history
        colour = st.VARIANT_CYCLE[i % len(st.VARIANT_CYCLE)]
        ax_a.plot(gens, gbest, color=colour, label=st.arch_label(case.architecture))
        drawn += 1
    if drawn:
        ax_a.set_yscale("log")
    ax_a.set_xlabel("Generation")
    ax_a.set_ylabel(r"Best objective $J'$")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    labels, evals, walls, colours = [], [], [], []
    for i, name in enumerate(present):
        case = cases[name]
        labels.append(st.arch_label(case.architecture))
        evals.append(case.row.get("n_evaluations") or 0)
        walls.append((case.row.get("wall_clock_s") or 0.0) / 60.0)
        colours.append(st.VARIANT_CYCLE[i % len(st.VARIANT_CYCLE)])

    y_pos = np.arange(len(labels))
    ax_b.barh(y_pos, walls, height=0.6, color=colours, edgecolor="white",
              linewidth=0.5)
    for i, (wall, n_eval) in enumerate(zip(walls, evals)):
        # Whole minutes are the right unit for a production solve and useless
        # for anything shorter, which is exactly what a reduced-budget check
        # produces -- so a sub-minute figure is reported in seconds.
        note = "%.0f min" % wall if wall >= 1.0 else "%.0f s" % (wall * 60.0)
        if n_eval:
            note += "  (%d evals)" % n_eval
        else:
            note += "  (no swarm)"
        ax_b.annotate(note, xy=(wall, i), xytext=(4, 0),
                      textcoords="offset points", fontsize=6.8, va="center",
                      color=st.INK)
    ax_b.set_yticks(y_pos)
    ax_b.set_yticklabels(labels)
    ax_b.invert_yaxis()
    ax_b.set_xlabel("Wall clock [min]")
    ax_b.set_xlim(0, max(walls + [1.0]) * 1.45)
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_solve_cost.png")


FIGURES = [showcase_laws, segmented_handoff, solve_cost]
