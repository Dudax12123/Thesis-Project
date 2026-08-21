"""Sections 6.3 and 6.4 figures -- the closed-loop law and the reference.

Outputs
-------
results_peg_vs_gt.png             F6.6
results_direct_contrast.png       F6.7
results_peg_environment.png       F6.8
results_reference_trajectory.png  F6.9
"""

import matplotlib.pyplot as plt

from . import _data
from . import _style as st


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def peg_vs_gt(cases):
    """F6.6 -- the closed-loop baseline against the passive one.

    Panel (b) is the point of the pair. The gravity turn commands alpha = 0
    after the kick, so its trace is flat by construction and the difference
    between the two curves is the whole of what the closed-loop law is doing.
    """
    missing = _data.missing_from(cases, "gt_baseline", "peg_baseline")
    if missing:
        return _skip("F6.6 peg vs gravity turn", missing)
    gt, peg = cases["gt_baseline"], cases["peg_baseline"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, colour in ((gt, st.BASELINE), (peg, st.VARIANT)):
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax_a.plot(s_km, alt_km, color=colour, label=st.law_label(case.law))
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    for case, colour in ((gt, st.BASELINE), (peg, st.VARIANT)):
        t, al = st.thin(case.time, case.alpha_deg)
        ax_b.plot(t, al, color=colour, label=st.law_label(case.law))
    ax_b.axhline(0.0, color=st.FAINT, linewidth=0.8)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Angle of attack $\alpha$ [deg]")
    st.add_events(ax_b, peg, coast=False)
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b)

    fig.tight_layout()
    return st.save(fig, "results_peg_vs_gt.png")


def direct_contrast(cases):
    """F6.7 -- both laws under both swarm architectures.

    The load-bearing figure of the architecture argument: the direct
    single-burn insertion closes for the law that enforces terminal constraints
    explicitly and cannot for the passive one, whatever budget it is given. Two
    laws times two architectures is the smallest comparison that separates the
    law from the architecture as the cause.
    """
    names = ("gt_baseline", "gt_direct", "peg_baseline", "peg_direct")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("F6.7 direct contrast", missing)

    entries = [
        (cases["gt_baseline"], st.BASELINE, "-", "Gravity turn, PSO coast"),
        (cases["gt_direct"], st.BASELINE, "--", "Gravity turn, direct"),
        (cases["peg_baseline"], st.VARIANT, "-", "PEG, PSO coast"),
        (cases["peg_direct"], st.VARIANT, "--", "PEG, direct"),
    ]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, colour, style, label in entries:
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax_a.plot(s_km, alt_km, color=colour, linestyle=style, label=label)
        if not case.reached_orbit:
            peri = case.row.get("periapsis_km")
            if peri is not None:
                ax_a.annotate("periapsis %.0f km" % peri,
                              xy=(case.downrange_km[-1], case.alt_km[-1]),
                              xytext=(-4, -12), textcoords="offset points",
                              fontsize=7, color=colour, ha="right")
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    for case, colour, style, label in entries:
        t, al = st.thin(case.time, case.alpha_deg)
        ax_b.plot(t, al, color=colour, linestyle=style, label=label)
    ax_b.axhline(0.0, color=st.FAINT, linewidth=0.8)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Angle of attack $\alpha$ [deg]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_direct_contrast.png")


def peg_environment(cases):
    """F6.8 -- the environment removed in two single-factor steps.

    Read left to right as a ladder: the vacuum run differs from the baseline
    only in the atmosphere, and the third differs from the vacuum run only in
    the rotation. Neither is read directly against the baseline, which would be
    a two-factor comparison.
    """
    names = ("peg_baseline", "peg_vacuum", "peg_vacuum_norot")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("F6.8 peg environment", missing)

    labels = ("Baseline", "No atmosphere", "No atmosphere, no rotation")
    colours = (st.BASELINE, st.VARIANT2, st.VARIANT)

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for name, colour, label in zip(names, colours, labels):
        case = cases[name]
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax_a.plot(s_km, alt_km, color=colour, label=label)
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    for name, colour, label in zip(names, colours, labels):
        case = cases[name]
        t, prop = st.thin(case.time, case.prop_kg)
        ax_b.plot(t, prop / 1e3, color=colour, label=label)
        remaining = case.row.get("prop_remaining_kg")
        if remaining is not None:
            ax_b.annotate("%.1f t" % (remaining / 1e3),
                          xy=(t[-1], prop[-1] / 1e3), xytext=(3, 0),
                          textcoords="offset points", fontsize=7, color=colour)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Propellant remaining [t]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_peg_environment.png")


def reference_trajectory(cases):
    """F6.9 -- the indirect optimum, with the flyable baselines behind it.

    The two flyable laws are drawn faint rather than omitted so the yardstick is
    seen against what it is a yardstick for. Panel (b) shows the pitch, because
    the question Chapter 4 raises about this trajectory is whether the optimal
    steering carries the near-linear-tangent signature the analytical treatment
    predicts.
    """
    missing = _data.missing_from(cases, "pmp_baseline")
    if missing:
        return _skip("F6.9 reference trajectory", missing)

    primary = [(cases["pmp_baseline"], st.REFERENCE, "-", "Indirect PMP")]
    if "pmp_vacuum" in cases:
        primary.append((cases["pmp_vacuum"], st.REFERENCE, "--",
                        "Indirect PMP, no atmosphere"))
    context = [(cases[n], st.FAINT, "-", st.law_label(cases[n].law))
               for n in ("gt_baseline", "peg_baseline") if n in cases]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for case, colour, style, label in context + primary:
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax_a.plot(s_km, alt_km, color=colour, linestyle=style, label=label,
                  linewidth=1.0 if colour == st.FAINT else 1.3)
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    for case, colour, style, label in context + primary:
        t, th = st.thin(case.time, case.theta_deg)
        ax_b.plot(t, th, color=colour, linestyle=style, label=label,
                  linewidth=1.0 if colour == st.FAINT else 1.3)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Pitch $\theta$ [deg]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_reference_trajectory.png")


FIGURES = [peg_vs_gt, direct_contrast, peg_environment, reference_trajectory]
