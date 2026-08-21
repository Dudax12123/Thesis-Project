"""Section 6.2 figures -- the gravity turn along its four secondary axes.

Five figures. The first is the reporting template of Section 6.1, shown in full
once here and cited by every later section rather than repeated; the other four
are each the baseline overlaid with the single case that differs from it.

Outputs
-------
results_gt_baseline_card.png   F6.1
results_gt_architecture.png    F6.2
results_gt_atmosphere.png      F6.3
results_gt_rotation.png        F6.4
results_gt_engine.png          F6.5
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _data
from . import _style as st


def _skip(name, missing):
    print("  [skip] %s -- missing %s" % (name, ", ".join(missing)))


def baseline_card(cases):
    """F6.1 -- the reporting template, in full, once.

    Four panels chosen so that the eleven independent quantities the per-run
    debugging suite spreads over twenty plots appear here in one place: the
    trajectory in space, the trajectory in time, all three angles together
    (theta = alpha + gamma, so two of the three are free), and the atmospheric
    severity that the drag and pressure losses are incurred against.
    """
    missing = _data.missing_from(cases, "gt_baseline")
    if missing:
        return _skip("F6.1 baseline card", missing)
    case = cases["gt_baseline"]

    fig, axes = plt.subplots(2, 2, figsize=st.WIDE_4)
    (ax_a, ax_b), (ax_c, ax_d) = axes

    # (a) the trajectory in space. The dashed line is the target orbit, not the
    # state reached: the curve continues past insertion along the terminal
    # ballistic arc, so marking the insertion state as a horizontal line would
    # suggest the trajectory stops there.
    s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
    ax_a.plot(s_km, alt_km, color=st.BASELINE)
    target = case.row.get("target_alt_km")
    if target is not None:
        ax_a.axhline(target, color=st.GREY, linestyle="--", linewidth=0.8,
                     label="Target orbit, %.0f km" % target)
    idx_seco = case.cutoff_index() - 1
    ax_a.plot(case.downrange_km[idx_seco], case.alt_km[idx_seco], "o",
              color=st.ACCENT, markersize=4.5, zorder=4,
              label="Insertion (%.0f km)" % case.alt_km[idx_seco])
    ax_a.set_xlabel("Downrange [km]")
    ax_a.set_ylabel("Altitude [km]")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    # (b) the trajectory in time, with the arc structure
    t, alt_km, v = st.thin(case.time, case.alt_km, case.v)
    st.shade_coast(ax_b, case, label="Coast")
    ax_b.plot(t, alt_km, color=st.BASELINE, label="Altitude")
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Altitude [km]", color=st.BASELINE)
    ax_bv = ax_b.twinx()
    ax_bv.plot(t, v / 1e3, color=st.ACCENT, linewidth=1.1)
    ax_bv.set_ylabel("Speed [km/s]", color=st.ACCENT)
    ax_bv.spines["top"].set_visible(False)
    st.add_events(ax_b, case)
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    # (c) all three angles -- theta is alpha + gamma, so only two are free
    t, gam, th, al = st.thin(case.time, case.gamma_deg, case.theta_deg,
                             case.alpha_deg)
    # Pitch is drawn wide and underneath: for the gravity turn alpha is zero
    # after the kick, so theta and gamma coincide by definition and a
    # same-weight trace would simply be hidden.
    ax_c.plot(t, th, color=st.BASELINE, linewidth=2.6, alpha=0.55,
              label=r"Pitch $\theta$")
    ax_c.plot(t, gam, color=st.GREEN, label=r"Flight path $\gamma$")
    ax_c.plot(t, al, color=st.ACCENT, label=r"Angle of attack $\alpha$")
    ax_c.axhline(0.0, color=st.FAINT, linewidth=0.8)
    ax_c.set_xlabel("Time [s]")
    ax_c.set_ylabel("Angle [deg]")
    st.add_events(ax_c, case, coast=False)
    st.panel_tag(ax_c, "c")
    st.tidy(ax_c)

    # (d) what the atmosphere does to the vehicle
    t, q, mach = st.thin(case.time, case.q, case.mach)
    in_atm = t <= (case.t_meco if case.t_meco else t[-1])
    ax_d.plot(t[in_atm], q[in_atm] / 1e3, color=st.BASELINE)
    ax_d.set_xlabel("Time [s]")
    ax_d.set_ylabel("Dynamic pressure [kPa]", color=st.BASELINE)
    ax_dm = ax_d.twinx()
    ax_dm.plot(t[in_atm], mach[in_atm], color=st.AMBER, linewidth=1.1)
    ax_dm.set_ylabel("Mach [-]", color=st.AMBER)
    ax_dm.spines["top"].set_visible(False)
    q_max = float(np.nanmax(q[in_atm])) if np.any(in_atm) else 0.0
    if q_max > 0:
        t_qmax = float(t[in_atm][int(np.nanargmax(q[in_atm]))])
        ax_d.annotate(r"max $q$ = %.1f kPa" % (q_max / 1e3),
                      xy=(t_qmax, q_max / 1e3), xytext=(6, -2),
                      textcoords="offset points", fontsize=7, color=st.INK)
    st.panel_tag(ax_d, "d")
    st.tidy(ax_d, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_gt_baseline_card.png")


def _overlay_trajectory(ax, entries):
    """Altitude against downrange for several cases, in the shared house style."""
    for case, colour, label, style in entries:
        s_km, alt_km = st.thin(case.downrange_km, case.alt_km)
        ax.plot(s_km, alt_km, color=colour, label=label, linestyle=style)
    ax.set_xlabel("Downrange [km]")
    ax.set_ylabel("Altitude [km]")


def architecture(cases):
    """F6.2 -- the same law under three optimization architectures.

    The direct run is expected to finish suborbital and is drawn, annotated
    with its periapsis, rather than suppressed: paired with ``peg_direct`` in
    Section 6.3 it is the chapter's argument for why coast arcs exist.
    """
    names = ("gt_baseline", "gt_apogee", "gt_direct")
    missing = _data.missing_from(cases, *names)
    if missing:
        return _skip("F6.2 architecture", missing)

    colours = (st.BASELINE, st.VARIANT2, st.VARIANT)
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)

    entries = []
    for name, colour in zip(names, colours):
        case = cases[name]
        label = st.arch_label(case.architecture)
        if not case.reached_orbit:
            label += " (suborbital)"
        entries.append((case, colour, label, "-"))
    _overlay_trajectory(ax_a, entries)

    failed = [c for c in (cases[n] for n in names) if not c.reached_orbit]
    for case in failed:
        peri = case.row.get("periapsis_km")
        if peri is not None:
            ax_a.annotate("periapsis %.0f km" % peri,
                          xy=(case.downrange_km[-1], case.alt_km[-1]),
                          xytext=(-4, -12), textcoords="offset points",
                          fontsize=7, color=st.VARIANT, ha="right")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    # (b) the same three in time, with each one's arc structure shaded. The
    # single continuous burn of the direct architecture reads here as the
    # absence of a coast, which is the whole of the explanation.
    for name, colour in zip(names, colours):
        case = cases[name]
        t, alt_km = st.thin(case.time, case.alt_km)
        ax_b.plot(t, alt_km, color=colour, label=st.arch_label(case.architecture))
        for t0, t1 in case.coast_intervals():
            ax_b.axvspan(t0, t1, color=colour, alpha=0.12, linewidth=0)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Altitude [km]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend=False)

    fig.tight_layout()
    return st.save(fig, "results_gt_architecture.png")


def atmosphere(cases):
    """F6.3 -- baseline against the drag-free run.

    The two runs differ on two counts and not one: ``INCLUDE_DRAG=False`` is the
    master no-atmosphere switch, so the vacuum case also drops the fairing and
    flies vacuum thrust and vacuum Isp. Panel (b) shows both consequences at
    once -- the drag integral vanishes, and the gravity loss changes because the
    trajectory it is flown along has changed.
    """
    missing = _data.missing_from(cases, "gt_baseline", "gt_vacuum")
    if missing:
        return _skip("F6.3 atmosphere", missing)
    base, vac = cases["gt_baseline"], cases["gt_vacuum"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    _overlay_trajectory(ax_a, [
        (base, st.BASELINE, "With atmosphere", "-"),
        (vac, st.VARIANT, "No atmosphere", "-"),
    ])
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    for case, colour, tag in ((base, st.BASELINE, "atm."), (vac, st.VARIANT, "vac.")):
        hist = case.loss_histories()
        t = case.time[:case.cutoff_index()]
        t, grav, drag = st.thin(t, hist["gravity"], hist["drag"])
        ax_b.plot(t, grav, color=colour, label="Gravity (%s)" % tag)
        ax_b.plot(t, drag, color=colour, linestyle="--",
                  label="Drag (%s)" % tag)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Cumulative loss [m/s]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b, legend_loc="upper left")

    fig.tight_layout()
    return st.save(fig, "results_gt_atmosphere.png")


def rotation(cases):
    """F6.4 -- baseline against the non-rotating Earth.

    Panel (b) is deliberately a null result for one of the two cases: the
    pseudo-force channels are recomputed under the same gate the equations of
    motion use, so the non-rotating run is identically zero rather than small.
    That is the evidence the switch did what it claims.
    """
    missing = _data.missing_from(cases, "gt_baseline", "gt_norot")
    if missing:
        return _skip("F6.4 rotation", missing)
    base, norot = cases["gt_baseline"], cases["gt_norot"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)
    _overlay_trajectory(ax_a, [
        (base, st.BASELINE, "Rotating Earth", "-"),
        (norot, st.VARIANT, "Non-rotating", "-"),
    ])
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a)

    lat = base.latitude_deg
    if lat is not None:
        ax_lat = ax_a.inset_axes([0.58, 0.14, 0.38, 0.30])
        t, lat_t = st.thin(base.time, lat)
        ax_lat.plot(t, lat_t, color=st.BASELINE, linewidth=1.0)
        ax_lat.set_title("Latitude [deg]", fontsize=6.5, pad=2)
        ax_lat.tick_params(labelsize=6)
        for side in ("top", "right"):
            ax_lat.spines[side].set_visible(False)

    for case, colour, tag in ((base, st.BASELINE, "rot."),
                              (norot, st.VARIANT, "non-rot.")):
        cor, cen = case.coriolis, case.centrifugal
        if cor is None or cen is None:
            continue
        t, cor, cen = st.thin(case.time, cor, cen)
        ax_b.plot(t, cor, color=colour, label="Coriolis (%s)" % tag)
        ax_b.plot(t, cen, color=colour, linestyle="--",
                  label="Centrifugal (%s)" % tag)
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel(r"Pseudo-force accel. [m/s$^2$]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b)

    fig.tight_layout()
    return st.save(fig, "results_gt_rotation.png")


def engine(cases):
    """F6.5 -- the pressure-dependent nozzle against a constant sea-level one.

    This is the one figure in which thrust and mass flow earn separate curves.
    Everywhere else mass flow is the thrust trace rescaled by a constant, since
    Isp is fixed per stage; under the pressure model Isp varies with altitude,
    so the two curves genuinely differ, and that difference is the case.
    """
    missing = _data.missing_from(cases, "gt_baseline", "gt_sea_level_engine")
    if missing:
        return _skip("F6.5 engine model", missing)
    base, sl = cases["gt_baseline"], cases["gt_sea_level_engine"]

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=st.WIDE_2)

    ax_mdot = ax_a.twinx()
    ax_mdot.set_ylabel("Mass flow [kg/s]  (dotted)")
    ax_mdot.spines["top"].set_visible(False)
    for case, colour in ((base, st.BASELINE), (sl, st.VARIANT)):
        label = "%s nozzle" % case.row.get("thrust_1_mode", "?")
        end = case.t_meco if case.t_meco else case.time[-1]
        sel = case.time <= end
        t, thr = st.thin(case.time[sel], case.thrust[sel])
        ax_a.plot(t, thr / 1e3, color=colour, label=label)
        # Mass flow from the mass trace itself, so it reflects the Isp actually
        # flown rather than a nominal value.
        mdot = -np.gradient(case.mass[sel], case.time[sel])
        t2, mdot = st.thin(case.time[sel], mdot)
        ax_mdot.plot(t2, mdot, color=colour, linestyle=":", linewidth=1.0)
    ax_a.set_xlabel("Time [s]")
    ax_a.set_ylabel("Stage-1 thrust [kN]  (solid)")
    st.panel_tag(ax_a, "a")
    st.tidy(ax_a, legend_loc="lower right")

    for case, colour in ((base, st.BASELINE), (sl, st.VARIANT)):
        t, m = st.thin(case.time, case.mass)
        ax_b.plot(t, m / 1e3, color=colour,
                  label="%s" % case.row.get("thrust_1_mode", "?"))
        if case.t_meco is not None:
            ax_b.axvline(case.t_meco, color=colour, linestyle=":", linewidth=0.9)
            ax_b.annotate("MECO %.0f s" % case.t_meco,
                          xy=(case.t_meco, 1.0), xycoords=("data", "axes fraction"),
                          xytext=(2, -9), textcoords="offset points",
                          fontsize=6.5, color=colour, rotation=90, va="top")
    ax_b.set_xlabel("Time [s]")
    ax_b.set_ylabel("Total mass [t]")
    st.panel_tag(ax_b, "b")
    st.tidy(ax_b)

    fig.tight_layout()
    return st.save(fig, "results_gt_engine.png")


FIGURES = [baseline_card, architecture, atmosphere, rotation, engine]
