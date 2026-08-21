"""The four-panel run card, for one trajectory.

This is the reporting template of Chapter 6 Section 6.1, and the only figure in
the set that describes a single case: everything else in the chapter is a
comparison between cases. It is therefore also what ``main.py`` can draw for an
interactive run under ``PLOT_SUITE = "new"``.

The four panels carry, between them, the quantities the twenty-plot diagnostic
suite spreads over twenty figures. Three of the angles reduce to two curves
(theta = alpha + gamma), dynamic pressure and Mach both follow from (v, h), and
propellant is the mass trace shifted by a constant -- so the compression costs
much less than the plot count suggests.
"""

import matplotlib.pyplot as plt
import numpy as np

from . import _style as st


def draw(case, filename, title=None):
    """Draw the run card for one :class:`~._data.Case` and save it.

    Panels absent from the data are handled rather than assumed: a run without
    an atmosphere has no dynamic pressure worth plotting, and a case with no
    recorded arc times simply gets no event markers.
    """
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

    # (c) all three angles -- theta is alpha + gamma, so only two are free.
    # Pitch is drawn wide and underneath: a law commanding alpha = 0 puts theta
    # and gamma on top of each other by definition, and a same-weight trace
    # would simply be hidden.
    t, gam, th, al = st.thin(case.time, case.gamma_deg, case.theta_deg,
                             case.alpha_deg)
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

    # (d) what the atmosphere does to the vehicle. A drag-free run has none of
    # this, and says so rather than drawing a flat zero.
    t, q, mach = st.thin(case.time, case.q, case.mach)
    end = case.t_meco if case.t_meco else t[-1]
    in_atm = t <= end
    q_max = float(np.nanmax(q[in_atm])) if np.any(in_atm) else 0.0

    if case.row.get("include_drag") is False or q_max <= 0.0:
        ax_d.text(0.5, 0.5, "no atmosphere modelled", transform=ax_d.transAxes,
                  ha="center", va="center", fontsize=8, color=st.GREY,
                  style="italic")
        ax_d.set_xticks([])
        ax_d.set_yticks([])
    else:
        ax_d.plot(t[in_atm], q[in_atm] / 1e3, color=st.BASELINE)
        ax_d.set_xlabel("Time [s]")
        ax_d.set_ylabel("Dynamic pressure [kPa]", color=st.BASELINE)
        ax_dm = ax_d.twinx()
        ax_dm.plot(t[in_atm], mach[in_atm], color=st.AMBER, linewidth=1.1)
        ax_dm.set_ylabel("Mach [-]", color=st.AMBER)
        ax_dm.spines["top"].set_visible(False)
        t_qmax = float(t[in_atm][int(np.nanargmax(q[in_atm]))])
        ax_d.annotate(r"max $q$ = %.1f kPa" % (q_max / 1e3),
                      xy=(t_qmax, q_max / 1e3), xytext=(6, -2),
                      textcoords="offset points", fontsize=7, color=st.INK)
    st.panel_tag(ax_d, "d")
    st.tidy(ax_d, legend=False)

    if title:
        fig.suptitle(title, fontsize=9.5, y=1.005)
    fig.tight_layout()
    return st.save(fig, filename)
