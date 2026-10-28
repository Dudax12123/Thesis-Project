"""Figure: the targets of a segmented schedule.

Spec: Thesis_Optimization_Background.tex, Section 4.5.3 (fig:segmented_waypoints).

A generic three-law schedule on a generic ascent (generic_ascent.py, not
simulator output; user requests 2026-10-28): law k steers from h_k to h_(k+1)
and aims at the reference's state there (its waypoint), its deadline is the
reference's time at that waypoint, and the last law aims at the target orbit.
The third hand-off falls during the coast, so law 2 flies the whole burn before
it and the burn after it is flown by a law of its own that inserts directly into
the orbit (the swarm-timed form). The axes carry no numbers.

Placed at 0.75\\linewidth (4.73 in); the script prints the scale so every label
can be checked against the 10 pt body text.
"""
import matplotlib.pyplot as plt

import generic_ascent as ga
from figstyle import use_thesis_style, save, INK, GREY, FAINT, GREEN, AMBER, ACCENT, VIOLET

use_thesis_style()
FS = 9.0
PW = 0.75 * 6.30                # placed at 0.75\linewidth

H2, H3 = 0.10, 0.75                       # hand-off altitudes, generic; h_3 is crossed coasting
T2, T3 = ga.t_at(H2), ga.t_at(H3)

fig, ax = plt.subplots(figsize=(4.8, 2.75))
ax.spines[["top", "right"]].set_visible(False)

# three laws: law 2 flies up to the coast, law 3 the burn after it
for (a, b), col in (((ga.T_KICK, T2), GREEN), ((T2, ga.T_COAST0), AMBER),
                    ((ga.T_COAST1, ga.T_END), ACCENT)):
    t = ga.grid(a, b)
    ax.plot(t, ga.h(t), color=col, lw=2.4, zorder=5, solid_capstyle="butt")
t = ga.grid(ga.T_COAST0, ga.T_COAST1)
ax.plot(t, ga.h(t), color=GREY, lw=1.6, ls=(0, (1.5, 2.2)), zorder=4)
ax.plot([ga.T_END, ga.T_END], [ga.H_ORB - 0.035, ga.H_ORB + 0.035], color=ACCENT, lw=1.8,
        zorder=7)

# hand-off altitudes, waypoints and their deadlines
for hk, tk, k in ((H2, T2, 2), (H3, T3, 3)):
    ax.plot([-0.05, tk], [hk, hk], color=FAINT, lw=0.8, ls=(0, (3, 2)), zorder=1)
    ax.text(-0.045, hk + 0.012, "$h_%d$" % k, fontsize=FS, color=INK, ha="left", va="bottom")
    ax.plot([tk, tk], [-0.12, hk], color=FAINT, lw=0.8, ls=(0, (3, 2)), zorder=1)
    ax.text(tk + 0.012, -0.105, r"$t_{\mathrm{ref}}(h_%d)$" % k, fontsize=FS, color=GREY,
            ha="left", va="bottom")
    ax.plot([tk], [hk], marker="o", ms=7, mfc="white", mec=VIOLET, mew=1.6, zorder=9)

# target orbit
ax.plot([0.9, ga.T_END + 0.08], [ga.H_ORB, ga.H_ORB], color=INK, lw=0.9, ls=(0, (6, 3)),
        zorder=1)
ax.text(0.9, ga.H_ORB + 0.02, "target orbit", fontsize=FS, color=INK, ha="left", va="bottom")

ax.text(0.0, 0.195, "law 1 aims at\nwaypoint 2", fontsize=FS, color=GREEN, ha="left",
        va="bottom", linespacing=1.15)
ax.text(0.50, 0.47, "law 2 aims at\nwaypoint 3", fontsize=FS, color=AMBER, ha="left",
        va="bottom", linespacing=1.15)
ax.text(1.21, 0.665, "coast", fontsize=FS, color=GREY, ha="right", va="center")
ax.text(ga.T_END + 0.07, 0.90, "law 3 aims\nat the orbit", fontsize=FS, color=ACCENT,
        ha="right", va="top", linespacing=1.15)
ax.plot([ga.T_KICK], [0.0], marker="o", ms=4.6, color=INK, zorder=8)
ax.text(ga.T_KICK + 0.01, -0.03, "kick", fontsize=FS, color=INK, ha="left", va="top")

ax.set_xlabel("time", fontsize=FS)
ax.set_ylabel("altitude", fontsize=FS)
ax.set_xticks([])
ax.set_yticks([])
ax.set_xlim(-0.05, ga.T_END + 0.08)
ax.set_ylim(-0.12, 1.10)

p = save(fig, "segmented_waypoints.png")
w_in = plt.imread(p).shape[1] / 300.0
print("printed scale %.3f -> labels at %.1f pt" % (PW / w_in, FS * PW / w_in))
