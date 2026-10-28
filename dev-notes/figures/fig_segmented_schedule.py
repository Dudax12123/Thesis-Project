"""Figure: a law-terminated segmented schedule.

Spec: Thesis_Optimization_Background.tex, Section 4.5.3 (fig:segmented_schedule);
dev-notes/segmented-4.5.3-draft-2026-10-28.tex.

The schedule of the Chapter 6 cases in its law-terminated form, drawn on a
generic ascent (generic_ascent.py; user request 2026-10-28, replacing the flown
show_seg_fixed_alt against its reference): the gravity turn from the kick to the
hand-off altitude, then vector PEG, which aims its first burn at the coast-start
waypoint, ends it on its own time-to-go, coasts, and ends the insertion burn the
same way. Each decision is marked with who makes it: the swarm (kick, coast) or
the law (both cut-offs). The axes carry no numbers.

The thesis places the figure at \\linewidth (6.30 in): the script prints the
scale, so every label can be checked against the 10 pt body text.
"""
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import generic_ascent as ga
from figstyle import use_thesis_style, save, INK, GREY, ACCENT, GREEN, VIOLET

use_thesis_style()
FS = 9.0
PW = 0.75 * 6.30                # placed at 0.75\linewidth

H_HAND = 0.30                               # the hand-off altitude, generic
T_HAND = ga.t_at(H_HAND)

fig, ax = plt.subplots(figsize=(4.8, 2.75))
ax.spines[["top", "right"]].set_visible(False)

t = ga.grid(ga.T_KICK, T_HAND)
ax.plot(t, ga.h(t), color=GREEN, lw=2.4, zorder=5, solid_capstyle="butt")
for a, b in ((T_HAND, ga.T_COAST0), (ga.T_COAST1, ga.T_END)):
    t = ga.grid(a, b)
    ax.plot(t, ga.h(t), color=ACCENT, lw=2.4, zorder=5, solid_capstyle="butt")
t = ga.grid(ga.T_COAST0, ga.T_COAST1)
ax.plot(t, ga.h(t), color=GREY, lw=1.6, ls=(0, (1.5, 2.2)), zorder=4)
ax.plot([ga.T_END, ga.T_END], [ga.H_ORB - 0.035, ga.H_ORB + 0.035], color=ACCENT, lw=1.8,
        zorder=7)

# target orbit
ax.plot([0.9, ga.T_END + 0.08], [ga.H_ORB, ga.H_ORB], color=INK, lw=0.9, ls=(0, (6, 3)),
        zorder=1)
ax.text(0.9, ga.H_ORB + 0.02, "target orbit", fontsize=FS, color=INK, ha="left", va="bottom")

# events and who decides them
ax.plot([ga.T_KICK], [0.0], marker="o", ms=4.6, color=INK, zorder=8)
ax.text(ga.T_KICK + 0.01, -0.03, r"kick, $\gamma_p$ (swarm)", fontsize=FS, color=INK,
        ha="left", va="top")
h_meco = float(ga.h(ga.T_MECO))
ax.plot([ga.T_MECO], [h_meco], marker="s", ms=6.0, color=INK, zorder=9)
ax.text(ga.T_MECO - 0.03, h_meco + 0.02, "MECO / staging", fontsize=FS, color=INK,
        ha="right", va="bottom")
ax.plot([T_HAND], [H_HAND], marker="o", ms=5.5, mfc="white", mec=INK, mew=1.2, zorder=8)
ax.text(T_HAND + 0.03, H_HAND - 0.015, "hand-off altitude\n(fixed or optimized)", fontsize=FS,
        color=INK, ha="left", va="top", linespacing=1.15)
h_c0 = float(ga.h(ga.T_COAST0))
ax.plot([ga.T_COAST0], [h_c0], marker="o", ms=7, mfc="white", mec=VIOLET, mew=1.7, zorder=9)
ax.text(ga.T_COAST0 - 0.02, h_c0 + 0.04, "coast-start waypoint:\ncut-off on PEG's $t_{go}$",
        fontsize=FS, color=VIOLET, ha="right", va="bottom", linespacing=1.15)
ax.text(1.28, 0.72, "coast,\n" r"$\Delta t_c$ (swarm)", fontsize=FS, color=GREY, ha="right",
        va="center", linespacing=1.15)
ax.text(ga.T_END + 0.07, ga.H_ORB - 0.10, "insertion:\ncut-off on\nPEG's $t_{go}$",
        fontsize=FS, color=ACCENT, ha="right", va="top", linespacing=1.15)

ax.legend(handles=[
    Line2D([], [], color=GREEN, lw=2.4, label="gravity turn"),
    Line2D([], [], color=ACCENT, lw=2.4, label="vector PEG"),
    Line2D([], [], color=GREY, lw=1.6, ls=(0, (1.5, 2.2)), label="coast")],
    loc="upper left", fontsize=FS, frameon=False, handlelength=2.6)

ax.set_xlabel("time", fontsize=FS)
ax.set_ylabel("altitude", fontsize=FS)
ax.set_xticks([])
ax.set_yticks([])
ax.set_xlim(-0.05, ga.T_END + 0.08)
ax.set_ylim(-0.12, 1.10)

p = save(fig, "segmented_schedule.png")
w_in = plt.imread(p).shape[1] / 300.0
print("printed scale %.3f -> labels at %.1f pt" % (PW / w_in, FS * PW / w_in))
