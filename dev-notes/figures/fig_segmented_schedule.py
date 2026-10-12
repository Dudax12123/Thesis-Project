"""Figure: the segmented schedule flown in Chapter 6.

Spec: Thesis_Optimization_Background.tex, Section 3.4.7 (fig:segmented_schedule).

One schedule, in its law-terminated form: the gravity turn from the kick to the
hand-off altitude, then vector PEG, which aims its first burn at the coast-start
waypoint read off the indirect reference, coasts, and inserts.

Since 2026-10-12 the curves are the flown archives, not an illustrative shape:
show_seg_fixed_alt and its reference pmp_baseline, from the Chapter 6 results
set (Output/results_matrix_chapter6_20261007). Read through the plot library's
loader, Plots/results_figures/_data.py. Time is the horizontal axis, compressed
over the coast, which would otherwise take nine tenths of the width.

The thesis places the figure at \\linewidth (16 cm = 6.30 in): every label is
drawn at FS so that none prints below the 10 pt body text.
"""
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

from figstyle import use_thesis_style, save, INK, GREY, ACCENT, GREEN, VIOLET

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "..", "Tese", "src"))
sys.path.insert(0, SRC)
from Plots.results_figures import _data  # noqa: E402

ROOT = os.path.join(SRC, "Output", "results_matrix_chapter6_20261007")
flown = _data.load("show_seg_fixed_alt", root=ROOT)
ref = _data.load("pmp_baseline", root=ROOT)

use_thesis_style()
FS = 9.0                         # prints at ~10.4 pt: the drawing is placed wider than it is drawn

# --- events, read off the archives ----------------------------------------
H_HAND = flown.segment_schedule[1][1] / 1000.0          # 120 km
T_KICK = 7.5
T_MECO = float(flown.row["t_meco"])
(TC0, TC1), = [iv for iv in flown.coast_intervals() if iv[1] - iv[0] > 100]
(RC0, RC1), = [iv for iv in ref.coast_intervals() if iv[1] - iv[0] > 100]
T_INS, R_INS = flown.t_insertion, ref.t_insertion

# --- the coast is drawn compressed ----------------------------------------
T_BREAK = 425.0                 # the axis is true time up to here
X_END = 600.0                   # where the latest insertion lands
K = (X_END - T_BREAK) / (R_INS - T_BREAK)


def x_of(t):
    t = np.asarray(t, dtype=float)
    return np.where(t <= T_BREAK, t, T_BREAK + (t - T_BREAK) * K)


def h_at(case, t):
    return float(np.interp(t, case.time, case.alt_km))


fig, ax = plt.subplots(figsize=(6.3, 4.0))
ax.spines[["top", "right"]].set_visible(False)
ax.tick_params(labelsize=FS, length=3, width=0.7)

# --- indirect reference, behind everything --------------------------------
m = ref.time <= R_INS
ax.plot(x_of(ref.time[m]), ref.alt_km[m], color=VIOLET, lw=1.5, ls=(0, (5, 3)),
        alpha=0.65, zorder=2)
ax.text(x_of(1500), 330, "reference", color=VIOLET, fontsize=FS, ha="left",
        va="center")

# --- flown schedule: gravity turn, then vector PEG -------------------------
t, h = flown.time, flown.alt_km
t_hand = float(t[np.argmax(h >= H_HAND)])
g = t <= t_hand
ax.plot(x_of(t[g]), h[g], color=GREEN, lw=3.0, zorder=5, solid_capstyle="butt")
b1 = (t >= t_hand) & (t <= TC0)
ax.plot(x_of(t[b1]), h[b1], color=ACCENT, lw=3.0, zorder=5, solid_capstyle="butt")
c = (t >= TC0) & (t <= TC1)
ax.plot(x_of(t[c]), h[c], color=ACCENT, lw=1.3, ls=(0, (1.5, 2.2)), zorder=4)
# the final burn, drawn to the same compressed time scale as the coast and on
# top of its dotted end: it lasts seconds, so it is a short stroke
fb = (t >= TC1) & (t <= T_INS)
ax.plot(x_of(t[fb]), h[fb], color=ACCENT, lw=3.0, zorder=6,
        solid_capstyle="butt")
xe = float(x_of(T_INS))
# and a short tick across the curve at the insertion burn, perpendicular to it
# on the page (worked in display coordinates, since the axes are not isotropic)
fig.canvas.draw()
p1 = ax.transData.transform((float(x_of(TC1 - 60.0)), h_at(flown, TC1 - 60.0)))
p2 = ax.transData.transform((xe, h_at(flown, T_INS)))
d = (p2 - p1) / np.hypot(*(p2 - p1))
# on the upper-left side only, away from the reference just below the curve
n = np.array([-d[1], d[0]]) * (0.06 * fig.dpi)              # 0.06 in long
ends = ax.transData.inverted().transform(np.array([p2, p2 + n]))
ax.plot(ends[:, 0], ends[:, 1], color=ACCENT, lw=1.8, zorder=7,
        solid_capstyle="butt")

ax.text(6, 70, "gravity turn", color=GREEN, fontsize=FS, ha="left",
        va="bottom")
ax.text(x_of(1000) - 14, 330, "coast", color=ACCENT, fontsize=FS, ha="right",
        va="center")
ax.text(xe - 4, h_at(flown, T_INS) + 16, "final burn,\norbit insertion",
        color=ACCENT, fontsize=FS, ha="right", va="bottom", linespacing=1.2)

# --- hand-off and coast-start waypoint ------------------------------------
ax.plot([t_hand], [H_HAND], marker="o", ms=5.5, mfc="white", mec=INK, mew=1.2,
        zorder=8)
ax.plot([x_of(RC0)], [h_at(ref, RC0)], marker="o", ms=7, mfc="white",
        mec=VIOLET, mew=1.7, zorder=9)
ax.text(x_of(RC0) - 8, h_at(ref, RC0) + 22, "coast-start\nwaypoint",
        fontsize=FS, color=VIOLET, ha="right", va="bottom", linespacing=1.2)

# --- kick and staging -----------------------------------------------------
ax.plot([T_KICK], [h_at(flown, T_KICK)], marker="o", ms=4.6, color=INK, zorder=8)
ax.text(T_KICK + 4, -14, "kick", fontsize=FS, color=INK, ha="left", va="top")
ax.plot([T_MECO], [h_at(flown, T_MECO)], marker="s", ms=6.0, color=INK, zorder=9)
ax.text(T_MECO + 8, h_at(flown, T_MECO) - 6, "MECO / staging", fontsize=FS,
        color=INK, ha="left", va="top")

# --- axes, with the coast compressed --------------------------------------
ax.set_xlabel("time from lift-off  [s]", fontsize=FS)
ax.set_ylabel("altitude  [km]", fontsize=FS)
ax.set_xlim(-30, X_END + 15)
ax.set_ylim(-40, 575)
ticks = [0, 100, 200, 300, 400, 1000, 2000]
ax.set_xticks(x_of(ticks))
ax.set_xticklabels([str(v) for v in ticks])
xb = float(x_of(T_BREAK)) + 6
for dx in (-4, 4):
    ax.plot([xb + dx - 4, xb + dx + 4], [-49, -31], color=INK, lw=0.9,
            clip_on=False, zorder=10)
ax.text(xb + 14, -34, "coast compressed", fontsize=FS, color=GREY,
        ha="left", va="bottom")

save(fig, "segmented_schedule.png")
