"""Figure: the segmented multi-law schedule.

Spec: Thesis_Optimization_Background.tex, the "% FIG-5 (TO DRAW)" block above
\\includegraphics{Figures/segmented_schedule.png} in Section 3.4.4.

The spec allows downrange or time on the horizontal axis.  Time is used: on a
downrange axis the whole hand-off structure collapses into the first few per
cent of the plot.  The trajectory shape is illustrative and the activation
altitudes are optimised, so they are labelled generically as h_1, h_2.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator

from figstyle import (use_thesis_style, save, INK, GREY, FAINT, THRUST,
                      ACCENT, GREEN, AMBER, VIOLET)

use_thesis_style()

# --- illustrative ascent shape, altitude against time -------------------
T = [0, 30, 60, 100, 130, 160, 200, 240, 300, 380, 460, 540]
H = [0, 3, 12, 32, 52, 75, 105, 140, 205, 290, 390, 500]
flown = PchipInterpolator(T, H)
ref = PchipInterpolator([1.045 * t for t in T], H)

TEND = 540.0
H1, H2 = 40.0, 120.0                 # activation altitudes (optimised)
MECO_T = 160.0

t = np.linspace(0, TEND, 900)
tr = np.linspace(0, 1.045 * TEND, 900)


def t_at_h(f, h, hi):
    ts = np.linspace(0, hi, 6000)
    return float(np.interp(h, f(ts), ts))


t1, t2 = t_at_h(flown, H1, TEND), t_at_h(flown, H2, TEND)

fig, ax = plt.subplots(figsize=(7.1, 4.8))
ax.spines[["top", "right"]].set_visible(False)
ax.tick_params(labelsize=8, length=3, width=0.7)

# --- cached PMP reference, behind everything ----------------------------
ax.plot(tr, ref(tr), color=VIOLET, lw=1.5, ls=(0, (5, 3)), alpha=0.6,
        zorder=2)
ax.text(452, ref(452) - 34, "PMP reference (cached)", color=VIOLET,
        fontsize=8.0, alpha=0.9, rotation=42, ha="center", va="center")

# --- flown trajectory, banded by the active guidance law ----------------
for t0, t1_, c in ((0.0, t1, GREEN), (t1, t2, AMBER), (t2, TEND, ACCENT)):
    m = (t >= t0) & (t <= t1_)
    ax.plot(t[m], flown(t[m]), color=c, lw=3.0, zorder=5,
            solid_capstyle="butt")

for h, lab in ((H1, "$h_1$"), (H2, "$h_2$")):
    ax.axhline(h, color=GREY, lw=0.7, ls=(0, (2.5, 2)), zorder=1)
    ax.text(-16, h, lab, fontsize=10, color=GREY, ha="right", va="center")

ax.text(30, 26, "law 1\n(gravity turn)", color=GREEN, fontsize=8.2,
        ha="left", va="bottom", linespacing=1.3)
ax.text(122, 88, "law 2", color=AMBER, fontsize=8.6, ha="left", va="bottom")
ax.text(252, 246, "law 3  (final segment)", color=ACCENT, fontsize=8.6,
        ha="left", va="bottom")

# --- waypoints read off the reference -----------------------------------
for h, tag, t_src, c in ((H1, "1", 0.55 * t1, GREEN),
                         (H2, "2", t1 + 0.45 * (t2 - t1), AMBER)):
    tw = t_at_h(ref, h, 1.045 * TEND)
    ax.plot([tw], [h], marker="o", ms=6.5, mfc="white", mec=VIOLET, mew=1.6,
            zorder=8)
    ax.annotate("", xy=(tw - 3, h - 2.5), xytext=(t_src, flown(t_src)),
                arrowprops=dict(arrowstyle="-|>", color=c, lw=1.2,
                                connectionstyle="arc3,rad=-0.35",
                                mutation_scale=9), zorder=7)
    ax.text(tw + 9, h - 7, r"$(h_%s,\ v_%s,\ \gamma_%s)$" % (tag, tag, tag),
            fontsize=8.4, color=VIOLET, ha="left", va="top")

ax.plot([TEND], [500], marker="*", ms=13, color=ACCENT, zorder=9)
ax.annotate("", xy=(TEND - 8, 494), xytext=(432, 405),
            arrowprops=dict(arrowstyle="-|>", color=ACCENT, lw=1.2,
                            connectionstyle="arc3,rad=-0.30",
                            mutation_scale=9), zorder=7)
ax.text(TEND - 6, 512, "orbit insertion", color=ACCENT, fontsize=8.4,
        ha="right", va="bottom")

# --- kick and staging ----------------------------------------------------
ax.plot([12], [flown(12)], marker="o", ms=4.6, color=INK, zorder=8)
ax.text(14, -16, "kick", fontsize=8.0, color=INK, ha="left", va="top")

ax.plot([MECO_T], [flown(MECO_T)], marker="s", ms=6.5, color=INK, zorder=9)
ax.text(MECO_T + 8, 72, "MECO / staging", fontsize=8.4, color=INK, ha="left",
        va="top")
ax.annotate("", xy=(t1 + 2, H1 - 2), xytext=(186, 6),
            arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.8,
                            mutation_scale=8, shrinkA=3, shrinkB=2))
ax.text(192, 4,
        "the hand-off at $h_1$ lies $\\it{below}$ staging:\n"
        "a closed-loop law flies during stage 1",
        fontsize=8.0, color=INK, ha="left", va="center", linespacing=1.4)

ax.set_xlabel("time from lift-off  [s]", fontsize=9)
ax.set_ylabel("altitude  [km]", fontsize=9)
ax.set_xlim(-35, 600)
ax.set_ylim(-40, 565)

# --- inset: the time-to-go problem across the stage boundary ------------
axi = ax.inset_axes([0.660, 0.055, 0.325, 0.315])
ts = np.linspace(0, 300, 600)
stage = 160.0
planned = 300.0 - ts
pre, post = ts < stage, ts >= stage
rocket = np.empty_like(ts)
rocket[pre] = 172.0 - 1.03 * ts[pre]
rocket[post] = 258.0 - 1.84 * (ts[post] - stage)
axi.plot(ts[pre], rocket[pre], color=AMBER, lw=1.6)
axi.plot(ts[post], rocket[post], color=AMBER, lw=1.6)
axi.plot([stage, stage], [rocket[pre][-1], 258.0], color=AMBER, lw=1.0,
         ls=(0, (2, 1.5)))
axi.plot(ts, planned, color=THRUST, lw=1.9)
axi.axvline(stage, color=GREY, lw=0.7, ls=(0, (2.5, 2)))
axi.text(stage - 5, 296, "staging", fontsize=6.5, color=GREY, va="top",
         ha="right")
axi.text(170, 118, "rocket-equation\nestimate —\ncollapses at\nstaging",
         fontsize=6.2, color=AMBER, ha="left", va="top", linespacing=1.3)
axi.text(6, 136, "planned\ndeadline —\nused here",
         fontsize=6.2, color=THRUST, ha="left", va="top", linespacing=1.3)
axi.set_xlim(0, 300)
axi.set_ylim(0, 310)
axi.set_xticks([])
axi.set_yticks([])
axi.set_xlabel("time", fontsize=6.8, labelpad=1.5)
axi.set_ylabel("$t_{go}$", fontsize=7.6, labelpad=2)
for sp in axi.spines.values():
    sp.set_linewidth(0.7)
    sp.set_color(GREY)
axi.set_title("time-to-go across the stage boundary", fontsize=7.2,
              color=GREY, pad=3)

save(fig, "segmented_schedule.png")
