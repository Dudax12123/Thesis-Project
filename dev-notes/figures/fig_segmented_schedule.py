"""Figure: the segmented schedule flown in Chapter 6.

Spec: Thesis_Optimization_Background.tex, the "% FIG-5 (TO DRAW)" block above
\\includegraphics{Figures/segmented_schedule.png} in Section 3.8.

One schedule, in its law-terminated form: the gravity turn from the kick to the
hand-off altitude, then vector PEG, which aims its first burn at the coast-start
waypoint read off the cached indirect reference, coasts, and inserts. The shape
is illustrative but placed on the flown events (MECO 146 s at about 62 km; the
coast starting at about 411 s and 165 km; a coast of about 1300 s to 500 km).
Time is the horizontal axis, compressed over the coast, which would otherwise
take three quarters of the width.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator

from figstyle import use_thesis_style, save, INK, GREY, ACCENT, GREEN, VIOLET

use_thesis_style()

# --- illustrative shape on the flown events ------------------------------
T = [0, 7.5, 40, 80, 120, 146, 154, 200, 250, 300, 360, 411]
H_FLOWN = [0, 0.1, 3.0, 15, 40, 63, 70, 95, 120, 139, 156, 165]
H_REF = [0, 0.1, 3.4, 17, 44, 64, 72, 100, 126, 145, 159, 165]
flown = PchipInterpolator(T, H_FLOWN)
ref = PchipInterpolator(T, H_REF)

T_KICK, T_MECO, T_COAST = 7.5, 146.0, 411.0
H_HAND, H_WP, H_ORBIT = 120.0, 165.0, 500.0
X_BREAK, X_INS = 432.0, 590.0          # the coast is drawn on a compressed axis


def coast_curve(x0, x1, h0, h1, n=200):
    """A ballistic rise to apoapsis: concave, flat at the top."""
    s = np.linspace(0.0, 1.0, n)
    return x0 + (x1 - x0) * s, h0 + (h1 - h0) * np.sin(0.5 * np.pi * s)


fig, ax = plt.subplots(figsize=(7.1, 4.6))
ax.spines[["top", "right"]].set_visible(False)
ax.tick_params(labelsize=8, length=3, width=0.7)

# --- cached indirect reference, behind everything -------------------------
t = np.linspace(0.0, T_COAST, 600)
ax.plot(t, ref(t), color=VIOLET, lw=1.5, ls=(0, (5, 3)), alpha=0.65, zorder=2)
xc, hc = coast_curve(T_COAST, X_INS + 6, H_WP, H_ORBIT)
ax.plot(xc, hc + 6, color=VIOLET, lw=1.5, ls=(0, (5, 3)), alpha=0.65, zorder=2)
ax.text(560, 395, "indirect\nreference\n(cached)", color=VIOLET, fontsize=8.0,
        ha="right", va="center", linespacing=1.3)

# --- flown schedule: gravity turn, then vector PEG ------------------------
t_hand = float(np.interp(H_HAND, flown(t), t))
m1 = t <= t_hand
ax.plot(t[m1], flown(t[m1]), color=GREEN, lw=3.0, zorder=5, solid_capstyle="butt")
m2 = t >= t_hand
ax.plot(t[m2], flown(t[m2]), color=ACCENT, lw=3.0, zorder=5, solid_capstyle="butt")
ax.plot(xc, hc, color=ACCENT, lw=1.3, ls=(0, (1.5, 2.2)), zorder=4)
ax.plot([X_INS - 4, X_INS], [H_ORBIT, H_ORBIT], color=ACCENT, lw=3.0, zorder=5,
        solid_capstyle="butt")

ax.text(4, 62, "gravity turn\n(from the kick)", color=GREEN, fontsize=8.4,
        ha="left", va="bottom", linespacing=1.3)
ax.text(330, 118, "vector PEG,\nfirst burn", color=ACCENT, fontsize=8.4,
        ha="left", va="top", linespacing=1.3)
ax.text(512, 300, "coast", color=ACCENT, fontsize=8.4, ha="left", va="center")

# --- hand-off altitude ---------------------------------------------------
ax.axhline(H_HAND, xmax=0.47, color=GREY, lw=0.7, ls=(0, (2.5, 2)), zorder=1)
ax.text(-24, H_HAND, "$h_h$", fontsize=10, color=GREY, ha="right", va="center")
ax.plot([t_hand], [H_HAND], marker="o", ms=5.5, mfc="white", mec=INK, mew=1.2,
        zorder=8)
ax.text(t_hand - 6, H_HAND + 14,
        "hand-off: fixed at 120 km, or optimised\nin $[10\\ \\mathrm{km},\\ 0.98\\,h_{wp}]$",
        fontsize=7.8, color=INK, ha="right", va="bottom", linespacing=1.35)

# --- coast-start waypoint read off the reference --------------------------
ax.plot([T_COAST], [H_WP], marker="o", ms=7, mfc="white", mec=VIOLET, mew=1.7,
        zorder=9)
ax.annotate("", xy=(T_COAST - 3, H_WP - 3), xytext=(318, 128),
            arrowprops=dict(arrowstyle="-|>", color=ACCENT, lw=1.1,
                            connectionstyle="arc3,rad=0.25", mutation_scale=9),
            zorder=7)
ax.text(T_COAST - 6, H_WP + 12,
        "coast-start waypoint\n$(h_{wp},\\ v_{wp},\\ \\gamma_{wp})$",
        fontsize=8.2, color=VIOLET, ha="right", va="bottom", linespacing=1.35)

# --- insertion ------------------------------------------------------------
ax.plot([X_INS], [H_ORBIT], marker="*", ms=13, color=ACCENT, zorder=9)
ax.text(X_INS - 4, H_ORBIT + 14, "final burn,\norbit insertion", color=ACCENT,
        fontsize=8.2, ha="right", va="bottom", linespacing=1.3)

# --- kick and staging -----------------------------------------------------
ax.plot([T_KICK], [flown(T_KICK)], marker="o", ms=4.6, color=INK, zorder=8)
ax.text(T_KICK + 4, -14, "kick", fontsize=8.0, color=INK, ha="left", va="top")
ax.plot([T_MECO], [flown(T_MECO)], marker="s", ms=6.0, color=INK, zorder=9)
ax.text(T_MECO + 8, flown(T_MECO) - 6, "MECO / staging", fontsize=8.2, color=INK,
        ha="left", va="top")

# --- axes, with the coast compressed --------------------------------------
ax.set_xlabel("time from lift-off  [s]", fontsize=9)
ax.set_ylabel("altitude  [km]", fontsize=9)
ax.set_xlim(-45, 615)
ax.set_ylim(-35, 575)
ax.set_xticks([0, 100, 200, 300, 400, X_INS])
ax.set_xticklabels(["0", "100", "200", "300", "400", "$\\approx$1700"])
for dx in (-5, 5):
    ax.plot([X_BREAK + dx - 4, X_BREAK + dx + 4], [-44, -26], color=INK, lw=0.9,
            clip_on=False, zorder=10)
ax.text(X_BREAK + 14, -30, "coast compressed", fontsize=7.0, color=GREY,
        ha="left", va="bottom")

save(fig, "segmented_schedule.png")
