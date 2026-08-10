"""Figure: mission phase timeline.

Spec: Thesis_Ascent_Background.tex, the "% FIG (TO DRAW)" block above
\\includegraphics{Figures/mission_phase_timeline.png} in Section 2.8.2.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from figstyle import (use_thesis_style, blank_axes, save, arrow, note, leader,
                      INK, GREY, FAINT, THRUST, ACCENT, GREEN, AMBER, COAST)

use_thesis_style()

fig, ax = plt.subplots(figsize=(7.2, 4.9))
blank_axes(ax)
ax.set_aspect("auto")

# --- main timeline -------------------------------------------------------
Y0, YH = 3.34, 0.44                     # band bottom and height
X_START, X_END = 0.40, 10.20

PHASES = [
    (0.40, 1.15, "#e9e9e9", "vertical\nrise"),
    (1.15, 2.15, "#fbeccd", "pitch-over\n(kick)"),
    (2.15, 6.35, "#e3ebf5", "gravity turn — unguided arc"),
    (6.35, 7.40, "#ffffff", "sep.\ncoast"),
]
for x0, x1, c, lab in PHASES:
    ax.add_patch(Rectangle((x0, Y0), x1 - x0, YH, fc=c, ec=INK, lw=0.8,
                           zorder=3))
    ax.text(0.5 * (x0 + x1), Y0 + YH / 2, lab, ha="center", va="center",
            fontsize=7.4, color=INK, zorder=5, linespacing=1.2)

ax.add_patch(Rectangle((7.40, Y0), X_END - 7.40, YH, fc="#dfe8f3", ec=INK,
                       lw=0.8, hatch="///", zorder=3))
ax.text(0.5 * (7.40 + X_END), Y0 + YH / 2, "guided second stage",
        ha="center", va="center", fontsize=7.6, color=INK, zorder=5,
        bbox=dict(boxstyle="round,pad=0.16", fc="white", ec="none"))

# --- events --------------------------------------------------------------
#   solid tick = located during integration by a condition on the state
#   dashed tick = prescribed duration, or set by a decision variable
EVENTS = [
    (0.40, "solid", 3.02, "lift-off\n$t = 0$", "top"),
    (1.15, "dash", 3.94, "kick start\n$T_k = 7.5$ s", "bottom"),
    (2.15, "solid", 4.30, "kick end", "bottom"),
    (3.30, "solid", 3.94, "max-$q$", "bottom"),
    (4.70, "solid", 4.30,
     "atmosphere exit\n$=$ fairing jettison\n$=$ guidance engagement", "bottom"),
    (6.35, "solid", 3.94, "MECO", "bottom"),
    (6.75, "dash", 4.34, "separation\n$+3$ s", "bottom"),
    (7.40, "dash", 4.86, "ignition\n$+8$ s", "bottom"),
]
for x, kind, ly, lab, side in EVENTS:
    solid = kind == "solid"
    top = Y0 + YH + (0.06 if side == "bottom" else 0.0)
    y_hi = ly - 0.04 if side == "bottom" else Y0 - 0.06
    if side == "bottom":
        ax.plot([x, x], [Y0 + YH, y_hi], color=INK if solid else GREY,
                lw=1.3 if solid else 1.0,
                ls="-" if solid else (0, (3, 2.2)), zorder=6)
    else:
        ax.plot([x, x], [Y0, ly + 0.06], color=INK if solid else GREY,
                lw=1.3 if solid else 1.0,
                ls="-" if solid else (0, (3, 2.2)), zorder=6)
    ax.text(x, ly, lab, ha="center",
            va="bottom" if side == "bottom" else "top",
            fontsize=7.2, color=INK if solid else GREY, zorder=7,
            linespacing=1.25)

# max-q is located by root-finding, not prescribed: draw it solid
ax.plot([3.30, 3.30], [Y0 + YH, 3.90], color=INK, lw=1.3, zorder=6)

ax.annotate("", xy=(X_END + 0.28, Y0 + YH / 2), xytext=(X_START - 0.30, Y0 + YH / 2),
            arrowprops=dict(arrowstyle="-|>", color=FAINT, lw=1.1,
                            mutation_scale=12), zorder=1)
ax.text(X_END + 0.34, Y0 + YH / 2, "$t$", fontsize=10, color=GREY, va="center")

# --- the final phase, expanded ------------------------------------------
DX0, DX1 = 2.20, 10.30
for a, b in ((7.40, DX0), (X_END, DX1)):
    ax.plot([a, b], [Y0 - 0.02, 2.72], color=FAINT, lw=0.9, ls=(0, (4, 3)),
            zorder=1)
ax.add_patch(Rectangle((DX0, 0.62), DX1 - DX0, 2.10, fc="#fafbfd", ec=FAINT,
                       lw=0.8, zorder=0))
ax.text(DX0 + 0.06, 2.62, "the final phase, by architecture", fontsize=8.4,
        color=GREY, ha="left", va="top", style="italic")

SH = 0.26
STRIPS = [
    (2.22, "(a)", [("T", 2.60, 4.20), ("C", 4.20, 6.95), ("T", 6.95, 7.40)],
     "event: apogee $= r_{\\rm target}$"),
    (1.62, "(b)", [("T", 2.60, 6.40)],
     "event: circular velocity"),
    (1.02, "(c)", [("T", 2.60, 4.05), ("C", 4.05, 5.95), ("T", 5.95, 7.40)],
     "planned: end of arc 3"),
]
for y, tag, arcs, cap in STRIPS:
    ax.text(DX0 + 0.12, y + SH / 2, tag, fontsize=8.4, color=INK, ha="left",
            va="center")
    for kind, x0, x1 in arcs:
        ax.add_patch(Rectangle((x0, y), x1 - x0, SH,
                               fc=THRUST if kind == "T" else COAST,
                               ec=INK, lw=0.8, zorder=4))
        ax.text(0.5 * (x0 + x1), y + SH / 2,
                "thrust" if kind == "T" else "coast",
                ha="center", va="center", fontsize=7.0,
                color="white" if kind == "T" else INK, zorder=5)
    xe = arcs[-1][2]
    ax.plot([xe, xe], [y - 0.09, y + SH + 0.09], color=INK, lw=1.3, zorder=6)
    ax.text(xe + 0.10, y + SH / 2, cap, fontsize=7.2, color=GREY, ha="left",
            va="center")
ax.plot([2.60, 2.60], [0.86, 2.56], color=GREY, lw=0.9, ls=(0, (3, 2.2)),
        zorder=3)
ax.text(2.70, 0.80, "second-stage ignition — common origin", fontsize=7.2,
        color=GREY, ha="left", va="top")

# --- legend --------------------------------------------------------------
ax.plot([0.45, 0.45], [0.34, 0.62], color=INK, lw=1.3)
ax.text(0.58, 0.48, "located during integration by a condition on the state",
        fontsize=7.6, color=INK, va="center")
ax.plot([0.45, 0.45], [-0.06, 0.22], color=GREY, lw=1.0, ls=(0, (3, 2.2)))
ax.text(0.58, 0.08,
        "prescribed duration, or set by an optimiser decision variable",
        fontsize=7.6, color=GREY, va="center")

ax.set_xlim(0.05, 10.85)
ax.set_ylim(-0.25, 5.45)
save(fig, "mission_phase_timeline.png")
