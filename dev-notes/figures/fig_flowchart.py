"""Figure: the brute-force kick-angle search.

Redraw of the original FlowChart.png, discharging the two artwork notes at
Thesis_Optimization_Background.tex:
  1. the input box carried the interval "[3o, 5o]", which contradicts the code
     (ALPHA_LOWEST/ALPHA_HIGHEST search [-5.5 deg, -2.5 deg]); the bracketed
     numbers are dropped so the box cannot drift out of date;
  2. "stoped" -> "stopped".
All other wording is kept verbatim from the original.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle

from figstyle import (use_thesis_style, blank_axes, save, INK, GREY, FAINT,
                      THRUST, ACCENT, GREEN)

use_thesis_style()

fig, ax = plt.subplots(figsize=(7.4, 5.0))
blank_axes(ax)
ax.set_aspect("auto")

PROC = "#dceafa"        # process boxes
IO = "#e2e2e2"          # input / output boxes
DEC = "#fce7cd"         # decisions
SHADE = "#f0f0f0"       # the guidance-law region

A, B, C, D = 1.70, 5.05, 8.40, 11.85
R1, R2, R3, R4 = 6.55, 4.85, 3.05, 1.05
BW, BH = 2.75, 1.05
DW, DH = 3.05, 1.15


def proc(x, y, text, fc=PROC, fs=7.2):
    ax.add_patch(FancyBboxPatch((x - BW / 2, y - BH / 2), BW, BH,
                                boxstyle="round,pad=0,rounding_size=0.08",
                                fc=fc, ec=INK, lw=0.9, zorder=5))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, color=INK,
            zorder=6, linespacing=1.35)


def decision(x, y, text, fs=7.2):
    ax.add_patch(Polygon([[x - DW / 2, y], [x, y + DH / 2],
                          [x + DW / 2, y], [x, y - DH / 2]],
                         closed=True, fc=DEC, ec=INK, lw=0.9, zorder=5))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, color=INK,
            zorder=6, linespacing=1.35)


def line(pts, arrowhead=True, color=INK, lw=1.0):
    pts = np.asarray(pts, dtype=float)
    ax.plot(pts[:, 0], pts[:, 1], color=color, lw=lw, zorder=4,
            solid_joinstyle="miter")
    if arrowhead:
        ax.annotate("", xy=pts[-1], xytext=pts[-2],
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                                    shrinkA=0, shrinkB=0, mutation_scale=10),
                    zorder=4)


# --- the region in which the guidance law acts --------------------------
ax.add_patch(Rectangle((6.55, 2.10), 6.85, 3.45, fc=SHADE, ec="none",
                       zorder=0))
ax.text(13.35, 5.62, "Guidance law in use", fontsize=9.0, color=GREY,
        ha="right", va="bottom", style="italic")

# --- boxes ---------------------------------------------------------------
proc(A, R1, "Simulate vertical\nascent")
proc(B, R1, "Input: choose an\ninitial kick angle", fc=IO)
proc(C, R1, "Simulate the ascent\ntrajectory until MECO\n& negligible atmosphere",
     fs=6.9)

proc(A, R2, "Discard the initial\nkick angle value")
proc(C, R2, "Calculate apogee of\nresulting orbit if\nengines stopped", fs=6.9)

decision(A, R3, "Minimum propellant\nused")
proc(B, R3, "Try another initial kick\nangle value to check if\nit is the minimum",
     fs=6.9)
decision(C, R3, "$R_{apogee}=R_{desired}$")
proc(D, R3, "Simulate ascent\ntrajectory if engine\ncontinues until next\ntime step",
     fs=6.9)

proc(A, R4, "Output: calculate\ntotal propellant used", fc=IO)
proc(B, R4, "Calculate $\\Delta v$ and\npropellant required for\ncircularization",
     fs=6.9)
proc(C, R4, "Simulate coasting\nuntil orbit reached")

# --- flow ----------------------------------------------------------------
line([[A + BW / 2, R1], [B - BW / 2, R1]])
line([[B + BW / 2, R1], [C - BW / 2, R1]])
line([[C, R1 - BH / 2], [C, R2 + BH / 2]])
line([[C, R2 - BH / 2], [C, R3 + DH / 2]])

# the inner guidance loop
line([[C + DW / 2, R3], [D - BW / 2, R3]], color=ACCENT)
ax.text(0.5 * (C + DW / 2 + D - BW / 2), R3 + 0.12, "False", fontsize=7.2,
        color=ACCENT, ha="center", va="bottom",
        bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none"))
line([[D, R3 + BH / 2 + 0.10], [D, R2], [C + BW / 2, R2]], color=ACCENT)

line([[C, R3 - DH / 2], [C, R4 + BH / 2]], color=GREEN)
ax.text(C + 0.10, 0.5 * (R3 - DH / 2 + R4 + BH / 2), "True", fontsize=7.2,
        color=GREEN, ha="left", va="center")

# the outer kick-angle loop
line([[C - BW / 2, R4], [B + BW / 2, R4]])
line([[B - BW / 2, R4], [A + BW / 2, R4]])
line([[A, R4 + BH / 2], [A, R3 - DH / 2]])

line([[A - DW / 2, R3], [A - DW / 2 - 0.55, R3], [A - DW / 2 - 0.55, R2],
      [A - BW / 2, R2]], color=ACCENT)
ax.text(A - DW / 2 - 0.62, 0.5 * (R2 + R3), "False", fontsize=7.2,
        color=ACCENT, ha="right", va="center", rotation=90)
line([[A + BW / 2, R2], [B, R2]], arrowhead=False)

line([[A + DW / 2, R3], [B - BW / 2, R3]], color=GREEN)
ax.text(0.5 * (A + DW / 2 + B - BW / 2), R3 + 0.12, "True", fontsize=7.2,
        color=GREEN, ha="center", va="bottom",
        bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none"))
line([[B, R3 + BH / 2], [B, R1 - BH / 2]])

ax.set_xlim(-0.45, 13.75)
ax.set_ylim(0.35, 7.30)
save(fig, "FlowChart.png")
