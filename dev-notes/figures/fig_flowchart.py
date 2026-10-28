"""Figure: the brute-force kick-angle search.

Redraw of the original FlowChart.png, discharging the two artwork notes at
Thesis_Optimization_Background.tex:
  1. the input box carried the interval "[3o, 5o]", which contradicts the code
     (ALPHA_LOWEST/ALPHA_HIGHEST search [-5.5 deg, -2.5 deg]); the bracketed
     numbers are dropped so the box cannot drift out of date;
  2. "stoped" -> "stopped".
All other wording is kept verbatim from the original.

C3-19 (2026-10-28): block labels name the method (numerical integration, root
search for the cut-off, impulsive circularization).

Figure review, 2026-10-12 (approved as is, to meet the 10 pt rule): the same
boxes, flow and wording, laid out in inches at the printed size. The thesis
places the figure at \\linewidth (16 cm = 6.30 in) and the drawing is that
wide, so every label prints at FS. The lines are re-broken to fit the boxes.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle

from figstyle import use_thesis_style, save, INK, GREY, ACCENT, GREEN

use_thesis_style()

FS = 10.2                # every label
LINE = 0.175             # height of one text line [in]
W, H = 6.30, 4.95
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")

PROC = "#dceafa"        # process boxes
IO = "#e2e2e2"          # input / output boxes
DEC = "#fce7cd"         # decisions
SHADE = "#f0f0f0"       # the guidance-law region

LOOP = 0.30                              # the outer False loop, left of A
A, B, C, D = 1.08, 2.58, 4.08, 5.63      # column centres
R1, R2, R3, R4 = 4.30, 3.18, 1.88, 0.68  # row centres
BW = 1.30                                # box width
DWA, DHA = 1.36, 0.82                    # decision at A
DWC, DHC = 1.30, 0.78                    # decision at C


def box_h(text):
    return LINE * (text.count("\n") + 1) + 0.16


def proc(x, y, text, fc=PROC, w=BW):
    h = box_h(text)
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle="round,pad=0,rounding_size=0.05",
                                fc=fc, ec=INK, lw=0.9, zorder=5))
    ax.text(x, y, text, ha="center", va="center", fontsize=FS, color=INK,
            zorder=6, linespacing=1.15)
    return h


def decision(x, y, w, h, text):
    ax.add_patch(Polygon([[x - w / 2, y], [x, y + h / 2],
                          [x + w / 2, y], [x, y - h / 2]],
                         closed=True, fc=DEC, ec=INK, lw=0.9, zorder=5))
    ax.text(x, y, text, ha="center", va="center", fontsize=FS, color=INK,
            zorder=6, linespacing=1.1)


def line(pts, arrowhead=True, color=INK, lw=1.0):
    pts = np.asarray(pts, dtype=float)
    ax.plot(pts[:, 0], pts[:, 1], color=color, lw=lw, zorder=4,
            solid_joinstyle="miter")
    if arrowhead:
        ax.annotate("", xy=pts[-1], xytext=pts[-2],
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                                    shrinkA=0, shrinkB=0, mutation_scale=9),
                    zorder=4)


def label(x, y, text, color, **kw):
    ax.text(x, y, text, fontsize=FS, color=color, zorder=7, **kw)


# --- the region in which the guidance law acts --------------------------
ax.add_patch(Rectangle((C - 0.78, R3 - 0.62), W - (C - 0.78), R2 + 0.48 - (R3 - 0.62),
                       fc=SHADE, ec="none", zorder=0))
label(W - 0.04, R2 + 0.52, "Guidance law in use", GREY, ha="right",
      va="bottom", style="italic")

# --- boxes ---------------------------------------------------------------
proc(A, R1, "Simulate vertical\nascent")
IN_TXT = "Input: Choose an\ninitial kick angle\nafter vertical\nphase"
proc(B, R1, IN_TXT, fc=IO)
h_c1 = proc(C, R1, "Simulate Gravity\nTurn until\natmosphere exit")

h_a2 = proc(A, R2, "Discard the initial\nkick angle value")
h_c2 = proc(C, R2, "Calculate apogee\nof resulting orbit\nif engines stopped")

decision(A, R3, DWA, DHA, "Minimum\npropellant\nused")
h_b3 = proc(B, R3, "Try another initial\nkick angle value\nto check if it is\nthe minimum")
decision(C, R3, DWC, DHC, "$R_{apogee}$\n$=R_{desired}$")
h_d3 = proc(D, R3, "Simulate ascent\ntrajectory if\nengine continues\nuntil next\ntime step",
            w=1.18)

h_a4 = proc(A, R4, "Output: Use total\n$\\Delta v$ to calculate\ntotal propellant\nused from the\nrocket equation", fc=IO)
proc(B, R4, "Impulsive\ncircularization at\napoapsis: $\\Delta v=$\n$|v_{circ}-v_{apo}|$")
h_c4 = proc(C, R4, "Place the cut-off\nby root search so\nthe coast reaches\nthe target apoapsis")
h_r1 = box_h(IN_TXT)

# --- flow ----------------------------------------------------------------
line([[A + BW / 2, R1], [B - BW / 2, R1]])
line([[B + BW / 2, R1], [C - BW / 2, R1]])
line([[C, R1 - h_c1 / 2], [C, R2 + h_c2 / 2]])
line([[C, R2 - h_c2 / 2], [C, R3 + DHC / 2]])

# the inner guidance loop
line([[C + DWC / 2, R3], [D - 1.18 / 2, R3]], color=ACCENT)
label(D - 1.18 / 2 - 0.03, R3 + 0.06, "False", ACCENT, ha="right", va="bottom")
line([[D, R3 + h_d3 / 2], [D, R2], [C + BW / 2, R2]], color=ACCENT)

line([[C, R3 - DHC / 2], [C, R4 + h_c4 / 2]], color=GREEN)
label(C + 0.06, 0.5 * (R3 - DHC / 2 + R4 + h_c4 / 2), "True", GREEN,
      ha="left", va="center")

# the outer kick-angle loop
line([[C - BW / 2, R4], [B + BW / 2, R4]])
line([[B - BW / 2, R4], [A + BW / 2, R4]])
line([[A, R4 + h_a4 / 2], [A, R3 - DHA / 2]])

line([[A - DWA / 2, R3], [LOOP, R3], [LOOP, R2], [A - BW / 2, R2]], color=ACCENT)
label(LOOP - 0.05, 0.5 * (R2 + R3), "False", ACCENT, ha="right", va="center",
      rotation=90)
line([[A + BW / 2, R2], [B, R2]], arrowhead=False)

line([[A + DWA / 2, R3], [B - BW / 2, R3]], color=GREEN)
label(B - BW / 2 - 0.03, R3 + 0.10, "True", GREEN, ha="right", va="bottom")
line([[B, R3 + h_b3 / 2], [B, R1 - h_r1 / 2]])

save(fig, "FlowChart.png")
