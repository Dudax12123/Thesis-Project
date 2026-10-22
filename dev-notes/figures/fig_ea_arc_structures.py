"""Figure (Extended Abstract): arc structure of the optimisation architectures.

Abstract-only variant of fig_architecture_arc_structures.py (review item F-1,
2026-10-22). The thesis figure is unchanged. Here the segmented row shows the
one schedule the abstract flies, gravity turn handing over to PEG, with each
law named inside its own thrust arc instead of in a band of generic law boxes
above the row; the space that band took is removed. Every label stays at the
thesis size, printed at 16 cm.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from figstyle import use_thesis_style, save, INK, GREY, THRUST, COAST

use_thesis_style()

FS = 10.5                  # every label
W, H = 6.30, 2.96
YB = 0.20                  # bottom of the drawing (one legend row)
fig = plt.figure(figsize=(W, H - YB))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(YB, H)
ax.axis("off")

X0 = 0.05                  # lift-off
IGN = 2.05                 # second-stage ignition
NAME_X = 1.95              # right edge of the row names
SH = 0.27                  # bar height
DY = -0.34                 # every row above the segmented one moves down by the removed band


def bar(x0, x1, y, kind):
    ax.add_patch(Rectangle((x0, y), x1 - x0, SH,
                           fc=THRUST if kind == "T" else COAST,
                           ec=INK, lw=0.8, zorder=4))


def impulse(x, y):
    ax.plot([x, x], [y - 0.07, y + SH + 0.07], color=INK, lw=1.8, zorder=7)


def name(y, text):
    ax.text(NAME_X, y + SH / 2, text, fontsize=FS, color=INK, ha="right",
            va="center", linespacing=1.15)


# --- the shared first stage, drawn once -----------------------------------
Y0 = 2.92 + DY
ax.add_patch(Rectangle((X0, Y0), IGN - X0, SH + 0.04, fc="#ececec",
                       ec=INK, lw=0.8, zorder=4))
ax.text(0.5 * (X0 + IGN), Y0 + (SH + 0.04) / 2, "stage 1 (common to all)",
        ha="center", va="center", fontsize=FS, color=INK, zorder=5)
ax.text(IGN + 0.08, Y0 + (SH + 0.04) / 2, "second-stage ignition",
        ha="left", va="center", fontsize=FS, color=GREY)

# --- one row per arc structure -------------------------------------------
y = 2.40 + DY                              # brute-force kick search
name(y, "Brute-force kick search")
bar(IGN, 3.45, y, "T")
bar(3.45, 5.60, y, "C")
impulse(5.60, y)

y = 1.88 + DY                              # coast-parameter and indirect PMP
name(y, "Coast-parameter PSO\nIndirect PMP PSO")
bar(IGN, 3.35, y, "T")
bar(3.35, 5.60, y, "C")
bar(5.60, 5.92, y, "T")

y = 1.38 + DY                              # direct insertion
name(y, "Direct-insertion PSO")
bar(IGN, 5.20, y, "T")

# a rule between the single-law architectures and the segmented mode
RULE = 1.305 + DY
ax.plot([X0, W - X0], [RULE, RULE], color=INK, lw=0.9, zorder=3)

y = 0.62                                   # segmented: gravity turn -> PEG
name(y, "Segmented\nmulti-law")
HAND = 3.10                                # hand-off, drawn after ignition
bar(IGN, 3.75, y, "T")
bar(3.75, 5.60, y, "C")
bar(5.60, 5.92, y, "T")
ax.plot([HAND, HAND], [y, y + SH], color="white", lw=1.0, ls=(0, (2, 1.6)),
        zorder=6)
for x0, x1, lab in ((IGN, HAND, "gravity turn"), (HAND, 3.75, "PEG")):
    ax.text(0.5 * (x0 + x1), y + SH / 2, lab, ha="center", va="center",
            fontsize=FS, color="white", zorder=7)

# ignition line through every row
ax.plot([IGN, IGN], [0.50, Y0], color=GREY, lw=1.0, ls=(0, (3, 2.2)),
        zorder=6)

# --- legend, centred on the figure ------------------------------------------
L1 = 0.27
items = [
    ax.add_patch(Rectangle((0.00, L1), 0.30, 0.16, fc=THRUST, ec=INK, lw=0.8)),
    ax.text(0.37, L1 + 0.08, "thrust arc", fontsize=FS, color=INK, va="center"),
    ax.add_patch(Rectangle((1.25, L1), 0.30, 0.16, fc=COAST, ec=INK, lw=0.8)),
    ax.text(1.62, L1 + 0.08, "coast arc", fontsize=FS, color=INK, va="center"),
    ax.plot([2.55, 2.55], [L1 - 0.03, L1 + 0.19], color=INK, lw=1.8)[0],
    ax.text(2.65, L1 + 0.08, "impulsive circularization burn", fontsize=FS,
            color=INK, va="center"),
]
r = fig.canvas.get_renderer()
right = items[-1].get_window_extent(r).x1 / fig.dpi      # inches = data units
dx = 0.5 * (W - right)
for it in items:
    if isinstance(it, Rectangle):
        it.set_x(it.get_x() + dx)
    elif hasattr(it, "set_xdata"):
        it.set_xdata([x + dx for x in it.get_xdata()])
    else:
        it.set_x(it.get_position()[0] + dx)

save(fig, "ea_architecture_arc_structures.png")
