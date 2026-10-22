"""Figure (Extended Abstract): arc structure of the optimisation architectures.

Abstract-only variant of fig_architecture_arc_structures.py (review item F-1,
2026-10-22). The thesis figure is unchanged. It is drawn for one column of the
abstract (3.08 in), so the row names sit above their bars instead of beside
them, the stage-1 box is shortened to its name, and the bars are shorter in
proportion. The segmented row shows the one schedule the abstract flies,
gravity turn handing over to PEG, each law named inside its thrust arc. Every
label is 10 pt at the printed size.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from figstyle import use_thesis_style, save, INK, GREY, THRUST, COAST

use_thesis_style()

FS = 10.0                  # every label, printed at the column width
W = 3.08                   # one column of the abstract
SH = 0.22                  # bar height
NAME_DY = 0.05             # gap between a row name and its bar
ROW = 0.53                 # row pitch: name + bar + gap

X0 = 0.03                  # lift-off
IGN = 0.86                 # second-stage ignition
XE = 3.03                  # right end of the longest bar

# rows from the top: (name, [(x0, x1, kind)], impulse x or None)
ROWS = [
    ("Brute-force kick search", [(IGN, 1.75, "T"), (1.75, 2.85, "C")], 2.85),
    ("Coast-parameter and indirect PMP PSO",
     [(IGN, 1.70, "T"), (1.70, 2.85, "C"), (2.85, XE, "T")], None),
    ("Direct-insertion PSO", [(IGN, 2.62, "T")], None),
]
SEG_NAME = "Segmented multi-law"
HAND = 1.80                # hand-off inside the first thrust arc
SEG = [(IGN, 2.20, "T"), (2.20, 2.85, "C"), (2.85, XE, "T")]

LEG = 0.50                 # height of the two legend rows
TOP = 0.40                 # stage-1 band
H = TOP + ROW * (len(ROWS) + 1) + 0.10 + LEG
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")


def bar(x0, x1, y, kind):
    ax.add_patch(Rectangle((x0, y), x1 - x0, SH,
                           fc=THRUST if kind == "T" else COAST,
                           ec=INK, lw=0.8, zorder=4))


def impulse(x, y):
    ax.plot([x, x], [y - 0.06, y + SH + 0.06], color=INK, lw=1.8, zorder=7)


def row(y, name, arcs, imp=None):
    """A row whose bar's bottom is at y, its name above it."""
    ax.text(X0, y + SH + NAME_DY, name, fontsize=FS, color=INK, ha="left", va="bottom")
    for x0, x1, kind in arcs:
        bar(x0, x1, y, kind)
    if imp is not None:
        impulse(imp, y)


# --- the shared first stage, shortened to its name --------------------------
y_top = H - TOP + 0.06
ax.add_patch(Rectangle((X0, y_top), IGN - X0, SH + 0.04, fc="#ececec",
                       ec=INK, lw=0.8, zorder=4))
ax.text(0.5 * (X0 + IGN), y_top + (SH + 0.04) / 2, "stage 1", ha="center",
        va="center", fontsize=FS, color=INK, zorder=5)
ax.text(IGN + 0.06, y_top + (SH + 0.04) / 2, "second-stage ignition",
        ha="left", va="center", fontsize=FS, color=GREY)

# --- one row per arc structure -------------------------------------------
y = y_top
bars_y = []
for name, arcs, imp in ROWS:
    y -= ROW
    row(y, name, arcs, imp)
    bars_y.append(y)

# a rule between the single-law architectures and the segmented mode
rule = y - 0.06
ax.plot([X0, XE], [rule, rule], color=INK, lw=0.9, zorder=3)

y -= ROW + 0.04
row(y, SEG_NAME, SEG)
ax.plot([HAND, HAND], [y, y + SH], color="white", lw=1.0, ls=(0, (2, 1.6)), zorder=6)
for x0, x1, lab in ((IGN, HAND, "gravity turn"), (HAND, 2.20, "PEG")):
    ax.text(0.5 * (x0 + x1), y + SH / 2, lab, ha="center", va="center",
            fontsize=FS, color="white", zorder=7)
y_seg = y

# the ignition line is drawn across each bar only, so it never crosses a row name
for yb in bars_y + [y_seg]:
    ax.plot([IGN, IGN], [yb - 0.04, yb + SH + 0.04], color=GREY, lw=1.0,
            ls=(0, (3, 2.2)), zorder=6)

# --- legend, two rows --------------------------------------------------------
L1, L2 = 0.30, 0.06
ax.add_patch(Rectangle((X0, L1), 0.26, 0.14, fc=THRUST, ec=INK, lw=0.8))
ax.text(X0 + 0.32, L1 + 0.07, "thrust arc", fontsize=FS, color=INK, va="center")
ax.add_patch(Rectangle((1.45, L1), 0.26, 0.14, fc=COAST, ec=INK, lw=0.8))
ax.text(1.45 + 0.32, L1 + 0.07, "coast arc", fontsize=FS, color=INK, va="center")
ax.plot([X0 + 0.13, X0 + 0.13], [L2 - 0.02, L2 + 0.16], color=INK, lw=1.8)
ax.text(X0 + 0.32, L2 + 0.07, "impulsive circularization burn", fontsize=FS,
        color=INK, va="center")

save(fig, "ea_architecture_arc_structures.png")
