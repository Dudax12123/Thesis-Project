"""Figure (thesis, Chapter 4): arc structure of the optimization architectures.

Compact variant of fig_architecture_arc_structures.py, taken from
fig_ea_arc_structures.py (C3 review, 2026-10-28). Same layout; it is drawn for one column of the
abstract (3.08 in), so the row names sit above their bars instead of beside
them, the stage-1 box opens every row (no separate stage-1 band), and the bars are
shorter in proportion. The segmented row keeps the generic three-law schedule of
the earlier thesis figure: law 1, law 2 and law 3 named inside its thrust arc. Every
label is 10 pt at the printed size. Review AG-4 (2026-10-24): the row names are
the abstract's architecture names, and the legend is one row.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from figstyle import use_thesis_style, save, INK, THRUST, COAST

STAGE1 = "#ececec"         # the first stage, common to every architecture

use_thesis_style()

FS = 10.0                  # every label, printed at the column width
W = 6.30                   # the thesis text width
K = W / 3.08               # the abstract layout stretched to it, labels unchanged
SH = 0.22                  # bar height
NAME_DY = 0.05             # gap between a row name and its bar
ROW = 0.53                 # row pitch: name + bar + gap

X0 = 0.03                  # lift-off
IGN = 0.86 * K                 # second-stage ignition
XE = 3.03 * K                  # right end of the longest bar

# rows from the top: (name, [(x0, x1, kind)], impulse x or None)
ROWS = [
    ("Apogee check", [(IGN, 1.75 * K, "T"), (1.75 * K, XE, "C")], XE),
    ("Coast parameter and indirect optimal control",
     [(IGN, 1.70 * K, "T"), (1.70 * K, 2.85 * K, "C"), (2.85 * K, XE, "T")], None),
    ("Direct insertion", [(IGN, XE, "T")], None),
]
SEG_NAME = "Segmented guidance"
HANDS = (1.32 * K, 1.76 * K)       # the two hand-offs inside the first thrust arc
SEG = [(IGN, 2.20 * K, "T"), (2.20 * K, 2.85 * K, "C"), (2.85 * K, XE, "T")]

LEG = 0.26                 # height of the one legend row
FIRST = 0.46               # top of the figure to the first bar's bottom
H = FIRST + ROW * len(ROWS) + 0.08 + LEG + 0.04
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
    # the shared first stage opens every row, so no separate stage-1 band is needed
    ax.add_patch(Rectangle((X0, y), IGN - X0, SH, fc=STAGE1, ec=INK, lw=0.8, zorder=4))
    ax.text(0.5 * (X0 + IGN), y + SH / 2, "stage 1", ha="center", va="center",
            fontsize=FS, color=INK, zorder=5)
    for x0, x1, kind in arcs:
        bar(x0, x1, y, kind)
    if imp is not None:
        impulse(imp, y)


y_top = H - FIRST + ROW     # the loop steps down one row before drawing

# --- one row per arc structure -------------------------------------------
y = y_top
bars_y = []
for name, arcs, imp in ROWS:
    y -= ROW
    row(y, name, arcs, imp)
    bars_y.append(y)

y -= ROW
row(y, SEG_NAME, SEG)
for xh in HANDS:
    ax.plot([xh, xh], [y, y + SH], color="white", lw=1.0, ls=(0, (2, 1.6)), zorder=6)
edges = (IGN,) + HANDS + (2.20 * K,)
for k, (x0, x1) in enumerate(zip(edges[:-1], edges[1:]), start=1):
    ax.text(0.5 * (x0 + x1), y + SH / 2, "law %d" % k, ha="center", va="center",
            fontsize=FS, color="white", zorder=7)
y_seg = y


# --- legend, one row (review AG-4, 2026-10-24) -------------------------------
L1 = 0.06
ax.add_patch(Rectangle((X0, L1), 0.20, 0.14, fc=THRUST, ec=INK, lw=0.8))
ax.text(X0 + 0.25, L1 + 0.07, "thrust", fontsize=FS, color=INK, va="center")
ax.add_patch(Rectangle((1.35, L1), 0.20, 0.14, fc=COAST, ec=INK, lw=0.8))
ax.text(1.35 + 0.25, L1 + 0.07, "coast", fontsize=FS, color=INK, va="center")
ax.plot([2.55, 2.55], [L1 - 0.02, L1 + 0.16], color=INK, lw=1.8)
ax.text(2.63, L1 + 0.07, "impulsive circularization burn", fontsize=FS, color=INK, va="center")

save(fig, "architecture_arc_structures.png")
