"""Figure (thesis, Chapter 5, Section 5.2): from the variables of the ascent to the cost.

Merges free_variables_cascade.png and optimization_loop.png. The optimizer (outer loop) proposes the
design vector x and receives the cost J(x). Inside each evaluation one ascent is flown. The variables
the ascent case fixes enter the flight unchanged (dashed box, straight to the ascent); x branches into
the variables the ascent case leaves free and those the guidance law adds; the final state goes to the
cost. Every label 10 pt at the printed width (6.3 in).
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from figstyle import use_thesis_style, save, INK

use_thesis_style()

FS = 10.0
W, H = 6.30, 3.62
B0 = 0.56
fig = plt.figure(figsize=(W, H - B0))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(B0, H)
ax.axis("off")

PROC = "#dceafa"           # optimizer and cost
SHADE = "#f0f0f0"          # the simulation region
LH = 0.175                 # line height inside a box
DASH = (0, (3, 2))


def box(x0, x1, y0, y1, lines, fc=PROC, ls="-"):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0,
                                boxstyle="round,pad=0,rounding_size=0.05",
                                fc=fc, ec=INK, lw=0.8, ls=ls, zorder=3))
    n = len(lines)
    for i, s in enumerate(lines):
        y = 0.5 * (y0 + y1) + (0.5 * (n - 1) - i) * LH
        ax.text(0.5 * (x0 + x1), y, s, ha="center", va="center",
                fontsize=FS, color=INK, zorder=4)


def arrow(xy0, xy1, ls="-"):
    ax.annotate("", xy=xy1, xytext=xy0, zorder=5,
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.9, linestyle=ls,
                                shrinkA=0, shrinkB=0, mutation_scale=9))


def line(xs, ys, ls="-"):
    ax.plot(xs, ys, color=INK, lw=0.9, ls=ls, zorder=5)


def text(x, y, s, **kw):
    kw.setdefault("ha", "center")
    kw.setdefault("va", "center")
    ax.text(x, y, s, fontsize=FS, color=INK, zorder=5, **kw)


YC = 1.65                                    # centre line of the main row
HB = 0.78
OPT = (0.04, 1.22)
REG = (1.72, 5.12)
COST = (5.62, 6.26)
RY = (0.80, 3.10)                            # the simulation region

box(OPT[0], OPT[1], YC - HB / 2, YC + HB / 2, ["Optimizer"])
box(COST[0], COST[1], YC - HB / 2, YC + HB / 2, ["Cost", r"$J(\mathbf{x})$"])

ax.add_patch(FancyBboxPatch((REG[0], RY[0]), REG[1] - REG[0], RY[1] - RY[0],
                            boxstyle="round,pad=0,rounding_size=0.06",
                            fc=SHADE, ec=INK, lw=0.8, ls=DASH, zorder=1))
text(0.5 * (REG[0] + REG[1]), RY[1] - 0.16,
     r"Inner loop: one ascent, flown with the values in $\mathbf{x}$")

BX = (2.12, 3.70)
BF = (2.31, 2.79)          # fixed by the ascent case
BA = (1.71, 2.19)          # left free by the ascent case
BB = (1.11, 1.59)          # added by the guidance law
box(*BX, *BF, ["Fixed by the", "ascent case"], fc="white", ls=DASH)
box(*BX, *BA, ["Left free by the", "ascent case"], fc="white")
box(*BX, *BB, ["Added by the", "guidance law"], fc="white")

# x enters and branches to the two sets of variables it holds
JX = 1.93
yA, yB = 0.5 * (BA[0] + BA[1]), 0.5 * (BB[0] + BB[1])
for yb in (yA, yB):
    arrow((JX, yb), (BX[0], yb))
line([JX, JX], [yB, yA])
line([OPT[1], JX], [YC, YC])
ax.plot([JX], [YC], "o", color=INK, ms=3, zorder=6)
text(0.5 * (OPT[1] + REG[0]), YC + 0.17, r"$\mathbf{x}$")

# the free and the added variables converge on the ascent; the fixed ones go straight to it
MX = 3.80
AF = (3.98, 4.96)
XF = 0.5 * (AF[0] + AF[1])
for yb in (yA, yB):
    line([BX[1], MX], [yb, yb])
line([MX, MX], [yB, yA])
ax.plot([MX], [YC], "o", color=INK, ms=3, zorder=6)
box(*AF, YC - 0.31, YC + 0.31, ["Ascent", "flown"], fc="white")
arrow((MX, YC), (AF[0], YC))
yF = 0.5 * (BF[0] + BF[1])
line([BX[1], XF], [yF, yF], ls=DASH)
arrow((XF, yF), (XF, YC + 0.31), ls=DASH)
line([AF[1], REG[1]], [YC, YC])

# the final state leaves
arrow((REG[1], YC), (COST[0], YC))
text(0.5 * (REG[1] + COST[0]), YC + 0.17, "final")
text(0.5 * (REG[1] + COST[0]), YC - 0.17, "state")

# outer loop: the cost goes back to the optimizer
OY = 3.27
xc, xo = 0.5 * (COST[0] + COST[1]), 0.5 * (OPT[0] + OPT[1])
line([xc, xc], [YC + HB / 2, OY])
line([xc, xo], [OY, OY])
arrow((xo, OY), (xo, YC + HB / 2))
text(0.5 * (xc + xo), OY + 0.17, r"Outer loop: the cost $J$ and the next $\mathbf{x}$")

save(fig, "ascent_optimization_loop.png")
