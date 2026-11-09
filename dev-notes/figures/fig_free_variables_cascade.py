"""Figure (thesis, Chapter 5, Section 5.2): from the free variables to the design vector.

General version: no variable is named, since Section 5.1 lists them. Three boxes
with the two operations on the arrows: the architecture fixes some variables
(which leave the search), and the guidance law adds its own. Every label 10 pt
at the printed width (6.3 in).
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from figstyle import use_thesis_style, save, INK

use_thesis_style()

FS = 10.0
W, H = 6.30, 1.95
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")

LH = 0.175
YC, HB = 1.25, 0.84
BW = 1.30
X = [0.03, 2.50, 4.97]
FC = ["white", "white", "#e3f0e8"]


def box(x0, w, yc, h, lines, fc, ls="-", ec=INK):
    ax.add_patch(FancyBboxPatch((x0, yc - h / 2), w, h,
                                boxstyle="round,pad=0,rounding_size=0.05",
                                fc=fc, ec=ec, lw=0.8, ls=ls, zorder=3))
    n = len(lines)
    for i, s in enumerate(lines):
        ax.text(x0 + w / 2, yc + (0.5 * (n - 1) - i) * LH, s, ha="center",
                va="center", fontsize=FS, color=INK, zorder=4)


def arrow(p0, p1, ls="-"):
    ax.annotate("", xy=p1, xytext=p0, zorder=5,
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.9, ls=ls,
                                shrinkA=0, shrinkB=0, mutation_scale=9))


box(X[0], BW, YC, HB, ["Variables of", "the ascent"], FC[0])
box(X[1], BW, YC, HB, ["Variables", "left free"], FC[1])
box(X[2], BW, YC, HB, ["Design vector", r"$\mathbf{x}$"], FC[2])

for i, lines in enumerate((["The architecture", "fixes some"],
                           ["The guidance law", "adds its own"])):
    x0, x1 = X[i] + BW, X[i + 1]
    arrow((x0 + 0.04, YC), (x1 - 0.04, YC))
    xm = 0.5 * (x0 + x1)
    for j, s in enumerate(lines):
        ax.text(xm, YC + 0.20 + (1 - j) * LH + 0.04, s, ha="center",
                va="center", fontsize=FS, color=INK, zorder=5)

# the fixed variables leave the search
xm = 0.5 * (X[0] + BW + X[1])
arrow((xm, YC - 0.02), (xm, 0.50), ls=(0, (3, 2)))
box(xm - 0.80, 1.60, 0.31, 0.34, ["Fixed variables"], "white", ls=(0, (3, 2)))
save(fig, "free_variables_cascade.png")
