"""Figure: layered module dependency graph.

Spec: Thesis_Methodology.tex, the "% FIG-7 (TO DRAW)" block above
\\includegraphics{Figures/module_dependency_graph.png} in Chapter 5.
It replaces the former ASCII listing, and its point is the intra-layer
structure that the listing could not show.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Ellipse, Rectangle

from figstyle import (use_thesis_style, blank_axes, save, INK, GREY, FAINT,
                      THRUST, ACCENT, VIOLET)

use_thesis_style()

fig, ax = plt.subplots(figsize=(7.4, 5.3))
blank_axes(ax)
ax.set_aspect("auto")


def box(x, y, w, h, text, fc="#ffffff", ec=INK, lw=0.9, fs=6.3, color=INK,
        zorder=5):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle="round,pad=0,rounding_size=0.06",
                                fc=fc, ec=ec, lw=lw, zorder=zorder))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, color=color,
            zorder=zorder + 1, linespacing=1.35)


def arr(p0, p1, color=FAINT, lw=0.8, ls="-", rad=0.0, scale=8):
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                                linestyle=ls, shrinkA=1, shrinkB=1,
                                mutation_scale=scale,
                                connectionstyle="arc3,rad=%g" % rad),
                zorder=3)


def layer(y, label):
    ax.text(-2.30, y, label, fontsize=7.8, color=GREY, ha="left",
            va="center", style="italic")


# ---------------------------------------------------------------- L4 ----
Y4 = 6.45
layer(Y4, "entry point")
box(6.75, Y4, 1.60, 0.44, "main.py", fs=7.4)
ax.text(5.80, Y4, "dispatches to one\nsolver per run  ", fontsize=6.9,
        color=GREY, ha="right", va="center", linespacing=1.3)

box(15.60, Y4, 2.90, 0.44, "Plots/new_plot_runner.py")
box(15.60, Y4 - 1.00, 2.90, 0.44, "Plots/new_metrics/*.py")
arr((7.59, Y4), (14.11, Y4), color=FAINT, ls=(0, (3, 2.2)), lw=1.0)
ax.text(10.85, Y4 + 0.09, "result arrays only", fontsize=6.9, color=GREY,
        ha="center", va="bottom")
arr((15.60, Y4 - 0.24), (15.60, Y4 - 0.76), color=GREY)
ax.text(17.25, Y4 - 0.50, "decoupled —\nreachable from\nno solver",
        fontsize=6.9, color=GREY, ha="center", va="center", linespacing=1.35)

# ---------------------------------------------------------------- L3 ----
Y3 = 4.35
layer(Y3, "solvers")
SOLVERS = [
    (1.35, "Simulation/\nsolver.py", "brute force"),
    (4.05, "Simulation/\ndirect_pso_\nsolver.py", None),
    (6.75, "Simulation/\npso_coast_\nsolver.py", "owns GuidanceState"),
    (9.45, "Simulation/\nindirect_pso_\nsolver.py", None),
    (12.15, "Simulation/\nsegmented_guidance_\nsolver.py", None),
]
for x, name, sub in SOLVERS:
    box(x, Y3, 2.55, 0.78, name, fs=6.0)
    if sub:
        ax.text(x, Y3 - 0.45, sub, fontsize=6.7, color=GREY, ha="center",
                va="top")
    arr((6.75, Y4 - 0.24), (x, Y3 + 0.43))

# intra-layer reuse -- the arrows the ASCII listing could not show
arr((5.35, Y3), (5.45, Y3), color=ACCENT, lw=1.3, scale=10)
arr((12.15, Y3 - 0.41), (7.30, Y3 - 0.41), color=ACCENT, lw=1.3, rad=-0.30,
    scale=10)
ax.text(9.70, Y3 - 1.28, "both reuse GuidanceState and the Stage-2 ODE",
        fontsize=7.1, color=ACCENT, ha="center", va="top")

# the reference cache, hanging off the segmented solver
box(15.60, Y3, 2.90, 0.62, "Simulation/\nsegment_reference.py", fs=6.0)
arr((13.45, Y3), (14.10, Y3), color=GREY)
CY = Y3 - 1.05
ax.add_patch(Rectangle((14.90, CY - 0.15), 1.40, 0.30, fc="#f1eaf7",
                       ec="none", zorder=4))
for xe in (14.90, 16.30):
    ax.plot([xe, xe], [CY - 0.15, CY + 0.15], color=VIOLET, lw=0.9, zorder=5)
ax.add_patch(Ellipse((15.60, CY - 0.15), 1.40, 0.19, fc="#f1eaf7", ec=VIOLET,
                     lw=0.9, zorder=5))
ax.add_patch(Ellipse((15.60, CY + 0.15), 1.40, 0.19, fc="#f1eaf7", ec=VIOLET,
                     lw=0.9, zorder=6))
ax.text(16.47, CY, "cached PMP\nreference", ha="left", va="center",
        fontsize=6.6, color=VIOLET, zorder=8, linespacing=1.3)
arr((15.60, Y3 - 0.41), (15.60, CY + 0.28), color=VIOLET)

# ---------------------------------------------------------------- L2 ----
Y2 = 2.25
layer(Y2, "physics hub")
box(6.75, Y2, 4.90, 0.64, "", zorder=5)
ax.text(6.75, Y2 + 0.14, "Simulation/rocket_ascent.py", ha="center",
        va="center", fontsize=7.2, color=INK, zorder=7)
for dx, ep in ((-1.05, "run()"), (1.05, "run_stage1()")):
    ax.add_patch(FancyBboxPatch((6.75 + dx - 0.85, Y2 - 0.26), 1.70, 0.22,
                                boxstyle="round,pad=0,rounding_size=0.05",
                                fc="#eef1f6", ec=THRUST, lw=0.8, zorder=6))
    ax.text(6.75 + dx, Y2 - 0.15, ep, ha="center", va="center", fontsize=5.9,
            color=THRUST, zorder=7)
ax.text(9.40, Y2 + 0.10, "imports every\nlayer-1 module", fontsize=6.9,
        color=GREY, ha="left", va="center", linespacing=1.3)

for x, _, _ in SOLVERS:
    arr((x, Y3 - 0.41), (float(np.clip(x, 4.70, 8.80)), Y2 + 0.34))

# ---------------------------------------------------------------- L1 ----
Y1 = 0.72
layer(Y1, "leaves")
LEAVES = [
    (2.05, 3.30, "Input_File/\nsimulation_parameters.py"),
    (6.75, 4.40, "Auxiliary/\nconstants · gravity · atmosphere\n"
                 "earth_rotation · rocket_specs"),
    (11.55, 3.40, "Guidance/*.py\nthe nine law modules"),
]
for x, w, name in LEAVES:
    box(x, Y1, w, 0.72, name, fs=6.1)
    arr((float(np.clip(x, 4.90, 8.60)), Y2 - 0.34), (x, Y1 + 0.38))

ax.set_xlim(-2.65, 19.55)
ax.set_ylim(0.15, 7.05)
save(fig, "module_dependency_graph.png")
