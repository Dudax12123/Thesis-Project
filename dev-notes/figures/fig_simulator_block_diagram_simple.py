"""Figure: the simulator as a Simulink-style block diagram (simplified version).

Same loop as fig_simulator_block_diagram_detailed.py, reduced to its essentials:
optimizer -> x -> guidance law -> alpha -> vehicle dynamics -> 1/s -> state -> cost -> J'
back to the optimizer, with the state fed back to the law and the dynamics. The
phase logic is part of the guidance block: the law flown depends on the ascent phase.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch

from figstyle import use_thesis_style, save, INK

use_thesis_style()

FS = 13.0
W, H = 11.0, 5.6
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.set_aspect("equal")
ax.axis("off")

SHADOW = "#b9bec6"


def block(x0, x1, y0, y1, lines, fc="#ffffff", fs=FS, name=None):
    ax.add_patch(Rectangle((x0 + 0.04, y0 - 0.04), x1 - x0, y1 - y0,
                           fc=SHADOW, ec="none", zorder=2))
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=fc, ec=INK, lw=1.1,
                           zorder=3))
    n = len(lines)
    for i, s in enumerate(lines):
        y = 0.5 * (y0 + y1) + (0.5 * (n - 1) - i) * 0.25
        ax.text(0.5 * (x0 + x1), y, s, ha="center", va="center",
                fontsize=fs if i else fs + 0.5,
                fontweight="bold" if (i == 0 and n > 1) else "normal",
                color=INK, zorder=4)
    if name:
        ax.text(0.5 * (x0 + x1), y0 - 0.22, name, ha="center", va="center",
                fontsize=FS - 1, color=INK, zorder=4)


def wire(pts, arrow=True):
    xs, ys = zip(*pts)
    ax.plot(xs[:-1] if arrow else xs, ys[:-1] if arrow else ys, color=INK,
            lw=1.1, zorder=5)
    if arrow:
        ax.annotate("", xy=pts[-1], xytext=pts[-2], zorder=6,
                    arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.1,
                                    shrinkA=0, shrinkB=0, mutation_scale=13))


def dot(x, y):
    ax.add_patch(Circle((x, y), 0.05, fc=INK, ec=INK, zorder=7))


def label(x, y, s, ha="center", va="bottom", fs=FS, **kw):
    ax.text(x, y, s, ha=ha, va=va, fontsize=fs, color=INK, zorder=8, **kw)


# outer loop
block(0.4, 2.4, 4.3, 5.0, ["Optimizer"], fc="#eaf1fb")
block(8.6, 10.6, 4.3, 5.0, ["Cost  $J'$"], fc="#eaf1fb")
wire([(9.6, 5.0), (9.6, 5.35), (1.4, 5.35), (1.4, 5.0)])
label(5.5, 5.39, r"$J'(\mathbf{x})$")

# the simulation
ax.add_patch(FancyBboxPatch((0.25, 0.3), 10.5, 3.65,
                            boxstyle="round,pad=0,rounding_size=0.08",
                            fc="#f2f4f7", ec=INK, lw=1.0, ls=(0, (5, 3)), zorder=1))
label(1.6, 3.72, "Ascent simulation", ha="left", va="center", style="italic")

GD = (1.7, 3.5, 1.85, 2.75)
DY = (4.3, 6.4, 1.85, 2.75)
IN = (7.2, 7.9, 1.95, 2.65)
block(*GD, ["Guidance law", "per ascent phase"], fs=FS - 1)
block(*DY, ["Vehicle dynamics"])
block(*IN, [r"$\frac{1}{s}$"], fs=FS + 6, name="Integrator")

# design vector
wire([(1.4, 4.3), (1.4, 2.55), (GD[0], 2.55)])
label(1.5, 4.13, r"$\mathbf{x}$", ha="left", va="center", fs=FS + 2)

# forward path
wire([(GD[1], 2.3), (DY[0], 2.3)])
label(3.9, 2.35, r"$\alpha$")
wire([(DY[1], 2.3), (IN[0], 2.3)])
label(6.8, 2.35, r"$\dot{\mathbf{x}}$")
wire([(IN[1], 2.3), (9.6, 2.3)], arrow=False)
dot(8.5, 2.3)
dot(9.6, 2.3)
wire([(9.6, 2.3), (9.6, 4.3)])
label(9.7, 3.3, r"$\mathbf{x}(t_f)$", ha="left", va="center")
label(9.05, 2.35, r"$\mathbf{x}(t)$")

# state feedback
wire([(8.5, 2.3), (8.5, 0.75), (1.0, 0.75), (1.0, 2.05), (GD[0], 2.05)])
dot(5.35, 0.75)
wire([(5.35, 0.75), (5.35, DY[2])])
label(3.2, 0.82, "state feedback (closed loop)", fs=FS - 1)

save(fig, "simulator_block_diagram_simple.png")
