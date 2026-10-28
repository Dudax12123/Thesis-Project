"""Figure (thesis, Chapter 4): the outer optimization loop and the inner simulation loop.

Horizontal counterpart of the Extended Abstract's fig_ea_optimisation_loop.py. The
optimizer (outer loop) proposes a design vector x and receives one scalar cost
J(x); it never sees the guidance law. Inside each evaluation the simulator flies
one trajectory: at every step the active law turns the state into the angle of
attack, the equations of motion advance the state, and the events (staging,
coast, cut-off) fall between integrations, until the final cut-off. The legend
line says what x and J are. Every label 10 pt at the printed width (6.3 in).
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from figstyle import use_thesis_style, save, INK

use_thesis_style()

FS = 10.0
W, H = 6.30, 2.95
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")

PROC = "#dceafa"           # optimizer and cost
SHADE = "#f0f0f0"          # the simulation region
LH = 0.175                 # line height inside a box


def box(x0, x1, y0, y1, lines, fc=PROC):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0,
                                boxstyle="round,pad=0,rounding_size=0.05",
                                fc=fc, ec=INK, lw=0.8, zorder=3))
    n = len(lines)
    for i, s in enumerate(lines):
        y = 0.5 * (y0 + y1) + (0.5 * (n - 1) - i) * LH
        ax.text(0.5 * (x0 + x1), y, s, ha="center", va="center",
                fontsize=FS, color=INK, zorder=4)


def arrow(xy0, xy1):
    ax.annotate("", xy=xy1, xytext=xy0, zorder=5,
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.9,
                                shrinkA=0, shrinkB=0, mutation_scale=9))


def text(x, y, s, **kw):
    kw.setdefault("ha", "center")
    kw.setdefault("va", "center")
    ax.text(x, y, s, fontsize=FS, color=INK, zorder=5, **kw)


YC = 1.40                                    # centre line of the main row
HB = 0.78                                    # height of the outer boxes
OPT = (0.04, 1.22)
REG = (1.72, 5.12)
COST = (5.62, 6.26)
RY = (0.50, 2.22)                            # the simulation region

box(OPT[0], OPT[1], YC - HB / 2, YC + HB / 2,
    ["Optimizer", "(swarm or", "grid search)"])
box(COST[0], COST[1], YC - HB / 2, YC + HB / 2, ["Cost", r"$J(\mathbf{x})$"])

# the simulation of one trajectory
ax.add_patch(FancyBboxPatch((REG[0], RY[0]), REG[1] - REG[0], RY[1] - RY[0],
                            boxstyle="round,pad=0,rounding_size=0.06",
                            fc=SHADE, ec=INK, lw=0.8, ls=(0, (3, 2)), zorder=1))
text(0.5 * (REG[0] + REG[1]), RY[1] - 0.15, "Inner loop: one trajectory, flown to cut-off")

IB = (YC - 0.31, YC + 0.31)                  # inner boxes
GL = (1.84, 2.72)
EOM = (2.88, 3.72)
EV = (3.88, 5.00)
box(*GL, *IB, ["Guidance law", r"sets $\alpha$"], fc="white")
box(*EOM, *IB, ["Equations", "of motion"], fc="white")
box(*EV, *IB, ["Events: staging,", "coast, cut-off"], fc="white")
arrow((GL[1], YC), (EOM[0], YC))
arrow((EOM[1], YC), (EV[0], YC))

# inner loop: the state goes back to the law at every step
LY = 0.80
xs, xe = 0.5 * (EV[0] + EV[1]), 0.5 * (GL[0] + GL[1])
ax.plot([xs, xs], [IB[0], LY], color=INK, lw=0.9, zorder=5)
ax.plot([xs, xe], [LY, LY], color=INK, lw=0.9, zorder=5)
arrow((xe, LY), (xe, IB[0]))
text(0.5 * (xs + xe), LY - 0.17, "state, at every step")

# the design vector enters, the final state leaves
arrow((OPT[1], YC), (REG[0], YC))
text(0.5 * (OPT[1] + REG[0]), YC + 0.17, r"$\mathbf{x}$")
arrow((REG[1], YC), (COST[0], YC))
text(0.5 * (REG[1] + COST[0]), YC + 0.17, "final")
text(0.5 * (REG[1] + COST[0]), YC - 0.17, "state")

# outer loop: the cost goes back to the optimizer
OY = 2.55
xc, xo = 0.5 * (COST[0] + COST[1]), 0.5 * (OPT[0] + OPT[1])
ax.plot([xc, xc], [YC + HB / 2, OY], color=INK, lw=0.9, zorder=5)
ax.plot([xc, xo], [OY, OY], color=INK, lw=0.9, zorder=5)
arrow((xo, OY), (xo, YC + HB / 2))
text(0.5 * (xc + xo), OY + 0.17,
     r"Outer loop: the cost $J$ and the next $\mathbf{x}$")

# what x and J are
text(0.06, 0.28, r"$\mathbf{x}$: kick angle, coast and burn timing, steering constants or costates",
     ha="left")
text(0.06, 0.08, r"$J$: propellant at insertion, with penalties on the terminal errors",
     ha="left")

save(fig, "optimization_loop.png")
