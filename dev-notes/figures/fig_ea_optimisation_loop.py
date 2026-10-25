"""Figure (Extended Abstract): how the optimisation loop and the guidance loop nest.

The optimiser (outer loop) proposes a design vector x and receives one scalar
cost J(x); it never sees the law. Inside each evaluation the simulator flies one
trajectory, and on every step the active guidance law turns the state into the
angle of attack (inner loop) until the final cut-off. Drawn for one column of
the abstract (3.08 in), every label 10 pt at the printed size.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from figstyle import use_thesis_style, save, INK, THRUST, COAST

use_thesis_style()

FS = 10.0
W, H = 3.08, 3.70
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")

PROC = "#dceafa"           # process boxes
SHADE = "#f0f0f0"          # the simulation region

OUT_LANE = 0.10            # J back to the optimiser
REG_X0, REG_X1 = 0.34, 3.02
IN_LANE = 0.50             # the state back to the law
BX0, BX1 = 0.76, 2.94      # inner boxes
XC = 0.5 * (BX0 + BX1)


def box(x0, x1, y0, y1, lines, fc=PROC):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0,
                                boxstyle="round,pad=0,rounding_size=0.05",
                                fc=fc, ec=INK, lw=0.8, zorder=3))
    n = len(lines)
    for i, s in enumerate(lines):
        y = 0.5 * (y0 + y1) + (0.5 * (n - 1) - i) * 0.175
        ax.text(0.5 * (x0 + x1), y, s, ha="center", va="center",
                fontsize=FS, color=INK, zorder=4)


def arrow(xy0, xy1):
    ax.annotate("", xy=xy1, xytext=xy0, zorder=5,
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.9,
                                shrinkA=0, shrinkB=0, mutation_scale=9))


def lane(x, y_from, x_from, y_to, x_to, label):
    """An elbow: out of a box to the left, up a lane, back in to the right."""
    ax.plot([x_from, x, x], [y_from, y_from, y_to], color=INK, lw=0.9, zorder=5)
    arrow((x, y_to), (x_to, y_to))
    ax.text(x + 0.10, 0.5 * (y_from + y_to), label, rotation=90, ha="center",
            va="center", fontsize=FS, color=INK, zorder=5)


# --- outer loop: the optimiser ------------------------------------------
OPT = (3.18, 3.60)
box(REG_X0, REG_X1, *OPT, ["Optimizer", "particle swarm or grid search"])

# --- the simulation of one trajectory -------------------------------------
REG = (0.86, 2.88)
ax.add_patch(FancyBboxPatch((REG_X0, REG[0]), REG_X1 - REG_X0, REG[1] - REG[0],
                            boxstyle="round,pad=0,rounding_size=0.06",
                            fc=SHADE, ec=INK, lw=0.8, ls=(0, (3, 2)), zorder=1))
ax.text(REG_X0 + 0.08, REG[1] - 0.06, "One trajectory, flown to cut-off",
        ha="left", va="top", fontsize=FS, color=INK, zorder=2)

GL = (2.12, 2.48)
EOM = (1.62, 1.92)
EV = (1.04, 1.40)
box(BX0, BX1, *GL, [r"Guidance law: $\alpha(t,\mathbf{y})$"], fc="white")
box(BX0, BX1, *EOM, ["Equations of motion"], fc="white")
box(BX0, BX1, *EV, ["Events: staging,", "coast, cut-off"], fc="white")
arrow((XC, GL[0]), (XC, EOM[1]))
arrow((XC, EOM[0]), (XC, EV[1]))

# inner loop: the state goes back to the law at every step
lane(IN_LANE, 0.5 * sum(EV), BX0, 0.5 * sum(GL), BX0, r"state $\mathbf{y}(t)$")

# the design vector enters at the top, the final state leaves at the bottom
arrow((XC, OPT[0]), (XC, REG[1]))
ax.text(XC + 0.06, 0.5 * (OPT[0] + REG[1]), r"$\mathbf{x}$",
        ha="left", va="center", fontsize=FS, color=INK)

COST = (0.12, 0.46)
box(REG_X0, REG_X1, *COST, [r"Cost $J(\mathbf{x})$ from the final state"])
arrow((XC, REG[0]), (XC, COST[1]))
ax.text(XC + 0.06, 0.5 * (REG[0] + COST[1]), r"$\mathbf{y}_f$",
        ha="left", va="center", fontsize=FS, color=INK)

# outer loop: the cost goes back to the optimiser
lane(OUT_LANE, 0.5 * sum(COST), REG_X0, 0.5 * sum(OPT), REG_X0, r"$J$, next $\mathbf{x}$")

save(fig, "ea_optimisation_loop.png")
