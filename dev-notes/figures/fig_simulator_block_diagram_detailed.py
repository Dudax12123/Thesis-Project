"""Figure: the simulator as a Simulink-style block diagram (detailed version).

Drawn from the code, not from a thesis block:
- outer loop: the optimizer (pso_coast_solver / direct_pso_solver / indirect_pso_solver
  through pygmo, or the grid + Brent search) proposes x and receives J'(x)
  (compute_coast_objective: burn time + altitude/velocity/FPA penalties);
- inner loop: one ascent. The guidance law returns alpha; the forces (engine thrust/Isp
  with ambient pressure, exponential atmosphere -> q -> drag, lift = 0, gravity mu/r^2)
  and alpha enter diff_eom_base; the planar Coriolis/centrifugal rates
  (earth_rotation.planar_pseudoforce_rates) are added; solve_ivp integrates the state
  [s, r, v, gamma, m];
- events (interrupt_* functions, root-found) drive the phase sequence and the mass
  resets (staging, fairing shed by ra.shed_fairing_if_due);
- every run archives itself (Archive/run_record).

Standalone landscape drawing (14 in wide), not sized for the 6.3 in thesis text width.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, FancyBboxPatch

from figstyle import use_thesis_style, save, INK, GREY

use_thesis_style()

FS = 11.0
W, H = 14.0, 8.9
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.set_aspect("equal")
ax.axis("off")

SUBSYS = "#f2f4f7"
BLOCK = "#ffffff"
SHADOW = "#b9bec6"
TAG = "#fff6d6"
LH = 0.21


def block(x0, x1, y0, y1, lines, fc=BLOCK, bold_first=True, fs=FS, name=None,
          lw=1.0, ls="-", shadow=True):
    if shadow:
        ax.add_patch(Rectangle((x0 + 0.04, y0 - 0.04), x1 - x0, y1 - y0,
                               fc=SHADOW, ec="none", zorder=2))
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=fc, ec=INK, lw=lw,
                           ls=ls, zorder=3))
    n = len(lines)
    for i, s in enumerate(lines):
        y = 0.5 * (y0 + y1) + (0.5 * (n - 1) - i) * LH
        ax.text(0.5 * (x0 + x1), y, s, ha="center", va="center",
                fontsize=fs if (i or not bold_first) else fs + 0.5,
                fontweight="bold" if (i == 0 and bold_first) else "normal",
                color=INK, zorder=4)
    if name:
        for j, s in enumerate(name):
            ax.text(0.5 * (x0 + x1), y0 - 0.17 - j * 0.19, s, ha="center",
                    va="center", fontsize=FS - 1, color=INK, zorder=4)


def tag(x0, x1, y0, y1, s, kind="from"):
    """Simulink Goto/From tag: a flag-shaped block."""
    ym = 0.5 * (y0 + y1)
    k = 0.12
    if kind == "goto":
        pts = [(x0, ym), (x0 + k, y1), (x1, y1), (x1, y0), (x0 + k, y0)]
    else:
        pts = [(x0, y0), (x0, y1), (x1 - k, y1), (x1, ym), (x1 - k, y0)]
    ax.add_patch(Polygon(pts, closed=True, fc=TAG, ec=INK, lw=0.9, zorder=3))
    ax.text(0.5 * (x0 + x1), ym, s, ha="center", va="center", fontsize=FS - 1,
            color=INK, zorder=4)


def wire(pts, arrow=True, color=INK, ls="-", lw=1.0):
    xs, ys = zip(*pts)
    if arrow:
        ax.plot(xs[:-1], ys[:-1], color=color, lw=lw, ls=ls, zorder=5)
        ax.annotate("", xy=pts[-1], xytext=pts[-2], zorder=6,
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                                    linestyle=ls, shrinkA=0, shrinkB=0,
                                    mutation_scale=11))
    else:
        ax.plot(xs, ys, color=color, lw=lw, ls=ls, zorder=5)


def dot(x, y):
    ax.add_patch(Circle((x, y), 0.045, fc=INK, ec=INK, zorder=7))


def label(x, y, s, ha="center", va="bottom", fs=FS, color=INK, **kw):
    ax.text(x, y, s, ha=ha, va=va, fontsize=fs, color=color, zorder=8, **kw)


# ------------------------------------------------------------------ outer loop
OPT = (0.4, 2.9, 7.35, 8.25)
CFG = (3.4, 5.9, 7.35, 8.25)
TGT = (6.5, 9.2, 7.35, 8.25)
CST = (11.0, 13.6, 7.35, 8.25)
block(*OPT, ["Optimizer", "PSO (pygmo) / grid + Brent", r"min $J'(\mathbf{x})$"])
block(*CFG, ["Configuration", "simulation_parameters.py", "rocket_specs.py"],
      ls=(0, (4, 2)))
block(*TGT, ["Target orbit", r"$h^* = 500$ km,  $\gamma^* = 0$",
             r"$v^* = \sqrt{\mu/r} - \omega r\cos\varphi_0$"], fc="#eef4ee")
block(*CST, ["Cost function", r"$J' = t_{\mathrm{burn}}/T + \Sigma$ penalties",
             r"($\Delta h,\ \Delta v,\ \Delta\gamma$)"], fc="#eaf1fb")

# J' back to the optimizer, over the top
wire([(12.3, 8.25), (12.3, 8.62), (1.65, 8.62), (1.65, 8.25)])
label(7.0, 8.66, r"$J'(\mathbf{x})$")
# target -> cost, with a Goto branch
wire([(9.2, 7.8), (11.0, 7.8)])
dot(9.75, 7.8)
wire([(9.75, 7.8), (9.75, 7.4)])
tag(9.35, 10.15, 7.05, 7.4, "[target]", kind="goto")

# ------------------------------------------------------------- the subsystem
RX, RY = (0.25, 13.75), (0.35, 6.9)
ax.add_patch(FancyBboxPatch((RX[0], RY[0]), RX[1] - RX[0], RY[1] - RY[0],
                            boxstyle="round,pad=0,rounding_size=0.08",
                            fc=SUBSYS, ec=INK, lw=1.0, ls=(0, (5, 3)), zorder=1))
label(2.05, 6.58, r"Ascent simulation  —  one trajectory per evaluation of $\mathbf{x}$",
      ha="left", va="center", fs=FS + 0.5, style="italic")
# configuration selects architecture / law / vehicle
wire([(4.65, 7.35), (4.65, 6.9)], ls=(0, (3, 2)), color=GREY)
label(4.75, 7.05, "architecture, law, vehicle", ha="left", va="center",
      fs=FS - 1, color=GREY)

# design vector x: down from the optimizer, to the sequencer and to the law
wire([(1.65, 7.35), (1.65, 6.05), (2.9, 6.05)])
dot(1.65, 6.05)
wire([(1.65, 6.05), (1.65, 4.55), (3.0, 4.55)])
label(1.72, 7.0, r"$\mathbf{x}$", ha="left", va="center", fs=FS + 2)
label(2.27, 6.1, r"$\gamma_p,\ \Delta t_c$")
label(2.05, 4.6, r"$\theta_0, \theta_f$")

# phase sequencer and event detection
SEQ = (2.9, 6.3, 5.45, 6.4)
EVT = (7.2, 10.7, 5.45, 6.4)
block(*SEQ, ["Phase sequencer",
             r"S1 burn $\to$ kick $\to$ MECO $\to$ separation",
             r"$\to$ arc 1 $\to$ coast $\Delta t_c$ $\to$ arc 2"])
block(*EVT, ["Event detection (zero crossings)",
             r"MECO $m = m_{bo}$  ·  fairing $h = 65$ km",
             r"SECO / insertion  ·  crash $r = R_E$"])
wire([(7.2, 5.92), (6.3, 5.92)])
label(6.75, 5.97, "events")
tag(5.4, 6.2, 5.0, 5.27, "[phase]", kind="goto")
wire([(5.8, 5.45), (5.8, 5.27)])

# guidance law
GDN = (3.0, 6.2, 3.85, 4.8)
block(*GDN, [r"Guidance law  $\to\ \alpha = \theta - \gamma$",
             "gravity turn, linear/bilinear tangent,",
             "Apollo, CPR, PEG, PEG-new,",
             "exp. shooting, indirect PMP"])
wire([(4.55, 5.45), (4.55, 4.8)])
label(4.62, 5.08, "arc restart", ha="left", va="center", fs=FS - 1)
tag(3.05, 3.85, 5.0, 5.27, "[target]", kind="from")
wire([(3.45, 5.0), (3.45, 4.8)])

# forces subsystem
FRC = (3.0, 6.2, 1.2, 3.4)
ax.add_patch(Rectangle((FRC[0] + 0.04, FRC[2] - 0.04), FRC[1] - FRC[0],
                       FRC[3] - FRC[2], fc=SHADOW, ec="none", zorder=2))
ax.add_patch(Rectangle((FRC[0], FRC[2]), FRC[1] - FRC[0], FRC[3] - FRC[2],
                       fc="#fbfbfb", ec=INK, lw=1.0, zorder=3))
label(4.6, 3.2, "Forces", va="center", fs=FS + 0.5, fontweight="bold")
for (y0, y1, s) in [(2.6, 3.0, r"Engine: $F_T,\ I_{sp}$ ($p_a$, stage)"),
                    (2.05, 2.45, r"Atmosphere $\to q \to$ drag $F_D$ ($L = 0$)"),
                    (1.45, 1.85, r"Gravity: $g = \mu / r^2$")]:
    block(3.35, 5.95, y0, y1, [s], bold_first=False, fs=FS - 0.5, shadow=False,
          lw=0.8)
tag(5.15, 5.95, 3.5, 3.72, "[phase]", kind="from")
wire([(5.55, 3.5), (5.55, 3.4)])

# mux
MUX = (6.75, 6.87, 1.4, 4.6)
ax.add_patch(Rectangle((MUX[0], MUX[2]), MUX[1] - MUX[0], MUX[3] - MUX[2],
                       fc=INK, ec=INK, zorder=4))
for y, s in [(4.3, r"$\alpha$"), (2.9, r"$F_T$"), (2.7, r"$I_{sp}$"),
             (2.25, r"$F_D$"), (1.65, r"$g$")]:
    x0 = 6.2 if y == 4.3 else 5.95
    wire([(x0, y), (MUX[0], y)])
    if y != 2.7:
        label(6.47, y + 0.03, s, fs=FS - 0.5)
    else:
        label(6.47, y - 0.05, s, va="top", fs=FS - 0.5)

# equations of motion, sum, pseudo-forces, integrator
EOM = (7.4, 9.35, 2.35, 3.65)
block(*EOM, ["Equations of motion",
             r"$\dot s,\ \dot r,\ \dot v,\ \dot\gamma,\ \dot m$",
             "(diff_eom_base)"])
wire([(MUX[1], 3.0), (EOM[0], 3.0)])
SX, SY, SR = 9.9, 3.0, 0.2
ax.add_patch(Circle((SX, SY), SR, fc=BLOCK, ec=INK, lw=1.0, zorder=3))
label(SX - 0.09, SY + 0.0, "+", va="center", fs=FS - 1)
label(SX, SY - 0.1, "+", va="center", fs=FS - 1)
wire([(EOM[1], 3.0), (SX - SR, 3.0)])
label(9.62, 3.05, r"$\dot{\mathbf{y}}$")
PSF = (8.95, 10.85, 1.2, 1.95)
block(*PSF, ["Pseudo-forces", "Coriolis + centrifugal",
             r"$\Omega = \omega\cos\varphi_0$ (rotation on)"], fs=FS - 0.5)
wire([(SX, PSF[3]), (SX, SY - SR)])
INT = (10.5, 11.3, 2.6, 3.4)
block(*INT, [r"$\frac{1}{s}$"], bold_first=False, fs=FS + 6,
      name=["Integrator", "solve_ivp (RK45)"])
wire([(SX + SR, 3.0), (INT[0], 3.0)])
tag(10.5, 11.3, 3.75, 4.0, "[phase]", kind="from")
wire([(10.9, 3.75), (10.9, 3.4)])
label(10.9, 4.08, "reset: staging, fairing", fs=FS - 1)

# state out: node to events, cost, archive and the feedback bus
wire([(INT[1], 3.0), (12.3, 3.0)], arrow=False)
dot(11.85, 3.0)
dot(12.3, 3.0)
wire([(11.85, 3.0), (11.85, 5.92), (EVT[1], 5.92)])
label(11.3, 5.97, r"$\mathbf{y}(t)$")
wire([(12.3, 3.0), (12.3, 7.35)])
label(12.4, 4.6, r"$\mathbf{y}(t_f)$", ha="left", va="center")
ARC = (12.05, 13.6, 1.05, 1.9)
block(*ARC, ["Run archive", ".npz · .json · .manifest"], fc="#f6f0fa")
wire([(12.3, 3.0), (12.3, 1.9)])
label(12.37, 2.45, "trajectory", ha="left", va="center", fs=FS - 1)

# feedback bus
wire([(11.85, 3.0), (11.85, 0.75), (2.45, 0.75), (2.45, 4.1), (3.0, 4.1)])
dot(2.45, 2.3)
wire([(2.45, 2.3), (3.0, 2.3)])
dot(SX, 0.75)
wire([(SX, 0.75), (SX, PSF[2])])
label(6.2, 0.55, r"state feedback  $\mathbf{y}(t) = [\,s,\ r,\ v,\ \gamma,\ m\,]$",
      va="center")

save(fig, "simulator_block_diagram_detailed.png")
