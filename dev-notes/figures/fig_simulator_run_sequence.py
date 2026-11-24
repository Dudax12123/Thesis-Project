"""Figure: one run of the simulator, start to end, as a Simulink/Stateflow-style diagram.

No configuration choices are drawn: one run of any Ascent Case, read from the code.
- Top: the run. The optimizer proposes x, one ascent is flown, the cost J'(x) returns;
  at convergence the best x* is flown once more on the fine output grid and archived
  (main.py, Archive/run_record).
- Middle: the phase sequence of one ascent as a Stateflow chart
  (rocket_ascent._fly_stage1 + _run_stage1a_with_kick for Stage 1, then
  pso_coast_solver.run_pso_coast_trajectory for Stage 2): vertical rise, kick as an
  instantaneous gamma jump at TIME_TO_START_KICK, gravity-turn burn with the fairing
  shed at 65 km, root-found MECO, 3 s coast to separation, 5 s to ignition, burn 1,
  coast, burn 2. A ground crossing ends any phase with the crash penalty.
- Bottom: the integration loop every phase runs until its transition fires.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, FancyBboxPatch

from figstyle import use_thesis_style, save, INK, GREY

use_thesis_style()

FS = 11.0
W, H = 14.0, 11.1
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.set_aspect("equal")
ax.axis("off")

SUBSYS = "#f2f4f7"
SHADOW = "#b9bec6"
TAG = "#fff6d6"
STATE = "#fffdf5"
LH = 0.21


def _text_lines(x0, x1, y0, y1, lines, fs):
    n = len(lines)
    for i, s in enumerate(lines):
        y = 0.5 * (y0 + y1) + (0.5 * (n - 1) - i) * LH
        ax.text(0.5 * (x0 + x1), y, s, ha="center", va="center",
                fontsize=fs if i else fs + 0.5,
                fontweight="bold" if (i == 0 and n > 1) else "normal",
                color=INK, zorder=4)


def block(x0, x1, y0, y1, lines, fc="#ffffff", fs=FS, name=None):
    ax.add_patch(Rectangle((x0 + 0.04, y0 - 0.04), x1 - x0, y1 - y0,
                           fc=SHADOW, ec="none", zorder=2))
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=fc, ec=INK, lw=1.0,
                           zorder=3))
    _text_lines(x0, x1, y0, y1, lines, fs)
    if name:
        ax.text(0.5 * (x0 + x1), y0 - 0.17, name, ha="center", va="center",
                fontsize=FS - 1, color=INK, zorder=4)


def state(x0, x1, y0, y1, lines, fc=STATE, ls="-"):
    """Stateflow state: rounded box."""
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0,
                                boxstyle="round,pad=0,rounding_size=0.14",
                                fc=fc, ec=INK, lw=1.1, ls=ls, zorder=3))
    _text_lines(x0, x1, y0, y1, lines, FS)


def terminal(x0, x1, y0, y1, s):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0,
                                boxstyle=f"round,pad=0,rounding_size={0.5 * (y1 - y0)}",
                                fc="#e9ecef", ec=INK, lw=1.0, zorder=3))
    ax.text(0.5 * (x0 + x1), 0.5 * (y0 + y1), s, ha="center", va="center",
            fontsize=FS, fontweight="bold", color=INK, zorder=4)


def tag(x0, x1, y0, y1, s, kind="from"):
    """Simulink Goto/From tag."""
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
                                    shrinkA=0, shrinkB=0, mutation_scale=11))
    else:
        ax.plot(xs, ys, color=color, lw=lw, ls=ls, zorder=5)


def dot(x, y, r=0.045):
    ax.add_patch(Circle((x, y), r, fc=INK, ec=INK, zorder=7))


def label(x, y, s, ha="center", va="bottom", fs=FS, color=INK, **kw):
    ax.text(x, y, s, ha=ha, va=va, fontsize=fs, color=color, zorder=8, **kw)


def transition(xa, xb, y, cond, action=None):
    """Horizontal Stateflow transition from xa to xb: [condition] above, {action} below."""
    wire([(xa, y), (xb, y)])
    xm = 0.5 * (xa + xb)
    label(xm, y + 0.06, cond, fs=FS - 1)
    if action:
        label(xm, y - 0.06, action, va="top", fs=FS - 1)


# =================================================================== the run
COST = (0.4, 2.4, 9.3, 10.1)
OPT = (3.0, 5.4, 9.3, 10.1)
FIN = (6.9, 9.3, 9.3, 10.1)
ARC = (9.9, 12.0, 9.3, 10.1)
block(*COST, ["Cost  $J'$", "burn time + penalties",
              r"($\Delta h,\ \Delta v,\ \Delta\gamma$)"], fc="#eaf1fb")
block(*OPT, ["Optimizer", r"proposes the next $\mathbf{x}$,", "keeps the best"],
      fc="#eaf1fb")
block(*FIN, ["Final flight", r"$\mathbf{x}^*$ through the same",
             r"simulation, output every 0.01 s"])
block(*ARC, ["Archive and plots", ".npz · .json · .manifest", "figures, report"],
      fc="#f6f0fa")
terminal(3.7, 4.7, 10.45, 10.8, "Start")
terminal(12.55, 13.55, 9.5, 9.9, "End")

wire([(4.2, 10.45), (4.2, 10.1)])
wire([(COST[1], 9.7), (OPT[0], 9.7)])
label(2.7, 9.75, r"$J'(\mathbf{x})$")
wire([(OPT[1], 9.7), (FIN[0], 9.7)])
label(6.15, 9.75, "converged", fs=FS - 1)
label(6.15, 9.65, r"$\mathbf{x}^*$", va="top")
wire([(FIN[1], 9.7), (ARC[0], 9.7)])
wire([(ARC[1], 9.7), (12.55, 9.7)])

# x down into the chart; x(t_f) up to the cost
wire([(4.2, 9.3), (4.2, 8.85)])
label(4.3, 9.07, r"$\mathbf{x}$: kick $\gamma_p$, burn split, coast $\Delta t_c$, …",
      ha="left", va="center", fs=FS - 0.5)

# ============================================================ phase sequence
BX, BY = (0.25, 13.75), (4.5, 8.85)
ax.add_patch(FancyBboxPatch((BX[0], BY[0]), BX[1] - BX[0], BY[1] - BY[0],
                            boxstyle="round,pad=0,rounding_size=0.08",
                            fc=SUBSYS, ec=INK, lw=1.0, ls=(0, (5, 3)), zorder=1))
label(1.0, 8.6, r"Ascent simulation  —  phase sequence of one trajectory",
      ha="left", va="center", fs=FS + 0.5, style="italic")
label(13.55, 8.6, "Stage 1", ha="right", va="center", fs=FS, fontweight="bold")
label(13.55, 4.72, "Stage 2", ha="right", va="center", fs=FS, fontweight="bold")

R1 = (7.1, 7.9)
R2 = (4.95, 5.75)
y1 = 0.5 * sum(R1)
y2 = 0.5 * sum(R2)
S1 = (1.0, 2.75)
S2 = (3.85, 6.05)
S3 = (7.25, 9.2)
S4 = (10.4, 12.5)

# row 1: Stage 1, left to right
dot(1.88, 8.3, r=0.06)
wire([(1.88, 8.3), (1.88, R1[1])])
state(*S1, *R1, ["Vertical rise", r"$\gamma = 90^\circ$, Stage 1 on"])
transition(S1[1], S2[0], y1, r"$[t = 7.5$ s$]$", r"$\{\gamma \leftarrow \gamma - \gamma_p\}$")
label(0.5 * (S1[1] + S2[0]), y1 + 0.3, "kick", fs=FS - 1, color=GREY)
state(*S2, *R1, ["Gravity turn", r"Stage 1 burn, $\alpha = 0$"])
# fairing: self-loop below the state
wire([(4.45, R1[0]), (4.45, 6.8), (5.45, 6.8), (5.45, R1[0])])
label(4.95, 6.72, r"fairing  $[h = 65$ km$]$  $\{m \leftarrow m - m_F\}$",
      va="top", fs=FS - 1)
transition(S2[1], S3[0], y1, r"$[m = m_{bo}]$", "MECO")
state(*S3, *R1, ["Coast", "engine off, 3 s"])
transition(S3[1], S4[0], y1, r"$[\Delta t = 3$ s$]$", r"$\{m \leftarrow m - m_{s,1}\}$")
label(0.5 * (S3[1] + S4[0]), y1 + 0.3, "separation", fs=FS - 1, color=GREY)
state(*S4, *R1, ["Coast", "engine off, 5 s"])

# down to Stage 2
xd = 0.5 * sum(S4)
wire([(xd, R1[0]), (xd, R2[1])])
label(xd + 0.1, 6.48, r"$[\Delta t = 5$ s$]$", ha="left", va="center", fs=FS - 1)
label(xd + 0.1, 6.24, "ignition", ha="left", va="center", fs=FS - 1)

# row 2: Stage 2, right to left
state(*S4, *R2, ["Burn 1", r"Stage 2, $\alpha$ from the law"])
transition(S4[0], S3[1], y2, r"$[t = t_{c1}]$", "cutoff")
state(*S3, *R2, ["Coast", r"engine off, $\Delta t_c$"])
transition(S3[0], S2[1], y2, r"$[\Delta t = \Delta t_c]$", "{restart law}")
state(*S2, *R2, ["Burn 2", r"Stage 2, $\alpha$ from the law"])
transition(S2[0], S1[1] - 0.15, y2, r"$[t = t_f]$", "cutoff")
# outport x(t_f)
OUT = (1.25, 2.6, 5.15, 5.55)
ax.add_patch(FancyBboxPatch((OUT[0], OUT[2]), OUT[1] - OUT[0], OUT[3] - OUT[2],
                            boxstyle="round,pad=0,rounding_size=0.12",
                            fc="#ffffff", ec=INK, lw=1.0, zorder=3))
label(0.5 * (OUT[0] + OUT[1]), y2, r"$\mathbf{y}(t_f)$", va="center")
wire([(OUT[0], y2), (0.75, y2), (0.75, COST[2])])

label(3.85, 4.72, r"any phase:  $[r = R_E]$  $\to$  crash, trajectory stops,"
      r"  $J'$ = crash penalty", ha="left", va="center", fs=FS - 1,
      style="italic")

# ========================================================== integration loop
CX, CY = (0.25, 13.75), (0.3, 4.25)
ax.add_patch(FancyBboxPatch((CX[0], CY[0]), CX[1] - CX[0], CY[1] - CY[0],
                            boxstyle="round,pad=0,rounding_size=0.08",
                            fc=SUBSYS, ec=INK, lw=1.0, ls=(0, (5, 3)), zorder=1))
label(0.5, 4.0, "Integration loop  —  runs inside every phase until its transition fires",
      ha="left", va="center", fs=FS + 0.5, style="italic")

GD = (1.9, 4.4, 2.75, 3.45)
FR = (1.9, 4.4, 1.15, 2.3)
block(*GD, ["Guidance", r"Stage 1: $\alpha = 0$", r"burns: law $\to\ \alpha$"])
block(*FR, ["Forces", r"thrust $F_T$, $I_{sp}(p_a)$ if on",
            r"drag $F_D(q)$, gravity $\mu/r^2$"])
tag(2.75, 3.55, 3.55, 3.8, "[phase]", kind="from")
wire([(3.15, 3.55), (3.15, GD[3])])
tag(2.75, 3.55, 0.7, 0.95, "[phase]", kind="from")
wire([(3.15, 0.95), (3.15, FR[2])])

EOM = (5.2, 7.0, 1.55, 2.65)
block(*EOM, ["Equations of motion",
             r"$\dot s,\ \dot r,\ \dot v,\ \dot\gamma,\ \dot m$"])
wire([(GD[1], 3.1), (4.8, 3.1), (4.8, 2.35), (EOM[0], 2.35)])
label(4.6, 3.15, r"$\alpha$")
wire([(FR[1], 1.85), (EOM[0], 1.85)])
label(4.8, 1.9, r"$F_T, F_D, g$", fs=FS - 1)

SX, SY, SR = 7.6, 2.1, 0.2
ax.add_patch(Circle((SX, SY), SR, fc="#ffffff", ec=INK, lw=1.0, zorder=3))
label(SX - 0.09, SY, "+", va="center", fs=FS - 1)
label(SX, SY - 0.1, "+", va="center", fs=FS - 1)
wire([(EOM[1], SY), (SX - SR, SY)])
PSF = (6.8, 8.4, 0.8, 1.45)
block(*PSF, ["Pseudo-forces", "Coriolis + centrifugal"], fs=FS - 0.5)
wire([(SX, PSF[3]), (SX, SY - SR)])

INT = (8.3, 9.0, 1.75, 2.45)
block(*INT, [r"$\frac{1}{s}$"], fs=FS + 6, name="Integrator")
wire([(SX + SR, SY), (INT[0], SY)])
label(8.05, 2.15, r"$\dot{\mathbf{y}}$")

# state out
NF, NE = 9.45, 10.0
wire([(INT[1], SY), (12.75, SY)])
dot(NF, SY)
dot(NE, SY)
label(11.3, 2.15, r"$\mathbf{y}(t)$")
EVT = (10.4, 12.6, 2.7, 3.65)
block(*EVT, ["Event detection", "zero crossing of the phase's",
             r"end condition and of $r = R_E$"], fs=FS - 0.5)
wire([(NE, SY), (NE, 3.17), (EVT[0], 3.17)])
tag(12.75, 13.6, 3.05, 3.29, "[event]", kind="goto")
wire([(EVT[1], 3.17), (12.75, 3.17)])
OUTP = (12.75, 13.55, 1.9, 2.3)
ax.add_patch(FancyBboxPatch((OUTP[0], OUTP[2]), OUTP[1] - OUTP[0], OUTP[3] - OUTP[2],
                            boxstyle="round,pad=0,rounding_size=0.12",
                            fc="#ffffff", ec=INK, lw=1.0, zorder=3))
label(0.5 * (OUTP[0] + OUTP[1]), SY, "to next", va="center", fs=FS - 1)
label(12.0, 1.62, "state at the event", va="center", fs=FS - 1, color=GREY)

# feedback
wire([(NF, SY), (NF, 0.5), (1.5, 0.5), (1.5, 2.95), (GD[0], 2.95)])
dot(1.5, 1.7)
wire([(1.5, 1.7), (FR[0], 1.7)])
dot(SX, 0.5)
wire([(SX, 0.5), (SX, PSF[2])])
label(5.3, 0.56, r"state feedback  $\mathbf{y}(t) = [\,s,\ r,\ v,\ \gamma,\ m\,]$")

save(fig, "simulator_run_sequence.png")
