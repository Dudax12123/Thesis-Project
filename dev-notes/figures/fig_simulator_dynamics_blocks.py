"""Figure: the simulator's dynamic and kinematic system as a Simulink-style block diagram.

Every block shows the signals that enter and leave it, read from the code:
- altitude h = r - R_E; exponential atmosphere rho, p_a (Auxiliary/atmosphere.py);
  q = rho v^2 / 2; drag F_D = q C_D A. Lift is drawn as zero, as the thesis states it (the code still
  flies q C_L A in Stage 1: thesis flag T68);
- engine F_T = F_vac - p_a A_e and I_sp(p_a) ("pressure" modes), on/off and stage
  from the schedule (rocket_ascent.thrust_Isp);
- gravity g = mu / r^2 (Auxiliary/gravity.py);
- guidance alpha = theta - gamma from t, the state, F_T and I_sp
  (_compute_alpha_stage2);
- translational dynamics, mass flow and kinematics (rocket_ascent.diff_eom_base);
- rotating-frame terms at Omega = omega cos(phi_0)
  (earth_rotation.planar_pseudoforce_rates), added to v-dot and gamma-dot;
- five integrators give the state [s, r, v, gamma, m]; Goto/From tags close the loops.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon

from figstyle import use_thesis_style, save, INK, GREY

use_thesis_style()

FS = 11.0
W, H = 17.8, 11.1
fig = plt.figure(figsize=(W, H))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.set_aspect("equal")
ax.axis("off")

SHADOW = "#b9bec6"
TAG = "#fff6d6"
ENV = "#eef4ee"
VEH = "#ffffff"
DYN = "#eaf1fb"
FB = FS + 2          # text size inside blocks


def block(x0, x1, y0, y1, lines, fc=VEH, fs=FB, title=True):
    LH = 0.024 * fs
    ax.add_patch(Rectangle((x0 + 0.04, y0 - 0.04), x1 - x0, y1 - y0,
                           fc=SHADOW, ec="none", zorder=2))
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=fc, ec=INK, lw=1.0,
                           zorder=3))
    n = len(lines)
    for i, s in enumerate(lines):
        y = 0.5 * (y0 + y1) + (0.5 * (n - 1) - i) * LH
        bold = title and i == 0 and n > 1
        ax.text(0.5 * (x0 + x1), y, s, ha="center", va="center",
                fontsize=fs + 0.5 if bold else fs,
                fontweight="bold" if bold else "normal", color=INK, zorder=4)


def tag(x0, x1, yc, s, kind="from", h=0.24):
    y0, y1 = yc - h / 2, yc + h / 2
    k = 0.11
    if kind == "goto":
        pts = [(x0, yc), (x0 + k, y1), (x1, y1), (x1, y0), (x0 + k, y0)]
    else:
        pts = [(x0, y0), (x0, y1), (x1 - k, y1), (x1, yc), (x1 - k, y0)]
    ax.add_patch(Polygon(pts, closed=True, fc=TAG, ec=INK, lw=0.9, zorder=3))
    ax.text(0.5 * (x0 + x1) - (0.04 if kind == "from" else -0.04), yc, s,
            ha="center", va="center", fontsize=FS - 1, color=INK, zorder=4)


def wire(pts, arrow=True):
    xs, ys = zip(*pts)
    ax.plot(xs[:-1] if arrow else xs, ys[:-1] if arrow else ys, color=INK, lw=1.0,
            zorder=5)
    if arrow:
        ax.annotate("", xy=pts[-1], xytext=pts[-2], zorder=6,
                    arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.0,
                                    shrinkA=0, shrinkB=0, mutation_scale=11))


def dot(x, y):
    ax.add_patch(Circle((x, y), 0.045, fc=INK, ec=INK, zorder=7))


def label(x, y, s, ha="center", va="bottom", fs=FS, color=INK, **kw):
    ax.text(x, y, s, ha=ha, va=va, fontsize=fs, color=color, zorder=8, **kw)


def summer(x, y, r=0.2):
    ax.add_patch(Circle((x, y), r, fc="#ffffff", ec=INK, lw=1.0, zorder=3))
    label(x - 0.09, y, "+", va="center", fs=FS - 1)
    label(x + 0.01, y - 0.1, "+", va="center", fs=FS - 1)


# ---------------------------------------------------------------- headers
for x, s in [(2.6, "Environment"), (6.6, "Vehicle models"), (10.75, "Dynamics"),
             (14.5, "Rotating frame"), (16.75, "Integration")]:
    label(x, 10.75, s, va="center", fs=FS + 1, fontweight="bold", color=GREY)

# ------------------------------------------------------------ environment
tag(0.15, 0.75, 8.95, r"$r$")
block(1.0, 2.0, 8.6, 9.3, ["Altitude", r"$h = r - R_E$"], fc=ENV, fs=FS + 1)
wire([(0.75, 8.95), (1.0, 8.95)])
block(2.35, 3.75, 8.5, 9.45, ["Atmosphere", r"$\rho = \rho_0 e^{-h/H}$",
                              r"$p_a = p_0 e^{-h/H}$"], fc=ENV)
wire([(2.0, 8.95), (2.35, 8.95)])
label(2.17, 9.0, r"$h$")

block(4.1, 5.35, 6.3, 7.1, ["Dyn. pressure", r"$q = \frac{1}{2}\rho v^2$"],
      fc=ENV, fs=FS + 1)
wire([(3.75, 8.7), (3.95, 8.7), (3.95, 6.85), (4.1, 6.85)])
label(3.88, 7.8, r"$\rho$", ha="right", va="center")
tag(3.2, 3.8, 6.55, r"$v$")
wire([(3.8, 6.55), (4.1, 6.55)])

# ---------------------------------------------------------- vehicle models
BX0, BX1 = 5.65, 7.55
ENG = (BX0, BX1, 8.8, 9.8)
GDN = (BX0, BX1, 7.4, 8.45)
AER = (BX0, BX1, 6.2, 7.2)
GRV = (BX0, BX1, 5.25, 5.95)
block(*ENG, ["Engine", r"$F_T = F_{vac} - p_a A_e$", r"$I_{sp}(p_a)$"])
block(*GDN, ["Guidance", "law of the phase", r"$\alpha = \theta - \gamma$"])
block(*AER, ["Aerodynamics", r"$F_D = q\,C_D A$"])
block(*GRV, ["Gravity", r"$g = \mu / r^2$"])

# atmosphere -> engine
wire([(3.75, 9.2), (BX0, 9.2)])
label(4.4, 9.25, r"$p_a$")
block(4.75, 5.3, 9.42, 9.68, ["Clock"], fs=FS, title=False)
wire([(5.3, 9.55), (BX0, 9.55)])
label(5.47, 9.58, r"$t$", fs=FS - 1)
# guidance inputs
block(4.75, 5.3, 8.17, 8.43, ["Clock"], fs=FS, title=False)
wire([(5.3, 8.3), (BX0, 8.3)])
label(5.47, 8.33, r"$t$", fs=FS - 1)
for y, s in [(8.03, r"$\mathbf{y}$"), (7.78, r"$F_T$"), (7.53, r"$I_{sp}$")]:
    tag(4.75, 5.35, y, s, h=0.2)
    wire([(5.35, y), (BX0, y)])
# q -> aero
wire([(5.35, 6.7), (BX0, 6.7)])
label(5.5, 6.75, r"$q$")
# r -> gravity
tag(4.75, 5.35, 5.6, r"$r$")
wire([(5.35, 5.6), (BX0, 5.6)])

# --------------------------------------------------------------- dynamics
CX0, CX1 = 8.9, 12.6
MFL = (CX0, CX1, 9.45, 10.2)
TRN = (CX0, CX1, 3.4, 9.25)
KIN = (CX0, CX1, 1.1, 2.7)
block(*MFL, ["Mass flow", r"$\dot m = -F_T / (I_{sp}\, g_0)$"], fc=DYN, fs=FS + 3)
block(*TRN, ["Translational dynamics", "",
             r"$\dot v = \dfrac{F_T\cos\alpha - F_D}{m} - g\sin\gamma$", "", "",
             r"$\dot\gamma = \dfrac{1}{v}\left[\dfrac{F_T\sin\alpha}{m}"
             r" - \left(g - \dfrac{v^2}{r}\right)\cos\gamma\right]$"], fc=DYN, fs=FS + 3)
block(*KIN, ["Kinematics", r"$\dot r = v\sin\gamma$",
             r"$\dot s = \dfrac{R_E}{r}\, v\cos\gamma$"], fc=DYN, fs=FS + 3)

# engine -> mass flow and dynamics, with Goto tags for the guidance
wire([(BX1, 9.55), (7.7, 9.55), (7.7, 10.0), (CX0, 10.0)])
label(8.3, 10.05, r"$I_{sp}$")
dot(7.7, 10.0)
wire([(7.7, 10.0), (7.7, 10.35), (7.85, 10.35)])
tag(7.85, 8.5, 10.35, r"$I_{sp}$", kind="goto", h=0.2)
wire([(BX1, 9.05), (CX0, 9.05)])
dot(7.9, 9.05)
wire([(7.9, 9.05), (7.9, 9.65), (CX0, 9.65)])
label(8.4, 9.7, r"$F_T$")
label(8.65, 9.1, r"$F_T$")
dot(8.1, 9.05)
wire([(8.1, 9.05), (8.1, 8.6), (8.2, 8.6)])
tag(8.2, 8.8, 8.6, r"$F_T$", kind="goto", h=0.2)
# guidance, aero, gravity -> dynamics
wire([(BX1, 7.95), (CX0, 7.95)])
label(8.2, 8.0, r"$\alpha$")
wire([(BX1, 6.7), (CX0, 6.7)])
label(8.2, 6.75, r"$F_D$")
wire([(BX1, 5.6), (CX0, 5.6)])
label(8.2, 5.65, r"$g$")
# state into the dynamics
for y, s in [(4.9, r"$m$"), (4.5, r"$v$"), (4.1, r"$\gamma$"), (3.7, r"$r$")]:
    tag(8.05, 8.65, y, s)
    wire([(8.65, y), (CX0, y)])
# state into the kinematics
for y, s in [(2.3, r"$v$"), (1.9, r"$\gamma$"), (1.5, r"$r$")]:
    tag(8.05, 8.65, y, s)
    wire([(8.65, y), (CX0, y)])

# ---------------------------------------------------------- rotating frame
SX = 14.5
Y_V, Y_G = 7.6, 5.2
PSF = (13.5, 15.5, 5.8, 7.0)
block(*PSF, ["Pseudo-forces", r"$\Omega = \omega\cos\varphi_0$",
             r"$\Delta\dot v = \Omega^2 r\sin\gamma$",
             r"$\Delta\dot\gamma = 2\Omega + \Omega^2 r\cos\gamma / v$"],
      fs=FS + 1)
for y, s in [(6.75, r"$v$"), (6.4, r"$\gamma$"), (6.05, r"$r$")]:
    tag(12.75, 13.35, y, s)
    wire([(13.35, y), (PSF[0], y)])
summer(SX, Y_V)
summer(SX, Y_G)
wire([(CX1, Y_V), (SX - 0.2, Y_V)])
label(13.1, Y_V + 0.05, r"$\dot v$")
wire([(CX1, Y_G), (SX - 0.2, Y_G)])
label(13.1, Y_G + 0.05, r"$\dot\gamma$")
wire([(SX, PSF[3]), (SX, Y_V - 0.2)])
label(SX + 0.07, 7.2, r"$\Delta\dot v$", ha="left", va="center", fs=FS - 1)
wire([(SX, PSF[2]), (SX, Y_G + 0.2)])
label(SX + 0.07, 5.6, r"$\Delta\dot\gamma$", ha="left", va="center", fs=FS - 1)

# ------------------------------------------------------------- integration
IX0, IX1 = 15.9, 16.5
GX0, GX1 = 16.85, 17.6
rows = [(9.82, r"$\dot m$", r"$m$", CX1),
        (Y_V, r"$\dot v$", r"$v$", SX + 0.2),
        (Y_G, r"$\dot\gamma$", r"$\gamma$", SX + 0.2),
        (2.4, r"$\dot r$", r"$r$", CX1),
        (1.4, r"$\dot s$", r"$s$", CX1)]
for y, sin_, sout, xsrc in rows:
    block(IX0, IX1, y - 0.3, y + 0.3, [r"$\frac{1}{s}$"], fs=FS + 7, title=False)
    wire([(xsrc, y), (IX0, y)])
    label(IX0 - 0.35, y + 0.05, sin_)
    tag(GX0, GX1, y, sout, kind="goto")
    wire([(IX1, y), (GX0, y)])

label(W / 2, 0.45,
      r"State $\mathbf{y} = [\,s,\ r,\ v,\ \gamma,\ m\,]$.  Goto and From tags with the"
      r" same name carry the same signal.  The initial state of each phase is the"
      r" state at the end of the previous one.", va="center", fs=FS - 0.5,
      color=GREY)

save(fig, "simulator_dynamics_blocks.png")
