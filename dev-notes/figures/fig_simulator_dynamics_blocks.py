"""Figures: the simulator as a Simulink-style block diagram, in five sheets, an overview and a full page.

Every sheet is drawn at its printed width (16 cm text width, 6.3 in), so a font size here is
the printed size: no text is below the 10 pt body size. Each sheet is cropped to its content.

1. simulator_dynamics_blocks.png   environment, engine, aerodynamics, gravity, mass flow,
                                   translational dynamics, pseudo-forces, kinematics, integrators
2. simulator_guidance_blocks.png   the guidance laws and the multiport switch that gives alpha;
                                   the state y and the design vector x
3. simulator_schedule_blocks.png   the phase schedules (switch input k, active law, engine delta,
                                   stage j) and the insertion schedule
4. simulator_parameter_blocks.png  the instant state changes and the fixed parameters
5. simulator_optimization_blocks.png  the optimizer, one ascent per design vector x, the cost J
6. simulator_overview_blocks.png  how the sheets relate: optimizer, guidance, dynamics, cost
                                   and the two loops
7. simulator_full_blocks.png       every sheet on one page (51 x 39 cm), the links between
                                   them wired: cost loop, kick/sep, law/k, alpha

Style: block fill = role (legend on each sheet), white = library block (integrator, clock,
merge, switch); every subsystem sits in a dashed group frame; every integrator takes its
initial value and reset from below.

Read from the code:
- h = r - R_E; rho = kappa rho_0 e^{-h/H}, p_a = kappa p_0 e^{-h/H} at every altitude
  (Auxiliary/atmosphere.py; kappa = INCLUDE_DRAG, rocket_ascent._ambient_pressure_for_run);
- F_D = q C_D A in both stages (drag is always modelled, STAGE2_DRAG on). Lift is zero;
- engine per stage j: F_T = delta (F_vac,j - p_a A_e,j), I_sp,j(p_a) ("pressure" modes);
- the 65 km crossing (h_atm) only jettisons the fairing (rocket_ascent._fairing_margin);
- pseudo-forces at Omega = eps omega cos(phi_0) (earth_rotation.planar_pseudoforce_rates),
  eps = ENABLE_EARTH_ROTATION;
- Stage 2: Delta t_1 = coast-start% Delta t_r% T_max,2, T_max,2 = m_p2 / mdot_2
  (pso_coast_solver.run_pso_coast_trajectory). Law-terminated burns are left out;
- cost: pso_coast_solver._coast_objective_terms, PENALTY_FORM "abs"; the burn-time term
  (t_f - t_cf)/T_max,2 = 1 - m_prop/m_p2;
- insertion: earth_rotation.rotating_to_inertial_planar, ra.get_orbital_elements.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, FancyBboxPatch
from matplotlib.text import Text
from matplotlib.spines import Spine
from matplotlib.axis import Axis

from figstyle import use_thesis_style, save, INK

use_thesis_style()
plt.rcParams["savefig.pad_inches"] = 0.0   # keep the sheet exactly W wide: printed = drawn

FS = 10.0            # every label: the printed body size
FT = 10.5            # block titles
TH = 0.19            # tag height [in]
TAGMIN = 0.36        # smallest tag width [in]
SHADOW = "#d3d7de"
TAGC = "#fff6d6"
FRAME = "#7d8590"
# block fill by role
ENV = "#e3f1e5"      # environment
VEH = "#eee7f7"      # vehicle
DYN = "#e2ebf8"      # equations of motion
GUI = "#f8e6ee"      # guidance laws
EVT = "#eceef2"      # schedules and event logic
PAR = "#ffffff"      # parameter sources and library blocks
OPZ = "#fdeedd"      # optimizer and cost
W = 6.3              # printed text width [in]


class Sheet:
    """One figure whose data units are printed inches; cropped to its content on save."""

    def __init__(self, name, width=W, height=14.0):
        self.name = name
        self.W = width
        self.ox = self.oy = 0.0      # offset added to every coordinate drawn
        self.fig = plt.figure(figsize=(width, height))
        ax = self.fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(0, width)
        ax.set_ylim(-3.0, height - 3.0)
        ax.set_aspect("equal")
        ax.axis("off")
        self.ax = ax
        self.fig.canvas.draw()
        self.R = self.fig.canvas.get_renderer()

    # -- measuring -------------------------------------------------------
    def tw(self, s, fs=FS, bold=False):
        t = self.ax.text(0, 0, s, fontsize=fs, fontweight="bold" if bold else "normal")
        bb = t.get_window_extent(self.R)
        t.remove()
        return bb.width / self.fig.dpi

    def tagw(self, s):
        return max(self.tw(s) + 0.2, TAGMIN)

    # -- primitives ------------------------------------------------------
    def text(self, x, y, s, ha="center", va="center", fs=FS, **kw):
        return self.ax.text(x + self.ox, y + self.oy, s, ha=ha, va=va, fontsize=fs, color=INK, zorder=8, **kw)

    def frame(self, x0, y0, x1, y1, title):
        """Dashed group frame, its title set on the top edge."""
        self.ax.add_patch(FancyBboxPatch((x0 + self.ox, y0 + self.oy), x1 - x0, y1 - y0,
                                         boxstyle="round,pad=0,rounding_size=0.06",
                                         fc="#fafbfc", ec=FRAME, lw=0.8, ls=(0, (4, 2.5)),
                                         zorder=1))
        self.text(x0 + 0.12, y1, title, ha="left", fs=FT + 0.5, fontweight="bold",
                  bbox=dict(fc="#ffffff", ec="none", pad=1.5))

    def wire(self, pts, arrow=True):
        pts = [(x + self.ox, y + self.oy) for x, y in pts]
        xs, ys = zip(*pts)
        n = len(pts) - 1 if arrow else len(pts)
        self.ax.plot(xs[:n], ys[:n], color=INK, lw=0.9, zorder=5,
                     solid_capstyle="butt")
        if arrow:
            self.ax.annotate("", xy=pts[-1], xytext=pts[-2], zorder=6,
                             arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.9,
                                             shrinkA=0, shrinkB=0, mutation_scale=8))

    def dot(self, x, y):
        self.ax.add_patch(Circle((x + self.ox, y + self.oy), 0.03, fc=INK, ec=INK, zorder=7))

    def block(self, x0, x1, y0, y1, lines, fc=PAR, title=True, lh=0.2, align="center",
              pad=0.12, indent=0.22):
        """Lines: str, (str, height) or (str, height, indent level). Returns the line centres.

        The first line is the bold title when `title`; with align="left" the other lines
        start at the left edge, indented by their level."""
        ax, ox, oy = self.ax, self.ox, self.oy
        ax.add_patch(Rectangle((x0 + 0.02 + ox, y0 - 0.02 + oy), x1 - x0, y1 - y0,
                               fc=SHADOW, ec="none", zorder=2))
        ax.add_patch(Rectangle((x0 + ox, y0 + oy), x1 - x0, y1 - y0, fc=fc, ec=INK, lw=0.9,
                               zorder=3))
        items = []
        for l in lines:
            if isinstance(l, str):
                l = (l, lh, 0)
            elif len(l) == 2:
                l = (l[0], l[1], 0)
            items.append(l)
        tot = sum(h for _, h, _ in items)
        y = 0.5 * (y0 + y1) + 0.5 * tot
        ys = []
        for i, (s, h, lev) in enumerate(items):
            bold = title and i == 0
            fs = FT if bold else FS
            w = self.tw(s, fs, bold)
            yc = y - 0.5 * h
            if align == "left" and not bold:
                x = x0 + pad + lev * indent
                room = x1 - x
                self.text(x, yc, s, ha="left", fs=fs)
            else:
                room = x1 - x0 - 0.06
                self.text(0.5 * (x0 + x1), yc, s, fs=fs,
                          fontweight="bold" if bold else "normal")
            if w > room:
                print(f"  [{self.name}] text wider than block: {s!r} {w:.2f} > {room:.2f}")
            ys.append(yc)
            y -= h
        if tot > y1 - y0 + 0.01:
            print(f"  [{self.name}] lines taller than block {lines[0]!r}")
        return ys

    def tag(self, x, y, s, kind="from", w=None):
        """From tag: x is its right tip.  Goto tag: x is its left tip.  Returns (x0, x1)."""
        w = w or self.tagw(s)
        k = 0.08
        y0, y1 = y - TH / 2, y + TH / 2
        if kind == "goto":
            x0, x1 = x, x + w
            pts = [(x0, y), (x0 + k, y1), (x1, y1), (x1, y0), (x0 + k, y0)]
            cx = 0.5 * (x0 + x1) + 0.03
        else:
            x0, x1 = x - w, x
            pts = [(x0, y0), (x0, y1), (x1 - k, y1), (x1, y), (x1 - k, y0)]
            cx = 0.5 * (x0 + x1) - 0.03
        pts = [(px + self.ox, py + self.oy) for px, py in pts]
        self.ax.add_patch(Polygon(pts, closed=True, fc=TAGC, ec=INK, lw=0.8, zorder=3))
        self.text(cx, y, s)
        return x0, x1

    def src(self, x, y, s, to, w=None):
        """From tag ending at x, wired horizontally into the point `to`."""
        self.tag(x, y, s, w=w)
        if to[1] == y:
            self.wire([(x, y), to])
        else:
            self.wire([(x, y), (to[0], y), to])

    def goto(self, frm, x, y, s, w=None):
        """Wire from `frm` to a Goto tag whose left tip is at (x, y)."""
        if frm[1] == y:
            self.wire([frm, (x, y)])
        else:
            self.wire([frm, (frm[0], y), (x, y)])
        self.tag(x, y, s, kind="goto", w=w)

    def stack(self, xtip, ys, labels, xin):
        """A column of From tags of one width, wired into the edge at xin."""
        w = max(self.tagw(s) for s in labels)
        for y, s in zip(ys, labels):
            self.src(xtip, y, s, (xin, y), w=w)

    def gotos(self, xout, x, ys, labels):
        """A column of Goto tags of one width, wired from the edge at xout."""
        w = max(self.tagw(s) for s in labels)
        for y, s in zip(ys, labels):
            self.goto((xout, y), x, y, s, w=w)

    def integ(self, x0, y, h=0.38, w=0.36):
        self.block(x0, x0 + w, y - h / 2, y + h / 2, [(r"$\dfrac{1}{s}$", 0.3)], title=False)

    def clock(self, x1, y):
        w = self.tw("Clock") + 0.14
        self.block(x1 - w, x1, y - 0.11, y + 0.11, ["Clock"], title=False)

    def summer(self, x, y, signs, r=0.13):
        self.ax.add_patch(Circle((x + self.ox, y + self.oy), r, fc="#ffffff", ec=INK, lw=0.9, zorder=3))
        for dx, dy, s in signs:
            self.text(x + dx, y + dy, s)

    def bar(self, x0, y0, y1, w=0.09):
        self.ax.add_patch(Rectangle((x0 + self.ox, y0 + self.oy), w, y1 - y0, fc=INK, ec=INK,
                                    zorder=4))

    def rect(self, x0, x1, y0, y1):
        """A plain white library block with no text (the Merge)."""
        self.ax.add_patch(Rectangle((x0 + self.ox, y0 + self.oy), x1 - x0, y1 - y0,
                                    fc="#ffffff", ec=INK, lw=0.9, zorder=3))

    def switch(self, x0, x1, y0, y1, ports, yout, active):
        """Multiport switch: open circles mark the data ports."""
        ax, ox, oy = self.ax, self.ox, self.oy
        ax.add_patch(Rectangle((x0 + ox, y0 + oy), x1 - x0, y1 - y0, fc="#ffffff", ec=INK,
                               lw=0.9, zorder=3))
        for y in ports:
            ax.plot([x0 + ox, x0 + 0.07 + ox], [y + oy, y + oy], color=INK, lw=0.9, zorder=4)
            ax.add_patch(Circle((x0 + 0.09 + ox, y + oy), 0.022, fc="#ffffff", ec=INK, lw=0.8,
                                zorder=4))
        ax.plot([x0 + 0.11 + ox, x1 - 0.07 + ox], [active + oy, yout + oy], color=INK, lw=1.1,
                zorder=4)
        ax.plot([x1 - 0.07 + ox, x1 + ox], [yout + oy, yout + oy], color=INK, lw=0.9,
                zorder=4)

    def notes(self, y, lines, dy=0.21):
        for i, s in enumerate(lines):
            if self.tw(s) > self.W:
                print(f"  [{self.name}] note wider than the sheet: {s[:40]!r}")
            self.text(0.0, y - dy * i, s, ha="left")

    def legend(self, y, items):
        """One row: (label, fill) for a block swatch, (label, None) for a signal tag."""
        x = 0.02
        ox, oy = self.ox, self.oy
        for label, fc in items:
            if fc is None:
                k = 0.07
                pts = [(x + ox, y - TH / 2 + oy), (x + ox, y + TH / 2 + oy),
                       (x + 0.3 - k + ox, y + TH / 2 + oy), (x + 0.3 + ox, y + oy),
                       (x + 0.3 - k + ox, y - TH / 2 + oy)]
                self.ax.add_patch(Polygon(pts, closed=True, fc=TAGC, ec=INK, lw=0.8,
                                          zorder=3))
            else:
                self.ax.add_patch(Rectangle((x + ox, y - 0.085 + oy), 0.3, 0.17, fc=fc, ec=INK,
                                            lw=0.8, zorder=3))
            x += 0.37
            self.text(x, y, label, ha="left")
            x += self.tw(label) + 0.28
        if x + ox > self.W:
            print(f"  [{self.name}] legend wider than the sheet")

    def save(self):
        """Crop the height to the content, keeping 1 data unit = 1 printed inch."""
        self.fig.canvas.draw()
        inv = self.ax.transData.inverted()
        lo, hi = 1e9, -1e9
        for a in self.ax.get_children():
            if not a.get_visible() or a is self.ax.patch or isinstance(a, (Spine, Axis)):
                continue
            if isinstance(a, Text) and not a.get_text():
                continue
            try:
                bb = a.get_window_extent(self.R)
            except Exception:
                continue
            if bb.width <= 0 and bb.height <= 0:
                continue
            (_, y0), (_, y1) = inv.transform([(bb.x0, bb.y0), (bb.x1, bb.y1)])
            lo, hi = min(lo, y0), max(hi, y1)
        m = 0.03
        self.ax.set_ylim(lo - m, hi + m)
        self.fig.set_size_inches(self.W, hi - lo + 2 * m)
        save(self.fig, self.name)


TAG_ITEM = ("signal tag (From / Goto)", None)
LIB_ITEM = ("library block", PAR)

# =============================================================== 1. dynamics
S = Sheet("simulator_dynamics_blocks.png")
IX = 5.28            # integrator column
GX = 5.78            # Goto column after the integrators
XT = 5.2             # right tip of the tags that feed an integrator from below


def below(yl, items):
    """Initial value and reset ports of the integrator at yl, both from below."""
    w = max(S.tagw(s) for s, _ in items)
    for i, (s, dx) in enumerate(items):
        y = yl - 0.33 - 0.24 * i
        S.src(XT, y, s, (IX + dx, yl - 0.19), w=w)


# ---- environment ---------------------------------------------------------
S.frame(0.01, 6.31, W - 0.01, 8.24, "Environment and aerodynamics")
YA = 7.25
ALT = (0.62, 1.38, 6.96, 7.54)
S.block(*ALT, ["Altitude", r"$h = r - R_E$"], fc=ENV)
S.src(0.52, YA, r"$r$", (ALT[0], YA))
S.src(0.86, 7.81, r"$R_E$", (1.0, ALT[3]))
ATM = (1.75, 2.92, 6.86, 7.64)
S.block(*ATM, ["Atmosphere", r"$\rho = \kappa\,\rho_0\, e^{-h/H}$",
               r"$p_a = \kappa\,p_0\, e^{-h/H}$"], fc=ENV)
S.wire([(ALT[1], YA), (ATM[0], YA)])
S.text(1.6, YA + 0.1, r"$h$")
S.dot(1.5, YA)
S.goto((1.5, YA), 1.62, 6.62, r"$h$")
S.src(2.15, 7.86, r"$\rho_0,\ p_0,\ H$", (2.25, ATM[3]))
S.src(2.65, 7.86, r"$\kappa$", (2.75, ATM[3]))
DPR = (3.22, 4.44, 7.2, 7.78)
S.block(*DPR, ["Dynamic pressure", r"$q = \rho v^2 / 2$"], fc=ENV)
S.wire([(ATM[1], 7.44), (DPR[0], 7.44)])
S.text(3.07, 7.53, r"$\rho$")
S.src(3.73, 7.98, r"$v$", (3.83, DPR[3]))
S.goto((ATM[1], 7.0), 3.02, 7.0, r"$p_a$")
AER = (4.6, 5.68, 7.2, 7.78)
S.block(*AER, ["Aerodynamics", r"$F_D = q\, C_D A$"], fc=ENV)
S.wire([(DPR[1], 7.48), (AER[0], 7.48)])
S.text(4.5, 7.57, r"$q$")
S.src(4.95, 7.98, r"$C_D,\ A$", (5.05, AER[3]))
S.goto((AER[1], 7.48), GX, 7.48, r"$F_D$")
GRV = (4.6, 5.68, 6.43, 6.93)
S.block(*GRV, ["Gravity", (r"$g = \mu / r^2$", 0.22)], fc=ENV)
S.stack(4.5, [6.8, 6.56], [r"$r$", r"$\mu$"], GRV[0])
S.goto((GRV[1], 6.68), GX, 6.68, r"$g$")

# ---- engine and mass -----------------------------------------------------
S.frame(0.01, 4.4, W - 0.01, 6.16, "Engine and mass flow")
ENG = (0.62, 2.68, 4.86, 5.62)
S.block(*ENG, ["Engine, stage $j$", r"$F_T = \delta\,(F_{vac,j} - p_a A_{e,j})$",
               r"$I_{sp} = I_{sp,j}(p_a)$"], fc=VEH)
S.stack(0.52, [5.48, 5.24, 5.0], [r"$p_a$", r"$\delta$", r"$j$"], ENG[0])
S.src(1.55, 4.56, r"$F_{vac,j},\ A_{e,j},\ I_{sp,vac,j}$", (1.65, ENG[2]))
MFL = (3.42, 4.42, 4.86, 5.62)
S.block(*MFL, ["Mass flow", (r"$\dot m = -\dfrac{F_T}{I_{sp}\, g_0}$", 0.36)], fc=VEH)
S.wire([(ENG[1], 5.38), (MFL[0], 5.38)])
S.text(3.24, 5.47, r"$F_T$")
S.wire([(ENG[1], 5.08), (MFL[0], 5.08)])
S.text(3.24, 5.17, r"$I_{sp}$")
S.dot(2.86, 5.38)
S.goto((2.86, 5.38), 2.96, 5.9, r"$F_T$")
S.dot(3.02, 5.08)
S.goto((3.02, 5.08), 3.12, 4.62, r"$I_{sp}$")
S.src(3.82, 5.9, r"$g_0$", (3.92, MFL[3]))
YM = 5.24
S.integ(IX, YM)
S.wire([(MFL[1], YM), (IX, YM)])
S.text(4.85, YM + 0.1, r"$\dot m$")
S.goto((IX + 0.36, YM), GX, YM, r"$m$")
below(YM, [(r"$m^+$", 0.1), (r"$m$ reset", 0.26)])

# ---- translational dynamics and pseudo-forces ----------------------------
S.frame(0.01, 1.54, W - 0.01, 4.24, "Dynamics")
TRN = (0.62, 2.85, 2.11, 4.08)
S.block(*TRN, ["Translational dynamics", ("", 0.1),
               (r"$\dot v = \dfrac{F_T\cos\alpha - F_D}{m} - g\sin\gamma$", 0.46),
               ("", 0.08),
               (r"$\dot\gamma = \dfrac{1}{v}\left[\dfrac{F_T\sin\alpha}{m}"
                r" - \left(g - \dfrac{v^2}{r}\right)\cos\gamma\right]$", 0.46)], fc=DYN)
S.stack(0.52, [3.94 - 0.24 * i for i in range(8)],
        [r"$F_T$", r"$F_D$", r"$g$", r"$\alpha$", r"$m$", r"$v$", r"$\gamma$", r"$r$"],
        TRN[0])
YV, YG = 3.92, 2.29
PSF = (3.45, 4.9, 2.6, 3.43)
S.block(*PSF, ["Pseudo-forces", (r"$\Omega = \varepsilon\,\omega\cos\varphi_0$", 0.19),
               (r"$\Delta\dot v = \Omega^2 r\sin\gamma$", 0.19),
               (r"$\Delta\dot\gamma = 2\Omega + \Omega^2 r\cos\gamma / v$", 0.21)], fc=DYN)
S.stack(3.35, [3.255, 3.015, 2.775], [r"$v$", r"$\gamma$", r"$r$"], PSF[0])
S.src(3.85, 3.62, r"$\varepsilon,\ \omega,\ \varphi_0$", (3.95, PSF[3]))
SXP = 4.5
S.summer(SXP, YV, [(-0.2, 0.1, "+"), (0.1, -0.21, "+")])
S.summer(SXP, YG, [(-0.2, -0.1, "+"), (0.1, 0.21, "+")])
for yl in (YV, YG):
    S.wire([(TRN[1], yl), (SXP - 0.13, yl)])
    S.integ(IX, yl)
    S.wire([(SXP + 0.13, yl), (IX, yl)])
S.text(3.15, YV + 0.1, r"$\dot v$")
S.text(3.15, YG + 0.1, r"$\dot\gamma$")
S.wire([(SXP, PSF[3]), (SXP, YV - 0.13)])
S.text(SXP - 0.2, 0.5 * (PSF[3] + YV - 0.13), r"$\Delta\dot v$")
S.wire([(SXP, PSF[2]), (SXP, YG + 0.13)])
S.text(SXP - 0.2, 0.5 * (PSF[2] + YG + 0.13), r"$\Delta\dot\gamma$")
S.goto((IX + 0.36, YV), GX, YV, r"$v$")
S.goto((IX + 0.36, YG), GX, YG, r"$\gamma$")
below(YV, [(r"$v_0$", 0.18)])
below(YG, [(r"$\gamma^+$", 0.1), (r"$\gamma$ reset", 0.26)])

# ---- kinematics ----------------------------------------------------------
S.frame(0.01, -0.06, W - 0.01, 1.38, "Kinematics")
KIN = (0.62, 2.85, 0.2, 1.22)
S.block(*KIN, ["Kinematics", (r"$\dot r = v\sin\gamma$", 0.26),
               (r"$\dot s = \dfrac{R_E}{r}\, v\cos\gamma$", 0.42)], fc=DYN)
S.stack(0.52, [1.07, 0.83, 0.59, 0.35], [r"$v$", r"$\gamma$", r"$r$", r"$R_E$"], KIN[0])
YR, YS = 1.0, 0.45
for yl, d, o, ic in [(YR, r"$\dot r$", r"$r$", r"$r_0$"), (YS, r"$\dot s$", r"$s$", r"$s_0$")]:
    S.integ(IX, yl)
    S.wire([(KIN[1], yl), (IX, yl)])
    S.text(4.0, yl + 0.1, d)
    S.goto((IX + 0.36, yl), GX, yl, o)
    below(yl, [(ic, 0.18)])
S.legend(-0.32, [("environment", ENV), ("vehicle", VEH), ("equations of motion", DYN),
                 LIB_ITEM])
S.legend(-0.6, [TAG_ITEM])
S.save()


# =============================================================== 2. guidance
S = Sheet("simulator_guidance_blocks.png")
S.frame(0.01, 2.2, W - 0.01, 7.45, "Guidance")
LX0, LX1 = 1.55, 3.55
VRT = (LX0, LX1, 6.7, 7.15)
ZER = (LX0, LX1, 6.1, 6.55)
CLL = (LX0, LX1, 4.41, 5.95)
OLL = (LX0, LX1, 3.44, 4.26)
OCL = (LX0, LX1, 2.47, 3.29)
S.block(*VRT, ["Vertical rise", r"$\theta = 90^\circ$"], fc=GUI)
S.block(*ZER, ["Thrust along velocity", r"$\theta = \gamma$, so $\alpha = 0$"], fc=GUI)
S.block(*CLL, ["Closed-loop law", "Apollo, PEG", r"$t_{go}$, coefficients"], fc=GUI)
S.block(*OLL, ["Open-loop law", r"$\theta(t;\ \mathbf{c})$"], fc=GUI)
S.block(*OCL, ["Optimal-control law", r"$\boldsymbol{\lambda}(t_{ign}) = \boldsymbol{\lambda}_0$",
               r"$\alpha$ from $\boldsymbol{\lambda}$"], fc=GUI)
S.src(1.45, 6.325, r"$\gamma$", (ZER[0], 6.325))
S.clock(1.45, 5.83)
S.wire([(1.45, 5.83), (CLL[0], 5.83)])
S.stack(1.45, [5.59 - 0.24 * i for i in range(5)],
        [r"$\mathbf{y}$", r"$F_T$", r"$I_{sp}$", r"$r_f,\ v_f,\ \gamma_f$", "law"], CLL[0])
S.clock(1.45, 4.09)
S.wire([(1.45, 4.09), (OLL[0], 4.09)])
S.stack(1.45, [3.85, 3.61], [r"$\mathbf{c}$", "law"], OLL[0])
S.stack(1.45, [3.12, 2.88, 2.64], [r"$\mathbf{y}$", r"$F_T$", r"$\boldsymbol{\lambda}_0$"],
        OCL[0])

PORTS = [0.5 * (b[2] + b[3]) for b in (VRT, ZER, CLL, OLL, OCL)]
SW = (4.3, 4.62, 2.6, 7.1)
AY = 4.6
S.switch(*SW, PORTS, yout=AY, active=PORTS[2])
S.text(0.5 * (SW[0] + SW[1]), SW[3] + 0.15, "Multiport switch")
for y in PORTS:
    S.wire([(LX1, y), (SW[0], y)])
S.tag(4.25, 2.4, r"$k$")
S.wire([(4.25, 2.4), (4.46, 2.4), (4.46, SW[2])])
AX = 5.2
S.summer(AX, AY, [(-0.2, 0.1, "+"), (0.1, 0.22, r"$-$")])
S.wire([(SW[1], AY), (AX - 0.13, AY)])
S.text(4.85, AY + 0.1, r"$\theta$")
S.src(5.12, 5.25, r"$\gamma$", (AX, AY + 0.13))
S.goto((AX + 0.13, AY), 5.62, AY, r"$\alpha$")
# alpha is the angle from the velocity vector to the thrust vector, with or without air
S.text(6.2, AY - 0.27, "angle from", ha="right")
S.text(6.2, AY - 0.47, "velocity to thrust", ha="right")

S.frame(0.01, -0.25, 1.75, 2.02, "State")
S.stack(0.5, [1.65 - 0.25 * i for i in range(5)],
        [r"$s$", r"$r$", r"$v$", r"$\gamma$", r"$m$"], 0.72)
S.bar(0.72, 0.48, 1.77)
S.text(0.77, 0.32, "Mux")
S.goto((0.81, 1.15), 1.2, 1.15, r"$\mathbf{y}$")
S.frame(1.9, -0.25, W - 0.01, 2.02, "Design vector, from the optimizer")
S.tag(2.6, 0.92, r"$\mathbf{x}$")
S.wire([(2.6, 0.92), (2.95, 0.92)])
S.bar(2.95, 0.05, 1.8)
S.gotos(3.04, 3.3, [1.67 - 0.3 * i for i in range(6)],
        [r"$\gamma_p$", r"$\Delta t_c$", r"$\Delta t_r\%$", r"coast-start$\%$",
         r"$\mathbf{c}$", r"$\boldsymbol{\lambda}_0$"])
S.text(3.0, -0.1, "Demux")
for i, s in enumerate(["An ascent case leaves", "some of these entries",
                       "free, and fixes the", "others."]):
    S.text(4.55, 1.37 - 0.2 * i, s, ha="left")
S.legend(-0.5, [("guidance law", GUI), LIB_ITEM, TAG_ITEM])
S.save()


# =============================================================== 3. schedules
S = Sheet("simulator_schedule_blocks.png")
SX0, SX1 = 1.55, 4.5
S1 = (SX0, SX1, 7.55, 8.6)
IS = (SX0, SX1, 6.28, 7.2)
S2 = (SX0, SX1, 4.37, 5.93)
INS = (SX0, 4.75, 1.32, 4.0)
XE = SX0 + 0.25               # the event arrows between the schedules
S.frame(0.01, 0.62, W - 0.01, 8.85, "Phase schedules")
S.block(*S1, ["Stage 1 phase schedule", r"$t < t_v$: vertical rise, kick at $t_v$",
              r"$t \geq t_v$: law1 steers",
              r"$\delta = 1$, $j = 1$; MECO when $m_{p1}$ is spent"], fc=EVT)
S.clock(1.45, 8.45)
S.wire([(1.45, 8.45), (S1[0], 8.45)])
S.stack(1.45, [8.21, 7.97, 7.73], [r"$m$", r"$t_v,\ m_{p1}$", "law1"], S1[0])
S.wire([(XE, S1[2]), (XE, IS[3])])
S.text(XE + 0.1, 0.5 * (S1[2] + IS[3]), "MECO", ha="left")
S.block(*IS, ["Interstage phase schedule", r"coast: $\theta = \gamma$, $\delta = 0$",
              r"separation at MECO $+\,\Delta t_{sep}$: $j = 2$",
              r"ignition at MECO $+\,\Delta t_{ign}$"], fc=EVT)
S.clock(1.45, 7.05)
S.wire([(1.45, 7.05), (IS[0], 7.05)])
S.src(1.45, 6.81, r"$\Delta t_{sep},\ \Delta t_{ign}$", (IS[0], 6.81))
S.wire([(XE, IS[2]), (XE, S2[3])])
S.text(XE + 0.1, 0.5 * (IS[2] + S2[3]), "ignition", ha="left")
S.block(*S2, [r"Stage 2 phase schedule, $j = 2$",
              r"burn 1: $\Delta t_1 = \mathrm{coast\text{-}start}\%\cdot\Delta t_r\%\cdot T_{\max,2}$",
              r"coast: $\Delta t_c$, $\theta = \gamma$, $\delta = 0$",
              r"burn 2: $\Delta t_3 = \Delta t_r\%\cdot T_{\max,2} - \Delta t_1$",
              "law2 steers both burns"], fc=EVT)
S.clock(1.45, 5.8)
S.wire([(1.45, 5.8), (S2[0], 5.8)])
S.stack(1.45, [5.56 - 0.24 * i for i in range(5)],
        [r"$\Delta t_c$", r"$\Delta t_r\%$", r"coast-start$\%$", r"$T_{\max,2}$", "law2"],
        S2[0])
S.wire([(XE, S2[2]), (XE, INS[3])])
S.text(XE + 0.1, 0.5 * (S2[2] + INS[3]), r"end of burn 2: $\delta = 0$", ha="left")
S.goto((S1[1], 8.45), 4.6, 8.45, "kick")
S.goto((IS[1], 7.05), 4.6, 7.05, "sep")
# Merge: three inputs at an even pitch, then the Demux
MG = (5.22, 5.45, 5.12, 6.08)
YMG = 0.5 * (MG[2] + MG[3])
S.rect(*MG)
S.wire([(S1[1], 7.7), (5.1, 7.7), (5.1, YMG + 0.3), (MG[0], YMG + 0.3)])
S.wire([(IS[1], 6.43), (4.98, 6.43), (4.98, YMG), (MG[0], YMG)])
S.wire([(S2[1], YMG - 0.3), (MG[0], YMG - 0.3)])
BX = 5.56
S.wire([(MG[1], YMG), (BX, YMG)])
S.bar(BX, YMG - 0.5, YMG + 0.5)
YLB = MG[3] + 0.17
S.text(0.5 * (MG[0] + MG[1]), YLB, "Merge")
S.text(BX + 0.06, YLB, "Demux", ha="left")
S.gotos(BX + 0.09, 5.74, [YMG + 0.375 - 0.25 * i for i in range(4)],
        [r"$k$", "law", r"$\delta$", r"$j$"])

# insertion: the final state, in the inertial frame, and what is read from it
ys = S.block(*INS, [("Insertion phase schedule", 0.28),
                    ("1. Rotating to inertial frame:", 0.24),
                    (r"$v_h = v\cos\gamma + \omega\, r\cos\varphi_0$", 0.22, 1),
                    (r"$v_I = \sqrt{v_h^2 + (v\sin\gamma)^2}$", 0.25, 1),
                    (r"$\gamma_I = \arctan\,(v\sin\gamma\, /\, v_h)$", 0.28, 1),
                    (r"2. Final orbit of $(r,\ v_I,\ \gamma_I)$: $a,\ e,\ h_a,\ h_p$", 0.28),
                    (r"3. Propellant left: $m_{prop} = m - m_{dry,2}$", 0.28),
                    ("4. Misses from the target:", 0.24),
                    (r"$\Delta r = r - r_f,\ \Delta v = v - v_f,\ \Delta\gamma = \gamma - \gamma_f$",
                     0.26, 1),
                    (r"5. Cost $J$ from $m_{prop}$ and the misses", 0.26)],
             fc=EVT, align="left")
S.stack(1.45, [INS[3] - 0.2 - 0.24 * i for i in range(5)],
        [r"$\mathbf{y}$", r"$\omega,\ \varphi_0$", r"$\mu$", r"$m_{dry,2}$",
         r"$r_f,\ v_f,\ \gamma_f$"], INS[0])
S.gotos(INS[1], 4.85, [ys[3], ys[5], ys[6], ys[8], ys[9]],
        [r"$\mathbf{y}_{f,I}$", r"$a,\ e,\ h_a,\ h_p$", r"$m_{prop}$",
         r"$\Delta r,\ \Delta v,\ \Delta\gamma$", r"$J$"])
XS = INS[1] - 0.45
S.wire([(XS, INS[2]), (XS, 0.98)])
S.block(XS - 0.28, XS + 0.28, 0.74, 0.98, ["Stop"], title=False)
S.notes(0.38, [
    r"Each schedule outputs $[k$, law, $\delta$, $j]$ in its own interval, and the Merge "
    r"passes the active one:",
    r"$k$ selects the guidance port, law is the active law (law1 in Stage 1, law2 in "
    r"Stage 2),",
    r"$\delta$ switches the engine on or off, and $j$ is the stage, 1 or 2. The cost $J$ "
    r"goes to the optimizer."], dy=0.22)
S.legend(-0.36, [("schedule and event logic", EVT), LIB_ITEM, TAG_ITEM])
S.save()


# ====================================================== 4. resets and parameters
S = Sheet("simulator_parameter_blocks.png")
S.frame(0.01, 6.75, W - 0.01, 8.92, "Instant State Changes")
JMP = (1.55, 3.75, 6.9, 8.68)
S.block(*JMP, ["Instant State Changes", r"start: $\gamma^+ = \gamma_0$, $m^+ = m_0$",
               r"kick: $\gamma^+ = \gamma_p$",
               r"fairing: $m^+ = m - m_{fair}$",
               r"separation: $m^+ = m - m_{s1}$"], fc=EVT)
S.stack(1.45, [8.55 - 0.24 * i for i in range(7)],
        ["kick", "sep", r"$h$", r"$m$", r"$\gamma_p$", r"$h_{atm},\ m_{fair},\ m_{s1}$",
         r"$\gamma_0,\ m_0$"], JMP[0])
YJ = 0.5 * (JMP[2] + JMP[3])
S.gotos(JMP[1], 3.9, [YJ + 0.54 - 0.36 * i for i in range(4)],
        [r"$\gamma$ reset", r"$\gamma^+$", r"$m$ reset", r"$m^+$"])
S.notes(6.48, [
    r"$\bullet$ Start, $t = 0$: the integrators begin from $\gamma_0$ and $m_0$.",
    r"$\bullet$ Kick, at $t_v$: the pitch-over sets $\gamma$ to $\gamma_p$ at once.",
    r"$\bullet$ Fairing: dropped when $h$ rises through $h_{atm}$.",
    r"$\bullet$ Separation: drops the first stage (sep, from the phase schedules).",
    r"At each event the block fires the $\gamma$ or $m$ reset, and that integrator restarts "
    r"from $\gamma^+$ or $m^+$.",
    r"Between events the integrators run on; $s$, $r$ and $v$ are never reset."])

PTOP = 5.12
S.frame(0.01, PTOP - 4.0, W - 0.01, PTOP, "Fixed parameters")   # bottom set below
DY = 0.255


def group(cursor, title_lines, names, bx):
    """A source block, its bus and one Goto per name; returns the next free y."""
    n = len(names)
    hb = 0.2 * len(title_lines) + 0.2
    half = max(DY * (n - 1) / 2 + 0.12, hb / 2)
    yc = cursor - half
    S.block(bx[0], bx[1], yc - hb / 2, yc + hb / 2, title_lines)
    xb = bx[1] + 0.15
    S.wire([(bx[1], yc), (xb, yc)])
    y_top = yc + DY * (n - 1) / 2
    S.bar(xb, y_top - DY * (n - 1) - 0.12, y_top + 0.12)
    S.gotos(xb + 0.09, xb + 0.22, [y_top - DY * i for i in range(n)], names)
    return yc - half - 0.24


LEFT, RIGHT = (0.12, 1.4), (3.55, 4.85)
c = PTOP - 0.3
c = group(c, ["Earth"], [r"$R_E$", r"$\mu$", r"$g_0$", r"$\omega,\ \varphi_0$"], LEFT)
c = group(c, ["Atmosphere"], [r"$\rho_0,\ p_0,\ H$", r"$h_{atm}$", r"$C_D,\ A$"], LEFT)
c = group(c, ["Vehicle", "stages 1 and 2"],
          [r"$F_{vac,j},\ A_{e,j},\ I_{sp,vac,j}$", r"$m_{fair}$", r"$m_{s1}$", r"$m_{p1}$",
           r"$m_{dry,2}$", r"$T_{\max,2}$"], LEFT)
c2 = PTOP - 0.3
c2 = group(c2, ["Model switches", "atmosphere, rotation"], [r"$\kappa$", r"$\varepsilon$"],
           RIGHT)
c2 = group(c2, ["Ascent Case", "laws, target"], ["law1, law2", r"$r_f,\ v_f,\ \gamma_f$"],
           RIGHT)
c2 = group(c2, ["Launch state", r"$\mathbf{y}_0$"],
           [r"$s_0$", r"$r_0$", r"$v_0$", r"$\gamma_0$", r"$m_0$"], RIGHT)
c2 = group(c2, ["Timing"], [r"$t_v$", r"$\Delta t_{sep},\ \Delta t_{ign}$"], RIGHT)
PBOT = min(c, c2) + 0.1
# fit the frame to the groups
for p in list(S.ax.patches):
    if isinstance(p, FancyBboxPatch) and abs(p.get_y() - (PTOP - 4.0)) < 1e-9:
        p.set_y(PBOT)
        p.set_height(PTOP - PBOT)
S.notes(PBOT - 0.25, [
    r"Subscript $j$ is the stage, 1 or 2. $T_{\max,2} = m_{p2}\, I_{sp,vac,2}\, g_0 "
    r"/ F_{vac,2}$, the longest Stage 2 burn.",
    r"$\kappa = 0$ without atmosphere, $\varepsilon = 0$ without rotation. $\omega$: "
    r"Earth's rotation rate, $\varphi_0$: launch latitude.",
    r"$m_{dry,2}$: Stage 2 structure and payload, the mass left when its propellant is spent."])
S.legend(PBOT - 1.0, [("event logic", EVT), ("parameter source", PAR), TAG_ITEM])
S.save()


# ============================================================= 5. optimization
S = Sheet("simulator_optimization_blocks.png")


def ports(b, labels, side, ys):
    """Port labels written inside a subsystem block, on its left or right edge."""
    for y, s in zip(ys, labels):
        if side == "left":
            S.text(b[0] + 0.08, y, s, ha="left")
        else:
            S.text(b[1] - 0.08, y, s, ha="right")


# outer loop: the optimizer proposes x and keeps the best
S.frame(0.01, 6.3, W - 0.01, 8.3, "Outer loop: optimizer")
OPT = (1.75, 4.3, 6.5, 8.0)
S.block(*OPT, ["Optimizer", r"particle swarm: $N_p$ particles,",
               r"$N_g$ generations, one $J$ per particle",
               r"$\mathbf{x}$ kept within $[\mathbf{x}_{lb},\ \mathbf{x}_{ub}]$",
               r"keeps the best: $\mathbf{x}^*,\ J^*$"], fc=OPZ)
S.stack(1.65, [7.6, 7.25, 6.9], [r"$J$", r"$\mathbf{x}_{lb},\ \mathbf{x}_{ub}$",
                                 r"$N_p,\ N_g$"], OPT[0])
S.gotos(OPT[1], 4.5, [7.6, 6.9], [r"$\mathbf{x}$", r"$\mathbf{x}^*,\ J^*$"])

# inner loop: one ascent per x
S.frame(0.01, 3.25, W - 0.01, 6.05, "Inner loop: one ascent")
ASC = (1.75, 4.3, 3.45, 5.75)
S.block(*ASC, [], fc=DYN)
S.text(0.5 * (ASC[0] + ASC[1]), ASC[3] - 0.2, "One ascent", fs=FT, fontweight="bold")
S.text(0.5 * (ASC[0] + ASC[1]), ASC[2] + 0.17, "sheets 1 to 4")
YIN = [5.25 - 0.3 * i for i in range(6)]
XIN = [r"$\gamma_p$", r"$\Delta t_c$", r"$\Delta t_r\%$", r"coast-start$\%$",
       r"$\mathbf{c}$", r"$\boldsymbol{\lambda}_0$"]
ports(ASC, XIN, "left", YIN)
S.tag(0.62, 0.5 * (YIN[0] + YIN[-1]), r"$\mathbf{x}$")
XD = 0.95
S.wire([(0.62, 0.5 * (YIN[0] + YIN[-1])), (XD, 0.5 * (YIN[0] + YIN[-1]))])
S.bar(XD, YIN[-1] - 0.14, YIN[0] + 0.14)
S.text(XD + 0.045, YIN[-1] - 0.3, "Demux")
for y in YIN:
    S.wire([(XD + 0.09, y), (ASC[0], y)])
YOUT = [5.25 - 0.36 * i for i in range(5)]
OUT = [r"$m_{prop}$", r"$\Delta r$", r"$\Delta v$", r"$\Delta\gamma$", "crash"]
ports(ASC, OUT, "right", YOUT)
S.gotos(ASC[1], 4.5, YOUT, [r"$m_{prop}$", r"$\Delta r$", r"$\Delta v$",
                            r"$\Delta\gamma$", "crash"])

# the cost of that ascent
S.frame(0.01, 0.97, W - 0.01, 3.0, "Cost")
CST = (1.75, 5.3, 1.17, 2.75)
S.block(*CST, ["Cost",
               r"$J = w_J\,(1 - m_{prop}/m_{p2})$",
               r"$+\ w_r\,|\Delta r|/h_f + w_v\,|\Delta v|/v_f$",
               r"$+\ w_\gamma\,|\Delta\gamma|/\gamma_{ref}$",
               r"$+\ J_{crash}$ if the ascent crashes"], fc=OPZ, lh=0.24)
S.stack(1.65, [2.62 - 0.22 * i for i in range(7)],
        [r"$m_{prop}$", r"$\Delta r$", r"$\Delta v$", r"$\Delta\gamma$", "crash",
         r"$w_J,\ w_r,\ w_v,\ w_\gamma$", r"$m_{p2},\ h_f,\ v_f,\ \gamma_{ref}$"], CST[0])
S.goto((CST[1], 1.96), 5.5, 1.96, r"$J$")

S.notes(0.67, [
    r"Each particle's $\mathbf{x}$ flies one ascent, and its cost $J$ returns to the "
    r"optimizer (tag $J$).",
    r"An ascent case fixes some entries of $\mathbf{x}$ as constants; the others are "
    r"searched.",
    r"$1 - m_{prop}/m_{p2}$ is the Stage 2 burn time over $T_{\max,2}$. $h_f$: target "
    r"altitude, $\gamma_{ref} = 1^\circ$.",
    r"Crash: the ascent ends below the surface, with $v < 0$, or fails to integrate."],
    dy=0.22)
S.legend(-0.33, [("optimizer and cost", OPZ), ("ascent (sheets 1 to 4)", DYN), TAG_ITEM])
S.save()


# ================================================================ 6. overview
# How the sheets relate, with no simulator specifics: optimizer and cost on top, one ascent
# (guidance -> dynamics -> integrator, the state fed back) below. The schedules, the
# insertion and the defined variables are left out.
S = Sheet("simulator_overview_blocks.png")


def lab(x, y, s, ha="center"):
    S.text(x, y, s, ha=ha)


# outer loop, top row
OPT = (0.1, 1.9, 4.6, 5.2)
CST = (5.0, 6.2, 4.6, 5.2)
S.block(*OPT, ["Optimizer"], fc=OPZ)
S.block(*CST, ["Cost"], fc=OPZ)
YC = 0.5 * (OPT[2] + OPT[3])
S.wire([(CST[0], YC), (OPT[1], YC)])
lab(0.5 * (OPT[1] + CST[0]), YC + 0.17, "cost")

# one ascent
S.frame(0.45, 2.4, W - 0.01, 4.15, "One ascent")
YR = 3.4
GU = (1.0, 2.2, 3.1, 3.7)
DY_ = (3.0, 4.1, 3.0, 3.8)
INT = 4.5
S.block(*GU, ["Guidance"], fc=GUI)
S.block(*DY_, ["Dynamics"], fc=DYN)
S.integ(INT, YR)
XG = 0.5 * (GU[0] + GU[1])
S.wire([(XG, OPT[2]), (XG, GU[3])])
lab(XG + 0.08, 4.38, "design variables", ha="left")
S.wire([(GU[1], YR), (DY_[0], YR)])
lab(0.5 * (GU[1] + DY_[0]), YR + 0.17, "steering")
S.wire([(DY_[1], YR), (INT, YR)])
lab(0.5 * (DY_[1] + INT), YR + 0.17, "rates")

# the state: to the cost, and back under the bottom
XS = 0.5 * (CST[0] + CST[1])
YB = 2.65
S.wire([(INT + 0.36, YR), (XS, YR), (XS, CST[2])])
S.dot(XS, YR)
lab(0.5 * (INT + 0.36 + XS), YR + 0.17, "state")
lab(XS + 0.08, 4.38, "final state", ha="left")
S.wire([(XS, YR), (XS, YB), (XG, YB), (XG, GU[2])])
XDB = 0.5 * (DY_[0] + DY_[1])
S.wire([(XDB, YB), (XDB, DY_[2])])
S.dot(XDB, YB)

S.save()


# ==================================================================== 7. full
# Every sheet on one page and joined. Wired between the sheets: the cost loop (insertion ->
# cost -> J -> optimizer), the schedule events (kick, sep -> instant state changes; law, k ->
# guidance) and alpha (guidance -> dynamics). The design entries, the state, the engine
# commands, the resets and the defined variables stay From/Goto tags, as within each sheet.
XL, XM, XR = 0.25, 7.0, 13.75          # left, middle and right column
S = Sheet("simulator_full_blocks.png", width=XR + W, height=17.0)


def at(ox=0.0, oy=0.0):
    S.ox, S.oy = ox, oy


# ---- left column: optimizer -------------------------------------------------
at(XL)
S.frame(0.01, 10.95, W - 0.01, 13.0, "Optimizer")
OPT = (1.6, 4.15, 11.3, 12.75)
S.block(*OPT, ["Optimizer", r"particle swarm: $N_p$ particles,",
               r"$N_g$ generations, one $J$ per particle",
               r"$\mathbf{x}$ kept within $[\mathbf{x}_{lb},\ \mathbf{x}_{ub}]$",
               r"keeps the best: $\mathbf{x}^*,\ J^*$"], fc=OPZ)
YJIN = 12.4
S.stack(1.5, [12.05, 11.75], [r"$\mathbf{x}_{lb},\ \mathbf{x}_{ub}$", r"$N_p,\ N_g$"], OPT[0])
YX = 0.5 * (OPT[2] + OPT[3])
XDX = 4.45
S.wire([(OPT[1], YX), (XDX, YX)])
S.text(4.29, YX + 0.11, r"$\mathbf{x}$")
YXS = [YX + 0.6 - 0.24 * i for i in range(6)]
S.bar(XDX, YXS[-1] - 0.13, YXS[0] + 0.13)
S.gotos(XDX + 0.09, 4.68, YXS, [r"$\gamma_p$", r"$\Delta t_c$", r"$\Delta t_r\%$",
                                r"coast-start$\%$", r"$\mathbf{c}$", r"$\boldsymbol{\lambda}_0$"])
S.text(XDX + 0.045, YXS[-1] - 0.3, "Demux")
S.goto((0.5 * (OPT[0] + OPT[1]), OPT[2]), 3.05, 11.12, r"$\mathbf{x}^*,\ J^*$")

# ---- left column: phase schedules and insertion -----------------------------
OYS = 1.95
at(XL, OYS)
SX0, SX1 = 1.55, 4.5
S1 = (SX0, SX1, 7.55, 8.6)
IS = (SX0, SX1, 6.28, 7.2)
S2 = (SX0, SX1, 4.37, 5.93)
INS = (SX0, 4.75, 1.45, 4.0)
XE = SX0 + 0.25
S.frame(0.01, 1.3, W - 0.01, 8.85, "Phase schedules and insertion")
S.block(*S1, ["Stage 1 phase schedule", r"$t < t_v$: vertical rise, kick at $t_v$",
              r"$t \geq t_v$: law1 steers",
              r"$\delta = 1$, $j = 1$; MECO when $m_{p1}$ is spent"], fc=EVT)
S.clock(1.45, 8.45)
S.wire([(1.45, 8.45), (S1[0], 8.45)])
S.stack(1.45, [8.21, 7.97, 7.73], [r"$m$", r"$t_v,\ m_{p1}$", "law1"], S1[0])
S.wire([(XE, S1[2]), (XE, IS[3])])
S.text(XE + 0.1, 0.5 * (S1[2] + IS[3]), "MECO", ha="left")
S.block(*IS, ["Interstage phase schedule", r"coast: $\theta = \gamma$, $\delta = 0$",
              r"separation at MECO $+\,\Delta t_{sep}$: $j = 2$",
              r"ignition at MECO $+\,\Delta t_{ign}$"], fc=EVT)
S.clock(1.45, 7.05)
S.wire([(1.45, 7.05), (IS[0], 7.05)])
S.src(1.45, 6.81, r"$\Delta t_{sep},\ \Delta t_{ign}$", (IS[0], 6.81))
S.wire([(XE, IS[2]), (XE, S2[3])])
S.text(XE + 0.1, 0.5 * (IS[2] + S2[3]), "ignition", ha="left")
S.block(*S2, [r"Stage 2 phase schedule, $j = 2$",
              r"burn 1: $\Delta t_1 = \mathrm{coast\text{-}start}\%\cdot\Delta t_r\%\cdot T_{\max,2}$",
              r"coast: $\Delta t_c$, $\theta = \gamma$, $\delta = 0$",
              r"burn 2: $\Delta t_3 = \Delta t_r\%\cdot T_{\max,2} - \Delta t_1$",
              "law2 steers both burns"], fc=EVT)
S.clock(1.45, 5.8)
S.wire([(1.45, 5.8), (S2[0], 5.8)])
S.stack(1.45, [5.56 - 0.24 * i for i in range(5)],
        [r"$\Delta t_c$", r"$\Delta t_r\%$", r"coast-start$\%$", r"$T_{\max,2}$", "law2"],
        S2[0])
S.wire([(XE, S2[2]), (XE, INS[3])])
S.text(XE + 0.1, 0.5 * (S2[2] + INS[3]), r"end of burn 2: $\delta = 0$", ha="left")
MG = (5.22, 5.45, 5.12, 6.08)
YMG = 0.5 * (MG[2] + MG[3])
S.rect(*MG)
S.wire([(S1[1], 7.7), (5.1, 7.7), (5.1, YMG + 0.3), (MG[0], YMG + 0.3)])
S.wire([(IS[1], 6.43), (4.98, 6.43), (4.98, YMG), (MG[0], YMG)])
S.wire([(S2[1], YMG - 0.3), (MG[0], YMG - 0.3)])
BX = 5.56
S.wire([(MG[1], YMG), (BX, YMG)])
S.bar(BX, YMG - 0.5, YMG + 0.5)
YLB = MG[3] + 0.17
S.text(0.5 * (MG[0] + MG[1]), YLB, "Merge")
S.text(BX + 0.06, YLB, "Demux", ha="left")
YD = [YMG + 0.375 - 0.25 * i for i in range(4)]          # law, k (wired), delta, j
S.gotos(BX + 0.09, 5.74, YD[2:], [r"$\delta$", r"$j$"])
ys = S.block(*INS, [("Insertion phase schedule", 0.28),
                    ("1. Rotating to inertial frame:", 0.24),
                    (r"$v_h = v\cos\gamma + \omega\, r\cos\varphi_0$", 0.22, 1),
                    (r"$v_I = \sqrt{v_h^2 + (v\sin\gamma)^2}$", 0.25, 1),
                    (r"$\gamma_I = \arctan\,(v\sin\gamma\, /\, v_h)$", 0.28, 1),
                    (r"2. Final orbit of $(r,\ v_I,\ \gamma_I)$: $a,\ e,\ h_a,\ h_p$", 0.28),
                    (r"3. Propellant left: $m_{prop} = m - m_{dry,2}$", 0.28),
                    ("4. Misses from the target:", 0.24),
                    (r"$\Delta r = r - r_f,\ \Delta v = v - v_f,\ \Delta\gamma = \gamma - \gamma_f$",
                     0.26, 1)],
             fc=EVT, align="left")
S.stack(1.45, [INS[3] - 0.2 - 0.24 * i for i in range(5)],
        [r"$\mathbf{y}$", r"$\omega,\ \varphi_0$", r"$\mu$", r"$m_{dry,2}$",
         r"$r_f,\ v_f,\ \gamma_f$"], INS[0])
S.gotos(INS[1], 4.85, [ys[3], ys[5]], [r"$\mathbf{y}_{f,I}$", r"$a,\ e,\ h_a,\ h_p$"])
S.wire([(INS[1], ys[8]), (5.02, ys[8])])
S.block(5.02, 5.58, ys[8] - 0.12, ys[8] + 0.12, ["Stop"], title=False)

# ---- left column: cost ------------------------------------------------------
at(XL)
S.frame(0.01, 0.6, W - 0.01, 2.85, "Cost")
CST = (1.55, 4.75, 0.8, 2.55)
S.block(*CST, ["Cost",
               r"$J = w_J\,(1 - m_{prop}/m_{p2})$",
               r"$+\ w_r\,|\Delta r|/h_f + w_v\,|\Delta v|/v_f$",
               r"$+\ w_\gamma\,|\Delta\gamma|/\gamma_{ref}$",
               r"$+\ J_{crash}$ if the ascent crashes"], fc=OPZ, lh=0.24)
S.stack(1.45, [2.05, 1.75, 1.45], ["crash", r"$w_J,\ w_r,\ w_v,\ w_\gamma$",
                                  r"$m_{p2},\ h_f,\ v_f,\ \gamma_{ref}$"], CST[0])
YINB = INS[2] + OYS
for xw, s in [(2.3, r"$m_{prop}$"), (3.5, r"$\Delta r,\ \Delta v,\ \Delta\gamma$")]:
    S.wire([(xw, YINB), (xw, CST[3])])
    S.text(xw + 0.08, 0.5 * (YINB + 2.85), s, ha="left")

# ---- middle column: instant state changes --------------------------------------------
OYR = 13.0 - 8.92
at(XM, OYR)
S.frame(0.01, 6.75, W - 0.01, 8.92, "Instant State Changes")
JMP = (1.55, 3.75, 6.9, 8.68)
S.block(*JMP, ["Instant State Changes", r"start: $\gamma^+ = \gamma_0$, $m^+ = m_0$",
               r"kick: $\gamma^+ = \gamma_p$",
               r"fairing: $m^+ = m - m_{fair}$",
               r"separation: $m^+ = m - m_{s1}$"], fc=EVT)
RR = [8.55 - 0.24 * i for i in range(7)]
S.stack(1.45, RR[2:], [r"$h$", r"$m$", r"$\gamma_p$", r"$h_{atm},\ m_{fair},\ m_{s1}$",
                       r"$\gamma_0,\ m_0$"], JMP[0])
YJ = 0.5 * (JMP[2] + JMP[3])
S.gotos(JMP[1], 3.9, [YJ + 0.54 - 0.36 * i for i in range(4)],
        [r"$\gamma$ reset", r"$\gamma^+$", r"$m$ reset", r"$m^+$"])

# ---- middle column: guidance ------------------------------------------------
OYG = 3.1
at(XM, OYG)
S.frame(0.01, 2.2, W - 0.01, 7.45, "Guidance")
LX0, LX1 = 1.55, 3.55
VRT = (LX0, LX1, 6.7, 7.15)
ZER = (LX0, LX1, 6.1, 6.55)
CLL = (LX0, LX1, 4.41, 5.95)
OLL = (LX0, LX1, 3.44, 4.26)
OCL = (LX0, LX1, 2.47, 3.29)
S.block(*VRT, ["Vertical rise", r"$\theta = 90^\circ$"], fc=GUI)
S.block(*ZER, ["Thrust along velocity", r"$\theta = \gamma$, so $\alpha = 0$"], fc=GUI)
S.block(*CLL, ["Closed-loop law", "Apollo, PEG", r"$t_{go}$, coefficients"], fc=GUI)
S.block(*OLL, ["Open-loop law", r"$\theta(t;\ \mathbf{c})$"], fc=GUI)
S.block(*OCL, ["Optimal-control law", r"$\boldsymbol{\lambda}(t_{ign}) = \boldsymbol{\lambda}_0$",
               r"$\alpha$ from $\boldsymbol{\lambda}$"], fc=GUI)
S.src(1.45, 6.325, r"$\gamma$", (ZER[0], 6.325))
S.clock(1.45, 5.83)
S.wire([(1.45, 5.83), (CLL[0], 5.83)])
S.stack(1.45, [5.59 - 0.24 * i for i in range(4)],
        [r"$\mathbf{y}$", r"$F_T$", r"$I_{sp}$", r"$r_f,\ v_f,\ \gamma_f$"], CLL[0])
S.clock(1.45, 4.09)
S.wire([(1.45, 4.09), (OLL[0], 4.09)])
S.src(1.45, 3.85, r"$\mathbf{c}$", (OLL[0], 3.85))
S.stack(1.45, [3.12, 2.88, 2.64], [r"$\mathbf{y}$", r"$F_T$", r"$\boldsymbol{\lambda}_0$"],
        OCL[0])
PORTS = [0.5 * (b[2] + b[3]) for b in (VRT, ZER, CLL, OLL, OCL)]
SW = (4.3, 4.62, 2.6, 7.1)
AY = 4.6
S.switch(*SW, PORTS, yout=AY, active=PORTS[2])
S.text(0.5 * (SW[0] + SW[1]), SW[3] + 0.15, "Multiport switch")
for y in PORTS:
    S.wire([(LX1, y), (SW[0], y)])
AX = 5.2
S.summer(AX, AY, [(-0.2, 0.1, "+"), (0.1, 0.22, r"$-$")])
S.wire([(SW[1], AY), (AX - 0.13, AY)])
S.text(4.85, AY + 0.1, r"$\theta$")
S.src(5.12, 5.25, r"$\gamma$", (AX, AY + 0.13))
S.text(6.2, AY - 0.27, "angle from", ha="right")
S.text(6.2, AY - 0.47, "velocity to thrust", ha="right")

# ---- middle column: defined variables ---------------------------------------
OYP = -0.22
at(XM, OYP)
c = PTOP - 0.3
c = group(c, ["Earth"], [r"$R_E$", r"$\mu$", r"$g_0$", r"$\omega,\ \varphi_0$"], LEFT)
c = group(c, ["Atmosphere"], [r"$\rho_0,\ p_0,\ H$", r"$h_{atm}$", r"$C_D,\ A$"], LEFT)
c = group(c, ["Vehicle", "stages 1 and 2"],
          [r"$F_{vac,j},\ A_{e,j},\ I_{sp,vac,j}$", r"$m_{fair}$", r"$m_{s1}$", r"$m_{p1}$",
           r"$m_{dry,2}$", r"$T_{\max,2}$"], LEFT)
c2 = PTOP - 0.3
c2 = group(c2, ["Model switches", "atmosphere, rotation"], [r"$\kappa$", r"$\varepsilon$"],
           RIGHT)
c2 = group(c2, ["Ascent Case", "laws, target"], ["law1, law2", r"$r_f,\ v_f,\ \gamma_f$"],
           RIGHT)
c2 = group(c2, ["Launch state", r"$\mathbf{y}_0$"],
           [r"$s_0$", r"$r_0$", r"$v_0$", r"$\gamma_0$", r"$m_0$"], RIGHT)
c2 = group(c2, ["Timing"], [r"$t_v$", r"$\Delta t_{sep},\ \Delta t_{ign}$"], RIGHT)
S.frame(0.01, min(c, c2) + 0.1, W - 0.01, PTOP, "Defined variables")

# ---- right column: dynamics -------------------------------------------------
OYD = 13.0 - 8.24
at(XR, OYD)
S.frame(0.01, 6.31, W - 0.01, 8.24, "Environment and aerodynamics")
YA = 7.25
ALT = (0.62, 1.38, 6.96, 7.54)
S.block(*ALT, ["Altitude", r"$h = r - R_E$"], fc=ENV)
S.src(0.52, YA, r"$r$", (ALT[0], YA))
S.src(0.86, 7.81, r"$R_E$", (1.0, ALT[3]))
ATM = (1.75, 2.92, 6.86, 7.64)
S.block(*ATM, ["Atmosphere", r"$\rho = \kappa\,\rho_0\, e^{-h/H}$",
               r"$p_a = \kappa\,p_0\, e^{-h/H}$"], fc=ENV)
S.wire([(ALT[1], YA), (ATM[0], YA)])
S.text(1.6, YA + 0.1, r"$h$")
S.dot(1.5, YA)
S.goto((1.5, YA), 1.62, 6.62, r"$h$")
S.src(2.15, 7.86, r"$\rho_0,\ p_0,\ H$", (2.25, ATM[3]))
S.src(2.65, 7.86, r"$\kappa$", (2.75, ATM[3]))
DPR = (3.22, 4.44, 7.2, 7.78)
S.block(*DPR, ["Dynamic pressure", r"$q = \rho v^2 / 2$"], fc=ENV)
S.wire([(ATM[1], 7.44), (DPR[0], 7.44)])
S.text(3.07, 7.53, r"$\rho$")
S.src(3.73, 7.98, r"$v$", (3.83, DPR[3]))
S.goto((ATM[1], 7.0), 3.02, 7.0, r"$p_a$")
AER = (4.6, 5.68, 7.2, 7.78)
S.block(*AER, ["Aerodynamics", r"$F_D = q\, C_D A$"], fc=ENV)
S.wire([(DPR[1], 7.48), (AER[0], 7.48)])
S.text(4.5, 7.57, r"$q$")
S.src(4.95, 7.98, r"$C_D,\ A$", (5.05, AER[3]))
S.goto((AER[1], 7.48), GX, 7.48, r"$F_D$")
GRV = (4.6, 5.68, 6.43, 6.93)
S.block(*GRV, ["Gravity", (r"$g = \mu / r^2$", 0.22)], fc=ENV)
S.stack(4.5, [6.8, 6.56], [r"$r$", r"$\mu$"], GRV[0])
S.goto((GRV[1], 6.68), GX, 6.68, r"$g$")

S.frame(0.01, 4.4, W - 0.01, 6.16, "Engine and mass flow")
ENG = (0.62, 2.68, 4.86, 5.62)
S.block(*ENG, ["Engine, stage $j$", r"$F_T = \delta\,(F_{vac,j} - p_a A_{e,j})$",
               r"$I_{sp} = I_{sp,j}(p_a)$"], fc=VEH)
S.stack(0.52, [5.48, 5.24, 5.0], [r"$p_a$", r"$\delta$", r"$j$"], ENG[0])
S.src(1.55, 4.56, r"$F_{vac,j},\ A_{e,j},\ I_{sp,vac,j}$", (1.65, ENG[2]))
MFL = (3.42, 4.42, 4.86, 5.62)
S.block(*MFL, ["Mass flow", (r"$\dot m = -\dfrac{F_T}{I_{sp}\, g_0}$", 0.36)], fc=VEH)
S.wire([(ENG[1], 5.38), (MFL[0], 5.38)])
S.text(3.24, 5.47, r"$F_T$")
S.wire([(ENG[1], 5.08), (MFL[0], 5.08)])
S.text(3.24, 5.17, r"$I_{sp}$")
S.dot(2.86, 5.38)
S.goto((2.86, 5.38), 2.96, 5.9, r"$F_T$")
S.dot(3.02, 5.08)
S.goto((3.02, 5.08), 3.12, 4.62, r"$I_{sp}$")
S.src(3.82, 5.9, r"$g_0$", (3.92, MFL[3]))
YM = 5.24
S.integ(IX, YM)
S.wire([(MFL[1], YM), (IX, YM)])
S.text(4.85, YM + 0.1, r"$\dot m$")
S.goto((IX + 0.36, YM), GX, YM, r"$m$")
below(YM, [(r"$m^+$", 0.1), (r"$m$ reset", 0.26)])

S.frame(0.01, 1.54, W - 0.01, 4.24, "Dynamics")
TRN = (0.62, 2.85, 2.11, 4.08)
S.block(*TRN, ["Translational dynamics", ("", 0.1),
               (r"$\dot v = \dfrac{F_T\cos\alpha - F_D}{m} - g\sin\gamma$", 0.46),
               ("", 0.08),
               (r"$\dot\gamma = \dfrac{1}{v}\left[\dfrac{F_T\sin\alpha}{m}"
                r" - \left(g - \dfrac{v^2}{r}\right)\cos\gamma\right]$", 0.46)], fc=DYN)
YTR = [3.94 - 0.24 * i for i in range(8)]
TRL = [r"$F_T$", r"$F_D$", r"$g$", r"$\alpha$", r"$m$", r"$v$", r"$\gamma$", r"$r$"]
wtr = max(S.tagw(s) for s in TRL)
for i in (0, 1, 2, 4, 5, 6, 7):                    # alpha is wired from the guidance
    S.src(0.52, YTR[i], TRL[i], (TRN[0], YTR[i]), w=wtr)
YV, YG = 3.92, 2.29
PSF = (3.45, 4.9, 2.6, 3.43)
S.block(*PSF, ["Pseudo-forces", (r"$\Omega = \varepsilon\,\omega\cos\varphi_0$", 0.19),
               (r"$\Delta\dot v = \Omega^2 r\sin\gamma$", 0.19),
               (r"$\Delta\dot\gamma = 2\Omega + \Omega^2 r\cos\gamma / v$", 0.21)], fc=DYN)
S.stack(3.35, [3.255, 3.015, 2.775], [r"$v$", r"$\gamma$", r"$r$"], PSF[0])
S.src(3.85, 3.62, r"$\varepsilon,\ \omega,\ \varphi_0$", (3.95, PSF[3]))
SXP = 4.5
S.summer(SXP, YV, [(-0.2, 0.1, "+"), (0.1, -0.21, "+")])
S.summer(SXP, YG, [(-0.2, -0.1, "+"), (0.1, 0.21, "+")])
for yl in (YV, YG):
    S.wire([(TRN[1], yl), (SXP - 0.13, yl)])
    S.integ(IX, yl)
    S.wire([(SXP + 0.13, yl), (IX, yl)])
S.text(3.15, YV + 0.1, r"$\dot v$")
S.text(3.15, YG + 0.1, r"$\dot\gamma$")
S.wire([(SXP, PSF[3]), (SXP, YV - 0.13)])
S.text(SXP - 0.2, 0.5 * (PSF[3] + YV - 0.13), r"$\Delta\dot v$")
S.wire([(SXP, PSF[2]), (SXP, YG + 0.13)])
S.text(SXP - 0.2, 0.5 * (PSF[2] + YG + 0.13), r"$\Delta\dot\gamma$")
S.goto((IX + 0.36, YV), GX, YV, r"$v$")
S.goto((IX + 0.36, YG), GX, YG, r"$\gamma$")
below(YV, [(r"$v_0$", 0.18)])
below(YG, [(r"$\gamma^+$", 0.1), (r"$\gamma$ reset", 0.26)])

S.frame(0.01, -0.06, W - 0.01, 1.38, "Kinematics")
KIN = (0.62, 2.85, 0.2, 1.22)
S.block(*KIN, ["Kinematics", (r"$\dot r = v\sin\gamma$", 0.26),
               (r"$\dot s = \dfrac{R_E}{r}\, v\cos\gamma$", 0.42)], fc=DYN)
S.stack(0.52, [1.07, 0.83, 0.59, 0.35], [r"$v$", r"$\gamma$", r"$r$", r"$R_E$"], KIN[0])
for yl, d, o, ic in [(1.0, r"$\dot r$", r"$r$", r"$r_0$"), (0.45, r"$\dot s$", r"$s$", r"$s_0$")]:
    S.integ(IX, yl)
    S.wire([(KIN[1], yl), (IX, yl)])
    S.text(4.0, yl + 0.1, d)
    S.goto((IX + 0.36, yl), GX, yl, o)
    below(yl, [(ic, 0.18)])

# ---- right column: state and termination ------------------------------------
at(XR)
S.frame(0.01, 2.1, W - 0.01, 4.45, "State and termination")
S.stack(0.5, [4.05 - 0.25 * i for i in range(5)],
        [r"$s$", r"$r$", r"$v$", r"$\gamma$", r"$m$"], 0.72)
S.bar(0.72, 2.93, 4.17)
S.text(0.77, 2.77, "Mux")
S.goto((0.81, 3.55), 1.2, 3.55, r"$\mathbf{y}$")
CRS = (2.6, 4.95, 2.8, 3.95)
S.block(*CRS, ["Crash check", r"$h < 0$ or $v < 0$,", "or a failed step:",
               "the ascent ends"], fc=EVT)
S.stack(2.5, [3.52, 3.22], [r"$h$", r"$v$"], CRS[0])
S.goto((CRS[1], 3.375), 5.12, 3.375, "crash")
S.legend(1.65, [("environment", ENV), ("vehicle", VEH), ("equations of motion", DYN)])
S.legend(1.3, [("guidance law", GUI), ("schedule and event logic", EVT)])
S.legend(0.95, [("optimizer and cost", OPZ), LIB_ITEM])
S.legend(0.6, [TAG_ITEM])

# ---- wires between the columns (page coordinates) ---------------------------
at()
# kick and sep: schedules -> instant state changes
for yfrom, xv, yto, s in [(8.45 + OYS, 6.7, RR[0] + OYR, "kick"),
                          (7.05 + OYS, 6.8, RR[1] + OYR, "sep")]:
    S.wire([(XL + SX1, yfrom), (xv, yfrom), (xv, yto), (XM + JMP[0], yto)])
    S.text(XL + 5.85, yfrom + 0.1, s)
    S.text(7.45, yto + 0.1, s)
# law: schedules -> closed- and open-loop laws
YLC, YLO = 4.63 + OYG, 3.61 + OYG
XLW = 6.9
S.wire([(XL + BX + 0.09, YD[0] + OYS), (XLW, YD[0] + OYS), (XLW, YLC), (XM + LX0, YLC)])
S.wire([(XLW, YLC), (XLW, YLO), (XM + LX0, YLO)])
S.dot(XLW, YLC)
S.text(XL + 6.0, YD[0] + OYS + 0.1, "law")
S.text(7.2, YLC + 0.1, "law")
S.text(7.2, YLO + 0.1, "law")
# k: schedules -> multiport switch
YKC = 5.12
S.wire([(XL + BX + 0.09, YD[1] + OYS), (6.6, YD[1] + OYS), (6.6, YKC),
        (XM + 4.46, YKC), (XM + 4.46, SW[2] + OYG)])
S.text(XL + 6.05, YD[1] + OYS + 0.1, r"$k$")
S.text(XM + 4.56, 0.5 * (YKC + SW[2] + OYG), r"$k$", ha="left")
# alpha: guidance -> translational dynamics
XAV = 13.5
YAL = YTR[3] + OYD
S.wire([(XM + AX + 0.13, AY + OYG), (XAV, AY + OYG), (XAV, YAL), (XR + TRN[0], YAL)])
S.text(XM + 5.75, AY + OYG + 0.11, r"$\alpha$")
# J: cost -> optimizer, round the left edge
YJB = 0.35
S.wire([(XL + CST[1], 1.675), (XL + 5.75, 1.675), (XL + 5.75, YJB), (0.1, YJB),
        (0.1, YJIN), (XL + OPT[0], YJIN)])
S.text(XL + 5.25, 1.785, r"$J$")
S.text(0.85, YJIN + 0.11, r"$J$")
S.save()
