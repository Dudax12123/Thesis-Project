"""Figure: arc structure of the optimisation architectures, side by side.

Spec: Thesis_Optimization_Background.tex, the "% FIG-4 (TO DRAW)" block above
\\includegraphics{Figures/architecture_arc_structures.png} in Section 3.4.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from figstyle import (use_thesis_style, blank_axes, save, INK, GREY, FAINT,
                      THRUST, ACCENT, GREEN, AMBER, VIOLET, COAST)

use_thesis_style()

fig, ax = plt.subplots(figsize=(7.3, 6.5))
blank_axes(ax)
ax.set_aspect("auto")

IGN = 3.15
END = 8.45
SH = 0.34
DIMX = 11.30


def span(x0, x1, yy, label, color=INK, fs=7.0):
    ax.annotate("", xy=(x0, yy), xytext=(x1, yy),
                arrowprops=dict(arrowstyle="|-|,widthA=0.22,widthB=0.22",
                                color=color, lw=0.8, shrinkA=0, shrinkB=0))
    ax.text(0.5 * (x0 + x1), yy - 0.05, label, fontsize=fs, color=color,
            ha="center", va="top")


def arcs(y, spec):
    for kind, x0, x1 in spec:
        ax.add_patch(Rectangle((x0, y), x1 - x0, SH,
                               fc=THRUST if kind == "T" else COAST,
                               ec=INK, lw=0.8, zorder=4))
        ax.text(0.5 * (x0 + x1), y + SH / 2,
                "thrust" if kind == "T" else "coast", ha="center",
                va="center", fontsize=6.9,
                color="white" if kind == "T" else INK, zorder=5)


def cutoff(x, y, label, event=True):
    ax.plot([x, x], [y - 0.11, y + SH + 0.11], color=INK if event else GREY,
            lw=1.4 if event else 1.1, ls="-" if event else (0, (3, 2.2)),
            zorder=7)
    ax.text(x + 0.16, y + SH / 2, label, fontsize=7.0,
            color=INK if event else GREY, ha="left", va="center",
            linespacing=1.3)


# --- the shared prefix, drawn once --------------------------------------
PY = 6.35
ax.add_patch(Rectangle((0.55, PY), IGN - 0.55, SH + 0.06, fc="#ececec",
                       ec=INK, lw=0.8, zorder=4))
ax.text(0.5 * (0.55 + IGN), PY + (SH + 0.06) / 2, "stage 1 · kick · staging",
        ha="center", va="center", fontsize=7.4, color=INK, zorder=5)
ax.text(IGN + 0.20, PY + (SH + 0.06) / 2,
        "identical for every architecture —\nthey differ only after ignition",
        fontsize=8.2, color=GREY, ha="left", va="center", style="italic",
        linespacing=1.35)
span(0.55, IGN, PY - 0.10, r"carries the kick variable ($\alpha$ or $\gamma_p$)",
     color=GREY)

ax.plot([IGN, IGN], [0.20, PY + SH + 0.10], color=GREY, lw=1.0,
        ls=(0, (3, 2.2)), zorder=2)
ax.text(IGN, PY + SH + 0.24, "second-stage ignition", fontsize=7.4,
        color=GREY, ha="center", va="bottom")
ax.text(DIMX, PY + SH + 0.24, "design-vector\ndimension", fontsize=7.6,
        color=INK, ha="center", va="bottom", linespacing=1.3)

NAMES = [(5.20, "Brute-force\nkick search", "1"),
         (4.15, "Coast-parameter\nPSO", "4"),
         (3.10, "Direct-insertion\nPSO", "2"),
         (1.95, "Indirect PMP\nPSO", "7"),
         (0.60, "Segmented\nmulti-law", "$4+(n\\!-\\!1)$")]
for y, name, dim in NAMES:
    ax.text(IGN - 0.20, y + SH / 2, name, fontsize=8.0, color=INK, ha="right",
            va="center", linespacing=1.3)
    ax.text(DIMX, y + SH / 2, dim, fontsize=9.5, color=INK, ha="center",
            va="center")

# (1) brute force -- the kick angle is the only variable, and it sits on the
#     shared prefix, which is the whole point
y = 5.20
arcs(y, [("T", IGN, 5.15), ("C", 5.15, 8.15), ("T", 8.15, 8.55)])
ax.plot([8.15, 8.15], [y - 0.11, y + SH + 0.11], color=INK, lw=1.4, zorder=7)
ax.text(8.70, y + SH / 2,
        "event: apogee $=r_{\\rm target}$;\nimpulsive circularisation",
        fontsize=7.0, color=INK, ha="left", va="center", linespacing=1.3)
span(0.55, IGN, y - 0.10, r"only variable: kick angle $\alpha$", color=ACCENT)

# (2) coast-parameter PSO
y = 4.15
arcs(y, [("T", IGN, 4.95), ("C", 4.95, 6.65), ("T", 6.65, END)])
cutoff(END, y, "planned cut-off", event=False)
span(IGN, 4.95, y - 0.10, "coast-start %")
span(4.95, 6.65, y - 0.10, r"$\Delta t_c$", fs=7.6)
span(IGN, END, y - 0.36, r"$\Delta t_r$ %", fs=7.6)

# (3) direct-insertion PSO
y = 3.10
arcs(y, [("T", IGN, 7.35)])
cutoff(7.35, y, "event: circular\nvelocity reached")
span(IGN, 7.35, y - 0.10, r"$t_{\rm burn}$ %", fs=7.6)

# (4) indirect PMP PSO
y = 1.95
arcs(y, [("T", IGN, 4.95), ("C", 4.95, 6.65), ("T", 6.65, END)])
cutoff(END, y, "planned cut-off", event=False)
ax.add_patch(Rectangle((IGN, y + SH + 0.06), END - IGN, 0.22, fc="#f1eaf7",
                       ec=VIOLET, lw=0.9, zorder=4))
ax.text(0.5 * (IGN + END), y + SH + 0.17,
        "costates propagated continuously — Weierstrass–Erdmann",
        ha="center", va="center", fontsize=6.2, color=VIOLET, zorder=5)
span(IGN, END, y - 0.10,
     r"the same four timing variables, plus $\lambda_{0r},\ \lambda_{0v},\ "
     r"\lambda_{0\gamma}$ seeded at ignition")

# (5) segmented multi-law
y = 0.60
arcs(y, [("T", IGN, 5.60), ("C", 5.60, 7.00), ("T", 7.00, END)])
cutoff(END, y, "planned cut-off", event=False)
for x0, x1, lab, c in ((IGN, 4.30, "law 1", GREEN), (4.30, 5.15, "law 2", AMBER),
                       (5.15, END, "law 3 (final)", ACCENT)):
    ax.add_patch(Rectangle((x0, y + SH + 0.06), x1 - x0, 0.22, fc="white",
                           ec=c, lw=1.0, zorder=4))
    ax.text(0.5 * (x0 + x1), y + SH + 0.17, lab, ha="center", va="center",
            fontsize=6.8, color=c, zorder=5)
for xb in (4.30, 5.15):
    ax.plot([xb, xb], [y, y + SH], color=INK, lw=0.9, ls=(0, (2, 1.6)),
            zorder=6)
ax.text(END + 0.16, y + SH + 0.17, "hand-off altitudes\n(optimised)",
        fontsize=6.9, color=GREY, ha="left", va="center", linespacing=1.25)
span(5.60, 7.00, y - 0.10, "a coast falls inside the final segment",
     color=ACCENT)

# --- legend --------------------------------------------------------------
LY = -0.28
ax.add_patch(Rectangle((0.55, LY), 0.42, 0.20, fc=THRUST, ec=INK, lw=0.8))
ax.text(1.05, LY + 0.10, "thrust arc", fontsize=7.4, color=INK, va="center")
ax.add_patch(Rectangle((2.05, LY), 0.42, 0.20, fc=COAST, ec=INK, lw=0.8))
ax.text(2.55, LY + 0.10, "coast arc", fontsize=7.4, color=INK, va="center")
ax.plot([3.75, 3.75], [LY, LY + 0.20], color=INK, lw=1.4)
ax.text(3.87, LY + 0.10, "cut-off located by an event", fontsize=7.4,
        color=INK, va="center")
ax.plot([7.15, 7.15], [LY, LY + 0.20], color=GREY, lw=1.1, ls=(0, (3, 2.2)))
ax.text(7.27, LY + 0.10, "cut-off planned in advance", fontsize=7.4,
        color=GREY, va="center")

ax.set_xlim(-1.75, 12.45)
ax.set_ylim(-0.45, 7.15)
save(fig, "architecture_arc_structures.png")
