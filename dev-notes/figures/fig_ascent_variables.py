"""Figure: the guidance variables placed on the phases of the ascent.

Spec: Chapter 5, Section "Simulator Capabilities" (draft 2026-11-06, Ascent Variables page).
One band of phases from lift-off to orbit; each guidance variable is marked where it acts,
coloured by what the simulator allows for it (fixed, value, choice, optimized). Text at 10 pt
for a figure printed at \\linewidth (6.3 in). Output: ascent_variables.png.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.lines import Line2D

from figstyle import use_thesis_style, save, INK, GREY, THRUST, GREEN

use_thesis_style()
FS = 10
plt.rcParams["font.size"] = FS

KIND = {"F": ("#8c8c8c", "Fixed"), "V": (INK, "Value"), "C": (THRUST, "Choice"), "O": (GREEN, "Optimized")}

fig, ax = plt.subplots(figsize=(6.3, 3.75))
ax.set_xlim(0, 100)
ax.set_ylim(-37, 25)
ax.axis("off")

# --- phase band ---------------------------------------------------------------
Y0, H = 4.0, 6.0
PH = [  # x0, x1, fill, label
    (0.0, 6.0, "#ececec", "Rise"),
    (6.0, 11.5, "#fbeccd", "Kick"),
    (11.5, 32.0, "#e3ebf5", "Stage-1 burn"),
    (32.0, 42.0, "#ffffff", "Staging"),
    (42.0, 62.0, "#e3ebf5", "Burn 1"),
    (62.0, 80.0, "#f4f6f9", "Coast"),
    (80.0, 93.0, "#e3ebf5", "Burn 2"),
]
for x0, x1, c, lab in PH:
    ax.add_patch(Rectangle((x0, Y0), x1 - x0, H, fc=c, ec=INK, lw=0.8))
    ax.text((x0 + x1) / 2, Y0 + H / 2, lab, ha="center", va="center", fontsize=FS)
ax.annotate("", xy=(99.5, Y0 + H / 2), xytext=(93, Y0 + H / 2),
            arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.9, mutation_scale=10))
ax.text(99.8, Y0 + H + 0.8, "Orbit,\n" + r"target $h_f$", ha="right", va="bottom", fontsize=FS,
        color=KIND["V"][0], linespacing=1.05)

# events above the band
ax.plot([32, 32], [Y0 + H, Y0 + H + 2.2], color=INK, lw=0.8)
ax.text(31.6, Y0 + H + 2.6, "MECO", ha="right", va="bottom", fontsize=FS)
ax.plot([42, 42], [Y0 + H, Y0 + H + 2.2], color=INK, lw=0.8)
ax.text(42.4, Y0 + H + 2.6, "Stage-2 ignition", ha="left", va="bottom", fontsize=FS)


def bracket(x0, x1, y, lab):
    ax.plot([x0, x0, x1, x1], [y - 1.0, y, y, y - 1.0], color=GREY, lw=0.8)
    ax.text((x0 + x1) / 2, y + 0.6, lab, ha="center", va="bottom", fontsize=FS, color=GREY)


bracket(0.3, 31.7, 21.0, "First stage")
bracket(42.3, 92.7, 21.0, "Second stage: one burn, or burn–coast–burn")


# --- variables below the band --------------------------------------------------
def var(x, y, text, kind, anchor="center", lead=True):
    col = KIND[kind][0]
    if lead:
        ax.plot([x, x], [Y0, y + 1.6], color=col, lw=0.8, ls=(0, (2, 1.5)))
        ax.plot([x], [Y0], marker="o", ms=3.2, color=col, zorder=5)
    ax.text(x, y, text, ha=anchor, va="top", fontsize=FS, color=col, linespacing=1.05)


var(6.0, 1.0, r"$t_k$", "V")
var(8.75, -6.2, "kick profile", "C")
var(8.75, -10.4, r"$\gamma_p$", "O", lead=False)
var(24.0, 1.0, r"hand-off $h_{\mathrm{ho}}$", "O")
var(48.0, 1.0, r"$\Delta t_r\%$", "O")
var(55.0, -6.2, "coast start", "O")
var(62.0, -11.6, "burn end", "C")
var(71.0, 1.0, r"$\Delta t_c$", "O")
var(80.0, -6.2, "coast end", "C")
var(93.0, -11.6, "orbit closure", "C", anchor="right")

# steering lane: law 1 then law 2, from the end of the kick to the final cut-off
YL = -20.0
ax.add_patch(FancyBboxPatch((11.5, YL - 1.6), 12.5, 3.2, boxstyle="round,pad=0,rounding_size=1.2",
                            fc="#eef3f9", ec=THRUST, lw=0.8))
ax.add_patch(FancyBboxPatch((24.0, YL - 1.6), 69.0, 3.2, boxstyle="round,pad=0,rounding_size=1.2",
                            fc="#eef3f9", ec=THRUST, lw=0.8))
ax.text(17.75, YL, "law 1", ha="center", va="center", fontsize=FS, color=THRUST)
ax.text(58.5, YL, r"law 2: any law, closed-loop, open-loop ($\mathbf{c}$) or optimal ($\boldsymbol{\lambda}_0$)",
        ha="center", va="center", fontsize=FS, color=THRUST)
ax.text(0.0, YL, "Steering", ha="left", va="center", fontsize=FS, color=INK)
# law 1 may be any law but the optimal-control one: a limitation, so drawn in the fixed colour
ax.text(11.5, YL - 4.4, "law 1: any law except optimal control", ha="left", va="center", fontsize=FS,
        color=KIND["F"][0])
ax.text(0.0, YL - 8.6, r"Control: angle of attack $\alpha$ only, in the plane of the trajectory",
        ha="left", va="center", fontsize=FS, color=KIND["F"][0])

handles = [Line2D([0], [0], marker="o", ls="", color=KIND[k][0], ms=5, label=KIND[k][1]) for k in "FVCO"]
ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, -0.03), ncol=4, frameon=False,
          fontsize=FS, handletextpad=0.3, columnspacing=1.4)

save(fig, "ascent_variables.png")
