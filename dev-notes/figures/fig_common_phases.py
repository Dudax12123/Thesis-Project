"""Figure: the phases and events common to every ascent, with their values (Ch 5 §5.3, Table 5.4).

Companion of fig_ascent_variables_b.py (Figure 5.1): the same altitude-against-time schematic, but
it marks the variables Table tab:events fixes for all four architectures, each with its value:
the 7.5 s vertical rise, the instantaneous kick to gamma_p, the atmospheric exit at 65 km, MECO at
first-stage propellant exhaustion, separation at MECO + 3 s, second-stage ignition at MECO + 8 s,
the 280 km circular orbit. The second stage is one possible burn-coast-burn, drawn faint (its
structure is what each architecture sets), with the coast marked.
Schematic, not simulator output: neither axis is to scale. Text at 10 pt for \\linewidth (6.3 in).
Output: common_phases.png.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator

from figstyle import use_thesis_style, save, INK, GREY, THRUST

use_thesis_style()
FS = 10
plt.rcParams["font.size"] = FS

# normalized time of each boundary; altitude 1 = target orbit
T_RISE, T_MECO, T_SEP, T_IGN, T_C0, T_C1, T_END = 0.14, 0.78, 0.86, 1.00, 1.36, 1.72, 1.92
H_ORB, H_FAIR = 1.0, 0.27
_curve = PchipInterpolator([0.0, T_RISE, 0.45, T_MECO, T_IGN, 1.18, T_C0, 1.55, T_C1, T_END],
                           [0.0, 0.04, 0.20, 0.38, 0.44, 0.55, 0.62, 0.84, 0.985, 1.0])
h = lambda t: _curve(np.asarray(t, dtype=float))
T_FAIR = float(np.interp(H_FAIR, h(np.linspace(T_RISE, T_MECO, 400)), np.linspace(T_RISE, T_MECO, 400)))

fig = plt.figure(figsize=(6.3, 3.3))
ax = fig.add_axes([0.07, 0.10, 0.91, 0.88])
ax.set_xlim(-0.02, 2.06)
ax.set_ylim(-0.06, 1.22)

PH = [(0.0, T_RISE, "#e6e6e6", "Rise"), (T_RISE, T_MECO, "#e3ebf5", "Stage-1 burn"),
      (T_MECO, T_IGN, "#ffffff", "Staging"), (T_IGN, T_END, "#f4f6f9", "Stage-2 arcs")]
for a, b, c, lab in PH:
    ax.axvspan(a, b, color=c, lw=0, zorder=0)
    ax.axvline(b, color="#d5dbe4", lw=0.6, zorder=0)
    ax.text((a + b) / 2, 1.15, lab, ha="center", va="center", fontsize=FS)


def seg(a, b, **kw):
    t = np.linspace(a, b, 200)
    ax.plot(t, h(t), solid_capstyle="butt", zorder=3, **kw)


seg(0.0, T_RISE, color=INK, lw=1.8)                        # vertical rise
seg(T_RISE, T_MECO, color=THRUST, lw=2.2)                  # stage-1 burn
seg(T_MECO, T_IGN, color=INK, lw=1.0, ls=(0, (1.5, 1.5)))  # staging, unpowered
# second stage: one possible burn-coast-burn, faint, since its arcs are the architecture's
seg(T_IGN, T_C0, color=THRUST, lw=2.2, alpha=0.35)
seg(T_C0, T_C1, color=GREY, lw=1.6, ls=(0, (4, 2.5)))
ax.axvspan(T_C0, T_C1, color="#ffffff", lw=0, zorder=0)   # the coast stands out of the stage-2 band
seg(T_C1, T_END, color=THRUST, lw=2.2, alpha=0.35)
ax.text(1.15, 0.25, "set by each\narchitecture", ha="center", va="center",
        fontsize=FS, color=GREY, style="italic", linespacing=1.05)

# target orbit
ax.axhline(H_ORB, color=INK, lw=0.7, ls=(0, (1.5, 2)), zorder=1)
ax.text(2.05, H_ORB + 0.025, r"circular orbit, $h_f = 280$ km", ha="right", va="bottom", fontsize=FS, color=INK)
ax.plot([T_END], [H_ORB], marker="o", ms=4.2, color=INK, zorder=5)

# fairing jettison altitude
ax.plot([T_RISE, T_MECO], [H_FAIR, H_FAIR], color=INK, lw=0.7, ls=(0, (1.5, 2)), zorder=1)
ax.plot([T_FAIR], [H_FAIR], marker="o", ms=4.2, color=INK, zorder=5)
ax.text(T_FAIR - 0.03, H_FAIR + 0.03, "atmospheric exit,\n$h = 65$ km", ha="right", va="bottom",
        fontsize=FS, linespacing=1.05)


def call(t, xy, text, ha="left"):
    ax.plot([t], [float(h(t))], marker="o", ms=4.2, color=INK, zorder=5)
    ax.annotate(text, xy=(t, float(h(t))), xytext=xy, ha=ha, va="center", fontsize=FS, color=INK,
                linespacing=1.05,
                arrowprops=dict(arrowstyle="-", color=INK, lw=0.7, shrinkA=2, shrinkB=3), zorder=4)


call(T_RISE, (0.02, 0.62), "kick: $\\gamma$ jumps\nto $\\gamma_p$ (searched)")
call(T_MECO, (0.50, 0.80), "MECO: stage-1\npropellant exhausted", ha="center")

# staging events
for t, lab in [(T_SEP, "separation"), (T_IGN, "ignition")]:
    ax.plot([t, t], [float(h(t)) + 0.03, 0.66], color=INK, lw=0.7, zorder=2)
ax.text(T_SEP + 0.015, 0.68, "separation", ha="right", va="bottom", fontsize=FS)
ax.text(T_IGN + 0.01, 0.68, "Stage-2\nignition", ha="left", va="bottom", fontsize=FS, linespacing=1.0)


def span(a, b, y, text):
    ax.annotate("", xy=(b, y), xytext=(a, y),
                arrowprops=dict(arrowstyle="<|-|>", color=INK, lw=0.8, mutation_scale=8, shrinkA=0, shrinkB=0))
    ax.text((a + b) / 2, y - 0.025, text, ha="center", va="top", fontsize=FS, color=INK)


span(0.0, T_RISE, 0.16, r"7.5 s")
span(T_MECO, T_SEP, 0.24, r"3 s")
span(T_MECO, T_IGN, 0.12, r"8 s")

# the second stage's arcs, named: burn, coast, burn
span(T_C0, T_C1, 0.50, "coast")
for a, b in [(T_C0, T_C0), (T_C1, T_C1)]:
    ax.plot([a, a], [0.50, float(h(a)) - 0.02], color=INK, lw=0.5, ls=(0, (1.5, 1.5)), zorder=1)
ax.text((T_IGN + T_C0) / 2, float(h((T_IGN + T_C0) / 2)) - 0.06, "burn", ha="center", va="top",
        fontsize=FS, color=THRUST)
ax.text((T_C1 + T_END) / 2, H_ORB - 0.06, "burn", ha="center", va="top", fontsize=FS, color=THRUST)

ax.set_xticks([])
ax.set_yticks([])
for k in ("top", "right"):
    ax.spines[k].set_visible(False)
ax.set_ylabel("Altitude (not to scale)", fontsize=FS)
ax.text(2.06, -0.08, "Time (not to scale)", ha="right", va="top", fontsize=FS, clip_on=False)

save(fig, "common_phases.png")
