"""Figure: where each guidance law is typically used along the ascent.

Spec (user, 2026-10-27): replaces Table 3.1 (tab:guidance_laws) of the guidance chapter,
Thesis_Guidance.tex. The figure only places the laws; each law's basis is in its own section.
A schematic ascent in altitude against downrange, split at the atmosphere exit into the
endo-atmospheric arc (gravity turn, constant pitch rate) and the exo-atmospheric arc, whose laws
are grouped as the chapter groups them: open-loop programs, explicit closed-loop laws and the
costate-optimal law. Monochrome on purpose (2026-10-27: the coloured version was too flashy, the
dot matrix too simple).

Schematic, not data: the altitude axis is stretched below the atmosphere exit. The thesis
places it at \linewidth: every label is drawn at FS so that none prints below the 10 pt body text.
"""
import numpy as np
import matplotlib.pyplot as plt

from figstyle import use_thesis_style, save, INK, GREY, FAINT

use_thesis_style()
FS = 10.5

fig, ax = plt.subplots(figsize=(4.9, 4.7))
ax.set_xlim(0, 10)
ax.set_ylim(0, 12)
ax.axis("off")

H_EXIT = 2.2                                   # atmosphere exit, drawn expanded

# endo-atmospheric band
ax.add_patch(plt.Rectangle((0, 0), 10, H_EXIT, fc="#eeeeee", ec="none", zorder=0))
ax.plot([0, 10], [H_EXIT, H_EXIT], color=GREY, lw=0.9, ls=(0, (4, 3)), zorder=1)
ax.plot([0, 10], [0, 0], color=INK, lw=1.0, zorder=2)

# schematic ascent: steep climb, then a progressive turn to the horizontal
x = np.linspace(0, 9.6, 400)
h = 10.6 * (1 - np.exp(-(x / 3.0) ** 1.25)) ** 0.9
ax.plot(x, h, color=INK, lw=1.6, zorder=3)
ax.annotate("", xy=(9.75, h[-1] + 0.03), xytext=(9.45, h[-1] - 0.02),
            arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.2), zorder=3)

ax.text(1.7, H_EXIT + 0.15, "Atmosphere exit", fontsize=FS, color=GREY, va="bottom")
ax.text(9.85, 0.15, "Endo-atmospheric arc", fontsize=FS, color=INK, ha="right", va="bottom")
ax.text(9.85, 11.6, "Exo-atmospheric arc", fontsize=FS, color=INK, ha="right", va="top")

# endo laws, in the band
ax.text(2.6, 1.1, "Gravity turn\nConstant pitch rate", fontsize=FS, color=INK, va="center",
        linespacing=1.25)

# exo laws, grouped as the chapter groups them
groups = [
    ("Open-loop programs", ["Linear tangent", "Bilinear tangent", "Exponential pitch"]),
    ("Explicit closed-loop", ["Polynomial (Apollo)", "Powered explicit (PEG)"]),
    ("Costate-optimal", ["Indirect (PMP)"]),
]
y = 8.2
for title, laws in groups:
    ax.text(4.9, y, title, fontsize=FS, color=GREY, style="italic", va="top")
    y -= 0.52
    for law in laws:
        ax.text(5.2, y, law, fontsize=FS, color=INK, va="top")
        y -= 0.52
    y -= 0.15

save(fig, "guidance_laws_arcs.png")
