"""Figure: geometry of the ground-relative azimuth correction.

Spec: Thesis_Ascent_Background.tex, the "% FIG (TO DRAW)" block above
\\includegraphics{Figures/azimuth_correction.png} in Section 2.2.3.
"""
import numpy as np
import matplotlib.pyplot as plt

from figstyle import (use_thesis_style, blank_axes, save, arrow, unit,
                      angle_arc, note, leader, INK, GREY, FAINT, THRUST,
                      ACCENT, GREEN, EARTH)

use_thesis_style()

# --- the velocity triangle, in the local horizontal plane ---------------
A_I = 55.0                     # inertial azimuth, clockwise from north
VI = 3.05                      # drawn length of V_I
EAST = 0.92                    # site term, deliberately exaggerated

# azimuth measured clockwise from north -> figure angle = 90 - azimuth
vi = VI * unit(90 - A_I)
e = np.array([EAST, 0.0])
vg = vi - e
A_G = np.rad2deg(np.arctan2(vg[0], vg[1]))

fig, ax = plt.subplots(figsize=(6.4, 4.5))
blank_axes(ax)
O = np.array([0.0, 0.0])

# --- compass axes --------------------------------------------------------
arrow(ax, O, np.array([0.0, 3.45]), color=FAINT, lw=1.0, scale=10, zorder=1)
arrow(ax, O, np.array([3.30, 0.0]), color=FAINT, lw=1.0, scale=10, zorder=1)
ax.text(0.0, 3.55, "N", fontsize=10, color=GREY, ha="center", va="bottom")
ax.text(3.40, 0.0, "E", fontsize=10, color=GREY, ha="left", va="center")

# --- the three vectors ---------------------------------------------------
arrow(ax, O, vg, color=THRUST, lw=2.1, scale=14, zorder=6)
ax.text(vg[0] - 0.34, vg[1] + 0.10, r"$V_{\rm rot}$", color=THRUST,
        fontsize=11, ha="right", va="bottom")

arrow(ax, vg, e, color=ACCENT, lw=2.1, scale=14, zorder=6)
ax.text(vg[0] + 0.5 * EAST, vg[1] + 0.13, r"$\omega_E R_E\cos\phi$",
        color=ACCENT, fontsize=10, ha="center", va="bottom")

arrow(ax, O, vi, color=INK, lw=2.1, scale=14, zorder=6)
ax.text(vi[0] + 0.10, vi[1] + 0.02, r"$V_I$", color=INK, fontsize=11,
        ha="left", va="center")

# --- the two azimuths ----------------------------------------------------
angle_arc(ax, O, 1.28, 90 - A_I, 90, color=INK, label=r"$A_I$",
          lab_scale=1.17, fontsize=10.5)
angle_arc(ax, O, 0.80, 90 - A_G, 90, color=THRUST, label=r"$A_G$",
          lab_scale=1.22, fontsize=10.5, lw=1.1)

note(ax, 0.36, 2.62,
     r"$A_G < A_I$: the site already supplies part of the" "\n"
     r"required eastward velocity, so the heading the" "\n"
     r"vehicle must fly is rotated $\it{toward\ north}$.",
     fontsize=8.4, va="center", ha="left", edge="#dddddd")

note(ax, 1.62, -0.62,
     r"The simulator flies $A_I$; $A_G$ is evaluated and reported"
     "\n"
     r"for comparison only. Site term drawn $\approx\!6\times$ oversize"
     "\n"
     r"(in practice $\omega_E R_E\cos\phi \approx 0.05\,V_I$).",
     fontsize=8.2, va="center", ha="center", color=GREY, edge="#eeeeee")

# --- inset: inclination is set by A_I alone -----------------------------
cx, cy, rad = 5.55, 1.72, 1.12
th = np.linspace(0, 2 * np.pi, 240)
ax.fill(cx + rad * np.cos(th), cy + rad * np.sin(th), color=EARTH, zorder=1)
ax.plot(cx + rad * np.cos(th), cy + rad * np.sin(th), color=GREY, lw=0.9,
        zorder=2)
# equator (edge-on ellipse) and the orbital plane through the launch site
t = np.linspace(0, 2 * np.pi, 240)
ax.plot(cx + rad * np.cos(t), cy + 0.30 * rad * np.sin(t), color=GREY,
        lw=0.9, ls=(0, (3, 2)), zorder=3)
inc = 42.0
ca, sa = np.cos(np.deg2rad(inc)), np.sin(np.deg2rad(inc))
ex, ey = rad * np.cos(t), 0.30 * rad * np.sin(t)
ax.plot(cx + ex * ca - ey * sa, cy + ex * sa + ey * ca, color=THRUST, lw=1.4,
        zorder=4)
angle_arc(ax, (cx, cy), 0.52 * rad, 180, 180 + inc, color=THRUST,
          label="$i$", lab_scale=1.34, fontsize=10.5, lw=1.0)
ax.plot([cx + 0.42 * rad], [cy + 0.38 * rad], marker="o", ms=3.4, color=ACCENT,
        zorder=6)
ax.text(cx + 0.52 * rad, cy + 0.46 * rad, "launch\nsite", fontsize=7.4,
        color=ACCENT, ha="left", va="bottom", linespacing=1.2)
ax.text(cx, cy - rad - 0.22,
        "the correction changes the commanded\nheading, not the target inclination",
        fontsize=7.8, color=GREY, ha="center", va="top", linespacing=1.3)
ax.plot([4.20, 4.20], [0.05, 3.30], color=FAINT, lw=0.9, zorder=1)

ax.set_xlim(-0.75, 6.95)
ax.set_ylim(-1.15, 3.70)
save(fig, "azimuth_correction.png")
