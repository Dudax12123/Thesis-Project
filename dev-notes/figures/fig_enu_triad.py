"""Figure: local East-North-Up triad and the velocity decomposition.

Replaces the scanned ENU_Bate.png.  The artwork note at
Thesis_Ascent_Background.tex asked only for the elevation angle to be
relabelled phi -> gamma, phi being reserved for latitude in this work; the
source was a low-resolution scan, so the construction is redrawn instead of
patched.  The geometry follows Bate, Mueller and White, which the caption
already credits.
"""
import numpy as np
import matplotlib.pyplot as plt

from figstyle import (use_thesis_style, blank_axes, save, arrow, angle_arc,
                      INK, GREY, FAINT, THRUST, ACCENT)

use_thesis_style()

# --- pseudo-3D basis -----------------------------------------------------
N2 = np.array([0.30, 0.52])       # north, receding up-right
E2 = np.array([0.92, -0.18])      # east, receding right
U2 = np.array([0.0, 1.0])         # up

GAM, BET = 35.0, 55.0             # elevation and azimuth of the velocity
L = 3.30

cg, sg = np.cos(np.deg2rad(GAM)), np.sin(np.deg2rad(GAM))
cb, sb = np.cos(np.deg2rad(BET)), np.sin(np.deg2rad(BET))

O = np.zeros(2)
P_N = L * cg * cb * N2                    # northward component
P_E = L * cg * sb * E2                    # eastward component
H = P_N + P_E                             # horizontal projection of v
V = H + L * sg * U2                       # the velocity vector tip

fig, ax = plt.subplots(figsize=(4.6, 4.0))
blank_axes(ax)

# --- the vertical plane containing v and its projection -----------------
ax.fill([O[0], H[0], V[0]], [O[1], H[1], V[1]], color="#eef1f6", zorder=1)

# --- axes ----------------------------------------------------------------
for d, ln, lab, ha, va in ((N2, 2.35, "north", "left", "bottom"),
                           (E2, 2.75, "east", "left", "top"),
                           (U2, 2.55, "up", "center", "bottom")):
    p = ln * d
    arrow(ax, O, p, color=GREY, lw=1.1, scale=9, zorder=3)
    ax.text(p[0] + (0.08 if ha == "left" else 0.0),
            p[1] + (0.06 if va == "bottom" else -0.06), lab, fontsize=9,
            color=GREY, ha=ha, va=va, style="italic")

# --- the velocity vector and its components -----------------------------
arrow(ax, O, V, color=INK, lw=2.4, scale=15, zorder=7)
ax.text(V[0] + 0.10, V[1] + 0.04, r"$\vec{v}$", fontsize=13, color=INK,
        ha="left", va="bottom")

for a, b in ((O, P_N), (P_N, H), (O, P_E), (P_E, H), (H, V), (O, H)):
    ax.plot([a[0], b[0]], [a[1], b[1]], color=GREY, lw=0.9, ls=(0, (3, 2.2)),
            zorder=4)

for p, lab, ha, va, off in ((P_N, r"$v_N$", "right", "bottom", (-0.09, 0.05)),
                            (P_E, r"$v_E$", "center", "top", (0.04, -0.10)),
                            (0.5 * (H + V), r"$v_U$", "left", "center",
                             (0.09, 0.0))):
    ax.plot([p[0]], [p[1]], marker="o", ms=3.2, color=THRUST, zorder=6)
    ax.text(p[0] + off[0], p[1] + off[1], lab, fontsize=9.5, color=THRUST,
            ha=ha, va=va)
ax.plot([H[0]], [H[1]], marker="o", ms=3.6, color=GREY, zorder=6)

# --- the two angles ------------------------------------------------------
a_h = np.rad2deg(np.arctan2(H[1], H[0]))
a_v = np.rad2deg(np.arctan2(V[1], V[0]))
a_n = np.rad2deg(np.arctan2(N2[1], N2[0]))
angle_arc(ax, O, 1.42, a_h, a_v, color=ACCENT, label=r"$\gamma$",
          lab_scale=1.20, fontsize=12, lw=1.2)
angle_arc(ax, O, 0.86, a_h, a_n, color=THRUST, label=r"$\beta$",
          lab_scale=1.30, fontsize=12, lw=1.2)

ax.plot([O[0]], [O[1]], marker="o", ms=4.6, color=INK, zorder=8)

ax.set_xlim(-0.60, 3.55)
ax.set_ylim(-0.95, 3.05)
save(fig, "ENU_Bate.png")
