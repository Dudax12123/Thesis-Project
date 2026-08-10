"""Figure: reference-frame strategy along one trajectory.

Spec: Thesis_Ascent_Background.tex, the "% FIG-3 (TO DRAW)" block above
\\includegraphics{Figures/frame_strategy.png} in Section 2.2.4.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from figstyle import (use_thesis_style, blank_axes, save, arrow, unit,
                      angle_arc, note, leader, INK, GREY, FAINT, THRUST,
                      ACCENT, GREEN, AMBER, EARTH)

use_thesis_style()

ROT_BG = "#eaf0f7"      # rotating-frame band
INE_BG = "#f5f0e6"      # inertial-frame band

fig = plt.figure(figsize=(7.1, 5.9))
gs = GridSpec(2, 2, height_ratios=[1.80, 1.0], hspace=0.14, wspace=0.08,
              figure=fig)
ax = fig.add_subplot(gs[0, :])
axg = fig.add_subplot(gs[1, 0])
axv = fig.add_subplot(gs[1, 1])
for a in (ax, axg, axv):
    blank_axes(a)

# ---------------------------------------------------------------- main ---
X0, X1, SECO = 1.10, 9.70, 6.85


def ground(x):
    return 0.32 - 0.0060 * (x - 4.0) ** 2


def traj(x):
    return ground(x) + 4.15 * ((x - X0) / (X1 - X0)) ** 0.60


xs = np.linspace(X0, X1, 400)

ax.axvspan(0.45, SECO, color=ROT_BG, zorder=0)
ax.axvspan(SECO, 10.55, color=INE_BG, zorder=0)

gx = np.linspace(0.45, 10.55, 200)
ax.fill_between(gx, -0.42, ground(gx), color=EARTH, zorder=1)
ax.plot(gx, ground(gx), color=INK, lw=1.2, zorder=2)

m1 = xs <= SECO
ax.plot(xs[m1], traj(xs[m1]), color=THRUST, lw=2.4, zorder=4)
ax.plot(xs[~m1], traj(xs[~m1]), color=AMBER, lw=2.4, zorder=4)
ax.plot([SECO, SECO], [-0.42, 5.35], color=INK, lw=1.1, ls=(0, (5, 3)),
        zorder=5)
ax.plot([SECO], [traj(SECO)], marker="o", ms=6.5, color=INK, zorder=7)

ax.plot([X0], [traj(X0)], marker="o", ms=4.2, color=INK, zorder=6)
ax.text(X0 - 0.08, traj(X0) - 0.10, "lift-off", fontsize=8, color=GREY,
        ha="right", va="top")

# band titles, placed in the empty corners
ax.text(1.05, 5.18, "rotating (ECEF) frame", color=THRUST, fontsize=10.5,
        ha="left", va="top", style="italic")
ax.text(10.42, 5.18, "inertial (ECI) frame,\ntwo-body propagation",
        color=AMBER, fontsize=10.5, ha="right", va="top", style="italic",
        linespacing=1.3)
ax.text(10.42, 2.95,
        "orbital elements $a$, $e$, apoapsis,\nperiapsis and $i$ are evaluated\nhere and only here",
        color=GREY, fontsize=8.1, ha="right", va="top", linespacing=1.4)

leader(ax, (SECO, 1.98), (SECO - 0.02, 1.46), color=GREY)
note(ax, SECO + 0.16, 1.36,
     "frame switch at SECO:\n"
     r"ECEF $\rightarrow$ ECI," "\n"
     "pseudo-forces deactivated",
     fontsize=8.2, ha="left", va="top", edge="#cccccc")

# --- point A: the ENU triad and the two pseudo-accelerations ------------
xa = 2.35
A = np.array([xa, traj(xa)])
for d, lab, la, va_ in (((0.60, 0.12), "E", "left", "center"),
                        ((0.0, 0.62), "U", "left", "bottom"),
                        ((-0.34, 0.34), "N", "right", "bottom")):
    d = np.array(d)
    arrow(ax, A, d, color=INK, lw=1.0, scale=7)
    ax.text(A[0] + d[0] + (0.05 if la == "left" else -0.05),
            A[1] + d[1] + (0.03 if va_ == "bottom" else 0.0), lab,
            fontsize=8.4, color=INK, ha=la, va=va_)
ax.plot([A[0]], [A[1]], marker="o", ms=4.8, color=THRUST, zorder=7)

arrow(ax, A, np.array([1.10, -0.50]), color=ACCENT, lw=1.7, scale=12)
ax.text(A[0] + 1.16, A[1] - 0.56, r"$-2\,\vec{\omega}\times\vec{v}_{\rm rot}$",
        fontsize=8.8, color=ACCENT, ha="left", va="top")
arrow(ax, A, np.array([0.88, 0.68]), color=GREEN, lw=1.7, scale=12)
ax.text(A[0] + 0.94, A[1] + 0.74,
        r"$-\vec{\omega}\times(\vec{\omega}\times\vec{r})$", fontsize=8.8,
        color=GREEN, ha="left", va="bottom")

# --- point B: which components are integrated ---------------------------
xb = 5.05
B = np.array([xb, traj(xb)])
ax.plot([B[0]], [B[1]], marker="o", ms=4.8, color=THRUST, zorder=7)
arrow(ax, B, np.array([0.92, 0.34]), color=INK, lw=1.9, scale=13)
ax.text(B[0] + 1.00, B[1] + 0.44, r"$a_{h,\parallel}$", fontsize=9.4,
        color=INK, ha="left", va="bottom")
arrow(ax, B, np.array([0.0, 0.95]), color=INK, lw=1.9, scale=13)
ax.text(B[0] - 0.07, B[1] + 1.00, r"$a_{\rm up}$", fontsize=9.4, color=INK,
        ha="right", va="bottom")
arrow(ax, B, np.array([-0.60, 0.62]), color=GREY, lw=1.5, scale=12,
      ls=(0, (3, 2)))
ax.text(B[0] - 0.66, B[1] + 0.66,
        r"$a_{h,\perp}$" + "\n" + "diagnostic only",
        fontsize=9.4, color=GREY, ha="right", va="bottom", linespacing=1.3)

ax.set_xlim(0.45, 10.55)
ax.set_ylim(-0.42, 5.45)

# ------------------------------------------------------- globe inset ----
axg.set_title("the rotating frame", fontsize=8.8, color=GREY, pad=2)
t = np.linspace(0, 2 * np.pi, 240)
axg.fill(np.cos(t), np.sin(t), color=EARTH, zorder=1)
axg.plot(np.cos(t), np.sin(t), color=GREY, lw=0.9, zorder=2)
axg.plot(np.cos(t), 0.30 * np.sin(t), color=GREY, lw=0.8, ls=(0, (3, 2)),
         zorder=3)
axg.plot([0, 0], [-1.28, 1.05], color=GREY, lw=0.8, zorder=2)
arrow(axg, (0.0, 1.05), (0.0, 0.52), color=ACCENT, lw=1.8, scale=12)
axg.text(0.14, 1.40, r"$\vec{\omega}_E$", color=ACCENT, fontsize=11,
         ha="left", va="center")
lat = 44.0
ls_ = np.array([np.cos(np.deg2rad(lat)) * 0.88, np.sin(np.deg2rad(lat))])
axg.plot([0, ls_[0]], [0, ls_[1]], color=GREY, lw=0.7, ls=(0, (2, 2)))
axg.plot([ls_[0]], [ls_[1]], marker="o", ms=4.4, color=THRUST, zorder=6)
angle_arc(axg, (0, 0), 0.42, 4, lat, color=GREY, label=r"$\phi$",
          lab_scale=1.40, fontsize=9.5, lw=0.8)
axg.text(ls_[0] + 0.13, ls_[1] + 0.05, "launch site", fontsize=7.6,
         color=THRUST, ha="left", va="bottom")
axg.set_xlim(-1.40, 2.05)
axg.set_ylim(-1.40, 1.75)

# --------------------------------------------- velocity-triangle inset --
axv.set_title("conversion at the frame switch", fontsize=8.8, color=GREY,
              pad=2)
o = np.array([0.0, 0.0])
vr = np.array([1.55, 1.02])
om = np.array([0.85, 0.0])
arrow(axv, o, vr, color=THRUST, lw=2.0, scale=13)
axv.text(vr[0] - 0.34, vr[1] + 0.06, r"$\vec{v}_{\rm rot}$", color=THRUST,
         fontsize=10, ha="right", va="bottom")
arrow(axv, vr, om, color=ACCENT, lw=2.0, scale=13)
axv.text(vr[0] + 0.42, vr[1] + 0.10, r"$\vec{\omega}_E\times\vec{r}$",
         color=ACCENT, fontsize=10, ha="center", va="bottom")
arrow(axv, o, vr + om, color=INK, lw=2.0, scale=13)
axv.text(vr[0] + om[0] + 0.12, vr[1] * 0.50, r"$\vec{v}_{\rm ECI}$", color=INK,
         fontsize=10, ha="left", va="center")
axv.text(1.15, -0.42,
         "only the east component gains\n"
         r"the surface term $\omega_E\,r\cos\phi$",
         fontsize=7.9, color=GREY, ha="center", va="top", linespacing=1.4)
axv.set_xlim(-0.40, 3.35)
axv.set_ylim(-1.30, 1.80)

save(fig, "frame_strategy.png")
