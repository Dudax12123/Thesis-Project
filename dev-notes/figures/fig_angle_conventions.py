"""Figure: angle and force conventions as implemented.

Spec: Thesis_Ascent_Background.tex, the "% FIG-2 (TO DRAW)" block above
\\includegraphics{Figures/angle_conventions.png} in Section 2.4.2.
"""
import numpy as np
import matplotlib.pyplot as plt

from figstyle import (use_thesis_style, blank_axes, save, arrow, unit,
                      angle_arc, note, leader, INK, GREY, FAINT, THRUST,
                      ACCENT, GREEN, AMBER, EARTH)

use_thesis_style()

# --- geometry ------------------------------------------------------------
RE = 2.15                      # drawn Earth radius
DELTA = 74.0                   # vehicle radial angle from +x
R = 3.38                       # drawn radial distance to the vehicle
LAUNCH = 103.0                 # launch site

GAM, THE = 31.0, 47.0          # flight-path and pitch angle above the horizon
HOR = DELTA - 90.0

P, S, L = R * unit(DELTA), RE * unit(DELTA), RE * unit(LAUNCH)
v_dir, b_dir = HOR + GAM, HOR + THE
u = unit(DELTA)

fig, ax = plt.subplots(figsize=(6.2, 4.7))
blank_axes(ax)

# --- planet: surface arc with a thin crust -------------------------------
th = np.linspace(np.deg2rad(48), np.deg2rad(120), 300)
xo, yo = RE * np.cos(th), RE * np.sin(th)
xi, yi = 0.92 * RE * np.cos(th), 0.92 * RE * np.sin(th)
ax.fill(np.r_[xo, xi[::-1]], np.r_[yo, yi[::-1]], color=EARTH, zorder=0)
ax.plot(xo, yo, color=INK, lw=1.3, zorder=2)

# radius to the vehicle, broken so that the centre can be shown compactly
ax.plot([P[0], (1.28 * u)[0]], [P[1], (1.28 * u)[1]], color=GREY, lw=0.8,
        ls=(0, (4, 2.5)), zorder=2)
ax.plot([(0.92 * u)[0], (0.66 * u)[0]], [(0.92 * u)[1], (0.66 * u)[1]],
        color=GREY, lw=0.8, ls=(0, (4, 2.5)), zorder=2)
perp = unit(DELTA - 90)
for d in (1.16, 1.04):
    a, b_ = d * u - 0.09 * perp, d * u + 0.09 * perp
    ax.plot([a[0] - 0.05 * u[0], b_[0] + 0.05 * u[0]],
            [a[1] - 0.05 * u[1], b_[1] + 0.05 * u[1]], color=GREY, lw=0.8,
            zorder=3)
ax.plot([(0.66 * u)[0]], [(0.66 * u)[1]], marker="o", ms=3.4, color=INK,
        zorder=5)
ax.text((0.66 * u)[0] - 0.13, (0.66 * u)[1] - 0.02, "$O$", ha="right",
        va="center", fontsize=10)
ax.text((1.55 * u)[0] - 0.24, (1.55 * u)[1], "$R_E$", color=GREY, fontsize=9.5,
        ha="center", va="center")
ax.text((2.85 * u)[0] - 0.26 * perp[0], (2.85 * u)[1] - 0.26 * perp[1], "$r$", color=GREY, fontsize=10,
        ha="center", va="center")

# altitude h
off = 0.34 * perp
arrow(ax, S + off, P - S, color=GREY, lw=0.9, scale=7)
arrow(ax, P + off, S - P, color=GREY, lw=0.9, scale=7)
hm = 0.5 * (S + P) + off
ax.text(hm[0] + 0.12, hm[1] - 0.04, "$h$", color=GREY, fontsize=10, ha="left",
        va="center")

# downrange s, measured along the surface
tha = np.linspace(np.deg2rad(DELTA + 1.5), np.deg2rad(LAUNCH), 120)
rr = RE * 1.09
ax.plot(rr * np.cos(tha), rr * np.sin(tha), color=ACCENT, lw=1.2, zorder=3)
arrow(ax, rr * unit(DELTA + 4), rr * (unit(DELTA + 1.2) - unit(DELTA + 4)),
      color=ACCENT, lw=1.2, scale=9)
sm = rr * 1.045 * unit(0.5 * (DELTA + LAUNCH))
ax.text(sm[0], sm[1], "$s$", color=ACCENT, fontsize=10.5, ha="center",
        va="bottom")
ax.plot([L[0]], [L[1]], marker="o", ms=3.2, color=INK, zorder=5)
ax.text(L[0] - 0.22, L[1] - 0.20, "launch site", fontsize=7.8, color=GREY,
        ha="right", va="top")

# --- local horizon -------------------------------------------------------
hh = unit(HOR)
ax.plot([P[0] - 0.85 * hh[0], P[0] + 2.42 * hh[0]],
        [P[1] - 0.85 * hh[1], P[1] + 2.42 * hh[1]],
        color=FAINT, lw=1.0, zorder=2)
ax.text(P[0] + 2.46 * hh[0], P[1] + 2.46 * hh[1] - 0.03, "local\nhorizon",
        fontsize=7.8, color=GREY, ha="left", va="top", linespacing=1.25)

# --- velocity, body axis, angles ----------------------------------------
LV, LB = 2.15, 1.65
v, b = LV * unit(v_dir), LB * unit(b_dir)
ax.plot([P[0] - 0.38 * b[0] / LB, P[0] + 1.10 * b[0]],
        [P[1] - 0.38 * b[1] / LB, P[1] + 1.10 * b[1]], color=FAINT, lw=0.9,
        zorder=2)

arrow(ax, P, v, color=INK, lw=1.7, scale=13)
ax.text(P[0] + v[0] + 0.08, P[1] + v[1] + 0.04, r"$\vec{v}$", fontsize=11,
        color=INK, ha="left", va="bottom")

angle_arc(ax, P, 0.78, v_dir, b_dir, color=ACCENT, label=r"$\alpha$",
          lab_scale=1.32, fontsize=11, lw=1.2)
angle_arc(ax, P, 1.22, HOR, v_dir, color=INK, label=r"$\gamma$",
          lab_scale=1.15, fontsize=11)
angle_arc(ax, P, 2.18, HOR, b_dir, color=GREY, label=r"$\theta$",
          lab_scale=1.10, fontsize=11, lw=0.8)

# --- forces --------------------------------------------------------------
FT = 1.65
ft = FT * unit(b_dir)
arrow(ax, P, ft, color=THRUST, lw=2.0, scale=14, zorder=6)
ax.text(P[0] + ft[0] + 0.04, P[1] + ft[1] + 0.07, r"$F_T$", fontsize=11,
        color=THRUST, ha="left", va="bottom")

c = FT * np.cos(np.deg2rad(THE - GAM))
Pc = P + c * unit(v_dir)
leader(ax, Pc, P + ft, color=THRUST, lw=0.9)
ax.plot([Pc[0]], [Pc[1]], marker="|", ms=6, color=THRUST, zorder=6,
        mew=1.1)
ax.text(Pc[0] - 0.04, Pc[1] - 0.13, r"$F_T\cos\alpha$", fontsize=8.4,
        color=THRUST, ha="center", va="top")
mm = 0.5 * (Pc + P + ft)
ax.text(mm[0] + 0.08, mm[1], r"$F_T\sin\alpha$", fontsize=8.4, color=THRUST,
        ha="left", va="center")

arrow(ax, P, 0.90 * unit(v_dir + 180), color=AMBER, lw=1.7, scale=13)
d = 0.90 * unit(v_dir + 180)
ax.text(P[0] + d[0] - 0.05, P[1] + d[1] - 0.04, r"$F_D$", fontsize=11,
        color=AMBER, ha="right", va="top")

arrow(ax, P, 0.90 * unit(v_dir + 90), color=GREEN, lw=1.7, scale=13)
lf = 0.90 * unit(v_dir + 90)
ax.text(P[0] + lf[0] - 0.09, P[1] + lf[1] + 0.02, r"$F_L$", fontsize=11,
        color=GREEN, ha="right", va="bottom")

arrow(ax, P, 0.78 * unit(DELTA + 180), color=INK, lw=1.7, scale=13)
g = 0.78 * unit(DELTA + 180)
ax.text(P[0] + g[0] - 0.12, P[1] + g[1] + 0.02, r"$g(r)$", fontsize=10.5,
        color=INK, ha="right", va="center")

ax.plot([P[0]], [P[1]], marker="o", ms=5.0, color=INK, zorder=7)

# --- the one thing the caption cannot carry ------------------------------
note(ax, -1.52, 4.38,
     r"$\alpha = \theta - \gamma$ drawn positive." "\n"
     r"$\alpha < 0$ pitches the nose below $\vec{v}$:" "\n"
     r"that is the kick manoeuvre.",
     fontsize=8.4, va="top", edge="#dddddd")

ax.set_xlim(-1.62, 4.05)
ax.set_ylim(0.32, 4.52)
save(fig, "angle_conventions.png")
