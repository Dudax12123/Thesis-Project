"""Figure: the two kick parametrisations.

Spec: Thesis_Guidance.tex, the "% FIG-6 (TO DRAW)" block above
\\includegraphics{Figures/kick_profiles.png} in Section 4.2.

The alpha traces are the configured profiles exactly (T_k, dT_k and the kick
amplitude are read from Input_File/simulation_parameters.py section 7).  The
gamma traces are ILLUSTRATIVE: a smooth monotone gravity-turn shape anchored on
the documented MECO flight-path angle, as the spec asks for.  They are not
simulator output and must not be read as such.
"""
import numpy as np
import matplotlib.pyplot as plt

from figstyle import (use_thesis_style, save, INK, GREY, FAINT, THRUST,
                      ACCENT, AMBER)

use_thesis_style()

# --- configured kick parameters (simulation_parameters.py, section 7) ----
T_K = 7.5          # TIME_TO_START_KICK        [s]
DT_K = 45.0        # DURATION_INITIAL_KICK     [s]
ALPHA_K = -3.0     # INITIAL_KICK_ANGLE        [deg]
GAMMA_P = 88.4     # instantaneous variant: gamma_p [deg]  (bound 88.24-89.95)

TEND = 165.0
GAM_MECO = 25.0    # documented MECO flight-path angle, degrees


def alpha_triangular(t):
    a = np.zeros_like(t)
    half = DT_K / 2.0
    up = (t >= T_K) & (t <= T_K + half)
    dn = (t > T_K + half) & (t <= T_K + DT_K)
    a[up] = ALPHA_K * (t[up] - T_K) / half
    a[dn] = ALPHA_K * (1.0 - (t[dn] - T_K - half) / half)
    return a


def gamma_shape(t, gam0):
    """Smooth monotone gravity turn from gam0 at T_K to GAM_MECO at TEND."""
    g = np.full_like(t, gam0)
    m = t >= T_K
    frac = ((t[m] - T_K) / (TEND - T_K)) ** 0.80
    g[m] = gam0 - (gam0 - GAM_MECO) * frac
    return g


t = np.linspace(0.0, TEND, 2000)
a_tri = alpha_triangular(t)
g_tri = gamma_shape(t, 90.0)
a_ins = np.zeros_like(t)
g_ins = gamma_shape(t, GAMMA_P)
g_ins[t < T_K] = 90.0

fig, axs = plt.subplots(2, 2, figsize=(7.0, 4.6), sharex=True,
                        gridspec_kw=dict(height_ratios=[1.0, 1.30],
                                         hspace=0.18, wspace=0.20))
(axa, axb), (axc, axd) = axs
for a in axs.ravel():
    a.spines[["top", "right"]].set_visible(False)
    a.tick_params(labelsize=8, length=3, width=0.7)
    a.set_xlim(0, TEND)

# ------------------------------------------------------- panel (a) -----
axa.set_title("(a) triangular profile — two parameters", fontsize=9.4,
              color=INK, pad=6)
axa.plot(t, a_tri, color=ACCENT, lw=1.9)
axa.axhline(0, color=FAINT, lw=0.8)
axa.set_ylabel(r"$\alpha$  [deg]", fontsize=9)
axa.set_ylim(-4.2, 1.5)
for x in (T_K, T_K + DT_K / 2, T_K + DT_K):
    axa.plot([x, x], [-4.2, np.interp(x, t, a_tri)], color=GREY, lw=0.7,
             ls=(0, (2.5, 2)))
axa.plot([0, T_K + DT_K / 2], [ALPHA_K, ALPHA_K], color=GREY, lw=0.7,
         ls=(0, (2.5, 2)))
axa.text(1.0, ALPHA_K + 0.20, r"$\alpha_k$", fontsize=9.5, color=ACCENT)
axa.annotate("", xy=(T_K, 0.85), xytext=(T_K + DT_K, 0.85),
             arrowprops=dict(arrowstyle="<|-|>", color=INK, lw=0.9,
                             mutation_scale=8))
axa.text(T_K + DT_K / 2, 1.00, r"$\Delta T_k$", fontsize=9, ha="center",
         va="bottom", color=INK)
axa.text(T_K - 1.0, -4.05, "$T_k$", fontsize=7.8, color=GREY, ha="right",
         va="bottom")
axa.text(T_K + DT_K + 2.0, -4.05, r"$T_k+\Delta T_k$", fontsize=7.8,
         color=GREY, ha="left", va="bottom")
axa.text(TEND * 0.98, -2.2,
         "two parameters:\n"
         r"amplitude $\alpha_k$ and duration $\Delta T_k$",
         fontsize=7.6, color=GREY, ha="right", va="center", linespacing=1.35)

axc.plot(t, g_tri, color=THRUST, lw=2.0)
axc.set_ylabel(r"$\gamma$  [deg]", fontsize=9)
axc.set_xlabel("time from lift-off  [s]", fontsize=9)
axc.set_ylim(15, 97)
for x in (T_K, T_K + DT_K):
    axc.axvline(x, color=GREY, lw=0.7, ls=(0, (2.5, 2)))
axc.plot([T_K], [90.0], marker="o", ms=4.2, color=THRUST, zorder=6)
axc.text(T_K + DT_K + 5, 88,
         r"$\alpha$ returns to zero, so $\gamma$ stays" "\n"
         "continuous: the gravity turn begins\nfrom a well-defined state",
         fontsize=7.6, color=GREY, va="top", linespacing=1.4)

# ------------------------------------------------------- panel (b) -----
axb.set_title("(b) instantaneous profile — one parameter", fontsize=9.4,
              color=INK, pad=6)
axb.plot(t, a_ins, color=ACCENT, lw=1.9)
axb.axhline(0, color=FAINT, lw=0.8)
axb.set_ylim(-4.2, 1.5)
axb.axvline(T_K, color=GREY, lw=0.7, ls=(0, (2.5, 2)))
axb.text(T_K - 1.0, -4.05, "$T_k$", fontsize=7.8, color=GREY, ha="right",
         va="bottom")
axb.text(TEND * 0.55, -1.9, r"$\alpha \equiv 0$ throughout", fontsize=10,
         color=ACCENT, ha="center", va="center")
axb.text(TEND * 0.98, -3.4,
         "one parameter: $\\gamma_p$,\n"
         r"with kick angle $=\gamma_p - \pi/2$",
         fontsize=7.6, color=GREY, ha="right", va="center", linespacing=1.35)

axd.plot(t, g_ins, color=THRUST, lw=2.0)
axd.plot([T_K, T_K], [90.0, GAMMA_P], color=AMBER, lw=2.4, zorder=6)
axd.plot([T_K], [90.0], marker="o", ms=4.2, mfc="white", mec=THRUST,
         mew=1.2, zorder=7)
axd.plot([T_K], [GAMMA_P], marker="o", ms=4.2, color=AMBER, zorder=7)
axd.set_xlabel("time from lift-off  [s]", fontsize=9)
axd.set_ylim(15, 97)
axd.axvline(T_K, color=GREY, lw=0.7, ls=(0, (2.5, 2)))
axd.text(TEND * 0.50, 86,
         "the state is discontinuous at $T_k$;\n"
         "the step is small in absolute terms\n"
         r"($\gamma_p$ is bounded to $88.2^\circ\!-\!90^\circ$)",
         fontsize=7.6, color=GREY, va="top", ha="left", linespacing=1.4)

# zoom inset: the step is only a degree or two, so magnify it
axz = axd.inset_axes([0.055, 0.10, 0.30, 0.36])
zt = np.linspace(3.0, 13.0, 400)
zg = gamma_shape(zt, GAMMA_P)
zg[zt < T_K] = 90.0
axz.plot(zt, zg, color=THRUST, lw=1.6)
axz.plot([T_K, T_K], [90.0, GAMMA_P], color=AMBER, lw=2.0)
axz.plot([T_K], [90.0], marker="o", ms=3.4, mfc="white", mec=THRUST, mew=1.0)
axz.plot([T_K], [GAMMA_P], marker="o", ms=3.4, color=AMBER)
axz.set_xlim(3.0, 13.0)
axz.set_ylim(87.4, 90.6)
axz.tick_params(labelsize=6, length=2, width=0.6, pad=1.5)
axz.set_yticks([88, 89, 90])
axz.set_xticks([5, 10])
for sp in axz.spines.values():
    sp.set_linewidth(0.7)
    sp.set_color(GREY)
axz.set_title(r"step to $\gamma_p$ (detail)", fontsize=6.8, color=GREY, pad=2)

fig.align_ylabels([axa, axc])
save(fig, "kick_profiles.png")
