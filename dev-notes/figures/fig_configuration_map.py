"""Figure: every configuration the simulator offers, in the order the selections are read.

Spec: Thesis_Appendix_Guidance.tex (Appendix A), Section "Configuration Map". Filled chips are
the options flown in Chapter 6; outlined chips are implemented but not flown. The left spine is
the selection order of Section A.1: the segmented schedule overrides the law and the
architecture, the indirect law overrides the architecture, and the remaining laws pair with
one of four architectures.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from figstyle import use_thesis_style, blank_axes, save, INK, GREY, FAINT, THRUST

use_thesis_style()

W, H = 10.0, 13.6
fig, ax = plt.subplots(figsize=(7.4, 7.4 * H / W))
blank_axes(ax)
ax.set_xlim(0, W)
ax.set_ylim(0, H)

FS = 6.1            # chip font size
CH = 0.27           # chip height
PANEL = "#f6f7f9"


RENDERER = fig.canvas.get_renderer()


def text_w(text, fs=FS):
    """Width of a text string in axis units, measured with the renderer."""
    t = ax.text(0, 0, text, fontsize=fs)
    bb = t.get_window_extent(renderer=RENDERER)
    t.remove()
    inv = ax.transData.inverted()
    return inv.transform((bb.x1, 0))[0] - inv.transform((bb.x0, 0))[0]


def chip_w(text):
    return 1.07 * text_w(text) + 0.26


def chip(x, y, text, flown):
    w = chip_w(text)
    ax.add_patch(FancyBboxPatch((x, y - CH / 2), w, CH,
                                boxstyle="round,pad=0,rounding_size=0.09",
                                fc=THRUST if flown else "white",
                                ec=THRUST if flown else GREY, lw=0.7, zorder=5))
    ax.text(x + w / 2, y, text, ha="center", va="center", fontsize=FS,
            color="white" if flown else INK, zorder=6)
    return w


def row(x0, y, label, options, xmax, lw):
    """A labelled row of chips; wraps onto further lines. Returns the y below it."""
    ax.text(x0, y, label, ha="left", va="center", fontsize=FS, color=GREY, zorder=6)
    x = x0 + lw
    for text, flown in options:
        w = chip_w(text)
        if x + w > xmax:
            x = x0 + lw
            y -= CH + 0.07
        chip(x, y, text, flown)
        x += w + 0.07
    return y - CH - 0.13


def panel(x0, y0, x1, title, rows, lw=None):
    """Grey panel with a title and rows of chips; returns its bottom edge."""
    if lw is None:
        lw = 1.05 * max(text_w(lab) for lab, _ in rows) + 0.28
    y = y0 - 0.30
    ax.text(x0 + 0.12, y0 - 0.16, title, ha="left", va="center", fontsize=7.0,
            color=INK, weight="bold", zorder=6)
    y -= 0.10
    for label, options in rows:
        y = row(x0 + 0.12, y, label, options, x1 - 0.10, lw=lw)
    bottom = y + 0.16
    ax.add_patch(FancyBboxPatch((x0, bottom), x1 - x0, y0 - bottom,
                                boxstyle="round,pad=0,rounding_size=0.10",
                                fc=PANEL, ec=FAINT, lw=0.8, zorder=1))
    return bottom


def step(x, y, text):
    ax.add_patch(FancyBboxPatch((x - 0.62, y - 0.25), 1.24, 0.50,
                                boxstyle="round,pad=0,rounding_size=0.12",
                                fc="white", ec=INK, lw=0.9, zorder=5))
    ax.text(x, y, text, ha="center", va="center", fontsize=6.4, color=INK, zorder=6,
            linespacing=1.2)


def arr(p0, p1, label=None, lab_off=(0.0, 0.10)):
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.8,
                                shrinkA=0, shrinkB=0, mutation_scale=8), zorder=4)
    if label:
        ax.text(0.5 * (p0[0] + p1[0]) + lab_off[0], 0.5 * (p0[1] + p1[1]) + lab_off[1],
                label, ha="center", va="bottom", fontsize=5.8, color=GREY, zorder=6)


T, F = True, False
L, R = 0.05, W - 0.05

# --- 1. mission and environment ------------------------------------------------
y = panel(L, H - 0.05, R, "Mission and environment (any mission; read by every architecture)", [
    ("Target orbit", [("circular, any altitude", T), ("inclination, for the azimuth", T)]),
    ("Earth rotation", [("rotating, with Coriolis and centrifugal", T), ("rotating, without them", F),
                        ("non-rotating", T)]),
    ("Atmosphere", [("drag and lift", T), ("drag only", F), ("none (vacuum)", T)]),
    ("Stage-1 engine", [("pressure-dependent", T), ("sea level", T), ("vacuum", F), ("average", F),
                        ("ramp in time", F)]),
    ("Atmosphere exit", [("altitude", T), ("dynamic pressure", F), ("aerothermal flux", F)]),
    ("Fairing jettison", [("at the exit altitude", T), ("at the atmosphere-exit criterion", F)]),
    ("First-stage kick", [("instantaneous", T), ("triangular (brute force)", F),
                          ("tabulated kick, no search", F)]),
])

# --- 2. the selection spine ----------------------------------------------------
SX = 0.72
X1 = 1.72
y_seg = y - 0.55
step(SX, y_seg, "Segmented\nschedule?")
yb = panel(X1, y_seg + 0.42, R, "Segmented mode: a schedule of laws by altitude", [
    ("Segment laws", [("gravity turn", T), ("Apollo", F), ("vector PEG", T), ("linear tangent", F),
                      ("bilinear tangent", F), ("indirect replay", F)]),
    ("Hand-off altitudes", [("fixed", T), ("chosen by the swarm", T)]),
    ("Burns", [("law-terminated, first burn to the waypoint", T), ("planned by the swarm", F)]),
])
arr((SX + 0.62, y_seg), (X1, y_seg), "yes", lab_off=(0.0, 0.05))

y_ind = yb - 0.62
arr((SX, y_seg - 0.25), (SX, y_ind + 0.25), "no", lab_off=(0.16, -0.06))
step(SX, y_ind, "Indirect\nlaw?")
yb = panel(X1, y_ind + 0.42, R, "Indirect architecture: costates, arc durations and kick by swarm", [
    ("Second-stage frame", [("rotating, with the pseudo-forces", T), ("inertial", F)]),
    ("Refinement", [("Levenberg-Marquardt + continuation", T), ("swarm point only", F)]),
])
arr((SX + 0.62, y_ind), (X1, y_ind), "yes", lab_off=(0.0, 0.05))

y_arc = yb - 0.62
arr((SX, y_ind - 0.25), (SX, y_arc + 0.25), "no", lab_off=(0.16, -0.06))
step(SX, y_arc, "Architecture")
yb = panel(X1, y_arc + 0.42, R, "Architecture (decides the arcs and who chooses them)", [
    ("Brute force", [("apogee check, grid over the kick", T)]),
    ("Coast-parameter", [("swarm over kick, burns and coast", T)]),
    ("Direct insertion", [("swarm", F), ("grid + Brent", T), ("cut-off planned", F),
                          ("law-terminated", T)]),
    ("Reference tracking", [("no search: the reference's plan", F), ("coast of fixed length", F),
                            ("coast to the target altitude", F)]),
])
arr((SX + 0.62, y_arc), (X1, y_arc))

y_law = yb - 0.62
arr((SX, y_arc - 0.25), (SX, y_law + 0.25))
step(SX, y_law, "Guidance\nlaw")
yb = panel(X1, y_law + 0.42, R, "Guidance law and its options", [
    ("Law", [("gravity turn", T), ("linear tangent", T), ("bilinear tangent", T),
             ("constant pitch rate", T), ("exponential pitch", T), ("Apollo", T),
             ("classical PEG", F), ("vector PEG", T)]),
    ("Time-to-go", [("rocket equation", T), ("PEG-derived", F), ("optimizer deadline", F)]),
    ("Update, freeze", [("2 s, 10 s", T), ("any period and threshold", F)]),
    ("Open-loop constants", [("chosen by the swarm", T), ("derived from the state", F)]),
    ("Law-specific", [("CPR rate: swarm", T), ("CPR rate: fixed or from t_go", F),
                      ("exp. pitch: shooting solve", F), ("classical PEG: damped or fixed iteration", F),
                      ("Apollo: thrust magnitude", F)]),
])
arr((SX + 0.62, y_law), (X1, y_law))

# --- 3. output and tooling ----------------------------------------------------
y_out = yb - 0.25
yb = panel(L, y_out, R, "Runs, records and tooling", [
    ("Every run", [("self-describing archive", T), ("list, show, compare, replay", F)]),
    ("Batches", [("results matrix, one process per case", T), ("smoke test", T), ("reduced budget", F),
                 ("subset of cases", T)]),
    ("Plots", [("Chapter 6 figures and tables", T), ("per-run diagnostic suites", F)]),
    ("Diagnostics", [("launch azimuth and achieved inclination", F), ("cross-heading force", T),
                     ("loss budget", T)]),
])

# --- legend --------------------------------------------------------------------
ly = yb - 0.32
w0 = chip(L + 0.10, ly, "used in this thesis", T)
chip(L + 0.10 + w0 + 0.12, ly, "available, not used", F)
ax.set_ylim(ly - 0.25, H)
save(fig, "configuration_map.png")
