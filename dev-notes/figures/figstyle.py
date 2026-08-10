"""Shared style and drawing helpers for the thesis figures.

Every figure script imports from here so that line weights, fonts and the colour
palette stay identical across the eight drawings.  Output location is taken from
the FIG_OUT environment variable, defaulting to a scratch preview directory, so
that drafts can be rendered without touching the Overleaf repository.
"""
import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyBboxPatch

# --- palette -------------------------------------------------------------
INK = "#1b1b1b"          # primary line/text
GREY = "#8c8c8c"         # construction lines, secondary
FAINT = "#c8c8c8"        # very light guides
THRUST = "#2f4b7c"       # thrust arcs, "active" things
COAST = "#eef1f6"        # coast arcs (fill)
ACCENT = "#b23a2e"       # the point of the figure
GREEN = "#3f7d55"        # second family
AMBER = "#c98a12"        # third family
VIOLET = "#6a4c93"       # fourth family
EARTH = "#e8e4dc"        # planet fill

LAW_COLORS = [THRUST, GREEN, AMBER, VIOLET, ACCENT]

OUT_DIR = os.environ.get(
    "FIG_OUT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "_preview"),
)


def use_thesis_style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 9,
        "axes.linewidth": 0.8,
        "lines.solid_capstyle": "round",
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.03,
    })


def blank_axes(ax):
    ax.set_aspect("equal")
    ax.axis("off")


def save(fig, name):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path)
    plt.close(fig)
    print("wrote " + path)
    return path


# --- drawing helpers -----------------------------------------------------
def arrow(ax, p, d, color=INK, lw=1.5, ls="-", scale=12, zorder=5, alpha=1.0):
    """Arrow from point p along displacement d."""
    ax.annotate(
        "", xy=(p[0] + d[0], p[1] + d[1]), xytext=(p[0], p[1]),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, linestyle=ls,
                        shrinkA=0, shrinkB=0, mutation_scale=scale, alpha=alpha),
        zorder=zorder,
    )


def unit(deg):
    a = np.deg2rad(deg)
    return np.array([np.cos(a), np.sin(a)])


def angle_arc(ax, center, radius, a1, a2, color=INK, label=None,
              lab_scale=1.32, fontsize=9.5, lw=0.9, ls="-"):
    """Arc between two absolute angles (degrees), with a label at its bisector."""
    ax.add_patch(Arc(center, 2 * radius, 2 * radius, angle=0,
                     theta1=a1, theta2=a2, color=color, lw=lw, linestyle=ls,
                     zorder=4))
    if label:
        am = np.deg2rad(0.5 * (a1 + a2))
        rr = radius * lab_scale
        ax.text(center[0] + rr * np.cos(am), center[1] + rr * np.sin(am),
                label, color=color, ha="center", va="center", fontsize=fontsize,
                zorder=6)


def note(ax, x, y, text, color=INK, fontsize=8.2, ha="left", va="center",
         box=True, boxcolor="#ffffff", edge=None, alpha=0.92, **kw):
    bbox = None
    if box:
        bbox = dict(boxstyle="round,pad=0.32", fc=boxcolor,
                    ec=edge if edge else "#00000000", lw=0.7, alpha=alpha)
    return ax.text(x, y, text, color=color, fontsize=fontsize, ha=ha, va=va,
                   bbox=bbox, zorder=8, **kw)


def leader(ax, p_from, p_to, color=GREY, lw=0.7, ls=(0, (2.5, 2))):
    ax.plot([p_from[0], p_to[0]], [p_from[1], p_to[1]], color=color, lw=lw,
            ls=ls, zorder=3)


def brace(ax, x0, x1, y, height=0.16, color=INK, lw=0.9, label=None,
          fontsize=8.2, below=True, labeloff=0.14):
    """Simple curly-ish brace spanning [x0, x1] at height y."""
    s = -1.0 if below else 1.0
    xm = 0.5 * (x0 + x1)
    n = 60
    t = np.linspace(0, 1, n)
    # two quarter-arcs meeting at a central spike
    xs = x0 + (xm - x0) * t
    ys = y + s * height * np.sin(np.pi * t / 2) ** 2 * 0.5
    ax.plot(xs, ys, color=color, lw=lw, zorder=5)
    xs2 = xm + (x1 - xm) * t
    ys2 = y + s * height * np.cos(np.pi * t / 2) ** 2 * 0.5
    ax.plot(xs2, ys2, color=color, lw=lw, zorder=5)
    ax.plot([xm, xm], [y + s * height * 0.5, y + s * height], color=color,
            lw=lw, zorder=5)
    ax.plot([x0, x0], [y, y + s * height * 0.16], color=color, lw=lw, zorder=5)
    ax.plot([x1, x1], [y, y + s * height * 0.16], color=color, lw=lw, zorder=5)
    if label:
        ax.text(xm, y + s * (height + labeloff), label, ha="center",
                va="top" if below else "bottom", fontsize=fontsize, color=color,
                zorder=6)


def box(ax, x, y, w, h, text, fc="#ffffff", ec=INK, lw=0.9, fontsize=8.4,
        color=INK, ls="-", radius=0.06, zorder=5, **kw):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle="round,pad=0,rounding_size=%g" % radius,
                                fc=fc, ec=ec, lw=lw, linestyle=ls, zorder=zorder))
    ax.text(x, y, text, ha="center", va="center", fontsize=fontsize,
            color=color, zorder=zorder + 1, **kw)
