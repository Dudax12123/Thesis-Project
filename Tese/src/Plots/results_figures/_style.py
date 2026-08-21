"""Shared style for the Chapter 6 results figures.

The palette and rcParams are those of ``dev-notes/figures/figstyle.py``, so the
figures built from simulator output sit beside the hand-drawn conceptual ones
without a visible seam. They are restated here rather than imported because
``dev-notes/`` is not part of the simulator and is not importable from it.

Output goes to ``FIG_OUT`` when set, so drafts render to a scratch directory and
a final pass writes straight into the thesis repository, exactly as the
conceptual figures do.
"""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# --- palette (identical to dev-notes/figures/figstyle.py) ----------------
INK = "#1b1b1b"
GREY = "#8c8c8c"
FAINT = "#c8c8c8"
THRUST = "#2f4b7c"
ACCENT = "#b23a2e"
GREEN = "#3f7d55"
AMBER = "#c98a12"
VIOLET = "#6a4c93"

# --- semantic roles ------------------------------------------------------
# Every figure in the chapter is a baseline with one thing changed, so the
# colours carry that meaning rather than being assigned per figure: the reader
# learns once that dark blue is the case being varied against.
BASELINE = THRUST
VARIANT = ACCENT
VARIANT2 = AMBER
VARIANT3 = VIOLET
REFERENCE = GREEN
FAILED = GREY

VARIANT_CYCLE = [VARIANT, VARIANT2, VARIANT3, GREEN, INK]

# Loss components, in the order they are stacked in the budget bars.
LOSS_COLORS = {
    "gravity": THRUST,
    "drag": ACCENT,
    "steering": AMBER,
    "pressure": VIOLET,
}

# --- figure sizes, in inches, against a ~6.3 in thesis text width --------
WIDE_1 = (6.3, 3.4)      # one panel, full width
WIDE_2 = (6.3, 2.9)      # two panels side by side
WIDE_4 = (6.3, 5.0)      # 2x2 card
TALL_1 = (6.3, 4.2)      # one panel, ranking bars

OUT_DIR = os.environ.get(
    "FIG_OUT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "_preview"),
)

# Pretty names for the guidance laws, used in every legend and table.
LAW_LABELS = {
    "gravity_turn": "Gravity turn",
    "linear_tangent": "Linear tangent",
    "bilinear_tangent": "Bilinear tangent",
    "apollo": "Apollo",
    "cpr": "Constant pitch rate",
    "peg": "PEG",
    "peg_new": "PEG (vector P-C)",
    "exp_shooting": "Polynomial shooting",
    "indirect_pmp": "Indirect (PMP)",
}

ARCH_LABELS = {
    "pso_coast": "PSO coast",
    "apogee_check": "Apogee check",
    "direct": "Direct",
    "indirect_pmp": "Indirect PMP",
    "segmented": "Segmented",
}


def law_label(name):
    return LAW_LABELS.get(name, str(name))


def arch_label(name):
    return ARCH_LABELS.get(name, str(name))


def use_thesis_style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 8.5,
        "axes.labelsize": 8.5,
        "axes.titlesize": 9,
        "legend.fontsize": 7.5,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "axes.linewidth": 0.8,
        "lines.linewidth": 1.3,
        "lines.solid_capstyle": "round",
        "legend.frameon": False,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.03,
    })


def tidy(ax, legend=True, legend_loc="best"):
    """The house treatment: light grid, no top/right spines, optional legend."""
    ax.grid(True, color=FAINT, linewidth=0.5, alpha=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK)
    if legend and ax.get_legend_handles_labels()[0]:
        ax.legend(loc=legend_loc)


def panel_tag(ax, letter):
    """The (a)/(b) label, placed outside the axes so it never covers a curve."""
    ax.set_title("(%s)" % letter, loc="left", color=INK, fontweight="bold")


def add_events(ax, case, meco=True, coast=True, seco=True):
    """Vertical markers for the arc boundaries on a time-axis panel.

    Events that never happened are stored as NaN by the harness rather than
    omitted, so an absent marker is skipped here rather than raising.
    """
    marks = []
    if meco:
        marks.append((case.t_meco, "MECO", AMBER))
    if coast:
        marks.append((case.t_coast_start, "Coast", GREY))
    if seco:
        marks.append((case.t_seco, "SECO", INK))
    for t_evt, label, colour in marks:
        if t_evt is None:
            continue
        ax.axvline(t_evt, color=colour, linestyle=":", linewidth=0.9, alpha=0.9)
        ax.annotate(label, xy=(t_evt, 1.0), xycoords=("data", "axes fraction"),
                    xytext=(2, -9), textcoords="offset points",
                    fontsize=6.5, color=colour, rotation=90, va="top")


def save(fig, name):
    """Write one PNG under FIG_OUT, named as the thesis includegraphics expects."""
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path)
    plt.close(fig)
    print("  wrote %s" % path)
    return path


def thin(*arrays, **kwargs):
    """Decimate parallel arrays for plotting, always keeping the last sample.

    A dense run carries tens of thousands of samples; at 300 dpi nothing below
    every fifth point is visible, and keeping them all makes the PNG large for
    no gain. The endpoint is preserved because insertion is the sample that
    matters most.
    """
    import numpy as _np

    step = kwargs.pop("step", 5)
    if step <= 1:
        return arrays if len(arrays) > 1 else arrays[0]

    # Whether the endpoint survives the slice is decided once, from the index,
    # and applied to every array. Deciding it per array by comparing values
    # would drop the appended sample from any array whose last two samples
    # happen to be equal, leaving parallel arrays of different lengths.
    lengths = {len(a) for a in arrays if a is not None}
    n = lengths.pop() if len(lengths) == 1 else None
    keep_last = n is not None and n > 0 and (n - 1) % step != 0

    out = []
    for a in arrays:
        if a is None:
            out.append(None)
            continue
        thinned = a[::step]
        if keep_last:
            thinned = _np.append(thinned, a[-1])
        out.append(thinned)
    return tuple(out) if len(out) > 1 else out[0]


def shade_coast(ax, case, x="time", color=FAINT, label=None):
    """Shade the unpowered arcs, so the arc structure is visible per case.

    Derived from the thrust trace rather than from a stored coast interval:
    every architecture produces a thrust history, and only some of them record
    where they put a coast.
    """
    for t0, t1 in case.coast_intervals():
        if x == "time":
            ax.axvspan(t0, t1, color=color, alpha=0.45, linewidth=0, label=label)
        label = None


def bar_size(n_rows, row_height=0.42, floor=1.6):
    """Figure size for a horizontal bar chart with *n_rows* bars.

    Fixed-height bar charts are wrong at both ends: a six-row budget is cramped
    at the height a two-row one needs, and a single-row chart drawn at a fixed
    height stretches its one bar over the whole canvas. The height follows the
    row count instead, with a floor so a very short chart still has room for its
    axis labels.
    """
    return (6.3, max(floor, floor + row_height * max(n_rows - 2, 0)))
