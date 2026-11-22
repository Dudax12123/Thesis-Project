"""Figure: waterfall of the variables of the ascent, from all of them down to each architecture's design vector.

Spec: Thesis_Implementation.tex, Section 5.2.5 (Summary of the Architectures). Top: every variable; below it
the variables defined in all cases (Table tab:events); then, for each architecture A-D, the variables it
defines and, in parallel, the variables left to the optimizer (its design vector). Text at 10 pt at \\linewidth.
Output arch_waterfall.png.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

from figstyle import use_thesis_style, save, INK

use_thesis_style()
FS = 10.2
W = 6.30
PW = 6.30
BAND, LINE, PADX, GAPX, BUL = 0.22, 0.20, 0.12, 0.15, 0.17

BLUE = dict(fill="#f3f6fb", line="#8aa4cc", band="#dce6f4", dot="#4f77b3")
GREEN = dict(fill="#f2f9f5", line="#86bb9c", band="#d6ecde", dot="#3f8a5f")
GREY = dict(fill="#ffffff", line="#b9b9b9", band="#ececec", dot="#777777")

COMMON = ["Vertical Rise", "Kick Profile", "Fairing Jettison", "MECO", "Stage Separation",
          "Second-Stage Ignition", "Final Orbit"]
LEFT = ["Pitch-Over Angle", "Coast Duration", "Burn Fraction", "Coast Start", "Guidance Laws", "Hand-Off Criteria",
        "Law Constants", "Initial Costates"]
SYM = {"Pitch-Over Angle": r"$\gamma_p$", "Coast Duration": r"$\Delta t_c$", "Burn Fraction": r"$\Delta t_r\%$",
       "Coast Start": r"coast-start$\%$", "Law Constants": r"$\mathbf{c}$", "Initial Costates": r"$\boldsymbol{\lambda}_0$"}

ARCH = [
    ("A", "Segmented Multi-Law",
     ["Guidance Laws: Two, Chosen", r"Hand-Off Criteria: Altitude $h_{\mathrm{ho}}$"],
     [r"$\gamma_p$", r"$\Delta t_c$", r"$\Delta t_r\%$", "Coast Start", r"$\mathbf{c}$"], "4 to 10"),
    ("B", "PSO Optimized Coast",
     ["Guidance Laws: One, Chosen", "Hand-Off Criteria: MECO"],
     [r"$\gamma_p$", r"$\Delta t_c$", r"$\Delta t_r\%$", "Coast Start", r"$\mathbf{c}$"], "4 to 7"),
    ("C", "Optimal Control",
     ["Guidance Laws: One, Optimal Control", "Hand-Off Criteria: MECO"],
     [r"$\gamma_p$", r"$\Delta t_c$", r"$\Delta t_r\%$", "Coast Start", r"$\boldsymbol{\lambda}_0$"], "7"),
    ("D", "Direct Insertion",
     ["Guidance Laws: One, Chosen", r"Coast Duration: $0$", "Hand-Off Criteria: MECO"],
     [r"$\gamma_p$", r"$\Delta t_r\%$", r"$\mathbf{c}$"], "1 to 5"),
]

fig = plt.figure(figsize=(W, 1))
ax = fig.add_axes([0, 0, 1, 1])
fig.canvas.draw()
rend = fig.canvas.get_renderer()


def tw(t, **kw):
    tx = ax.text(0, 0, t, fontsize=FS, **kw)
    w = tx.get_window_extent(rend).width / fig.dpi
    tx.remove()
    return w


def flow(items, width):
    lines, cur, x = [], [], 0.0
    for it in items:
        w = BUL + tw(it)
        if cur and x + w > width:
            lines.append(cur)
            cur, x = [], 0.0
        cur.append((it, x))
        x += w + GAPX
    lines.append(cur)
    return lines


def height(items, w):
    return BAND + 0.03 + len(flow(items, w - 2 * PADX)) * LINE + 0.03


def rbox(x0, y0, x1, y1, fc, ec, lw=1.1, r=0.09, z=1):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=%g" % r,
                                fc=fc, ec=ec, lw=lw, zorder=z))


def box(x0, y0, w, title, items, pal):
    """Card with a tinted title band; y grows downward. Returns its height."""
    h = height(items, w)
    rbox(x0, y0, x0 + w, y0 + h, pal["fill"], pal["line"], z=1)
    rbox(x0, y0, x0 + w, y0 + BAND, pal["band"], pal["line"], z=2)
    ax.add_patch(Rectangle((x0 + 0.012, y0 + BAND * 0.5), w - 0.024, BAND * 0.5, fc=pal["band"], ec="none", zorder=3))
    ax.plot([x0 + 0.012, x0 + w - 0.012], [y0 + BAND] * 2, color=pal["line"], lw=0.8, zorder=4)
    ax.text(x0 + PADX, y0 + BAND / 2, title, fontsize=FS, fontweight="bold", color=INK, va="center", zorder=5)
    for li, line in enumerate(flow(items, w - 2 * PADX)):
        yy = y0 + BAND + 0.03 + li * LINE + LINE / 2
        for it, xo in line:
            ax.plot([x0 + PADX + 0.04 + xo], [yy], "o", ms=4.0, color=pal["dot"], zorder=5)
            ax.text(x0 + PADX + BUL + xo, yy, it, fontsize=FS, color=INK, va="center", zorder=5)
    return h


def arrow(xy, xytext, col):
    ax.annotate("", xy=xy, xytext=xytext, zorder=6,
                arrowprops=dict(arrowstyle="-|>", color=col, lw=1.2, shrinkA=0, shrinkB=0, mutation_scale=9))


X0, X1 = 0.04, W - 0.04
TR = X0 + 0.14                      # x of the trunk
IND = 0.38                          # indent of the architecture rows
y = 0.02
LEFTSYM = [("%s (%s)" % (n, SYM[n])) if n in SYM else n for n in LEFT]
wbox = X1 - X0
secs = [("Defined in All Ascent Cases:", COMMON, BLUE), ("Defined by the Ascent Case or Left to Its Design Vector:", LEFTSYM, GREY)]
nl_ = [len(flow([h_] + its, wbox - 2 * PADX)) for h_, its, _ in secs]
SEP = 0.05
h = BAND + 0.03 + sum(nl_) * LINE + SEP + 0.03
rbox(X0, y, X1, y + h, GREY["fill"], GREY["line"], z=1)
rbox(X0, y, X1, y + BAND, GREY["band"], GREY["line"], z=2)
ax.add_patch(Rectangle((X0 + 0.012, y + BAND * 0.5), wbox - 0.024, BAND * 0.5, fc=GREY["band"], ec="none", zorder=3))
ax.plot([X0 + 0.012, X1 - 0.012], [y + BAND] * 2, color=GREY["line"], lw=0.8, zorder=4)
ax.text(X0 + PADX, y + BAND / 2, "All Variables of the Ascent", fontsize=FS, fontweight="bold", color=INK, va="center", zorder=5)
yy0 = y + BAND + 0.03
for si, (head, its, pal) in enumerate(secs):
    for li, line in enumerate(flow([head] + its, wbox - 2 * PADX)):
        yy = yy0 + li * LINE + LINE / 2
        for k, (it, xo) in enumerate(line):
            if si == 0 and li == 0 and k == 0 or si == 1 and li == 0 and k == 0:
                ax.text(X0 + PADX + 0.04 + xo - 0.04, yy, it, fontsize=FS, fontweight="bold", color=INK, va="center", zorder=5)
                continue
            ax.plot([X0 + PADX + 0.04 + xo], [yy], "o", ms=4.0, color=pal["dot"], zorder=5)
            ax.text(X0 + PADX + BUL + xo, yy, it, fontsize=FS, color=INK, va="center", zorder=5)
    yy0 += nl_[si] * LINE
    if si == 0:
        ax.plot([X0 + 0.012, X1 - 0.012], [yy0 + SEP / 2] * 2, color=GREY["line"], lw=0.6, zorder=4)
        yy0 += SEP
y += h
GAP = 0.13
trunk_top = y
rows = []
y += GAP * 0.6
avail = X1 - (X0 + IND)
for key, name, dfn, dv, nv in ARCH:
    wl = 2.75
    wr = avail - wl - 0.22
    title_l = "Ascent Case %s Defines" % key
    title_r = r"Design Vector $\mathbf{x}$ (%s Variables)" % nv
    hl, hr = height(dfn, wl), height(dv, wr)
    hh = max(hl, hr)
    ys = y
    # equalise heights: draw with the larger height
    for (x0, w, t, its, pal) in ((X0 + IND, wl, title_l, dfn, BLUE), (X0 + IND + wl + 0.22, wr, title_r, dv, GREEN)):
        box(x0, ys, w, t, its, pal)
    rows.append((ys, hl, hr))
    arrow((X0 + IND + wl + 0.22, ys + BAND / 2), (X0 + IND + wl, ys + BAND / 2), GREEN["dot"])
    y += hh + 0.08

# trunk: from the common box down to the last row, with a branch into every row
last_mid = rows[-1][0] + BAND / 2
ax.plot([TR, TR], [trunk_top, last_mid], color=BLUE["dot"], lw=1.2, zorder=0)
for ys, hl, hr in rows:
    arrow((X0 + IND, ys + BAND / 2), (TR, ys + BAND / 2), BLUE["dot"])
H = y - 0.08 + 0.04
fig.set_size_inches(W, H)
ax.set_xlim(0, W)
ax.set_ylim(H, 0)
ax.axis("off")
p = save(fig, "arch_waterfall.png")
w_in = plt.imread(p).shape[1] / 300.0
print("printed scale %.3f -> labels at %.1f pt, %.2f in tall" % (PW / w_in, FS * PW / w_in, plt.imread(p).shape[0] / 300.0 * PW / w_in))
