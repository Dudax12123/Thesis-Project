"""Figures: for each architecture, what it does with the variables of Table 5.2.

Spec: Thesis_Implementation.tex, Section 5.2 (Table tab:left_variables) and the four architecture
subsections. Compact, horizontal layout, at \\linewidth with 10 pt text:
  top card    "Variables Left to Define": a red "Not Needed" panel and a green "Not Defined: Design
              Vector x" panel side by side (items flow in rows), and below them the defined variables;
  bottom card "Defined by the Ascent Case": only the defined values, each under its variable and
              reached by an arrow; the card is as wide as those values need.
The design vector itself is given in the text. Outputs arch_variables_{A,B,C,D}.png.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

from figstyle import use_thesis_style, save, INK

use_thesis_style()
FS = 10.2
PW = 6.30
W = 6.30

G, CO, BF, CS, GL, HO, LC, L0 = (r"Pitch-Over Angle $\gamma_p$", r"Coast Duration $\Delta t_c$",
                                 r"Burn Fraction $\Delta t_r\%$", "Coast Start", "Guidance Laws", "Hand-Off Criteria",
                                 r"Law Constants $\mathbf{c}$ (Open-Loop Law)", r"Initial Costates $\boldsymbol{\lambda}_0$")
NN = "Not Needed"
ARCH = {
    "A": ([(GL, "Two Laws, Chosen"), (HO, r"Altitude, Chosen ($h_{\mathrm{ho}}$)"), (L0, NN)], [G, CO, BF, CS, LC]),
    "B": ([(GL, "One Law, Second Stage"), (HO, "MECO"), (L0, NN)], [G, CO, BF, CS, LC]),
    "C": ([(GL, "Optimal Control, Second Stage"), (HO, "MECO"), (LC, NN)], [G, CO, BF, CS, L0]),
    "D": ([(GL, "One Law"), (CO, "0 (No Coast)"), (HO, "MECO"), (CS, NN), (L0, NN)], [G, BF, LC]),
}

NV = {"A": "4 to 10", "B": "4 to 7", "C": "7", "D": "1 to 5"}   # fewest and most variables of the design vector
FOOT = {"A": "Defined by Ascent Case (Schedule)", "B": "Defined by Ascent Case (Single Law)",
        "C": "Defined by Ascent Case (Optimal Control)", "D": "Defined by Ascent Case (Direct Insertion)"}
BLUE = dict(fill="#f3f6fb", line="#8aa4cc", band="#dce6f4", dot="#4f77b3")
RED = dict(fill="#fdf3f2", line="#dd9a93", band="#f8dcd9", dot="#c8584d")
GREEN = dict(fill="#f2f9f5", line="#86bb9c", band="#d6ecde", dot="#3f8a5f")
GREY = dict(fill="#ffffff", line="#b9b9b9", band="#ececec", dot="#777777")

BAND = 0.25
LINE = 0.245         # row pitch inside a panel
PADX = 0.14
GAPX = 0.17          # between flowing items
BUL = 0.17           # bullet + space


def rbox(ax, x0, y0, x1, y1, fc, ec, lw=1.0, r=0.09, z=1):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=%g" % r,
                                fc=fc, ec=ec, lw=lw, zorder=z))


def band_card(ax, x0, x1, y0, y1, title, pal, r=0.09, band_top=True):
    """Card from y0 to y1 with a tinted title band at the top (or the bottom)."""
    rbox(ax, x0, y0, x1, y1, pal["fill"], pal["line"], lw=1.1, r=r, z=1)
    by0, by1 = (y1 - BAND, y1) if band_top else (y0, y0 + BAND)
    rbox(ax, x0, by0, x1, by1, pal["band"], pal["line"], lw=1.1, r=r, z=2)
    # square off the band's inner edge
    ys = by0 if band_top else by1 - BAND * 0.5
    ax.add_patch(Rectangle((x0 + 0.012, by0 if band_top else by0 + BAND * 0.5), x1 - x0 - 0.024, BAND * 0.5,
                           fc=pal["band"], ec="none", zorder=3))
    yl = by0 if band_top else by1
    ax.plot([x0 + 0.012, x1 - 0.012], [yl, yl], color=pal["line"], lw=0.8, zorder=4)
    ax.text(x0 + PADX, (by0 + by1) / 2, title, fontsize=FS, fontweight="bold", color=INK, va="center", zorder=5)


def draw(key):
    dfn_all, free = ARCH[key]
    dfn = [(n, v) for n, v in dfn_all if v != NN]
    na = [n for n, v in dfn_all if v == NN]
    fig = plt.figure(figsize=(W, 1))
    ax = fig.add_axes([0, 0, 1, 1])
    fig.canvas.draw()
    r = fig.canvas.get_renderer()

    def tw(t, **kw):
        tx = ax.text(0, 0, t, fontsize=FS, **kw)
        w = tx.get_window_extent(r).width / fig.dpi
        tx.remove()
        return w

    def flow(items, width):
        """Line breaks of items in a panel body of the given inner width -> list of lists of (item, x offset)."""
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

    X0, X1 = 0.04, W - 0.04
    ginner = 0.08
    avail = (X1 - X0) - 2 * ginner - 0.14          # two panels and the gap between them
    minw = lambda its: max(BUL + tw(i) for i in its) + 2 * PADX
    if na:
        best = None
        rmin, gmin = minw(na), max(minw(free), tw(r"Not Defined: Design Vector $\mathbf{x}$ (%s Variables)" % NV[key], fontweight="bold") + 2 * PADX + 0.05)
        rmax = avail - gmin
        rw = rmin
        while rw <= rmax + 1e-9:
            gw = avail - rw
            h = max(len(flow(na, rw - 2 * PADX)), len(flow(free, gw - 2 * PADX)))
            cand = (h, abs(rw - gw))
            if best is None or cand < best[0]:
                best = (cand, rw, gw)
            rw += 0.05
        if best is None:
            best = (None, None, None)
        side = best[1] is not None
    else:
        side = False
    if side:
        rw, gw = best[1], best[2]
        nlines = max(len(flow(na, rw - 2 * PADX)), len(flow(free, gw - 2 * PADX)))
        panels = [(na, "Not Needed", RED, rw), (free, (r"Not Defined: Design Vector $\mathbf{x}$ (%s Variables)" % NV[key]), GREEN, gw)]
    else:
        gw = (X1 - X0) - 2 * ginner
        nlines = len(flow(free, gw - 2 * PADX))
        panels = [(free, (r"Not Defined: Design Vector $\mathbf{x}$ (%s Variables)" % NV[key]), GREEN, gw)]
        if na:
            panels = [(na, "Not Needed", RED, gw)] + panels
    # vertical stacks of panels if not side by side
    pan_h = BAND + 0.05 + nlines * LINE + 0.03
    if side:
        body_h = pan_h
    else:
        body_h = sum(BAND + 0.05 + len(flow(p[0], p[3] - 2 * PADX)) * LINE + 0.03 for p in panels) + 0.10 * (len(panels) - 1)

    # defined variables: a third panel inside the same card, "variable: value"
    dfn_items = ["%s: %s" % (n, v) for n, v in dfn]
    gwid = (X1 - X0) - 2 * ginner
    dl = flow(dfn_items, gwid - 2 * PADX) if dfn else []
    h_def = (BAND + 0.05 + len(dl) * LINE + 0.03) if dfn else 0
    h_top = BAND + 0.07 + body_h + ((0.10 + h_def) if dfn else 0) + 0.06
    H = h_top + 0.06
    fig.set_size_inches(W, H)
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")

    ytop = H - 0.03
    ytop_bot = ytop - h_top
    band_card(ax, X0, X1, ytop_bot, ytop, "Variables Left Free", GREY, r=0.12)

    py = ytop - BAND - 0.07
    px = X0 + ginner
    for items, title, pal, pw in panels:
        ls = flow(items, pw - 2 * PADX)
        ph = BAND + 0.05 + len(ls) * LINE + 0.03
        band_card(ax, px, px + pw, py - ph, py, title, pal)
        for li, line in enumerate(ls):
            yy = py - BAND - 0.05 - li * LINE - LINE / 2 + 0.02
            for it, xo in line:
                ax.plot([px + PADX + 0.04 + xo], [yy], "o", ms=4.0, color=pal["dot"], zorder=5)
                ax.text(px + PADX + BUL + xo, yy, it, fontsize=FS, color=INK, va="center", zorder=5)
        if side:
            px += pw + 0.14
        else:
            py -= ph + 0.10

    if dfn:
        pb = ytop_bot + 0.06 + h_def
        px = X0 + ginner
        band_card(ax, px, px + gwid, pb - h_def, pb, FOOT.get(key, "Defined by the Ascent Case"), BLUE)
        for li, line in enumerate(dl):
            yy = pb - BAND - 0.05 - li * LINE - LINE / 2 + 0.02
            for it, xo in line:
                ax.plot([px + PADX + 0.04 + xo], [yy], "o", ms=4.0, color=BLUE["dot"], zorder=5)
                ax.text(px + PADX + BUL + xo, yy, it, fontsize=FS, color=INK, va="center", zorder=5)
    p = save(fig, "arch_variables_%s.png" % key)
    w_in = plt.imread(p).shape[1] / 300.0
    print(key, "printed scale %.3f -> labels at %.1f pt, %.2f in tall" % (PW / w_in, FS * PW / w_in, plt.imread(p).shape[0] / 300.0 * PW / w_in))


for k in "ABCD":
    draw(k)
