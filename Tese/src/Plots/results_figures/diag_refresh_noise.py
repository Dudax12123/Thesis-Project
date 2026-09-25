"""Diagnostic figures for GUIDANCE_REFRESH_MODE: the objective with the coefficient
refresh inside the ODE right-hand side ("in_rhs", off) against once per guidance cycle
("cycle", on).

Drawn from the per-case JSON that ``dev-notes/refresh_ab.py`` writes
(``Output/refresh_ab/<case>/<case>.refresh_ab.json``); its "cycle" mode is the production
switch bit for bit.

  refresh_noise_scans.png    J along each case's noisiest decision coordinate
                             (41 points over +/-1e-3 of the bound range), off vs on
  refresh_noise_summary.png  the difference-noise estimate of J (worst coordinate,
                             3rd difference) and the RHS evaluations per trajectory

Usage (from the repository root):
    python Tese/src/Plots/results_figures/diag_refresh_noise.py [--root DIR] [--out DIR]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from Plots.results_figures import _style as st  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402  (after _style selects Agg)

CASES = ("peg_baseline", "peg_vacuum", "peg_vacuum_norot", "peg_direct",
         "show_apollo", "show_seg_fixed_alt", "show_seg_opt_alt")
COORD_NAMES = {
    "pso_coast": ["coast $\\Delta t_c$ [s]", "burn [%]", "coast start [%]",
                  "kick $\\gamma_p$ [rad]"],
    "segmented": ["coast $\\Delta t_c$ [s]", "burn [%]", "coast start [%]",
                  "kick $\\gamma_p$ [rad]", "switch-altitude fraction"],
    "direct": ["kick $\\gamma_p$ [rad]", "burn [%]"],
}
OFF, ON = ("in_rhs", "in-RHS refresh (off)"), ("cycle", "cycle refresh (on)")
DEFAULT_ROOT = Path(__file__).resolve().parents[2] / "Output" / "refresh_ab"


def _sigma(entry, scale, k=3):
    d = entry["scales"][scale]["sigma_J"]
    v = d.get(str(k), d.get(k))
    return float("nan") if v is None else float(v)


def load(root):
    out = []
    for name in CASES:
        p = Path(root) / name / f"{name}.refresh_ab.json"
        if p.exists():
            out.append(json.loads(p.read_text(encoding="utf-8")))
    return out


def fig_scans(cases):
    n = len(cases)
    cols = 2
    rows = -(-n // cols)
    fig, axes = plt.subplots(rows, cols, figsize=(6.3, 1.9 * rows + 0.4), squeeze=False)
    for ax in axes.flat[n:]:
        ax.set_visible(False)
    for i, (ax, d) in enumerate(zip(axes.flat, cases)):
        scan_off = d["modes"][OFF[0]].get("scan")
        scan_on = d["modes"][ON[0]].get("scan")
        if not scan_off or not scan_on:
            ax.set_visible(False)
            continue
        # the coordinate where the in-RHS objective is noisiest
        c = int(np.nanargmax([_sigma(r, "coarse") for r in scan_off]))
        x0 = d["x"][c]
        for scan, (mode, label), color in ((scan_off, OFF, st.BASELINE),
                                           (scan_on, ON, st.VARIANT)):
            line = scan[c]["scales"]["line"]
            pts = np.asarray(line["points"], dtype=float) - x0
            ax.plot(pts, line["J"], color=color, linewidth=1.1, label=label)
            ax.plot([0.0], [d["modes"][mode]["at_x"]["J"]], "o", color=color, markersize=3)
        ax.set_title(d["case"] + ("  (borrowed x)" if d.get("borrowed") else ""), fontsize=8)
        ax.set_xlabel("offset in " + COORD_NAMES[d["arch"]][c], fontsize=7)
        ax.set_ylabel("$J$", fontsize=7)
        ax.tick_params(labelsize=6)
        ax.ticklabel_format(axis="x", style="sci", scilimits=(-2, 3))
        st.tidy(ax, legend=(i == 0), legend_kw={"fontsize": 6})
    fig.tight_layout()
    return fig


def fig_summary(cases):
    names = [d["case"] for d in cases]
    x = np.arange(len(names))
    w = 0.38
    fig, (a1, a2) = plt.subplots(1, 2, figsize=st.WIDE_2)
    for j, (mode, label) in enumerate((OFF, ON)):
        color = st.BASELINE if mode == OFF[0] else st.VARIANT
        sig = []
        for d in cases:
            scan = d["modes"][mode].get("scan") or []
            vals = [_sigma(r, "fine") for r in scan]
            vals = [v for v in vals if v == v]
            sig.append(max(vals) if vals else np.nan)
        nfev = [d["modes"][mode]["at_x"]["stats"]["nfev_stage2"] for d in cases]
        a1.bar(x + (j - 0.5) * w, sig, w, color=color, label=label)
        a2.bar(x + (j - 0.5) * w, nfev, w, color=color, label=label)
    a1.set_yscale("log")
    a1.set_ylabel("noise in $J$, worst coordinate\n(step $10^{-6}$ of its range)", fontsize=7)
    a2.set_ylabel("RHS evaluations per trajectory", fontsize=7)
    for ax, tag in ((a1, "a"), (a2, "b")):
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=40, ha="right", fontsize=6)
        ax.tick_params(axis="y", labelsize=6)
        st.tidy(ax, legend=(ax is a2), legend_kw={"fontsize": 6})
        st.panel_tag(ax, tag)
    fig.tight_layout()
    return fig


def make(root=DEFAULT_ROOT, out_dir=None):
    st.use_thesis_style()
    if out_dir is not None:
        st.OUT_DIR = str(out_dir)
    cases = load(root)
    if not cases:
        raise SystemExit(f"no refresh_ab results under {root}")
    return [st.save(fig_scans(cases), "refresh_noise_scans.png"),
            st.save(fig_summary(cases), "refresh_noise_summary.png")]


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default=str(DEFAULT_ROOT))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    make(a.root, a.out)
