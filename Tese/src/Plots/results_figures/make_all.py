"""Build every Chapter 6 figure from the results-matrix output.

The figures are produced offline, from the .npz and .json files that
``run_results_matrix.py`` writes, and never from a live run. That separation is
the point: a figure can be redesigned, recoloured or re-panelled as many times
as the chapter needs without re-solving anything, and the twelve-hour batch is
run once.

This is not the per-run debugging suite. ``Plots/new_metrics`` still exists and
is still what ``main.py`` fires for a single interactive run; it draws twenty
plots for one trajectory, which is right for finding a bug and wrong for a
chapter. These sixteen draw comparisons across cases instead.

Usage
-----
Render drafts to the scratch preview directory::

    python -m Plots.results_figures.make_all

Render straight into the thesis repository::

    FIG_OUT=".../Thesis_Overleaf/Figures" python -m Plots.results_figures.make_all

Options: ``--root`` to read a different results directory, ``--only`` to filter
by figure function name. Cases the batch has not produced yet are skipped with a
note rather than raising, so a partial batch still draws what it can.
"""

import argparse
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent.parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from Plots.results_figures import _data
from Plots.results_figures import _style as st
from Plots.results_figures import sec62_gravity_turn as sec62
from Plots.results_figures import sec63_peg as sec63
from Plots.results_figures import sec65_losses as sec65
from Plots.results_figures import sec67_capabilities as sec67

# Every case the sixteen figures can draw on, in chapter order.
ALL_CASES = [
    "gt_baseline", "gt_apogee", "gt_direct", "gt_vacuum", "gt_norot",
    "gt_sea_level_engine",
    "peg_baseline", "peg_direct", "peg_vacuum", "peg_vacuum_norot",
    "pmp_baseline", "pmp_vacuum",
    "show_cpr", "show_linear_tangent", "show_bilinear_tangent", "show_apollo",
    "show_peg", "show_exp_shooting",
    "show_seg_fixed_alt", "show_seg_opt_alt",
]

SECTIONS = [
    ("6.2  Gravity turn", sec62.FIGURES),
    ("6.3  Powered explicit guidance / 6.4  Reference", sec63.FIGURES),
    ("6.5  Losses / 6.6  Comparison", sec65.FIGURES),
    ("6.7  Capabilities", sec67.FIGURES),
]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", help="results directory "
                                       "(default: Output/results_matrix)")
    parser.add_argument("--only", help="substring filter over figure names")
    args = parser.parse_args()

    st.use_thesis_style()
    cases = _data.load_many(ALL_CASES, root=args.root)

    print("=" * 70)
    print("CHAPTER 6 FIGURES -- %d of %d cases available"
          % (len(cases), len(ALL_CASES)))
    print("output: %s" % st.OUT_DIR)
    print("=" * 70)
    absent = [n for n in ALL_CASES if n not in cases]
    if absent:
        print("not yet produced: %s" % ", ".join(absent))

    written, skipped = 0, 0
    for title, figures in SECTIONS:
        selected = [f for f in figures
                    if args.only is None or args.only in f.__name__]
        if not selected:
            continue
        print("\n%s" % title)
        for figure in selected:
            if figure(cases) is None:
                skipped += 1
            else:
                written += 1

    print("\n" + "=" * 70)
    print("%d figure(s) written, %d skipped" % (written, skipped))
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
