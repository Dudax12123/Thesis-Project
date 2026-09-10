#!/usr/bin/env python
"""Compare two results-matrix roots case by case.

    python dev-notes/compare_arms.py <root_A> <root_B> [--label-a NAME --label-b NAME]

Built for paired A/B arms produced with --out, e.g. the pseudo-force ON/OFF
pair or a penalty-weight A/B. Prints the mission-relevant columns side by side
plus the delta, and flags any case that reached only one arm.

Ranking is on propellant delivered, not on J', because J' is not comparable
across arms whose objective weights differ -- dropping a penalty term removes
it from J' without any trajectory having changed.
"""

import argparse
import csv
import sys
from pathlib import Path

# Columns worth pairing, and how to show them.
COLUMNS = [
    ("prop_remaining_kg", "propellant [kg]", "%12.1f", "%+10.1f"),
    ("obj_J",             "obj_J",           "%12.6f", "%+10.6f"),
    ("t_meco",            "t_meco [s]",      "%12.4f", "%+10.4f"),
    ("insertion_v_ms",    "v_ins [m/s]",     "%12.3f", "%+10.3f"),
    ("insertion_alt_km",  "alt_ins [km]",    "%12.5f", "%+10.5f"),
    ("eccentricity",      "ecc",             "%12.3e", "%+10.2e"),
    ("dv_gravity",        "dv_gravity",      "%12.3f", "%+10.3f"),
    ("dv_drag",           "dv_drag",         "%12.3f", "%+10.3f"),
    ("dv_steering",       "dv_steering",     "%12.3f", "%+10.3f"),
    ("residual",          "residual",        "%12.3f", "%+10.3f"),
]


def load(root):
    path = Path(root) / "results_matrix.csv"
    if not path.exists():
        sys.exit("no results_matrix.csv under %s" % root)
    with open(path, newline="", encoding="utf-8") as fh:
        return {row["case"]: row for row in csv.DictReader(fh)}


def num(row, key):
    try:
        return float(row[key])
    except (TypeError, ValueError, KeyError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root_a")
    ap.add_argument("root_b")
    ap.add_argument("--label-a", default=None)
    ap.add_argument("--label-b", default=None)
    args = ap.parse_args()

    a, b = load(args.root_a), load(args.root_b)
    la = args.label_a or Path(args.root_a).name
    lb = args.label_b or Path(args.root_b).name

    only_a = sorted(set(a) - set(b))
    only_b = sorted(set(b) - set(a))
    shared = [c for c in a if c in b]

    print("A = %s   (%d cases)" % (args.root_a, len(a)))
    print("B = %s   (%d cases)" % (args.root_b, len(b)))
    if only_a:
        print("only in A: %s" % ", ".join(only_a))
    if only_b:
        print("only in B: %s" % ", ".join(only_b))
    print()

    for case in shared:
        ra_, rb = a[case], b[case]
        crashed = [lbl for lbl, row in ((la, ra_), (lb, rb))
                   if str(row.get("crashed", "")).lower() in ("true", "1")]
        head = case
        if crashed:
            head += "   [CRASHED in %s]" % ", ".join(crashed)
        print(head)
        print("  %-16s %12s %12s %10s" % ("", la[:12], lb[:12], "B - A"))
        for key, label, fmt, dfmt in COLUMNS:
            va, vb = num(ra_, key), num(rb, key)
            if va is None or vb is None:
                continue
            print(("  %-16s " + fmt + " " + fmt + " " + dfmt)
                  % (label, va, vb, vb - va))
        print()

    # Ranking on propellant, per arm.
    print("=" * 62)
    print("RANKING on propellant delivered [kg]")
    print("=" * 62)
    for lbl, arm in ((la, a), (lb, b)):
        rows = [(num(r, "prop_remaining_kg"), c) for c, r in arm.items()
                if num(r, "prop_remaining_kg") is not None
                and str(r.get("crashed", "")).lower() not in ("true", "1")]
        rows.sort(reverse=True)
        print("\n%s:" % lbl)
        for i, (v, c) in enumerate(rows, 1):
            print("  %2d. %-24s %10.1f" % (i, c, v))


if __name__ == "__main__":
    main()
