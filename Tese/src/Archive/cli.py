"""Command line over the archive: list, show, compare, replay, index.

Run it from the repository root::

    python Tese/src/run_archive.py list
    python Tese/src/run_archive.py show pso_coast_peg_new_20260824
    python Tese/src/run_archive.py compare gt_baseline pso_coast_gravity_turn_2026
    python Tese/src/run_archive.py replay pso_coast_peg_new_20260824

Names are matched by unique prefix, so the timestamped ids do not have to be
typed in full. ``<dir>::<stem>`` names an archive in any directory, which is how
a results-matrix case and an interactive run end up in the same comparison.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

from . import run_record
from . import store


# =========================================================================
#  list / index
# =========================================================================

_LIST_COLUMNS = [
    ("run_id", 44), ("created_local", 20), ("guidance_mode", 22),
    ("architecture", 14), ("periapsis_km", 13), ("prop_remaining_kg", 18),
    ("wall_clock_s", 13), ("label", 24),
]


def cmd_list(args):
    root = Path(args.root) if args.root else None
    entries = store.list_runs(root=root, sim_params=_sim_params())
    if args.limit:
        entries = entries[:args.limit]
    if not entries:
        print("no archives under %s" % (root or store.archive_root(_sim_params())))
        return 0

    header = "  ".join(name.ljust(width) for name, width in _LIST_COLUMNS)
    print(header)
    print("-" * len(header))
    for entry in entries:
        cells = []
        for name, width in _LIST_COLUMNS:
            value = entry.get(name)
            if isinstance(value, float):
                value = "%.4g" % value
            cells.append(str("" if value is None else value)[:width].ljust(width))
        print("  ".join(cells))
    print("")
    print("%d archive(s)" % len(entries))
    return 0


def cmd_index(args):
    path = store.write_index(root=Path(args.root) if args.root else None,
                             sim_params=_sim_params())
    print("wrote %s" % path)
    return 0


# =========================================================================
#  show
# =========================================================================

def cmd_show(args):
    root, stem = store.resolve(args.name, root=args.root, sim_params=_sim_params())
    case = store.load_run(stem, root=root)
    man = case.manifest

    print("=" * 70)
    print(stem)
    print("=" * 70)
    print("  directory      %s" % root)
    if man:
        git = man.get('git') or {}
        print("  flown          %s   (%s s wall clock)"
              % (man.get('created_local'), man.get('wall_clock_s')))
        print("  source         %s" % man.get('source'))
        print("  git            %s%s"
              % (git.get('describe'), "  [dirty tree]" if git.get('dirty') else ""))
        if man.get('label'):
            print("  label          %s" % man['label'])
    else:
        print("  (no manifest -- this archive predates them, so what it was "
              "flown under is not recorded)")

    print("")
    print("  RESULTS")
    for key in ("architecture", "guidance_mode", "crashed", "J_prime",
                "insertion_alt_km", "insertion_v_ms", "insertion_fpa_deg",
                "eccentricity", "periapsis_km", "apoapsis_km",
                "prop_remaining_kg", "dv_achieved", "residual",
                "t_meco", "t_seco", "n_evaluations", "wall_clock_s"):
        if case.row.get(key) is not None:
            print("    %-22s %s" % (key, case.row[key]))

    print("")
    print("  CHANNELS  (%d samples on the state grid)" % len(case.time))
    for key in sorted(case._z.files):
        arr = np.asarray(case._z[key])
        shape = "scalar" if arr.ndim == 0 else "x".join(str(n) for n in arr.shape)
        print("    %-26s %s" % (key, shape))

    if args.config and man:
        print("")
        print("  CONFIGURATION")
        for key, value in (man.get('config') or {}).items():
            print("    %-34s %s" % (key, value))
    return 0


# =========================================================================
#  replay / compare
# =========================================================================

def cmd_replay(args):
    """Redraw the twenty-plot diagnostic suite from an archive.

    The suite itself is untouched: the archive stores every channel on the grid
    the architecture produced it on, so the saved arrays are handed straight to
    run_new_plot_suite exactly as main.py would have handed them over at the end
    of the run. That is the difference between archiving the data and archiving
    pictures of it.
    """
    if not args.show:
        import matplotlib
        matplotlib.use("Agg")
    from Plots import new_plot_runner

    root, stem = store.resolve(args.name, root=args.root, sim_params=_sim_params())
    with np.load(root / (stem + ".npz")) as z:
        time = np.asarray(z["time"], dtype=float)
        data = np.asarray(z["data"], dtype=float)
        kwargs = {}
        for npz_key, kwarg in run_record.REPLAY_KEYS.items():
            if npz_key in z.files:
                kwargs[kwarg] = np.asarray(z[npz_key])
        # Channels the writer dropped because they were bit-identical to
        # something already stored. Following the alias reproduces exactly what
        # the run handed to the suite; guessing would not.
        if "suite_aliases" in z.files:
            for entry in z["suite_aliases"]:
                npz_key, _, target = str(entry).partition("=")
                kwarg = run_record.REPLAY_KEYS.get(npz_key)
                if kwarg is not None and target in z.files:
                    kwargs[kwarg] = np.asarray(z[target])
        if "apollo_freeze_threshold" in z.files:
            kwargs["apollo_freeze_threshold"] = float(z["apollo_freeze_threshold"])
        if "pso_gen" in z.files:
            kwargs["pso_history"] = {'gen': np.asarray(z["pso_gen"]),
                                     'gbest': np.asarray(z["pso_gbest"])}
        # The four positional channels. Fall back to the state-grid copies for
        # an archive written before the native-cadence set existed.
        thrust_data = kwargs.pop("thrust_data", np.asarray(z["thrust"]))
        time_thrust = kwargs.pop("time_thrust", time)
        alpha_data = kwargs.pop("alpha_data", np.asarray(z["alpha"]))
        alpha_time_data = kwargs.pop("alpha_time_data", time)

    # Into the run's own plot folder, not next to its data: Output/ is the
    # trajectory, Output_Plots/<run_id>/ is everything drawn from it. Replaying
    # a results-matrix case therefore lands in Output_Plots/<case_name>/.
    out_dir = args.out or str(store.run_plots_dir(stem, _sim_params()))
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    new_plot_runner.run_new_plot_suite(
        time, data, thrust_data, time_thrust, alpha_data, alpha_time_data,
        output_dir=out_dir, show=args.show, close_after=not args.show, **kwargs)
    print("replayed %s into %s" % (stem, out_dir))
    if args.show:
        import matplotlib.pyplot as plt
        plt.show()
    return 0


def cmd_compare(args):
    import matplotlib
    matplotlib.use("Agg")
    from . import compare as compare_mod

    labels = args.labels.split(",") if args.labels else None
    compare_mod.compare(args.names, root=args.root, out_dir=args.out,
                        labels=labels, cards=args.cards,
                        sim_params=_sim_params())
    return 0


def _sim_params():
    """The config module, for ARCHIVE_DIR. Absent is not fatal -- the default is."""
    try:
        from Input_File import simulation_parameters as sim_params
        return sim_params
    except Exception:                              # noqa: BLE001
        return None


# =========================================================================
#  Entry point
# =========================================================================

def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="run_archive.py", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", help="archive directory (default: ARCHIVE_DIR)")
    sub = parser.add_subparsers(dest="command")

    p_list = sub.add_parser("list", help="every archive, newest first")
    p_list.add_argument("--limit", type=int, default=0)
    p_list.set_defaults(func=cmd_list)

    p_index = sub.add_parser("index", help="rewrite index.csv from a scan")
    p_index.set_defaults(func=cmd_index)

    p_show = sub.add_parser("show", help="one archive in detail")
    p_show.add_argument("name")
    p_show.add_argument("--config", action="store_true",
                        help="also print every recorded configuration value")
    p_show.set_defaults(func=cmd_show)

    p_cmp = sub.add_parser("compare", help="overlay N archives and diff them")
    p_cmp.add_argument("names", nargs="+")
    p_cmp.add_argument("--out", help="output directory")
    p_cmp.add_argument("--labels", help="comma-separated legend labels")
    p_cmp.add_argument("--cards", action="store_true",
                       help="also draw the run card for each archive")
    p_cmp.set_defaults(func=cmd_compare)

    p_rep = sub.add_parser("replay", help="redraw the 20-plot suite from an archive")
    p_rep.add_argument("name")
    p_rep.add_argument("--out", help="output directory")
    p_rep.add_argument("--show", action="store_true", help="display instead of save")
    p_rep.set_defaults(func=cmd_replay)

    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        parser.print_help()
        return 1
    try:
        return args.func(args)
    except (FileNotFoundError, ValueError) as exc:
        print("error: %s" % exc)
        return 2
