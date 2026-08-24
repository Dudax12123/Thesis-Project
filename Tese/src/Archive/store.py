"""Where archives live, how they are named, and how one is found again.

Naming has one job: a second run of the same configuration must not destroy the
first. The id is ``<architecture>_<law>_<date>_<time>``, so runs sort
chronologically, read at a glance, and collide only if two finish in the same
second (in which case a counter is appended).

The directory is resolved against the project source root, not the working
directory. ``SAVE_PLOTS_DIR`` is cwd-relative and that is a documented trap --
running main.py from Tese/src produced the nested Tese/src/Tese/src/Output/plots
that is still in the repository. An archive is worth more than a PNG, so this
one resolves the way segment_reference.py resolves its cache.
"""

import csv
import json
from datetime import datetime
from pathlib import Path

import numpy as np

from . import run_record

_SRC = Path(__file__).resolve().parent.parent

DEFAULT_ARCHIVE_DIR = "Output/runs"
INDEX_NAME = "index.csv"

# Directories a bare name is looked up in, in order, when the caller does not
# say where to look. The results-matrix output is included so that a batch case
# and an interactive run can be named side by side in one comparison.
def search_roots(sim_params=None):
    roots = [archive_root(sim_params), _SRC / "Output" / "results_matrix"]
    seen, out = set(), []
    for root in roots:
        key = str(root).lower()
        if key not in seen:
            seen.add(key)
            out.append(root)
    return out


def archive_root(sim_params=None):
    """Where archives are written. Relative ARCHIVE_DIR is project-root-relative."""
    configured = DEFAULT_ARCHIVE_DIR
    if sim_params is not None:
        configured = getattr(sim_params, "ARCHIVE_DIR", DEFAULT_ARCHIVE_DIR)
    path = Path(configured)
    return path if path.is_absolute() else (_SRC / path)


# =========================================================================
#  Naming
# =========================================================================

def _law_slug(sim_params, extra):
    """The guidance half of a run id, as a filename-safe token."""
    label = run_record.guidance_label(sim_params, extra)
    slug = label.replace(" -> ", "+").replace(" ", "")
    return "".join(ch if (ch.isalnum() or ch in "+_-") else "_" for ch in slug)


def make_run_id(sim_params, extra=None, when=None, root=None):
    """``<architecture>_<law>_<YYYYmmdd>_<HHMMSS>``, unique within *root*.

    The law half is dropped when it only repeats the architecture, so an
    indirect-PMP run is ``indirect_pmp_20260824_142530`` rather than saying it
    twice.
    """
    arch = run_record.architecture(sim_params)
    law = _law_slug(sim_params, extra)
    stem = arch if law == arch else "%s_%s" % (arch, law)
    stamp = (when or datetime.now()).strftime("%Y%m%d_%H%M%S")
    base = "%s_%s" % (stem, stamp)

    root = Path(root) if root is not None else archive_root(sim_params)
    candidate, n = base, 1
    while (root / (candidate + ".npz")).exists():
        n += 1
        candidate = "%s_%d" % (base, n)
    return candidate


# =========================================================================
#  Writing
# =========================================================================

def save_run(sim_params, time_a, data, thrust, alpha, result,
             J=None, history=None, extra=None, wall_clock=None, suite=None,
             source=None, label=None, root=None, name=None, tags=None,
             index=None, verbose=True):
    """Write one complete archive and return ``{'stem', 'row', 'manifest'}``.

    *name* fixes the stem, which is what the results matrix wants: its case
    names are the identity of the experiment and re-running a case should
    replace it. Left None -- the interactive path -- a timestamped id is
    generated and nothing is ever overwritten.

    Deliberately independent of SAVE_PLOTS and PLOT_SUITE. Storing a run and
    drawing it are different concerns, and the setting named for images should
    not decide whether tens of minutes of solving survive the process.
    """
    root = Path(root) if root is not None else archive_root(sim_params)
    root.mkdir(parents=True, exist_ok=True)

    generated = name is None
    if generated:
        name = make_run_id(sim_params, extra, root=root)

    row = run_record.collect_row(name, sim_params, time_a, data, thrust, alpha,
                                 result, J, history, wall_clock or 0.0, extra,
                                 tags=tags)
    payload = run_record.channels(sim_params, time_a, data, extra, history,
                                  thrust=thrust, alpha=alpha, suite=suite)
    man = run_record.manifest(sim_params, name, wall_clock=wall_clock,
                              source=source, label=label, extra=extra)

    np.savez_compressed(root / (name + ".npz"),
                        time=np.asarray(time_a, dtype=float),
                        data=np.asarray(data, dtype=float),
                        thrust=np.asarray(thrust, dtype=float),
                        alpha=np.asarray(alpha, dtype=float),
                        **payload)
    _write_json(root / (name + ".json"), row)
    _write_json(root / (name + ".manifest.json"), man)

    # The index is a convenience for browsing, always rebuildable by scanning,
    # so it is refreshed rather than appended to and a stale one is never fatal.
    refresh_index = generated if index is None else bool(index)
    if refresh_index:
        try:
            write_index(root)
        except Exception as exc:                   # noqa: BLE001 — never fatal
            print("  [archive] index not refreshed: %s" % exc)

    if verbose:
        size_kb = (root / (name + ".npz")).stat().st_size / 1024.0
        print("  [archive] %s  (%.0f kB, %d channels)"
              % (root / (name + ".npz"), size_kb, len(payload) + 4))
    return {'stem': root / name, 'row': row, 'manifest': man, 'name': name}


def _write_json(path, obj):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2)


# =========================================================================
#  Finding and reading
# =========================================================================

def resolve(name, root=None, sim_params=None):
    """Turn a user-typed name into ``(root, stem)``.

    Accepted, in order: ``<dir>::<stem>``; a path to the .npz or .json; an exact
    stem in *root*; a unique stem prefix in any search root. The prefix form is
    what makes timestamped ids usable by hand -- nobody types
    ``pso_coast_peg_new_20260824_142530`` in full.
    """
    text = str(name)
    if "::" in text:
        head, stem = text.rsplit("::", 1)
        return Path(head), stem

    path = Path(text)
    if path.suffix in (".npz", ".json") and path.exists():
        stem = path.name
        for suffix in (".manifest.json", ".json", ".npz"):
            if stem.endswith(suffix):
                stem = stem[: -len(suffix)]
                break
        return path.parent, stem

    roots = [Path(root)] if root is not None else search_roots(sim_params)
    for candidate in roots:
        if (candidate / (text + ".npz")).exists():
            return candidate, text

    matches = []
    for candidate in roots:
        for stem in _stems(candidate):
            if stem.startswith(text):
                matches.append((candidate, stem))
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise FileNotFoundError(
            "no archive named %r under %s"
            % (text, ", ".join(str(r) for r in roots)))
    raise ValueError("%r matches %d archives: %s"
                     % (text, len(matches),
                        ", ".join(stem for _r, stem in matches)))


def _stems(root):
    """Every archive stem in a directory, newest first, older layouts included."""
    root = Path(root)
    if not root.is_dir():
        return []
    stems = sorted((p.stem for p in root.glob("*.npz")),
                   key=lambda s: (root / (s + ".npz")).stat().st_mtime,
                   reverse=True)
    return stems


def read_manifest(root, stem):
    """The manifest, or ``{}`` for an archive written before manifests existed."""
    path = Path(root) / (stem + ".manifest.json")
    if not path.exists():
        return {}
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def load_run(name, root=None, sim_params=None):
    """One archive as a :class:`Plots.results_figures._data.Case`.

    The same loader the chapter figures use, so a batch case and an interactive
    run are the same kind of object and any figure written for one works on the
    other.
    """
    from Plots.results_figures import _data as rf_data

    found_root, stem = resolve(name, root=root, sim_params=sim_params)
    case = rf_data.load(stem, root=found_root)
    if case is None:
        raise FileNotFoundError("archive %s is incomplete (need %s.npz and %s.json)"
                                % (Path(found_root) / stem, stem, stem))
    return case


# =========================================================================
#  Listing
# =========================================================================

# What the index shows: enough to pick a run out of a hundred without opening
# it, and no more. Everything else is in the manifest.
INDEX_COLUMNS = [
    'run_id', 'created_local', 'architecture', 'guidance_mode', 'target_alt_km',
    'include_drag', 'earth_rotation', 'thrust_1_mode', 'crashed',
    'periapsis_km', 'eccentricity', 'prop_remaining_kg', 'dv_achieved',
    'wall_clock_s', 'n_evaluations', 'git_commit', 'label',
]


def describe(root, stem):
    """One index line for one archive, tolerating a missing manifest or row."""
    root = Path(root)
    row = {}
    json_path = root / (stem + ".json")
    if json_path.exists():
        try:
            with open(json_path, encoding="utf-8") as fh:
                row = json.load(fh)
        except Exception:                          # noqa: BLE001 — listed anyway
            row = {}
    man = read_manifest(root, stem)
    git = man.get('git') or {}
    commit = git.get('commit')

    entry = {key: row.get(key) for key in INDEX_COLUMNS}
    entry['run_id'] = stem
    entry['created_local'] = man.get('created_local') or _mtime(root, stem)
    entry['git_commit'] = None if commit is None else commit[:9]
    entry['label'] = man.get('label') or ""
    entry['wall_clock_s'] = row.get('wall_clock_s', man.get('wall_clock_s'))
    return entry


def _mtime(root, stem):
    path = Path(root) / (stem + ".npz")
    if not path.exists():
        return None
    return datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec='seconds')


def list_runs(root=None, sim_params=None):
    """Every archive in a directory, newest first.

    Produced by scanning rather than by reading index.csv, so an index that was
    never written or has gone stale cannot hide a run.
    """
    root = Path(root) if root is not None else archive_root(sim_params)
    return [describe(root, stem) for stem in _stems(root)]


def write_index(root=None, sim_params=None):
    """Rewrite index.csv from a scan of *root*. Returns the path."""
    root = Path(root) if root is not None else archive_root(sim_params)
    root.mkdir(parents=True, exist_ok=True)
    path = root / INDEX_NAME
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=INDEX_COLUMNS, extrasaction='ignore')
        writer.writeheader()
        for entry in list_runs(root):
            writer.writerow(entry)
    return path
