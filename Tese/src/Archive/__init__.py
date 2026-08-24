"""Run archiving: persist a flown trajectory, completely, so it never has to be re-flown.

A production solve is tens of minutes and the results-matrix batch is most of a
day. Everything a run produces beyond the state history -- the PSO convergence
curve, the arc boundary times, the theta and t_go histories, the pseudo-force
diagnostics, the segmented schedule -- lives in module globals that die with the
interpreter. This package writes all of it, plus a manifest describing the
configuration that produced it, under a name that never collides with an earlier
run.

An archive is three files sharing a stem:

    <run_id>.npz            trajectory and every captured channel
    <run_id>.json           the scalar results row
    <run_id>.manifest.json  configuration, vehicle, git commit, timing

which is deliberately the same layout ``run_results_matrix.py`` already writes,
so ``Plots.results_figures._data.load`` reads a batch case and an interactive run
through one code path. An archive written before the manifest existed simply
loads with ``Case.manifest == {}``.

Entry points::

    python Tese/src/run_archive.py list
    python Tese/src/run_archive.py show <run_id>
    python Tese/src/run_archive.py compare <run_id> <run_id> [...]
    python Tese/src/run_archive.py replay <run_id>
"""

from . import run_record          # noqa: F401
from . import store               # noqa: F401

__all__ = ["run_record", "store"]
