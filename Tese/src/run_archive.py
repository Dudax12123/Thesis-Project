"""Browse, compare and replay archived runs.

Every run main.py flies writes a complete archive of itself -- the trajectory,
every diagnostic channel, the PSO convergence curve, and a manifest holding the
whole of simulation_parameters.py, the vehicle constants and the git commit.
This is the way back in.

    python Tese/src/run_archive.py list
    python Tese/src/run_archive.py show <run_id> [--config]
    python Tese/src/run_archive.py compare <run_id> <run_id> [...] [--cards]
    python Tese/src/run_archive.py replay <run_id>
    python Tese/src/run_archive.py index

Run ids match by unique prefix. ``<dir>::<stem>`` reaches into any directory, so
a results-matrix case can be compared against a run flown by hand:

    python Tese/src/run_archive.py compare gt_baseline direct_peg_new_20260824
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# Prints carry Greek letters and the degree sign; the Windows console defaults
# to a codepage that cannot encode them. Same treatment as main.py.
if sys.stdout.encoding is not None and sys.stdout.encoding.lower() != "utf-8":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

from Archive.cli import main

if __name__ == "__main__":
    sys.exit(main() or 0)
