"""Tests that wait for the re-fly under EARTH_ROTATION_MODEL = "launch_site".

On 2026-10-05 the latitude was frozen at its launch value and the pseudo-force terms
began crediting the launch-site speed, like the target and the frame conversion
(earth_rotation.planar_pseudoforce_rates). The tracked PMP reference
(Output/pmp_reference.npz) and every results-matrix archive were flown before that:
the reference is refused under the current cache key (and its Stage 1 is no longer
the one the solvers fly, which the segmented solver checks on its own), and the
archived numbers no longer reproduce.

``needs_refly`` skips a test while an archive it depends on predates the current model.
The test re-enables itself once that archive is re-flown, and any number it pins then
has to be re-pinned to the new archive. The tracked reference is pmp_baseline's
extremal, so ``needs_new_reference`` keys on that case.
"""

import json
from pathlib import Path

import pytest

from Input_File import simulation_parameters as sim_params

_MATRIX = Path(__file__).resolve().parents[1] / "Output" / "results_matrix"


def archive_predates_rotation_model(case):
    """True when results_matrix/<case>'s manifest was not flown under the current model."""
    manifest = _MATRIX / case / ("%s.manifest.json" % case)
    try:
        config = json.loads(manifest.read_text(encoding="utf-8")).get("config", {})
    except OSError:
        return True
    return config.get("EARTH_ROTATION_MODEL") != sim_params.EARTH_ROTATION_MODEL


def needs_refly(*cases):
    """Skip while any of ``cases`` is archived under an earlier rotation model."""
    stale = [case for case in cases if archive_predates_rotation_model(case)]
    return pytest.mark.skipif(
        bool(stale),
        reason="archived %s predates EARTH_ROTATION_MODEL='launch_site' (2026-10-05); "
               "re-fly, then re-pin" % ", ".join(stale))


# The tracked PMP reference cache is pmp_baseline's extremal and is rebuilt with it.
needs_new_reference = needs_refly("pmp_baseline")
