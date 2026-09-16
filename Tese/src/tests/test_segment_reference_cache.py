"""
The PMP reference can be seeded from an archived indirect_pmp run (2026-09-16).

With PMP_REFERENCE_PSO_* equal to the PMP swarm's own budget and seed, the reference
build and a pmp_baseline case of the results matrix are one deterministic run, so the
archive stands in for the ~2 h rebuild. The cache is written under the key of the
configuration in force, is refused when the archive's manifest disagrees with that
configuration on any key input, and is ignored by the loader as soon as the
configuration moves.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from Input_File import simulation_parameters as sim_params
import Simulation.segment_reference as segref

_NAMES = ("PSO_SEED", "PSO_LB", "PSO_UB", "INCLUDE_DRAG", "ENABLE_EARTH_ROTATION",
          "INCLUDE_PSEUDO_FORCES", "INDIRECT_PMP_STAGE2_FRAME",
          "INDIRECT_PMP_STAGE1_PSEUDO_FORCES", "INDIRECT_PMP_TRANSVERSALITY",
          "TARGET_ORBITAL_ALTITUDE", "TARGET_ORBIT_INCLINATION", "LAUNCH_LATITUDE",
          "ISP_1_MODE", "THRUST_1_MODE")


def _write_archive(folder, n=50, manifest=True, **config_overrides):
    """A small archive shaped like the harness writes one: six-row data (state plus
    the appended latitude), a dense time grid and the control history."""
    t = np.linspace(0.0, 600.0, n)
    data = np.vstack([t * 7e3, 6.45e6 + t * 100.0, 3000.0 + t, np.deg2rad(20.0) - t * 1e-4,
                      1e5 - t * 10.0, np.full(n, 0.5)])
    alpha = 0.01 * np.sin(t / 100.0)
    npz = folder / "pmp_baseline.npz"
    np.savez(npz, time=t, data=data, alpha=alpha, thrust=np.zeros(n))
    if manifest:
        config = {name: getattr(sim_params, name) for name in _NAMES}
        p, g = segref._reference_pso_settings()
        config.update({"GUIDANCE_MODE": "indirect_pmp",
                       "PSO_N_PARTICLES": p, "PSO_MAX_GENERATIONS": g})
        config.update(config_overrides)
        (folder / "pmp_baseline.manifest.json").write_text(
            json.dumps({"config": config}), encoding="utf-8")
    return npz, t, data, alpha


@pytest.fixture
def cache_in_tmp(tmp_path, monkeypatch):
    monkeypatch.setattr(sim_params, "PMP_REFERENCE_CACHE", str(tmp_path / "ref.npz"))
    monkeypatch.setattr(sim_params, "PMP_REFERENCE_USE_CACHE", True)
    monkeypatch.setattr(sim_params, "PMP_REFERENCE_FORCE_RERUN", False)

    def _no_rebuild(verbose):
        raise AssertionError("the loader tried to REBUILD the reference")
    monkeypatch.setattr(segref, "_run_pmp_reference", _no_rebuild)
    return tmp_path


def test_archive_seeds_a_cache_the_reference_loader_accepts(cache_in_tmp):
    npz, t, data, alpha = _write_archive(cache_in_tmp)
    path, _key = segref.cache_from_archive(npz, verbose=False)
    assert path == cache_in_tmp / "ref.npz" and path.exists()
    t2, d2, a2 = segref.get_pmp_reference_full(verbose=False)
    np.testing.assert_array_equal(t2, t)
    np.testing.assert_array_equal(d2, data[:5])          # state rows only, no latitude
    np.testing.assert_array_equal(a2, alpha)


def test_seeded_cache_is_ignored_once_the_configuration_moves(cache_in_tmp, monkeypatch):
    npz, *_ = _write_archive(cache_in_tmp)
    segref.cache_from_archive(npz, verbose=False)
    monkeypatch.setattr(sim_params, "TARGET_ORBITAL_ALTITUDE",
                        sim_params.TARGET_ORBITAL_ALTITUDE + 1e3)
    assert segref._load_cache(segref._abs_cache_path(), segref._reference_input_key()) is None


@pytest.mark.parametrize("override, named", [
    ({"INCLUDE_DRAG": not sim_params.INCLUDE_DRAG}, "INCLUDE_DRAG"),
    ({"PSO_MAX_GENERATIONS": 4}, "PSO_MAX_GENERATIONS"),
    ({"GUIDANCE_MODE": "gravity_turn"}, "GUIDANCE_MODE"),
    ({"INDIRECT_PMP_STAGE2_FRAME": "inertial"}, "INDIRECT_PMP_STAGE2_FRAME"),
    ({"PSO_UB": [1.0, 1.0, 1.0, 1000.0, 100.0, 100.0, 1.57]}, "PSO_UB"),
])
def test_an_archive_of_a_different_problem_is_refused(cache_in_tmp, override, named):
    npz, *_ = _write_archive(cache_in_tmp, **override)
    with pytest.raises(ValueError, match=named):
        segref.cache_from_archive(npz, verbose=False)
    assert not (cache_in_tmp / "ref.npz").exists()


def test_an_archive_without_a_manifest_is_still_cached(cache_in_tmp, capsys):
    npz, t, *_ = _write_archive(cache_in_tmp, manifest=False)
    segref.cache_from_archive(npz, verbose=True)
    assert "cannot be checked" in capsys.readouterr().out
    t2, _d, _a = segref.get_pmp_reference_full(verbose=False)
    np.testing.assert_array_equal(t2, t)
