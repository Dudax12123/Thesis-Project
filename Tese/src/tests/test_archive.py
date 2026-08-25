"""Unit tests for the run archive.

The archive exists so that a solve measured in tens of minutes never has to be
flown twice, which makes exactly one property worth testing above all others:
what goes in comes back out. Everything here is built from a synthetic
trajectory rather than a real solve, so the tests run in milliseconds and test
the archiving rather than the physics.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Archive import compare as compare_mod
from Archive import run_record
from Archive import store
from Input_File import simulation_parameters as sim_params
from Plots.results_figures import _data as rf_data
from Simulation import rocket_ascent as ra


N = 201

# The rocket_ascent globals the collector reads. They are module state, so every
# test that sets them has to put them back -- the same hazard the simulator has.
_RA_STATE = ("time_main_engine_cutoff", "TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL",
             "PSO_COAST_ARC2_START_TIME", "time_guidance_start",
             "time_kick_start", "time_atmosphere_exit",
             "PROPAGATING_IN_INERTIAL_FRAME", "theta_history",
             "theta_time_history", "tgo_history", "tgo_time_history",
             "cross_heading_counter_force_history", "cross_heading_accel_history")


@pytest.fixture
def flown():
    """A synthetic ascent with every history global populated, then restored."""
    saved = {name: getattr(ra, name) for name in _RA_STATE}

    t = np.linspace(0.0, 400.0, N)
    data = np.vstack([
        t * 900.0,                                   # downrange
        6378e3 + t * 1200.0,                         # radius
        50.0 + t * 18.0,                             # speed
        np.deg2rad(89.0 - 0.2 * t),                  # flight-path angle
        520e3 - t * 900.0,                           # mass
    ])
    thrust = np.full(N, 7607e3)
    alpha = np.linspace(0.0, -0.05, N)

    ra.time_main_engine_cutoff = 150.0
    ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL = 380.0
    ra.PSO_COAST_ARC2_START_TIME = 250.0
    ra.time_guidance_start = 60.0
    ra.time_kick_start = 12.0
    ra.time_atmosphere_exit = 95.0
    ra.PROPAGATING_IN_INERTIAL_FRAME = False
    # Deliberately on a different cadence from the state grid: this is what the
    # native-grid replay set is for, and storing it on the wrong grid would be
    # invisible unless a test checks the length that comes back.
    ra.theta_time_history = list(np.linspace(0.0, 400.0, 77))
    ra.theta_history = list(np.linspace(1.5, 0.1, 77))
    ra.tgo_time_history = list(np.linspace(60.0, 380.0, 41))
    ra.tgo_history = list(np.linspace(320.0, 0.0, 41))
    ra.cross_heading_counter_force_history = list(np.linspace(0.0, 900.0, 63))
    ra.cross_heading_accel_history = list(np.linspace(0.0, 0.02, 63))

    result = {'crashed': False, 'state_final': data[:, -1],
              'circularisation_dv': 0.0}
    history = {'gen': np.arange(5.0), 'gbest': np.array([9.0, 5.0, 3.0, 2.5, 2.4])}

    yield dict(time=t, data=data, thrust=thrust, alpha=alpha, result=result,
               history=history)

    for name, value in saved.items():
        setattr(ra, name, value)


def _save(flown, root, **kwargs):
    return store.save_run(sim_params, flown['time'], flown['data'],
                          flown['thrust'], flown['alpha'], flown['result'],
                          J=1.25, history=flown['history'],
                          wall_clock=12.5, source="test", root=root,
                          verbose=False, **kwargs)


# =========================================================================
#  Round trip
# =========================================================================

def test_every_channel_survives_the_round_trip(flown, tmp_path):
    saved = _save(flown, tmp_path)
    case = rf_data.load(saved['name'], root=tmp_path)
    assert case is not None

    with np.load(tmp_path / (saved['name'] + ".npz")) as z:
        keys = set(z.files)
        assert np.allclose(z['time'], flown['time'])
        assert np.allclose(z['data'], flown['data'])
        assert np.allclose(z['thrust'], flown['thrust'])
        assert np.allclose(z['alpha'], flown['alpha'])
        # A history with its OWN time axis comes back on the cadence it was
        # recorded at, not resampled onto the state grid.
        assert len(z['suite_theta']) == 77
        assert len(z['suite_tgo']) == 41
        # The cross-heading channels have no axis of their own -- the suite
        # plots them against the shared time_thrust -- so the 63-sample history
        # cannot be kept, and the state-grid equivalent is stored instead. See
        # test_every_replay_channel_fits_the_axis_it_is_plotted_against.
        assert len(z['suite_cross_force']) == N
        assert np.allclose(z['pso_gbest'], flown['history']['gbest'])

    # Arc times, including the two the results matrix never captured.
    for key in ('t_meco', 't_seco', 't_coast_start', 't_guidance_start',
                't_kick_start', 't_atmosphere_exit'):
        assert key in keys
    assert case.t_meco == 150.0
    assert case.t_seco == 380.0
    assert case.t_coast_start == 250.0
    assert case.t_guidance_start == 60.0


def test_an_event_that_never_happened_reads_back_as_none(flown, tmp_path):
    ra.PSO_COAST_ARC2_START_TIME = None
    saved = _save(flown, tmp_path)
    case = rf_data.load(saved['name'], root=tmp_path)
    assert case.t_coast_start is None


def test_the_row_carries_the_orbit_and_the_budget(flown, tmp_path):
    saved = _save(flown, tmp_path)
    with open(tmp_path / (saved['name'] + ".json"), encoding="utf-8") as fh:
        row = json.load(fh)
    for key in ('architecture', 'guidance_mode', 'eccentricity', 'periapsis_km',
                'prop_remaining_kg', 'dv_ideal', 'dv_gravity', 'dv_achieved',
                'wall_clock_s', 'pso_gbest_last'):
        assert row.get(key) is not None, key
    assert row['J_prime'] == 1.25
    assert row['wall_clock_s'] == 12.5


def test_tags_reach_the_row_without_a_matrix_case(flown, tmp_path):
    """The generalisation that let main.py call the harness's collector."""
    saved = _save(flown, tmp_path, tags={'section': '6.2', 'factor': 'baseline'})
    assert saved['row']['section'] == '6.2'
    assert saved['row']['factor'] == 'baseline'


# =========================================================================
#  The replay set
# =========================================================================

def test_deduplicated_channels_resolve_through_their_alias(flown, tmp_path):
    """A dropped channel must come back as the original, not as a guess.

    The PSO solvers hand thrust and alpha back already on the state grid, so
    half the replay set would be byte-identical to arrays already stored. They
    are dropped and the substitution recorded, which only works if the alias is
    followed on the way out.
    """
    saved = _save(flown, tmp_path)
    with np.load(tmp_path / (saved['name'] + ".npz")) as z:
        aliases = [str(a) for a in z['suite_aliases']]
        # thrust and alpha were passed on the state grid, so they alias
        assert 'suite_thrust=thrust' in aliases
        assert 'suite_alpha=alpha' in aliases
        assert 'suite_thrust' not in z.files

        resolved = {}
        for npz_key, kwarg in run_record.REPLAY_KEYS.items():
            if npz_key in z.files:
                resolved[kwarg] = np.asarray(z[npz_key])
        for entry in aliases:
            npz_key, _, target = entry.partition("=")
            resolved[run_record.REPLAY_KEYS[npz_key]] = np.asarray(z[target])

    assert np.allclose(resolved['thrust_data'], flown['thrust'])
    assert np.allclose(resolved['alpha_data'], flown['alpha'])
    # Every channel the twenty-plot suite takes positionally must be recoverable
    # or the archive is not replayable.
    for kwarg in ('thrust_data', 'time_thrust', 'alpha_data', 'alpha_time_data',
                  'theta_data', 'theta_time_data', 'tgo_data', 'tgo_time_data'):
        assert kwarg in resolved, kwarg
    # and each pair has to be the same length or the suite raises on the plot
    for values, times in (('thrust_data', 'time_thrust'),
                          ('alpha_data', 'alpha_time_data'),
                          ('theta_data', 'theta_time_data'),
                          ('tgo_data', 'tgo_time_data')):
        assert len(resolved[values]) == len(resolved[times]), values


# =========================================================================
#  Naming -- runs must accumulate, never overwrite
# =========================================================================

def test_a_second_run_of_the_same_configuration_does_not_overwrite(flown, tmp_path):
    first = _save(flown, tmp_path)
    second = _save(flown, tmp_path)
    assert first['name'] != second['name']
    assert (tmp_path / (first['name'] + ".npz")).exists()
    assert (tmp_path / (second['name'] + ".npz")).exists()


def test_a_named_run_is_replaced_rather_than_accumulated(flown, tmp_path):
    """The results matrix needs the opposite rule: its case names are identities."""
    _save(flown, tmp_path, name="gt_baseline")
    _save(flown, tmp_path, name="gt_baseline")
    assert len(list(tmp_path.glob("*.npz"))) == 1


def test_the_run_id_names_the_architecture_and_the_law(flown, tmp_path):
    saved = _save(flown, tmp_path)
    assert saved['name'].startswith(run_record.architecture(sim_params))
    assert sim_params.GUIDANCE_MODE in saved['name']


def test_the_index_lists_every_archive(flown, tmp_path):
    _save(flown, tmp_path)
    _save(flown, tmp_path)
    entries = store.list_runs(root=tmp_path)
    assert len(entries) == 2
    assert all(entry['run_id'] for entry in entries)
    path = store.write_index(root=tmp_path)
    assert path.exists()
    assert len(path.read_text(encoding="utf-8").strip().splitlines()) == 3


# =========================================================================
#  Finding an archive again
# =========================================================================

def test_resolve_accepts_a_unique_prefix(flown, tmp_path):
    saved = _save(flown, tmp_path)
    root, stem = store.resolve(saved['name'][:12], root=tmp_path)
    assert stem == saved['name']
    assert root == tmp_path


def test_resolve_refuses_an_ambiguous_prefix(flown, tmp_path):
    _save(flown, tmp_path, name="gt_baseline")
    _save(flown, tmp_path, name="gt_vacuum")
    with pytest.raises(ValueError):
        store.resolve("gt_", root=tmp_path)


def test_resolve_reaches_into_a_named_directory(flown, tmp_path):
    _save(flown, tmp_path, name="gt_baseline")
    root, stem = store.resolve("%s::gt_baseline" % tmp_path)
    assert stem == "gt_baseline"
    assert Path(root) == tmp_path


def test_resolve_says_so_when_there_is_nothing_there(tmp_path):
    with pytest.raises(FileNotFoundError):
        store.resolve("no_such_run", root=tmp_path)


# =========================================================================
#  The manifest
# =========================================================================

def test_the_manifest_records_the_whole_configuration(flown, tmp_path):
    saved = _save(flown, tmp_path)
    case = rf_data.load(saved['name'], root=tmp_path)
    man = case.manifest
    assert man['config']['GUIDANCE_MODE'] == sim_params.GUIDANCE_MODE
    assert man['config']['TARGET_ORBITAL_ALTITUDE'] == sim_params.TARGET_ORBITAL_ALTITUDE
    assert man['vehicle']['M_PROP_1']
    assert man['constants']['R_EARTH']
    assert man['env']['python']
    assert 'commit' in man['git']
    assert man['source'] == "test"
    # Captured by iterating the module, so a setting added later is archived
    # without anyone having to remember this file exists.
    assert len(man['config']) > 100


def test_unserialisable_settings_are_left_out_not_stringified():
    """An address in a manifest would make two identical runs look different."""
    class FakeConfig(object):
        pass
    fake = FakeConfig()
    fake.GOOD_FLOAT = 1.5
    fake.GOOD_LIST = [("a", 1.0), ("b", 2.0)]
    fake.GOOD_NUMPY = np.float64(3.5)
    fake.BAD_CALLABLE = lambda: None
    fake.lowercase_ignored = 7
    snapshot = run_record.module_snapshot(fake)
    assert snapshot['GOOD_FLOAT'] == 1.5
    assert snapshot['GOOD_LIST'] == [["a", 1.0], ["b", 2.0]]
    assert snapshot['GOOD_NUMPY'] == 3.5
    assert 'BAD_CALLABLE' not in snapshot
    assert 'lowercase_ignored' not in snapshot
    json.dumps(snapshot)


def test_an_archive_without_a_manifest_still_loads(flown, tmp_path):
    """The results-matrix batch was written before manifests existed."""
    saved = _save(flown, tmp_path)
    (tmp_path / (saved['name'] + ".manifest.json")).unlink()
    case = rf_data.load(saved['name'], root=tmp_path)
    assert case is not None
    assert case.manifest == {}
    assert len(case.time) == N          # everything else is still there


# =========================================================================
#  Comparison
# =========================================================================

def _case_with(manifest, name="run"):
    """A minimal Case carrying only a manifest, for the diff tests."""
    t = np.linspace(0.0, 1.0, 4)
    return rf_data.Case.from_arrays(name, t, np.zeros((5, 4)), np.zeros(4),
                                    np.zeros(4), row={}, manifest=manifest)


def test_the_diff_names_the_setting_that_actually_differs():
    a = _case_with({'config': {'INCLUDE_DRAG': True, 'TARGET_ORBITAL_ALTITUDE': 500e3},
                    'vehicle': {'M_PROP_1': 395.7e3}}, "a")
    b = _case_with({'config': {'INCLUDE_DRAG': False, 'TARGET_ORBITAL_ALTITUDE': 500e3},
                    'vehicle': {'M_PROP_1': 395.7e3}}, "b")
    compared, rows, unrecorded = compare_mod.manifest_diff([a, b])
    assert len(compared) == 2
    assert not unrecorded
    assert rows == [("config", "INCLUDE_DRAG", [True, False])]


def test_identical_configurations_produce_an_empty_diff():
    block = {'config': {'INCLUDE_DRAG': True}, 'vehicle': {'M_PROP_1': 1.0}}
    _compared, rows, _unrecorded = compare_mod.manifest_diff(
        [_case_with(dict(block), "a"), _case_with(dict(block), "b")])
    assert rows == []


def test_a_setting_present_in_only_one_manifest_shows_as_absent():
    a = _case_with({'config': {'ARCHIVE_RUNS': True}}, "a")
    b = _case_with({'config': {}}, "b")
    _compared, rows, _unrecorded = compare_mod.manifest_diff([a, b])
    assert rows == [("config", "ARCHIVE_RUNS", [True, compare_mod.ABSENT])]


def test_a_run_without_a_manifest_is_named_not_diffed_against():
    """Otherwise it would appear to disagree about all 129 settings at once."""
    a = _case_with({'config': {'INCLUDE_DRAG': True}}, "a")
    b = _case_with({}, "no_manifest")
    compared, rows, unrecorded = compare_mod.manifest_diff([a, b])
    assert [case.name for case in compared] == ["a"]
    assert unrecorded == ["no_manifest"]
    assert rows == []


# =========================================================================
#  Directory layouts
# =========================================================================

def test_a_case_in_its_own_folder_loads_the_same_way(flown, tmp_path):
    """The results-matrix layout: <root>/<case>/<case>.npz.

    Twenty cases as sixty-one files in one directory is unreadable, so the
    harness gives each its own folder. Nothing downstream may need to know:
    the figures, the CLI and the comparison all go through one resolver.
    """
    case_root = tmp_path / "gt_baseline"
    _save(flown, case_root, name="gt_baseline")

    assert (case_root / "gt_baseline.npz").exists()
    assert store.case_dir(tmp_path, "gt_baseline") == case_root

    case = rf_data.load("gt_baseline", root=tmp_path)
    assert case is not None
    assert len(case.time) == N
    assert case.manifest                       # the sidecar came with it

    found_root, stem = store.resolve("gt_baseline", root=tmp_path)
    assert (found_root, stem) == (tmp_path, "gt_baseline")
    assert store.load_run("gt_baseline", root=tmp_path).name == "gt_baseline"


def test_flat_and_nested_archives_coexist(flown, tmp_path):
    """A directory holding both layouts lists and loads completely.

    Interactive archives stay flat -- their timestamped id is already unique and
    a folder per run would only add a level -- so a root can legitimately hold
    both at once.
    """
    _save(flown, tmp_path / "gt_baseline", name="gt_baseline")
    flat = _save(flown, tmp_path)

    listed = {entry['run_id'] for entry in store.list_runs(root=tmp_path)}
    assert listed == {"gt_baseline", flat['name']}
    assert rf_data.load("gt_baseline", root=tmp_path) is not None
    assert rf_data.load(flat['name'], root=tmp_path) is not None


def test_a_stray_npz_in_a_case_folder_is_not_an_archive(flown, tmp_path):
    """Only the folder's own case counts, or any scratch file becomes a run."""
    _save(flown, tmp_path / "gt_baseline", name="gt_baseline")
    np.savez(tmp_path / "gt_baseline" / "scratch.npz", x=np.arange(3))

    listed = {entry['run_id'] for entry in store.list_runs(root=tmp_path)}
    assert listed == {"gt_baseline"}


def test_every_replay_channel_fits_the_axis_it_is_plotted_against(flown, tmp_path):
    """The archive must not store a channel the suite cannot plot.

    Five channels share one ``time_thrust`` axis, and they do not all arrive on
    the same grid: the fixture records the cross-heading history at ODE cadence
    while thrust comes in on the state grid, exactly as the results-matrix
    worker produces them for apogee_check. Stored unreconciled, that archive
    loads fine and only dies at replay -- long after the run is gone.
    """
    saved = _save(flown, tmp_path)
    with np.load(tmp_path / (saved['name'] + ".npz")) as z:
        resolved = {}
        for npz_key, kwarg in run_record.REPLAY_KEYS.items():
            if npz_key in z.files:
                resolved[kwarg] = np.asarray(z[npz_key])
        for entry in z.get('suite_aliases', []):
            npz_key, _, target = str(entry).partition("=")
            resolved[run_record.REPLAY_KEYS[npz_key]] = np.asarray(z[target])

    for channel, axis_name in run_record.SUITE_AXIS.items():
        if channel not in resolved or axis_name not in resolved:
            continue
        assert len(resolved[channel]) == len(resolved[axis_name]), (
            "%s does not fit %s" % (channel, axis_name))

    # The cross-heading pair was recorded at 63 samples against a 201-sample
    # axis, so it must have been replaced by its state-grid equivalent rather
    # than stored as-is or silently dropped.
    assert len(resolved['cross_heading_counter_force_data']) == N
    assert len(resolved['cross_heading_accel_data']) == N
