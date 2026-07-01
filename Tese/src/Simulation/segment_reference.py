"""
Segment Reference — indirect-PMP optimal trajectory as a waypoint source.

Builds (and disk-caches) the indirect-PMP optimal-control reference trajectory and
exposes a helper to read the optimal (radius, speed, flight-path-angle) state at an
arbitrary ascent altitude.  The segmented guidance driver
(``segmented_guidance_solver``) uses these waypoints as the terminal target for each
intermediate guidance segment, so every segment aims at the optimal-control values
rather than at the far-away circular orbit.

This is the ONLY module in the codebase that serialises a trajectory to disk; the
serialisation is deliberately isolated here.  Nothing runs unless
``MULTI_GUIDANCE_ENABLED`` is set and the segmented driver asks for a reference.

PyGMO is required to BUILD the reference (same dependency as the indirect PMP solve),
but only the first time — subsequent runs load the npz cache, so no PyGMO is needed
once a valid cache exists.
"""

import hashlib
import warnings
from pathlib import Path

import numpy as np

import sys
_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE.parent))

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r
from Input_File import simulation_parameters as sim_params


# ---------------------------------------------------------------------------
# Waypoint extraction
# ---------------------------------------------------------------------------

def waypoint_at_altitude(data_full, time_full, activation_alt):
    """Interpolate the PMP reference to the optimal state at a given altitude.

    Parameters
    ----------
    data_full   : ndarray (5 x N)  rows [s, r, v, gamma, m]  (SI; r = radius, gamma rad)
    time_full   : ndarray (N,)     time stamps [s]
    activation_alt : float         altitude above the surface [m]

    Returns
    -------
    dict {r, alt, v, gamma, m, t} — the reference state at ``activation_alt``
    (rotating-frame velocity, gamma in rad). ``r`` is the exact target radius
    ``R_EARTH + activation_alt``.
    """
    data_full = np.asarray(data_full, dtype=float)
    time_full = np.asarray(time_full, dtype=float)

    alt_col = data_full[1] - c.R_EARTH
    # Altitude is monotonic-increasing over the ascent; restrict to that prefix so
    # any post-apogee descent cannot break np.interp's required ascending x array.
    i_top = int(np.argmax(alt_col))
    if i_top < 1:
        i_top = len(alt_col) - 1
    x = alt_col[: i_top + 1]

    if activation_alt > x[-1]:
        warnings.warn(
            f"[segment_reference] activation altitude {activation_alt/1e3:.1f} km exceeds "
            f"the PMP reference apogee {x[-1]/1e3:.1f} km; clamping to the endpoint.",
            RuntimeWarning,
        )

    v_wp     = float(np.interp(activation_alt, x, data_full[2, : i_top + 1]))
    gamma_wp = float(np.interp(activation_alt, x, data_full[3, : i_top + 1]))
    m_wp     = float(np.interp(activation_alt, x, data_full[4, : i_top + 1]))
    t_wp     = float(np.interp(activation_alt, x, time_full[: i_top + 1]))

    return {
        "r":     c.R_EARTH + activation_alt,
        "alt":   activation_alt,
        "v":     v_wp,
        "gamma": gamma_wp,
        "m":     m_wp,
        "t":     t_wp,
    }


# ---------------------------------------------------------------------------
# Cache plumbing
# ---------------------------------------------------------------------------

def _project_root():
    # .../Tese/src/Simulation/segment_reference.py -> parents[3] = project root
    return Path(__file__).resolve().parents[3]


def _abs_cache_path():
    p = Path(sim_params.PMP_REFERENCE_CACHE)
    if not p.is_absolute():
        p = _project_root() / p
    return p


def _reference_pso_settings():
    """(particles, generations) for the PMP-reference build: the dedicated
    PMP_REFERENCE_PSO_* knobs when set, else the indirect-PMP PSO defaults."""
    p = getattr(sim_params, "PMP_REFERENCE_PSO_PARTICLES", None)
    g = getattr(sim_params, "PMP_REFERENCE_PSO_GENERATIONS", None)
    p = int(sim_params.PSO_N_PARTICLES if p is None else p)
    g = int(sim_params.PSO_MAX_GENERATIONS if g is None else g)
    return p, g


def _reference_input_key():
    """Hash of every input that changes the PMP reference trajectory."""
    _ref_particles, _ref_generations = _reference_pso_settings()
    payload = (
        ("TARGET_ORBITAL_ALTITUDE", float(sim_params.TARGET_ORBITAL_ALTITUDE)),
        ("TARGET_ORBIT_INCLINATION", float(sim_params.TARGET_ORBIT_INCLINATION)),
        ("LAUNCH_LATITUDE", float(sim_params.LAUNCH_LATITUDE)),
        ("ENABLE_EARTH_ROTATION", bool(sim_params.ENABLE_EARTH_ROTATION)),
        ("INCLUDE_DRAG", bool(sim_params.INCLUDE_DRAG)),
        ("PSO_SEED", int(getattr(sim_params, "PSO_SEED", 0))),
        # Labels kept as PSO_N_PARTICLES/PSO_MAX_GENERATIONS so a default build
        # (knobs=None ⇒ same values as the indirect PSO) keeps the existing cache
        # valid; raising PMP_REFERENCE_PSO_* changes the value here and rebuilds.
        ("PSO_N_PARTICLES", _ref_particles),
        ("PSO_MAX_GENERATIONS", _ref_generations),
        ("PSO_LB", tuple(float(x) for x in getattr(sim_params, "PSO_LB", ()))),
        ("PSO_UB", tuple(float(x) for x in getattr(sim_params, "PSO_UB", ()))),
        # Vehicle (rocket_specs) — anything that changes the optimal trajectory
        ("M_PAYLOAD", float(r.M_PAYLOAD)), ("M_FAIRING", float(r.M_FAIRING)),
        ("ISP_1_SL", float(r.ISP_1_SL)), ("ISP_1_VAC", float(r.ISP_1_VAC)),
        ("F_THRUST_1_SL", float(r.F_THRUST_1_SL)), ("F_THRUST_1_VAC", float(r.F_THRUST_1_VAC)),
        ("M_STRUCTURE_1", float(r.M_STRUCTURE_1)), ("M_PROP_1", float(r.M_PROP_1)),
        ("ISP_2", float(r.ISP_2)), ("F_THRUST_2", float(r.F_THRUST_2)),
        ("M_STRUCTURE_2", float(r.M_STRUCTURE_2)), ("M_PROP_2", float(r.M_PROP_2)),
    )
    return hashlib.sha256(repr(payload).encode("utf-8")).hexdigest()


def _load_cache(path, key):
    if not path.exists():
        return None
    try:
        with np.load(path, allow_pickle=False) as npz:
            if str(npz["key"]) != key:
                return None
            return np.asarray(npz["time_full"]), np.asarray(npz["data_full"])
    except Exception as exc:  # corrupt/old cache -> rebuild
        warnings.warn(f"[segment_reference] ignoring unreadable cache {path}: {exc}",
                      RuntimeWarning)
        return None


def _save_cache(path, key, time_full, data_full):
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, key=np.array(key), time_full=time_full, data_full=data_full)


def _run_pmp_reference(verbose):
    """Run the indirect-PMP PSO + dense re-run. Requires PyGMO."""
    import Simulation.indirect_pso_solver as ips
    n_particles, n_gen = _reference_pso_settings()
    if verbose:
        print(f"[segment_reference] reference PSO fidelity: "
              f"{n_particles} particles x {n_gen} generations")
    best_x, _J = ips.run_pso_optimization(
        verbose=verbose, n_particles=n_particles, n_gen=n_gen)
    out = ips.run_indirect_full(best_x, verbose=verbose)
    time_full, data_full = out[0], out[1]
    return np.asarray(time_full, dtype=float), np.asarray(data_full, dtype=float)


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def get_pmp_reference(verbose=True):
    """Return (time_full, data_full) for the indirect-PMP reference trajectory.

    Loads the npz cache when present, valid (input hash matches) and enabled;
    otherwise runs the PMP solve (PyGMO required) and caches the result.
    """
    cache_path = _abs_cache_path()
    key = _reference_input_key()

    if sim_params.PMP_REFERENCE_USE_CACHE and not sim_params.PMP_REFERENCE_FORCE_RERUN:
        cached = _load_cache(cache_path, key)
        if cached is not None:
            if verbose:
                print(f"[segment_reference] loaded PMP reference from cache: {cache_path}")
            return cached

    if verbose:
        print("[segment_reference] building PMP reference (PyGMO PSO) — this is slow; "
              "the result will be cached for reuse.")
    time_full, data_full = _run_pmp_reference(verbose)
    _save_cache(cache_path, key, time_full, data_full)
    if verbose:
        print(f"[segment_reference] cached PMP reference to: {cache_path}")
    return time_full, data_full
