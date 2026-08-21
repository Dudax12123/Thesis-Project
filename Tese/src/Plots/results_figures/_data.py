"""Loading layer for the Chapter 6 figures.

One :class:`Case` wraps the pair of files ``run_results_matrix.py`` writes per
case -- the ``.npz`` trajectory with its captured channels, and the ``.json``
scalar row -- and derives the plotted quantities on demand.

The derived channels are computed with the same helpers the interactive plot
suite uses (``Plots.plot_state_utils``, ``Auxiliary.losses``) rather than
reimplemented here, so a figure in the thesis and the corresponding debugging
plot cannot disagree about what a quantity means.

A missing case returns ``None`` rather than raising. That is deliberate: the
production batch takes most of a day, and a partially complete run must still
be able to draw the figures it has.
"""

import json
import sys
from pathlib import Path

import numpy as np

_SRC = Path(__file__).resolve().parent.parent.parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from Auxiliary import constants as c
from Auxiliary import losses as loss_mod
from Plots import plot_state_utils as psu

DEFAULT_ROOT = _SRC / "Output" / "results_matrix"


def _scalar(z, key):
    """A 0-d array back to a float, with the harness NaN convention as None."""
    if key not in z.files:
        return None
    value = float(z[key])
    return None if np.isnan(value) else value


class Case:
    """One flown trajectory, with everything a figure needs derived lazily."""

    def __init__(self, name, npz, row):
        self.name = name
        self.row = row
        self._z = npz

        self.time = np.asarray(npz["time"], dtype=float)
        self.data = np.asarray(npz["data"], dtype=float)
        self.thrust = np.asarray(npz["thrust"], dtype=float)
        self.alpha = np.asarray(npz["alpha"], dtype=float)

        self.t_meco = _scalar(npz, "t_meco")
        self.t_seco = _scalar(npz, "t_seco")
        self.t_coast_start = _scalar(npz, "t_coast_start")
        self.t_guidance_start = _scalar(npz, "t_guidance_start")

        self._ch = psu.extract_state_channels(self.data)

    # --- identity -------------------------------------------------------
    @property
    def law(self):
        return self.row.get("guidance_mode", "")

    @property
    def architecture(self):
        return self.row.get("architecture", "")

    @property
    def reached_orbit(self):
        """Whether the case is eligible to be ranked on propellant at all.

        A negative periapsis is a trajectory that intersects the Earth, and
        unspent propellant on a vehicle that failed to arrive is not a saving,
        so the figures annotate these rather than ranking them.
        """
        if self.row.get("crashed"):
            return False
        peri = self.row.get("periapsis_km")
        return peri is not None and peri > 0.0

    # --- state channels -------------------------------------------------
    @property
    def alt_km(self):
        return self._ch["alt_km"]

    @property
    def downrange_km(self):
        return self._ch["s_km"]

    @property
    def v(self):
        return self._ch["v"]

    @property
    def gamma_deg(self):
        return np.rad2deg(self._ch["gamma"])

    @property
    def alpha_deg(self):
        return np.rad2deg(self.alpha)

    @property
    def theta_deg(self):
        """Pitch, not stored: it is alpha + gamma by definition."""
        return np.rad2deg(self.alpha + self._ch["gamma"])

    @property
    def mass(self):
        return self._ch["m"]

    @property
    def latitude_deg(self):
        lat = self._ch["lat"]
        return None if lat is None else np.rad2deg(lat)

    @property
    def prop_kg(self):
        return psu.compute_propellant_mass(self._ch["m"], time_steps=self.time)

    @property
    def q(self):
        return psu.compute_dynamic_pressure(self._ch["v"], self._ch["alt"])

    @property
    def mach(self):
        return psu.compute_mach(self._ch["v"], self._ch["alt"])

    # --- captured diagnostics -------------------------------------------
    @property
    def coriolis(self):
        return np.asarray(self._z["coriolis"]) if "coriolis" in self._z.files else None

    @property
    def centrifugal(self):
        return (np.asarray(self._z["centrifugal"])
                if "centrifugal" in self._z.files else None)

    @property
    def pso_history(self):
        """(generations, best objective), or None for a solve with no swarm."""
        if "pso_gen" not in self._z.files:
            return None
        return np.asarray(self._z["pso_gen"]), np.asarray(self._z["pso_gbest"])

    @property
    def segment_schedule(self):
        """[(law, activation altitude [m]), ...] for a segmented run, else None."""
        if "segment_laws" not in self._z.files:
            return None
        laws = [str(x) for x in self._z["segment_laws"]]
        alts = [float(a) for a in self._z["segment_altitudes"]]
        return list(zip(laws, alts))

    @property
    def optimized_altitudes(self):
        if "optimized_altitudes" not in self._z.files:
            return None
        alts = [float(a) for a in np.atleast_1d(self._z["optimized_altitudes"])]
        return alts or None

    # --- derived accounting ---------------------------------------------
    def cutoff_index(self):
        """Index one past SECO -- the powered ascent, excluding the final coast.

        The loss integrals are only defined over the powered arc; carrying them
        through the ballistic coast would add a gravity loss that no propellant
        paid for.
        """
        if self.t_seco is None:
            return len(self.time)
        return max(int(np.searchsorted(self.time, self.t_seco, "right")), 2)

    def coast_intervals(self, floor_frac=0.01):
        """Unpowered spans of the ascent, as [(t_start, t_end), ...].

        Derived from the thrust trace instead of from a recorded coast window,
        because every architecture produces a thrust history while only
        ``pso_coast`` records where it placed a coast -- and the arc structure
        is exactly what the architecture comparison is about. Everything after
        SECO is excluded: the terminal ballistic arc is not a coast the
        optimiser chose.
        """
        end = self.cutoff_index()
        thrust = self.thrust[:end]
        time = self.time[:end]
        if not len(thrust):
            return []

        unpowered = thrust <= floor_frac * float(np.max(thrust))
        spans, start = [], None
        for i, quiet in enumerate(unpowered):
            if quiet and start is None:
                start = time[i]
            elif not quiet and start is not None:
                spans.append((start, time[i]))
                start = None
        if start is not None:
            spans.append((start, time[-1]))
        # Staging is a discontinuity, not a coast; anything shorter than a few
        # seconds is the separation transient rather than a chosen arc.
        return [(a, b) for a, b in spans if (b - a) > 5.0]

    def loss_histories(self):
        """Cumulative gravity/drag/steering/pressure losses over the powered arc."""
        idx = self.cutoff_index()
        alt = self.data[1, :idx] - c.R_EARTH
        return loss_mod.loss_histories(
            self.time[:idx], alt, self.data[2, :idx], self.data[3, :idx],
            self.data[4, :idx], self.thrust[:idx], self.alpha[:idx],
            t_meco=self.t_meco,
            include_drag=bool(self.row.get("include_drag", True)),
            thrust_mode=self.row.get("thrust_1_mode"),
        )

    def budget(self):
        """The scalar delta-v budget, read from the row rather than recomputed."""
        keys = ("dv_ideal", "dv_gravity", "dv_drag", "dv_steering", "dv_pressure",
                "dv_losses", "dv_gain", "dv_achieved", "residual")
        return {k: self.row.get(k) for k in keys}

    def __repr__(self):
        return "<Case %s (%s / %s)>" % (self.name, self.law, self.architecture)


def load(name, root=None):
    """One case, or None if the batch has not produced it yet."""
    root = Path(root) if root is not None else DEFAULT_ROOT
    npz_path = root / (name + ".npz")
    json_path = root / (name + ".json")
    if not npz_path.exists() or not json_path.exists():
        return None
    with open(json_path, encoding="utf-8") as fh:
        row = json.load(fh)
    return Case(name, np.load(npz_path), row)


def load_many(names, root=None):
    """Several cases as a dict, silently omitting those not yet produced."""
    out = {}
    for name in names:
        case = load(name, root=root)
        if case is not None:
            out[name] = case
    return out


def missing_from(cases, *names):
    """Which of *names* are absent, so a figure can skip itself cleanly.

    This is what lets the suite run against a partially complete batch instead
    of failing on the first case that has not been flown yet.
    """
    return [n for n in names if n not in cases]
