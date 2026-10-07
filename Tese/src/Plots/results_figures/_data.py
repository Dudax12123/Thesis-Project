"""Loading layer for the Chapter 6 figures.

One :class:`Case` wraps the archive that ``Archive/store.py`` writes -- the
``.npz`` trajectory with its captured channels, the ``.json`` scalar row, and
the optional ``.manifest.json`` describing the configuration it was flown under
-- and derives the plotted quantities on demand.

That archive is written both by ``run_results_matrix.py``, once per matrix case,
and by ``main.py``, once per interactive run, so this is the one loader for
both. The manifest is optional because archives written before it existed must
still load; an absent one is ``Case.manifest == {}``.

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

# The seventeen cases Chapter 6 reports, in chapter order. The matrix flies three
# more -- gt_direct, gt_vacuum and peg_vacuum_norot -- which stay archived but
# are not reported (decision 2026-09-29), so no figure draws them. The two
# reference-tracking cases, show_ref_track and show_ref_track_apollo, left the
# matrix on 2026-10-07 (user decision); their archives are kept.
REPORTED_CASES = [
    "pmp_baseline", "pmp_vacuum", "pmp_norot",
    "gt_baseline", "gt_apogee", "gt_norot", "gt_sea_level_engine",
    "peg_baseline", "peg_direct", "peg_vacuum",
    "show_cpr", "show_linear_tangent", "show_bilinear_tangent",
    "show_exp_shooting", "show_apollo",
    "show_seg_fixed_alt", "show_seg_opt_alt",
]

# Drawn beside a reported case but never ranked or tabulated, so no figure that
# iterates REPORTED_CASES and no table sees them. pmp_vacuum_c3500 is the
# drag-free reference refined from the same swarm point with the coast bound at
# 3500 s instead of 2000 s: without drag it coasts from a lowest point 4.4 km
# above the surface (Section 6.1, user decision 2026-10-07).
SUPPLEMENTARY_CASES = ["pmp_vacuum_c3500"]


def _scalar(z, key):
    """A 0-d array back to a float, with the harness NaN convention as None."""
    if key not in z.files:
        return None
    value = float(z[key])
    return None if np.isnan(value) else value


class Case:
    """One flown trajectory, with everything a figure needs derived lazily."""

    def __init__(self, name, npz, row, manifest=None):
        self.name = name
        self.row = row
        # The configuration the run was flown under, when the archive carries
        # one. Archives written before manifests existed -- including the
        # results-matrix batch -- simply have {}, which is why every reader
        # must treat an empty manifest as "not recorded" rather than "no
        # settings differ".
        self.manifest = dict(manifest or {})
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
        """Steering angle, 0 wherever the engine is off: an unpowered point mass
        has no attitude, and the stored channel there is only the interpolated
        guidance log (Plots.plot_state_utils.zero_alpha_when_unpowered)."""
        return np.rad2deg(psu.zero_alpha_when_unpowered(self.time, self.alpha,
                                                        self.time, self.thrust))

    @property
    def theta_deg(self):
        """Pitch, not stored: it is alpha + gamma by definition -- the flight-path
        angle wherever the engine is off."""
        return self.alpha_deg + np.rad2deg(self._ch["gamma"])

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
    def extremal_budget(self):
        """(particles, generations) of the swarm a re-flown PMP extremal came from.

        The results matrix re-flies the indirect-PMP rows from a stored, polished
        decision vector instead of swarming them, so their wall clock and
        evaluation count describe the re-flight, not the search. The search
        itself ran offline; this is the part of its cost the archive records.
        None for every case that ran its own search.
        """
        if "extremal_swarm_budget" not in self._z.files:
            return None
        budget = np.atleast_1d(self._z["extremal_swarm_budget"])
        return tuple(int(b) for b in budget)

    @property
    def optimized_altitudes(self):
        if "optimized_altitudes" not in self._z.files:
            return None
        alts = [float(a) for a in np.atleast_1d(self._z["optimized_altitudes"])]
        return alts or None

    @property
    def decision_vector(self):
        """The optimiser's full-precision decision vector, or None."""
        if "decision_vector" not in self._z.files:
            return None
        return np.atleast_1d(np.asarray(self._z["decision_vector"], dtype=float))

    @property
    def gamma_p(self):
        """The flight-path angle the kick leaves [rad], wherever the case stores it.

        Each architecture puts it in a different slot of its decision vector:
        fourth for pso_coast, seventh for the PMP, alone for law-terminated
        direct insertion, second for the law-terminated segmented schedule. The
        reference-tracking cases fly the reference's, and record it in the
        schedule they realised. The apogee check stores the kick itself,
        gamma_p - pi/2 (solver.find_initial_kick_angle_coast_single_burn).
        """
        x = self.decision_vector
        arch = self.architecture
        if arch == "reference_track" and "realised_schedule" in self._z.files:
            return float(self._z["realised_schedule"][3])
        if x is None:
            return None
        if arch == "apogee_check":
            return float(x[0]) + np.pi / 2.0
        slot = {"pso_coast": 3, "indirect_pmp": 6, "direct": 0}.get(arch)
        if arch == "segmented":
            # [delta_tc, gamma_p (, hand-off)] law-terminated; the default form
            # is pso_coast's four base variables.
            slot = 1 if len(x) <= 3 else 3
        return None if slot is None or slot >= len(x) else float(x[slot])

    def waypoint_miss(self):
        """(dh [m], dv [m/s], dgamma [deg]) at the end of the first burn against
        the coast-start waypoint it aimed at, or None for a case that aimed at
        none. Read from the stored target and achieved state, not re-derived."""
        if "arc1_target" not in self._z.files or "arc1_achieved" not in self._z.files:
            return None
        target = np.asarray(self._z["arc1_target"], dtype=float)
        achieved = np.asarray(self._z["arc1_achieved"], dtype=float)
        dr, dv, dgamma = (achieved[:3] - target[:3])
        return float(dr), float(dv), float(np.rad2deg(dgamma))

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

    @property
    def t_insertion(self):
        """When the vehicle reaches its orbit [s].

        SECO for every architecture but one. The apogee check cuts its engine
        below the target and coasts up to apogee, where an impulsive
        circularisation inserts it. Archives since 2026-09-30 record that
        instant (``t_circularisation``). Older ones did not, so it is read as
        the one velocity jump after SECO (every archive carries ~1000 s of orbit
        after insertion, so the end of the time axis is not it either).
        """
        if self.architecture != "apogee_check" or self.t_seco is None:
            return self.t_seco
        recorded = _scalar(self._z, "t_circularisation")
        if recorded is not None:
            return recorded
        after = np.where(self.time > self.t_seco + 1.0)[0]
        if len(after) < 2:
            return self.t_seco
        jumps = np.abs(np.diff(self.v[after]))
        return float(self.time[after[int(np.argmax(jumps)) + 1]])

    def insertion_index(self):
        """Index one past insertion: the ascent, excluding the orbit after it."""
        t_ins = self.t_insertion
        if t_ins is None:
            return len(self.time)
        return max(int(np.searchsorted(self.time, t_ins, "right")), 2)

    def coast_intervals(self, floor_frac=0.01):
        """Unpowered spans of the ascent, as [(t_start, t_end), ...].

        Derived from the thrust trace instead of from a recorded coast window,
        because every architecture produces a thrust history while only
        ``pso_coast`` records where it placed a coast -- and the arc structure
        is exactly what the architecture comparison is about. Everything after
        SECO is excluded: the terminal ballistic arc is not a coast the
        optimiser chose. The one exception is the apogee check, whose coast to
        apogee follows its SECO and ends at the impulsive circularisation.
        The staging interval after MECO is left out: it is a planned separation
        delay, not a coast.
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
        # Staging is a discontinuity, not a coast. The separation delay after
        # MECO runs 8 s, longer than the 5 s transient floor, so it is dropped
        # by where it starts rather than by how long it lasts.
        spans = [(a, b) for a, b in spans
                 if (b - a) > 5.0
                 and not (self.t_meco is not None and abs(a - self.t_meco) < 1.0)]
        t_ins = self.t_insertion
        if (self.architecture == "apogee_check" and t_ins is not None
                and self.t_seco is not None and t_ins > self.t_seco + 5.0):
            spans.append((self.t_seco, t_ins))
        return spans

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

    @classmethod
    def from_arrays(cls, name, time, data, thrust, alpha, row=None, **channels):
        """A Case built in memory, for a live run that never went through the harness.

        main.py can draw the run card for a single interactive run this way,
        without writing an .npz first. Channels the caller does not have --
        typically the pseudo-force diagnostics and the arc times -- are simply
        absent, and the figures skip whatever depends on them.
        """
        store = {"time": np.asarray(time, dtype=float),
                 "data": np.asarray(data, dtype=float),
                 "thrust": np.asarray(thrust, dtype=float),
                 "alpha": np.asarray(alpha, dtype=float)}
        manifest = channels.pop("manifest", None)
        for key, value in channels.items():
            if value is not None:
                store[key] = np.asarray(value)
        return cls(name, _InMemoryNpz(store), dict(row or {}), manifest=manifest)


class _InMemoryNpz:
    """The little of numpy's NpzFile interface that Case actually uses.

    Case reads ``.files`` and indexes by key; wrapping a plain dict in that
    shape keeps one code path for cases loaded from disk and cases handed over
    in memory, rather than a parallel set of getters that could drift.
    """

    def __init__(self, store):
        self._store = store

    @property
    def files(self):
        return list(self._store)

    def __getitem__(self, key):
        return self._store[key]

    def __contains__(self, key):
        return key in self._store


def load(name, root=None):
    """One case, or None if the batch has not produced it yet.

    The optional third file, ``<name>.manifest.json``, carries the configuration
    the run was flown under. It is read when present and skipped when not, which
    is what lets one loader serve an interactive archive, a results-matrix case
    written today, and a case written before the manifest existed.

    Both directory layouts are accepted -- ``<root>/<name>/<name>.npz`` as the
    results matrix writes it, and ``<root>/<name>.npz`` flat as an interactive
    archive does -- so a figure never has to know which produced the case.
    """
    # Imported here rather than at module scope: Archive.store pulls in the
    # collector, and the results-matrix harness deliberately keeps matplotlib
    # and everything downstream of it out of a batch worker.
    from Archive.store import case_dir

    root = Path(root) if root is not None else DEFAULT_ROOT
    # The results matrix gives each case its own folder; interactive archives
    # sit flat. One helper decides which, so no caller has to know.
    holder = case_dir(root, name)
    npz_path = holder / (name + ".npz")
    json_path = holder / (name + ".json")
    if not npz_path.exists() or not json_path.exists():
        return None
    with open(json_path, encoding="utf-8") as fh:
        row = json.load(fh)
    manifest = {}
    manifest_path = holder / (name + ".manifest.json")
    if manifest_path.exists():
        with open(manifest_path, encoding="utf-8") as fh:
            manifest = json.load(fh)
    return Case(name, np.load(npz_path), row, manifest=manifest)


def load_many(names, root=None):
    """Several cases as a dict, silently omitting those not yet produced."""
    out = {}
    for name in names:
        case = load(name, root=root)
        if case is not None:
            out[name] = case
    return out


def check_one_rotation_model(cases):
    """Refuse a set of cases whose rotation-on rows were flown under different models.

    EARTH_ROTATION_MODEL (2026-10-05) changed the physics of every case flown with
    the rotation on; a manifest without it predates the change. A figure or table
    built from a partly re-flown matrix would otherwise compare the two models
    without saying so. Rotation-off rows are exempt: the change does not touch them.
    """
    models = {}
    for name, case in cases.items():
        if not case.row.get("earth_rotation"):
            continue
        model = (case.manifest.get("config") or {}).get("EARTH_ROTATION_MODEL")
        models.setdefault(model, []).append(name)
    if len(models) > 1:
        raise ValueError(
            "rotation-on cases flown under different EARTH_ROTATION_MODEL values "
            "(None = before 2026-10-05): %s. Re-fly the stale ones, or draw from a "
            "root that holds one model only (--root)."
            % "; ".join("%s: %s" % (m, ", ".join(sorted(n))) for m, n in models.items()))


def missing_from(cases, *names):
    """Which of *names* are absent, so a figure can skip itself cleanly.

    This is what lets the suite run against a partially complete batch instead
    of failing on the first case that has not been flown yet.
    """
    return [n for n in names if n not in cases]


# The physics switches that decide whether two cases were flown under the same
# equations of motion, and how to name each one when it differs.
_FORCE_MODEL_KEYS = (
    ("pseudo_forces_flown", "without the rotating-frame pseudo-forces",
     "with the rotating-frame pseudo-forces"),
    ("include_drag", "without drag", "with drag"),
)


def force_model_note(ref, others):
    """How *ref*'s force model differs from the cases it is drawn against.

    The indirect-PMP architecture integrates costate equations derived for the
    drag-free, non-rotating equations of motion. Three force models of its
    Stage 2 exist in the archives (manifest ``config.INDIRECT_PMP_STAGE2_FRAME``):
    until 2026-09-13 the whole ascent flew without the pseudo-forces
    (``pseudo_forces_flown`` False); until 2026-09-16 Stage 2 was propagated in
    the inertial frame, where no such term exists but where the frame transform
    credits the full, unprojected omega*r*cos(lat) along-track -- about 121 m/s
    more than the azimuth-resolved pseudo-force terms of the time credited at the
    baseline site; since 2026-09-16 ("rotating_pseudo_forces", the default) Stage 2
    carries the same terms as every ``pso_coast`` case, with costate equations that
    omit their sub-percent partials, and the force models agree. Since
    EARTH_ROTATION_MODEL = "launch_site" (2026-10-05) the terms credit the same
    launch-site speed as the transform, so an inertial Stage 2 flown under it is
    no longer a different force model either.

    Where they do not, the PMP result is a comparison and not an optimality
    bound: some of any gap between it and a closed-loop law is the force model
    rather than the guidance. Every figure that draws the line asks for this
    note and prints it, so the distinction cannot be lost between the figure and
    its caption.

    Returns a phrase naming the difference, or None when the models agree (in
    which case the line really is a like-for-like reference and says so).
    """
    peers = [c for c in others if c is not None and c is not ref]
    if ref is None or not peers:
        return None

    differences = []
    for key, when_false, when_true in _FORCE_MODEL_KEYS:
        ref_value = ref.row.get(key)
        peer_values = {c.row.get(key) for c in peers}
        # Only a switch every peer agrees on can be said to differ from the
        # reference; a mixed field is the figure's own business to explain.
        if ref_value is None or len(peer_values) != 1:
            continue
        peer_value = peer_values.pop()
        if peer_value is None or bool(ref_value) == bool(peer_value):
            continue
        differences.append(when_true if ref_value else when_false)

    # The flag alone no longer separates the PMP from a pso_coast case once its
    # Stage 1 carries the terms; the form its Stage 2 was flown in still can.
    # The manifest records it; an archive without one predates the setting and
    # was flown pseudo-force-free, which the loop above already reports.
    config = ref.manifest.get("config") or {}
    stage2_frame = config.get("INDIRECT_PMP_STAGE2_FRAME")
    if (ref.row.get("architecture") == "indirect_pmp"
            and ref.row.get("pseudo_forces_flown")
            and stage2_frame == "inertial"
            and config.get("EARTH_ROTATION_MODEL") is None
            and all(c.row.get("architecture") != "indirect_pmp" for c in peers)):
        differences.append("with Stage 2 propagated in the inertial frame "
                           "(unprojected rotation credit)")

    if not differences:
        return None
    return "flown " + " and ".join(differences)
