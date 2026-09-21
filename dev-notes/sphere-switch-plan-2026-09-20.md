# Plan: the costate sphere switch in the indirect-PMP swarm

Written 2026-09-20. Backlog item 2 of `memory/backlog-on-hold-2026-09-19.md`, planned but **not
started**. Supersedes nothing; the λ_r bound-narrowing (Experiment 2 of handoff 19b) stays on hold
and this is the alternative the user preferred to it.

## Motivation, measured

All ten converged `pmp_baseline` solutions (five seeds at 250x1000, five at 750x1500) mapped onto
the unit sphere `u = λ_r/‖λ‖`, `φ = atan2(λ_γ, λ_v)`:

| | u = λ_r/‖λ‖ | latitude | φ |
|---|---|---|---|
| range over 10 runs | −1.86e-3 … −4.67e-4 | **−0.107° … −0.027°** | −165° … +157° (unconstrained) |

Every solution ever found sits in a **0.08°-wide band about the equator**, always negative. φ is
free; only u is a needle.

For uniformly distributed directions that band is 0.19 % of the sphere, so at the initial
population:

- 250 particles → **0.47** expected particles in the band
- 750 particles → **1.40**

This reframes the 750x1500 budget result (`memory/pmp-budget-750x1500-2026-09-20.md`): tripling the
population tripled the in-band count from half a particle to one and a half, which is exactly the
"more scatter, one lucky seed" behaviour measured. The swarm is not failing to converge, it is
failing to sample.

A warp on u fixes it. With `u = sign(t)·|t|^p` for t uniform on [−1, 1]:

| p | fraction in band | particles at 750 |
|---|---|---|
| 1 (plain sphere) | 0.19 % | 1.4 |
| 2 | 4.3 % | 32 |
| **3** | **12.3 %** | **92** |
| 4 | 20.8 % | 156 |

**It remains a bijection onto the whole sphere.** Nothing is excluded, only resampled — this is a
reparameterisation, not a restriction, which is the user's stated requirement (they rejected
narrowing the λ_r bounds for fear of losing optima).

## Design decision that shapes everything

**The sphere is internal to the search only.** `run_indirect_trajectory` keeps its 7-argument
Cartesian signature; every archive keeps the canonical 7-long Cartesian `decision_vector`. The
swarm searches 6 variables and converts before anything else sees the point.

Unchanged as a result: `dev-notes/pmp_swarm_polish.py --start <npz>`,
`segment_reference.cache_from_archive`, `run_archive compare/replay`, the results-matrix row
builder, and every existing test holding a hard-coded x.

The solver already calls `_normalize_costates` (`Tese/src/Simulation/indirect_pso_solver.py:91`), so
the costate magnitude is searched and then discarded. That redundant radial dimension is what the
switch deletes (7 → 6).

## Phase 1 — the map and its tests (no behaviour change)

New in `indirect_pso_solver.py`:

```
sphere_to_costates(u, phi, p)  -> (λ_r, λ_v, λ_γ)   # λ_r = w(u), rest = √(1−w²)·(cos φ, sin φ)
costates_to_sphere(λ_r, λ_v, λ_γ, p) -> (u, phi)     # exact inverse after normalisation
sphere_x_to_cartesian_x(x6, p) -> x7
```

Tests — `Tese/src/tests/test_pmp_sphere.py`, no trajectories, fast:

- round-trip on all ten archived x vectors, agreeing to 1e-12 after normalisation;
- round-trip on random directions, including near-pole cases (φ is undefined at u = ±1; measure
  zero, but assert it does not raise);
- the warp's in-band fraction matches the closed form for p = 1, 2, 3;
- `p = 1` is the identity warp.

## Phase 2 — remove the positional-index hazard BEFORE the switch exists

The live trap. `indirect_pso_solver.py:730,732` read `PSO_UB[3]` / `PSO_LB[3]` as the **coast**
bounds for the one-sided transversality condition. In sphere form the coast is index **2**, so a
6-long bounds vector silently turns the coast bound into a γ_p bound. Same positional pattern in
`dev-notes/pmp_swarm_polish.py:73-74` and `Tese/src/tests/test_pmp_transversality.py:65,72`.

Add named accessors — `coast_bounds()`, `gamma_p_bounds()` — that always return the physical bounds
whatever parameterisation is active, and route every call site through them.

**Land this as its own commit**, with the existing 182 tests green. It is a pure refactor and must
not be merged with Phase 3.

## Phase 3 — the switch

`Tese/src/Input_File/simulation_parameters.py`, beside the other `INDIRECT_PMP_*` settings
(lines 553 / 569 / 581):

```python
INDIRECT_PMP_COSTATE_PARAM = "cartesian"   # or "sphere"
INDIRECT_PMP_COSTATE_U_EXP = 1.0           # warp exponent p; 1.0 = plain sphere
PSO_SPHERE_LB = [-1.0, -pi,    0.0,   0.0,   0.0, 1.50]
PSO_SPHERE_UB = [ 1.0,  pi, 2000.0, 100.0, 100.0, 1.57]
```

`PSO_LB`/`PSO_UB` stay exactly as they are, so the default run's reference cache key is unchanged.

`IndirectTPBVPProblem.fitness` and `get_bounds` consult the switch; `run_pso_optimization` converts
the champion back to the 7-long Cartesian form before returning, so its documented return contract
holds.

## Phase 4 — provenance

- `Tese/src/Simulation/segment_reference.py:156-157,248` — add `INDIRECT_PMP_COSTATE_PARAM`,
  `INDIRECT_PMP_COSTATE_U_EXP` and `PSO_SPHERE_LB/UB` to the cache key **and** to
  `_ARCHIVE_MUST_MATCH`. Without this a sphere run can silently reuse a Cartesian reference.
- Archive `decision_vector_sphere` alongside the canonical vector. The manifest already captures
  the whole config, so the parameterisation is recorded automatically.
- `Tese/worktree.md` §2.10 (settings table) and the compatibility notes.

## Phase 5 — proof the default is untouched

The project's established gate. With the switch off, re-fly `pmp_baseline` seed 42 and reproduce the
archived **J′ = 0.7794598056967313** and 20 561.5 kg exactly. Plus a new equivalence test: a sphere x
mapped to Cartesian produces a bit-identical result dict to flying the Cartesian x directly.

## Phase 6 — the A/B that settles it

5 seeds (1, 2, 3, 4, 42) of `pmp_baseline` at **250x1000** with `sphere` + `p = 3`, against the
existing 250x1000 Cartesian study. Same seeds, same budget, one factor changed.

**~3.7 h per seed, ~4 h wall five-wide** — a third of the 750x1500 run's cost.

Criteria fixed in advance, against Cartesian 250x1000 (mean 20 388.2, sd 144.8, best 20 525.5) and
gt_baseline's 20 687.5 kg:

- **worked:** mean up *and* sd down — the sampling fix showing as reliability;
- **reliably beats gt:** median > 20 687.5;
- **null:** overlapping distributions → the costate band was not the binding constraint, and the
  coast / γ_p ridges are.

## What this will not do

The sphere fixes **costate** sampling only. It does nothing about the coast ridge (±76 s feasible
inside a 2000 s box) or walking the γ_p family, where the ~870 kg beyond the best 750x1500 seed
sits. Expect lower variance and a better mean at equal budget; do not expect the polish's 22.2 t.

Honest caveat for the write-up: the dimension drops 7 → 6, so PSO's velocity dynamics differ
slightly. The A/B is not a perfectly controlled single-factor change.

## Sequencing and cost

Phases 1–2 are safe and independently committable. Phase 3–4 is the real change. Phase 5 gates
Phase 6. Implementation is roughly half a day plus the 4 h run.
