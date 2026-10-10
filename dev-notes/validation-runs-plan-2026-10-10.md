# Validation runs: plan (2026-10-10, not run)

Follow-up to thesis §6.8 "Results Validation" (`sec:validation`), which compares the Chapter 6
set (`Output/results_matrix_chapter6_20261007/`) with the SpaceX user's guide sample LEO
timeline and three expendable 2017 flights (`Tese/src/Reference_Data/falcon9_webcast/`).
Figure and tables: `Plots/results_figures/validation_falcon9.py`,
`tables.validation_timeline` / `validation_states`. Both runs below are user requests to be
planned only. Nothing launched.

## Run 1. PMP reference refined with a 2500 s coast bound

**Aim.** See whether the reference's coast moves towards the guide's 2 572 s.

**Recipe.** The chosen 2026-10-07 one, with only the bound changed:
- the seed-3 750×1500 swarm (`Output/pmp_budget_750x1500/pmp_baseline/seed_3/`), refined with
  `dev-notes/pmp_swarm_polish.py --case pmp_baseline`;
- `PSO_UB[3]` = 2500 s;
- first pass only;
- written into a new root (`Output/pmp_polish_c2500_<date>/`).

**Cost.** One refinement, about 1 h; no swarm.

**Expectation, before running.** The reference does not sit on its bound:
- with 2 000 s it coasts 1 823 s;
- with 3 500 s (2026-10-06) it coasted 1 741 s;
- so a 2 500 s bound will most likely return the 1 823 s extremal or the 1 741 s one, not a
  longer coast.

The guide's coast is for an unstated orbit and payload, so it is not a target for this model.

**The case that is on its bound is the gravity turn** (`gt_baseline`, 2 000 s). Re-refining it at
2 500 s, with the same recipe and offline, is the run that can actually change a coast. Offer it
alongside.

**After the run.** If adopted, re-fly the dependent rows the way the 2026-10-07 set was built,
then re-run `make_all` and `tables`.

## Run 2. Payload-matched configuration (6.8 t)

**Aim.** Separate the payload's share of the speed and max-Q gap in `tab:validation_states`
from the throttle-down and vehicle-version shares.

**Configuration.**
- `M_PAYLOAD` = 6 800 kg (about Intelsat 35e).
- Mission: the 500 km baseline. Optionally a second variant with a 170 km target, closer to
  the flights' first-burn altitude.
- Cases: gt_baseline (coast-parameter swarm, 250×1000, then the refinement recipe) and the PMP
  reference.

**Hazards to settle first.**
- `M_PAYLOAD` lives in `Auxiliary/rocket_specs.py`, and constants must not be changed at run
  time (CLAUDE.md). This needs an edited copy in a frozen worktree, or a harness override that
  every module sees.
- The vehicle is part of the PMP reference cache key. A payload run rebuilds and **overwrites the
  tracked `Output/pmp_reference.npz`** unless the cache path is redirected. Fly it from a
  separate worktree, as in `long-run-launch-procedure`.
- The Stage-2 propellant can run out with 6.8 t. Check that the 500 km case still inserts before
  committing hours to it.

**Cost.**
- gt swarm: 2.6–11 h at 250×1000, plus its refinement.
- PMP: one 750×1500 swarm plus refinement, about 13–22 h. A 250×1000 single seed is enough for
  a validation figure, about 2–4 h.

**Output.** Add the rows to `validation_falcon9.SIM_CASES` (or a second root) and redraw the
figure and tables.

## Not planned: throttle-down

Throttle-down is listed as Future Work in Chapter 7 (2026-10-10). It needs a thrust schedule in
both dispatchers, so it is out of scope here.
