# Plan: 750×1500 refined PMP references for every environment (2026-10-04)

**Superseded 2026-10-07 (user decision):** the references are the same seed-3 swarms refined with the coast bound `PSO_UB[3]` = 2000 s, first pass only:
- pmp_baseline 22 649.0 kg, pmp_vacuum 24 536.4 kg, pmp_norot 19 345.3 kg.
- Archives: `Tese/src/Output/pmp_polish_launchsite_20261006/<case>/seed_3_c2000/`.
- Re-flown as matrix rows into `Tese/src/Output/results_matrix_chapter6_20261007/`.
- Not yet adopted into `build_matrix` or the tracked `pmp_reference.npz`.

**Status: COMPLETE 2026-10-05.** All three references come from 750×1500 seed-3 swarms, refined at half step:
- `pmp_baseline`: 22 261.2 kg.
- `pmp_vacuum`: 23 928.9 kg. Code 9a52002, row c9a7ea3.
- `pmp_norot`: 19 384.5 kg. Committed by another session: b99acce, 724707a.
- Thesis: 8198fa1.
- Seed 3 was the user's rule (2026-10-05) "so it is the same as the baseline". The best-of-five plan below was not applied.

History below. Phase 1 was done on 2026-10-04 (0ff7c8a); 276 tests passed then.

**Smoke** (`Output/results_matrix_smoke_pmp_20261004`, untracked):
- Both cases dispatch.
- `--swarm-extremal` is refused on `pmp_norot`.
- `pmp_swarm_polish.py` re-flies each smoke archive to the harness's J′ exactly (151.981494, 139.647590).
- It refuses the rotating `pmp_budget_750x1500/pmp_baseline/seed_3` start under `pmp_norot`.
- `pmp_reference.npz` sha256 unchanged (ae9d1f86…).

D1 and D2 decided by the user on 2026-10-04:
- **D1:** build `pmp_norot` only.
- **D2:** five seeds per environment.

**Scope:** re-do `pmp_vacuum` and build `pmp_norot`. That is 10 swarms.

## Goal

Every Chapter 6 reference is produced by the recipe that produced `pmp_baseline`:

1. a 750×1500 swarm from five seeds;
2. each seed refined with `dev-notes/pmp_swarm_polish.py`, followed by a half-step pass from the best;
3. the best extremal kept.

The non-rotating environment gets a reference, which closes T17 for `gt_norot`. `gt_sea_level_engine` stays without one by the user's choice, and keeps its local treatment in `tab:gt_results`.

## Where things stand

| Reference | Environment | Swarm | Refined | Propellant left | Action |
|---|---|---|---|---|---|
| `pmp_baseline` | baseline | 750×1500, seeds 1/2/3/4/42; seed 3 kept | yes, half-step | 22 261.2 kg | none; already meets the recipe |
| `pmp_vacuum` | no atmosphere | 250×1000, seeds 1/2/3/4/42; seed 3 kept | yes, half-step | 23 952.1 kg | re-do at 750×1500 |
| none | non-rotating (`gt_norot`, reported) | none | none | none | new `pmp_norot` |
| none | no atmosphere, non-rotating (`peg_vacuum_norot`, archived and not reported since 2026-09-29) | none | none | none | not built (D1) |
| none | sea-level engine (`gt_sea_level_engine`, reported; also in T17) | none | none | none | not built (D1) |

`pmp_baseline` stays as it is, so `pmp_reference.npz` does not change. No segmented or reference-tracking case needs re-flying.

## Probe findings (scratchpad only; nothing written)

`norot_pmp_probe2.py` flew the two stored extremals once in each environment, one process per environment.

- **The code path runs.**
  - With the rotation off, `indirect_pmp` flies with the pseudo-forces inert in both stages and raises nothing.
  - The target is √(μ/r) = 7612.68 m/s.
  - A flight takes 0.03–0.05 s, the same as with the rotation on. The sea-level engine flies too.
  - As a sanity check, both stored extremals still hit their own targets to the metre.
- **The stored extremals do not work as starting points.**
  - With the rotation off, the baseline extremal's kick stages 9 km lower and 4° flatter: 55.8 km / 9.9° against 64.9 km / 13.9°. Stage 2 then hits the ground.
  - The sea-level engine gives 51.5 km / 9.6°, with the same result.
  - So the swarms cannot be skipped: each environment needs its own kick.
- **Expected difference with the rotation off.**
  - The target becomes a true circular orbit, so the last burn cannot vanish.
  - The γ_p continuation should therefore stop at an interior maximum ("past the optimum"). Every rotating extremal instead ends at the D3 → 0 wall.
  - These references also carry no coast-to-apoapsis margin, the T18 effect. Their budget residual should close near 0, as `gt_norot`'s does (−0.03 m/s against −125 m/s on the rotating Earth).

## Decisions

- **D1. DECIDED: `pmp_norot` only.** `pmp_sea_level_engine` and `pmp_vacuum_norot` are not built.
- **D2. DECIDED: five seeds** (1/2/3/4/42) per environment. This is how the baseline reference was chosen: all five refined, the best kept.
- **D3. Adoption gate, decided at Phase 4.** If the 750×1500 `pmp_vacuum` refines below the current 23 952.1 kg, I stop and show you both before replacing anything.

## Phase 1: code, before launch

Commit on your word, so that every manifest records a clean commit.

- **C1. `build_matrix`: the new §6.4 case `pmp_norot`.** It is `GUIDANCE_MODE = "indirect_pmp"` plus `gt_norot`'s three rotation switches. It gets no `extremal` until Phase 5, so `--case` swarms it.
- **C2. `run_results_matrix.py --swarm-extremal`.** A case with a stored extremal runs its swarm instead of replaying the extremal. `pmp_vacuum` needs this: the harness has replayed it since 2026-09-25.
- **C3. `pmp_swarm_polish.py`.**
  - `--case` accepts the matrix's `indirect_pmp` cases instead of the hard-coded pair.
  - `load_start` refuses a start whose drag, rotation or engine settings differ from the case's. Today it checks `INCLUDE_DRAG` only, so a rotating swarm would be polished under a non-rotating configuration without any warning.
- **C4. Tests (`test_results_matrix_config.py`).**
  - The case count.
  - `pmp_norot`'s `include_drag`/`earth_rotation`/`thrust_1_mode` equal `gt_norot`'s. These are the keys `sec65_losses` matches on.
  - `--swarm-extremal` drops the extremal.
- **C5. Docs.** `CLAUDE.md` and `worktree.md`: the case count and the §6.4 description.
- **Smoke.** Run `pmp_norot`, and `pmp_vacuum` with `--swarm-extremal`, both with `--smoke --out Output/results_matrix_smoke` (8×4 swarm, seconds).

## Phase 2: swarms (about 14–18 h)

One detached process per (case, seed), seeds {1, 2, 3, 4, 42}, from the repository root:

```
python -u Tese/src/run_results_matrix.py --case <case> --budget 750,1500 --set PSO_SEED=<s> --out Output/pmp_budget_750x1500/<case>/seed_<s> [--swarm-extremal]
```

- **Layout.** It mirrors the baseline's `Output/pmp_budget_750x1500/pmp_baseline/seed_N/`. Every root is untracked, and `indirect_pmp` never reads `pmp_reference.npz`, so the tree stays clean throughout.
- **Load.**
  - 10 processes: `pmp_vacuum` with `--swarm-extremal`, and `pmp_norot`.
  - The CPU is an i9-14900HX: 8 performance and 16 efficiency cores, 32 threads.
- **Runtime.** The baseline's five-process batch took 12.6 h. With 10 processes, some of them on efficiency cores, expect about 14–18 h for the slowest.
- **Keep-awake.** Use the same helper as on 2026-09-25.

## Phase 3: refinement (about 1–2 h)

One process per case, the two in parallel:

1. **First pass:** all five seeds at step 0.0005, as `b750`, archived under `Output/pmp_polish_750x1500/<case>/`.
2. **Retries:** a start that stops at the Levenberg–Marquardt cap is re-run with a larger `--max-nfev`, as `b750nfev`.
3. **Half-step pass:** step 0.00025 from the best seed, as `b750half`.

## Phase 4: gate (show, then wait)

- **What I show for each case:**
  - the swarm best and the refined propellant per seed;
  - γ_p, the coast, D3 and the orbit error;
  - how each continuation stopped.
- **Overlays:** drawn with `run_archive.py compare`, against `gt_*` and the old `pmp_vacuum`.
- **I stop and ask if any of these happens:**
  - a best γ_p lies within 0.005 rad of the bounds [1.50, 1.57];
  - no seed of a case refines;
  - D3 triggers.

## Phase 5: adoption (after your go)

- **A1. The extremals go into `build_matrix`.** Each records its x vector, seed, `[750, 1500]` budget and search cost (a `PMP_*_SEARCH` dict). `tables.SWARM_POINT_KG` gets the matching entries.
- **A2. Re-fly the §6.4 rows.**
  - About 1 s each, into an untracked root.
  - Copy them in, rebuild `results_matrix.csv` and force-add the rows, following the clean-hash procedure.
  - Check the sha256 of `pmp_reference.npz` is unchanged.
- **A3. Figures and tables.**
  - `sec65_losses._reference_for` matches on (drag, rotation). The "no reference" exception is kept only for the constant-nozzle case.
  - Update `REPORTED_CASES`, the rows of `tables.reference_results` and the `_style` labels.
  - Re-render `make_all` and `tables` into the thesis repo.
- **A4. Thesis, wording shown first.**
  - The Ch. 5 case table.
  - Ch. 5's optimal-reference paragraph: one budget for all, best of five refined seeds. This also settles T9's seed sentence.
  - The `tab:gt_results` caption, which says the non-rotating and constant-nozzle cases have no reference; only the constant-nozzle case is left without one.
  - The §6.4 text.
  - T17 closed for `gt_norot`; `gt_sea_level_engine` stays as decided.
- **A5.** Update `thesis-flags.md`, the memory, and these docs.

## Risks

- **`pmp_vacuum` may move.** It is the reference for `peg_vacuum` and for the vacuum curves of the §6.3 and §6.5 figures, so those figures move with it.
- **A refinement may stop at "solve failed".** The 250×1000 baseline's half-step pass did. The fallback is the next-best seed.
- **Page budget.** Chapter 6 is over it. `pmp_norot` costs one table row; no new figure is planned.
