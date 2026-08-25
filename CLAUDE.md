# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A thesis project: a 3-DOF two-stage launch-vehicle ascent simulator + trajectory optimizer
(Python), plus the LaTeX/Markdown thesis documents that describe it. The point of the code is
**fair comparison** — many guidance laws and many optimization strategies flown on the same
vehicle, same mission, same physics.

- Simulator source: `Tese/src/`
- Design docs and superseded drafts: `Tese/` (see "Documentation map" below)
- **The LaTeX thesis itself is in a second repository** — see "The thesis is a separate repo"
- `dev-notes/` — session handoffs and scratch scripts, **not** part of the simulator and often stale

## The thesis is a separate repo

The LaTeX thesis is **not** in this repository. It lives at
`C:\Users\eduar\Desktop\Tese\Thesis_Overleaf`, its own git repo whose remote is the Overleaf git
bridge. That path is **outside this working directory**, so a session cannot read it until the
directory is added (`/add-dir` in-session, or `claude --add-dir <path>` at launch).

Chapter order, and the file behind each — `Thesis.tex` is the master that `\input`s them:

1 Introduction · 2 Ascent Flight Mechanics (`Thesis_Ascent_Background.tex`) · 3 Trajectory
Optimization (`Thesis_Optimization_Background.tex`) · 4 Ascent Guidance Laws
(`Thesis_Guidance.tex`) · 5 Implementation (**`Thesis_Methodology.tex`** — the filename no longer
matches the chapter it renders) · 6 Results · 7 Conclusions. Bibliography:
`Thesis_Bibliography_DB.bib`.

- `Tese/Ongoing_Chapters/` in *this* repo is superseded — do not edit it for thesis work.
- The user also edits in the Overleaf web UI, which produces "Update on Overleaf." commits, so
  **`git fetch` and read the incoming diff before editing or rebasing.**
- **Never handle the user's Overleaf password or git token.** The token is already embedded in the
  remote URL, so git authenticates without it being typed; mask it out of any command output with
  `sed -E 's#//[^@]*@#//***@#'`.
- **No LaTeX toolchain exists on this machine** — Overleaf is the only place the document
  compiles. Verify edits with `\ref` / `\label` / `\cite` sweeps instead of a build.

## Commands

Run from the **repository root** (see the cwd gotcha below):

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe Tese/src/main.py
```

`main.py` inserts `Tese/src` on `sys.path` itself, so `python main.py` from inside `Tese/src`
also works — but then `SAVE_PLOTS_DIR` resolves relative to that cwd.

Environment: `pygmo` is **not** on the default Python; use the `pygmo-env` conda env above for any
PSO path (`pso_coast`, `direct`, `indirect_pmp`, segmented). Only `COAST_METHOD="apogee_check"`
runs without PyGMO — plain `python` is fine there. There is no requirements file; deps are
`numpy`, `scipy`, `matplotlib` (+ `pygmo` for PSO).

Dependency/import sanity check:

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/check_readiness.py
```

Tests — `Tese/src/tests/` holds `test_apollo_tgo.py` and `test_losses.py` (31 tests). pytest is
installed in `pygmo-env` only:

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe -m pytest Tese/src/tests/ -q
```

Single test: append the file and `::TestApolloTgo::test_nominal_case`.

**Every run archives itself.** `main.py` writes `<arch>_<law>_<date>_<time>.npz` + `.json` +
`.manifest.json` to `Tese/src/Output/runs/` regardless of `PLOT_SUITE` and `SAVE_PLOTS` (switch:
`ARCHIVE_RUNS`, default `True`). The manifest holds the whole of `simulation_parameters.py`, the
vehicle constants and the git commit, so a run stays interpretable months later. Runs accumulate;
nothing is overwritten. Browse and compare them with:

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe Tese/src/run_archive.py list
```

`show <id> [--config]`, `compare <id> <id> [...]` (overlays + a table of the settings that actually
differ), `replay <id>` (redraws the legacy 20-plot suite from the archive). Ids match by unique
prefix; `<dir>::<stem>` reaches into any directory, so a results-matrix case and a hand-flown run go
into one comparison. Figures land in `Tese/src/Output_Plots/<run_id>/`, never beside the data.
See `Tese/worktree.md` §2.12a (the two roots) and §2.12b (the archive format).

Multi-mode batch scripts (older, cover only the four classical laws):
`Tese/src/all_guidance_plotting/run_all_guidance_methods.py`,
`Tese/src/guidance_comparison/compare_guidance_methods.py`.

The Chapter 6 results set is produced by `Tese/src/run_results_matrix.py` — 23 cases, one frozen
baseline with one factor changed at a time, each case in its **own subprocess** so no module
global can leak between them. Prove every case dispatches before committing a night to it:

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe Tese/src/run_results_matrix.py --smoke
```

Then drop `--smoke` for the real thing (~13-14 h). `--case <name>` runs one case in-process with
solver output on screen; `--only <substring>` filters.

**Runtimes are long.** A production PSO solve is tens of minutes (documented: coast PSO ~31 min at
100×250; the indirect-PMP reference build ~1 h at 250×500). Drop `PSO_*_N_PARTICLES` /
`PSO_*_MAX_GENERATIONS` for smoke tests, but note that a reduced budget changes results — several
"this combination fails" conclusions in the docs turned out to be under-convergence.

## Configuration is the interface

`Tese/src/Input_File/simulation_parameters.py` is a single hand-edited control panel (~650 lines,
numbered sections with a table of contents). Changing what the simulator does normally means
editing that file, not the code. Vehicle constants live in `Tese/src/Auxiliary/rocket_specs.py`
(one flat Falcon-9-like two-stage vehicle — no vehicle registry, no stage-count switch, Earth-only).

**Read `Tese/worktree.md` before touching the config.** It is the authoritative, up-to-date map of
every setting, which combinations are valid, which raise, and which are *silent no-ops*. Keep it
updated when config semantics change.

Dispatch order (from `main.py`) — each level overrides the ones below it:

1. `MULTI_GUIDANCE_ENABLED=True` → segmented multi-law world; ignores `GUIDANCE_MODE` and
   `COAST_METHOD` entirely.
2. `GUIDANCE_MODE="indirect_pmp"` → its own 7-variable PSO (costates + timing + kick); ignores
   `COAST_METHOD`, `KICK_PROFILE_MODE`, `RUN_FAST`.
3. `COAST_METHOD` ∈ {`pso_coast`, `direct`, `apogee_check`} → picks the solver for all other laws.

Nine guidance laws: `gravity_turn`, `linear_tangent`, `bilinear_tangent`, `apollo`, `cpr`, `peg`,
`peg_new`, `exp_shooting`, `indirect_pmp`. Not all pair with all coast methods — see the
compatibility matrix in `Tese/worktree.md` §3 (e.g. `apollo`+`apogee_check` raises; under `direct`
only `apollo`/`peg`/`peg_new` reach orbit, the rest converge to a genuinely suborbital optimum).

## Architecture

State vector is `[s, r, v, γ, m]` — downrange, geocentric radius, speed, flight-path angle, mass —
with latitude appended as a 6th element when `ENABLE_EARTH_ROTATION`. Every guidance law's single
output is the angle of attack `α = θ − γ`. Trajectory arrays are `data[row, step]` with those rows.

```
main.py                  branch per dispatch level, all printing/reporting, then the plot suite
Simulation/
  rocket_ascent.py       the physics: EOM, atmosphere/thrust/staging events, legacy run() and
                         run_stage1(); ~2500 lines of module-global state
  solver.py              apogee_check brute-force kick-angle search
  pso_coast_solver.py    4-var PSO thrust→coast→thrust, direct insertion; owns GuidanceState
  direct_pso_solver.py   2-var PSO, single continuous burn — imports GuidanceState & the Stage-2
                         ODE from pso_coast_solver
  indirect_pso_solver.py 7-var PSO over PMP costates; propagates [s,r,v,γ,m,λ_r,λ_v,λ_γ]
  segmented_guidance_solver.py  multi-law schedule; reuses ra.run_stage1 + pso_coast Stage-2
  segment_reference.py   builds/caches the indirect-PMP reference that supplies segment waypoints
Guidance/                one module per law, pure functions returning α (or coefficients)
Auxiliary/               constants, atmosphere, gravity, earth_rotation, rocket_specs
Plots/new_metrics/       one file per metric; new_plot_runner.py runs the ~20-plot suite
Plots/results_figures/   the Chapter 6 figure set; _data.Case is THE loader for any archive
Archive/                 run_record (row + channels + manifest, shared with the harness),
                         store (naming, writing, finding), compare (generic N-way overlay
                         + manifest diff), cli; entry point Tese/src/run_archive.py
```

### Two parallel guidance dispatchers — the thing to know

The same guidance laws are driven through **two independent implementations**, and a change to one
does not affect the other:

- **Legacy path** (`apogee_check`): `rocket_ascent.rocket_dynamics()` dispatches on
  `sim_params.GUIDANCE_MODE` and keeps all guidance state (PEG coefficients, Apollo freeze flags,
  CPR pitch rate, t_go history …) in **module globals**, reset by big blocks at the top of `run()`
  and `run_stage1()`.
- **PSO path** (`pso_coast`, `direct`, segmented): `pso_coast_solver._compute_alpha_stage2()` with
  a per-trajectory `GuidanceState` object, plus `restart_for_new_burn()` at each arc boundary.

Consequence: the same `GUIDANCE_MODE` can fly differently depending on `COAST_METHOD` (documented
example: `cpr`'s initial pitch is hardcoded vertical in the legacy path but the current γ in the
PSO path). When fixing a guidance bug, check whether both dispatchers need the fix.

All three PSO solvers share Stage 1 via `ra.run_stage1()` (always the instantaneous γ-jump kick);
only the legacy `run()` honours `KICK_PROFILE_MODE`.

**The force model is now shared, and that took a fix.** `diff_eom_base` is documented as the EOM
*"WITHOUT Earth rotation"* — the rotating-frame pseudo-forces were added one layer up, in
`rocket_dynamics`. The PSO Stage-2 ODEs call the kernel directly (they cannot use the 500-line
`rocket_dynamics` inside a swarm inner loop), so for a long time they silently flew a non-rotating
model while Stage 1 did not. Coriolis and centrifugal are now applied in `_stage2_ode_guidance` too,
and are all-or-nothing per architecture — carried for the whole ascent everywhere except
`indirect_pmp`, whose costate equations assume the drag-free EOM. The switch is
`ra.set_pseudo_forces_for_run()`, set explicitly by the driving solver and **never inferred from
config** (building the segmented PMP reference runs the *indirect* solver, so a config-derived gate
would mislabel it).

**When adding a new term to the equations of motion, put it in `diff_eom_base`, not in
`rocket_dynamics`** — otherwise it silently misses every population-based architecture, which is
exactly how the pseudo-force gap arose. There is no test that would catch it.

### Segmented (multi-law) guidance

`segmented_guidance_solver` flies an ordered `GUIDANCE_SEGMENTS` schedule of `(law, altitude)`.
Two mechanisms make it work: `ra._SEGMENTED_ALPHA_HOOK` (a callable installed into
`rocket_dynamics` so a law can steer *during Stage 1*, sub-MECO) and a **planned-deadline t_go**
(`deadline − t`, deadlines from the PMP reference) instead of the rocket-equation estimate that
collapses at the stage boundary. Non-final segments aim at indirect-PMP `(alt, v, γ)` waypoints;
the final segment inserts to orbit. The PMP reference is cached to
`Tese/src/Output/pmp_reference.npz`, keyed by target orbit + vehicle + reference-PSO budget
(path is resolved against the project root, so it is cwd-independent).

## Invariants and gotchas

- **Module-global state in `rocket_ascent.py` is the main hazard.** A PSO run evaluates thousands
  of trajectories in one process, so anything cached in a global must be reset per trajectory
  (`reset_stage1_ramp_state()` exists precisely because the Isp/thrust ramp leaked across particle
  evaluations). The reset blocks in `run()` and `run_stage1()` must stay in sync; adding a new
  global means adding it to both.
- **Feature flags exist to keep old paths byte-identical**, and comments say so explicitly:
  `_IN_PSO_STAGE1` (suppresses legacy CPR Stage-1 behaviour that otherwise crashes `brentq` event
  bracketing), `_stage1_kick_handled_by_gamma_jump` (prevents a double kick),
  `_SEGMENTED_ALPHA_HOOK` (`None` on every non-segmented run). Preserve that property when editing.
- **Two output roots, and neither depends on cwd.** `Tese/src/Output/` is **data only** — run
  archives, the results-matrix batch, `pmp_reference.npz`. `Tese/src/Output_Plots/` is **every
  figure** — `<run_id>/` per run, plus `comparisons/` and `chapter_figures/`. Both are gitignored
  (`pmp_reference.npz` is force-added) and both resolve relative to `Tese/src`, so running from
  inside `Tese/src` is no longer a trap. It used to be: `SAVE_PLOTS_DIR` was cwd-relative and
  produced a nested `Tese/src/Tese/src/Output/plots` tree that reached git before anyone noticed.
- **UTF-8 stdout is forced in `main.py`** because prints use Greek letters and `°`. Standalone
  scripts that print those need `PYTHONIOENCODING=utf-8` on this Windows console.
- **`main.py` is mostly reporting.** Most of its ~1100 lines are per-branch result printing; new
  solver branches follow the same shape and reuse the shared `_print_*` helpers at the top.
- `dev-notes/handoff.md` describes a configuration that no longer exists (e.g.
  `DIRECT_OPTIMIZATION_MODE`, since removed). Trust `Tese/worktree.md` over `dev-notes/`.

## Documentation map

- `Tese/worktree.md` — **start here for configuration**: decision tree, full parameter catalog,
  compatibility matrix, gotchas, and the recorded results of validation sweeps.
- `Tese/README.md` — short user-facing overview (note: its `COAST_METHOD="direct"` description
  still mentions a brute-force sub-mode that no longer exists).
- `Tese/Code_Overview/code_overview_detailed.md` — conceptual, no-source explanation of the model,
  useful for thesis-facing prose.
- `Tese/Project_Description/` — guidance-mode reference, optimization-process explanation,
  Earth-rotation notes, EOM/kinematics LaTeX.
- `Tese/Ongoing_Chapters/`, `Tese/Thesis_Outline/`, `Tese/legacy_*` — **all superseded** by the
  Overleaf repo above; kept only for history.
