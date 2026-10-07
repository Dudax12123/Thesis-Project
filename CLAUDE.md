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

Tests — `Tese/src/tests/` holds twenty-two files (273 tests as of 2026-10-01). pytest is installed
in `pygmo-env` only:

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

The Chapter 6 results set is produced by `Tese/src/run_results_matrix.py` — 22 cases (classical
`peg`'s `show_peg` was dropped 2026-09-22, so Chapter 6 flies eight of the nine laws; §6.7's
`show_ref_track` and `show_ref_track_apollo`, added 2026-09-23, are peg_new and apollo flying the
PMP reference's plan with no optimiser; §6.4's `pmp_norot`, added 2026-10-04, is the PMP of
`gt_norot`'s non-rotating environment),
one frozen baseline with one factor changed at a time, each case in its **own subprocess** so no module
global can leak between them, and each writing its archive into its **own folder**
(`Output/results_matrix/<case>/<case>.npz` + `.json` + `.manifest.json`), with one
`results_matrix.csv` at the top. Prove every case dispatches before committing a night to it:

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe Tese/src/run_results_matrix.py --smoke
```

Then drop `--smoke` for the real thing (measured ~25.7 h at 100×500 for the whole matrix, plus the
PMP swarms and the ~1 h reference rebuild). `--case <name>` runs one case in-process with
solver output on screen; `--only` takes a **comma-separated** list of substrings
(`--only gt_,peg_` is exactly the ten cases of §6.2 and §6.3, in one invocation — filtering across
two invocations would leave `results_matrix.csv` holding only the second one's rows).

**The production configuration was fixed with the user on 2026-09-25;** see worktree.md §4.
- Re-flown swarms run at `--budget 250,1000`.
- The refresh fix is on.
- The six §6.2 archives are kept.
- `peg_direct` is law-terminated with the grid + Brent kick search.
- The three §6.4 PMP rows are **polished extremals re-flown from stored decision vectors, not
  swarmed** (`pmp_baseline` = the tracked reference cache's extremal).
  - Every reference follows one recipe: a 750×1500 swarm from five seeds, each refined, seed 3's
    half-step extremal kept (user decision). See
    `dev-notes/pmp-references-750x1500-plan-2026-10-04.md`.
  - The refinement runs with the PMP coast bound `PSO_UB[3]` = 3500 s (2026-10-06).
    `pmp_vacuum` overrides it to 2500 s in its own case: without drag the family runs on to a
    coast that grazes the surface, 4.4 km at 3500 s. The law swarms keep 2000 s
    (`PSO_COAST_UB`, `PSO_MG_UB`), which none of them approaches (0–820 s).
  - The stored extremals (2026-10-06, `Output/pmp_polish_launchsite_20261006/`):

    | Reference | Propellant left | Coast | Final burn | Lowest point after MECO |
    |---|---|---|---|---|
    | `pmp_baseline` | 22 620.6 kg | 1 741 s | 4.1 s | 59 km |
    | `pmp_vacuum` | 24 822.4 kg | 2 500 s (at its bound) | 3.7 s | 57 km |
    | `pmp_norot` | 19 560.5 kg | 2 380 s | 2.9 s | 60 km |

  - All three are circular at 500 km. The half-step pass changed none of them (0 kg).
  - **Chosen 2026-10-07, not yet adopted here:** the same seed-3 swarms refined with
    `PSO_UB[3]` = 2000 s, first pass only. pmp_baseline 22 649.0 kg (coast 1 823 s), pmp_vacuum
    24 536.4 kg (1 987 s), pmp_norot 19 345.3 kg (1 879 s).
    - The Chapter 6 results set flown with them is
      `Tese/src/Output/results_matrix_chapter6_20261007/`: untracked, 20 cases at `f350460`,
      with a README. Its three coast-parameter gravity-turn rows are their refined points
      (2026-10-07): the same recipe without costates, which a closed-loop law such as peg_new
      cannot use.
    - `build_matrix`'s stored extremals, the tracked `pmp_reference.npz` and this table still
      hold the 3500 s ones above.
  - `--swarm-extremal` swarms a case that has one (how `pmp_vacuum` was re-searched); the harness
    refuses it on any other case.
  - `dev-notes/pmp_swarm_polish.py --case` takes any `indirect_pmp` case, and refuses a start
    flown in another environment (drag, rotation, pseudo-forces, engine modes).
- Both segmented cases fly `SEGMENTED_LAW_TERMINATED_ARCS = True`: peg_new ends arc 1 at the
  reference's coast start and arc 3 at the orbit, on its own t_go, and the swarm picks
  `[Δt_c, γ_p]` (+ the hand-off altitude).
- `--smoke` flies a copy of the tracked reference (`_prepare_smoke_reference`), not a token one.
- `gt_apogee` flies `APOGEE_CHECK_COAST_FRAME = "rotating"` (`BASELINE`, 2026-09-30): its coast
  is the other architectures' coast, not a converted inertial one (see Architecture below).

Chapter 6's figures and tables are drawn offline from these archives, straight into the thesis
repo. After re-flying any case, re-run both (from `Tese/src`, with `PYTHONPATH=.`):

```bash
FIG_OUT=C:/Users/eduar/Desktop/Tese/Thesis_Overleaf/Figures C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe -m Plots.results_figures.make_all
```

```bash
TAB_OUT=C:/Users/eduar/Desktop/Tese/Thesis_Overleaf/Tables C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe -m Plots.results_figures.tables
```

The thesis draws each table body with `\resultstable{<name>}` (an `\input` of `Tables/<name>.tex`),
so no number in a Chapter 6 table is typed by hand; never edit those files in Overleaf. Captions
and labels stay in `Thesis_Results.tex`.

Two flags exist so a subset can be rehearsed without editing config or endangering the real batch:
`--budget P,G` sets every swarm architecture's PSO budget in memory (`--budget 50,100`), and
`--out DIR` writes into a different root. `--budget` deliberately does **not** touch
`PMP_REFERENCE_PSO_*`, which is part of the PMP cache key — changing it rebuilds and overwrites
the tracked `pmp_reference.npz`.

Note what `--budget` cannot reach. `apogee_check` runs no PSO at all: its cost is the
1000-point brute grid in `solver.py` (`solver.BRUTE_GRID_POINTS`, which the archive records as
`n_evaluations`), and every grid point is a complete `ra.run()` ascent, so `gt_apogee` costs the
same at any budget (**measured: 57–61 s** with the rotating coast; 48 s before it). And `PSO_DIRECT_*` ships at 50×100 already, so
`--budget 50,100` leaves the two `direct` cases at full production fidelity. `show_ref_track` runs no
search at all (~1 s each), and neither does `show_ref_track_apollo`; they fly the plan stored in
the reference cache.

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
3. `COAST_METHOD` ∈ {`pso_coast`, `direct`, `apogee_check`, `reference_track`} → picks the solver for
   all other laws. `reference_track` (2026-09-23) searches nothing: peg_new or apollo flies the PMP
   reference's kick, arc-1 end state and coast length (`Simulation/reference_track_solver.py`), and
   it raises for any other law. apollo is a fixed-time law, so its arc 1 ends at the reference's
   cutoff instant. Its coefficients are refreshed outside the ODE (`GuidanceState.apollo_external`),
   because the `pso_coast` in-RHS refresh fires on `solve_ivp`'s speculative trial points. On this
   flight that ended apollo's arc 1 966 m/s short.
   - `REFERENCE_TRACK_COAST_MODE` (2026-10-07) chooses how the coast ends. The config default is
     `"duration"`, the reference's Δt_c. The matrix's two cases fly `"target_altitude"`: the coast
     ends where the flight climbs through the target altitude, or at its apoapsis if that is lower.
   - Why: after a long, low coast, arc 1's ~0.06° miss moves the apoapsis by ~6 km. A short final
     burn can correct velocity, not position, so a fixed-length coast cost peg_new 5.6 t.
   - apollo still misses: its apoapsis is 2.2 km low, and its 3.4 s final burn, frozen at
     ignition, spends all its thrust vertically (thesis flag K12).
   - `GUIDANCE_REFRESH_MODE = "cycle"` (2026-09-24) applies the same fix to every guided arc of
     `pso_coast`, `direct` and the segmented Stage 2, through `pso_coast_solver.solve_guided_arc`.
   - The config default is `"in_rhs"`, which is byte-identical to before. The results matrix
     flies `"cycle"` (`run_results_matrix.BASELINE`, decision 2026-09-25), so every
     results-matrix archive flown before that date is `"in_rhs"`.
   - Measured by `dev-notes/refresh_ab.py`; see worktree.md §4. `main.py`'s final `else` runs the apogee-check search for any value it does not
   recognise, so a new value needs its own branch there.
   - `apogee_check`'s coast from SECO to apogee is chosen by `APOGEE_CHECK_COAST_FRAME`
     (2026-09-30). The config default is `"rotating"` since 2026-10-03, as the matrix flies;
     `"inertial"` is the old path, kept bit-identical. See "The apogee check" under Architecture.

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
  reference_track_solver.py  no search: peg_new or apollo flies the PMP reference's plan
  indirect_pso_solver.py 7-var PSO over PMP costates; propagates [s,r,v,γ,m,λ_r,λ_v,λ_γ]
  segmented_guidance_solver.py  multi-law schedule; reuses ra.run_stage1 + pso_coast Stage-2
  segment_reference.py   builds/caches the indirect-PMP reference that supplies segment waypoints
Guidance/                one module per law, pure functions returning α (or coefficients)
Auxiliary/               constants, atmosphere, gravity, earth_rotation, rocket_specs
Plots/new_metrics/       one file per metric; new_plot_runner.py runs the ~20-plot suite
Plots/results_figures/   the Chapter 6 figures (make_all) and tables (tables); _data.Case is
                         THE loader for any archive
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

**Two laws were re-aligned with their sources on 2026-09-10** (`dev-notes/guidance-audit-2026-09-10.md`
has the audit of all nine). Classical `peg` now flies `sin(pitch) = A + B·t + C`, where
`C = (μ/r² − ω²r)/a₀` is the gravity/centrifugal fraction the Orbiter-wiki reference adds in the
steering because its guide step leaves gravity out; `peg_alpha` takes `C` as a required argument
(`peg_guidance.compute_gravity_term`, evaluated at the current state on every call). Without it
the law opened Stage 2 at α = −97°. Its estimate step is now term-for-term the reference; note
that at this vehicle's Stage-2 ignition `C ≈ 0.85–0.91` and for the shallower kicks `A + C > 1`,
where the reference's small-pitch expansion `f_θ = 1 − f_r²/2` is outside its validity and the
guide–estimate iteration has no unique fixed point — a property of the classical algorithm on a
T/W ≈ 1 stage, to be reported, not patched. `apollo` now subtracts the local-frame kinematics of
`(v cos γ, v sin γ)` — `−g + v_x²/r` vertically, `−v_x v_y/r` horizontally
(`apollo_guidance.local_frame_accelerations`, tested against `diff_eom_base`) — instead of a
gravity vector rotated by `s/R_E`, and resolves the thrust-magnitude constraint the way Luminary
P12 does: vertical channel first, downrange takes `sqrt(a_T² − a_y²)`; pass
`a_thrust_available=F_T/m` from both dispatchers. **Documented, deliberately not fixed:** under
the coast architecture every closed-loop law steers the pre-coast Stage-2 burn towards the *final*
orbit as a direct insertion, then the swarm's coast discards that plan (comment above Arc 1 in
`run_pso_coast_trajectory`). Only `reference_track` and the segmented mode under
`SEGMENTED_LAW_TERMINATED_ARCS` give that arc an intermediate target; a plain segmented schedule does
not, since its final law aims at the orbit in both burns.

**`peg_new` was re-aligned with its source on 2026-09-23.** The source is Mahajan & Condon,
AAS 25-844 (`Desktop/Tese/References/PEG_ASC25_Mahajan - PEG_recent.pdf`), not the "Sagliano et
al." the docstring used to cite. Two defects are fixed:
- `S1` is eq. 69, `S0·τ − c·t_go²/2`; it was `S0·t_go`, which made `λ'_r` 2.8× too small.
- The gravity integrals follow Algorithm 1 steps 15–20. `v_G` and `r_G` come from quadrature
  along the predicted powered trajectory, in the law's own planar model (radial `−μ/r² + v_θ²/r`,
  tangential `−v_r·v_θ/r`, no Earth-rotation terms). The velocity miss is fed back until it
  converges, with a secant step length. Before, `v_G` was a two-point average that drifted to its
  burnout value and `r_G` was `½·ḡ·t_go²`, which commanded a 20° pitch-down at Stage-2 ignition.

Measured with `dev-notes/arc1_reference_track.py` (peg_new tracking the PMP reference's arc 1):
the waypoint miss fell from 24 km / 21 m/s to 0.03 km / 0.1 m/s. That flight is now
`COAST_METHOD="reference_track"` and the matrix case `show_ref_track`; the script imports it and
adds the controls. The reference cache stores the reference's `decision_vector` beside its
trajectory for it (re-seeded 2026-09-23 from the same archive, arrays unchanged). **Every `peg_new` archive flown
before this change is stale**, and so is anything that used `TGO_ESTIMATOR="peg_new"`.
`tests/test_peg_new_predictor.py` pins the change.

Two consequences:
- A major loop now costs 0.02–0.45 ms instead of 0.02 ms: +37 % per `peg_new` trajectory, and 2.8×
  for `apollo` under `TGO_ESTIMATOR="peg_new"`, which calls it on every RHS evaluation.
- Freeze thresholds below ~10 s break the endgame, because `λ'_r` grows like `r_go/t_go³`. This
  covers the segmented `SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD = 2.0`; see worktree.md.

**The tangent laws are open-loop under `pso_coast` since 2026-09-11**, in the form the
bibliography gives them: `tan θ` linear (`linear_tangent`) or a ratio of linear functions
(`bilinear_tangent`) in time, with the constants chosen by the swarm (`θ0`, `θf`, plus the
mid-span fraction `μ` for bilinear, appended to the decision vector) rather than derived from the
current γ. The old closed-loop form matched `tan θ = tan γ_now` with the terminal pitch pinned at
zero, which leaves nothing to target and returns α ≡ 0 on refresh; it survives only as the
fallback for architectures that supply no constants (`apogee_check`, `direct`, segmented). Two
conventions, both measured against the indirect-PMP optimum: time runs **continuously from first
Stage-2 ignition to the planned final cutoff, through the coast** — unlike `exp_shooting`, it is
not re-epoched (re-epoching fits the PMP steering 4–8° worse) — and pitch is from the local
horizontal, as in Chapter 4's equation. `GuidanceState.tan_t0`/`tan_tf` carry the span and are not
touched by `restart_for_new_burn`.

**The force model is now shared, and that took a fix.** `diff_eom_base` is documented as the EOM
*"WITHOUT Earth rotation"* — the rotating-frame pseudo-forces were added one layer up, in
`rocket_dynamics`. The PSO Stage-2 ODEs call the kernel directly (they cannot use the 500-line
`rocket_dynamics` inside a swarm inner loop), so for a long time they silently flew a non-rotating
model while Stage 1 did not. Coriolis and centrifugal are now applied in `_stage2_ode_guidance` too,
and are all-or-nothing per architecture — carried for the whole ascent by every architecture,
`indirect_pmp` included since 2026-09-16 (Stage 1 via `INDIRECT_PMP_STAGE1_PSEUDO_FORCES`, Stage 2
via the default `INDIRECT_PMP_STAGE2_FRAME="rotating_pseudo_forces"`, whose state ODE makes the very
pseudo-force call the coast solver makes; the earlier whole-ascent exemption had handed Stage 2 a
state 9.1 km lower, 44 m/s faster and 4.5° shallower than every other case's). The switch is
`ra.set_pseudo_forces_for_run()`, set explicitly by the driving solver and **never inferred from
config** (building the segmented PMP reference runs the *indirect* solver, so a config-derived gate
would mislabel it).

**The rotation model is `EARTH_ROTATION_MODEL = "launch_site"` since 2026-10-05** (a label, not a
switch; any other value raises). The ascent is planar.
- The latitude is held at `LAUNCH_LATITUDE` for the whole flight. Nothing follows a great circle,
  and the `data[5]` row is constant.
- The rotation is credited as the launch-site speed ω·r·cos φ₀, along-track in full and never
  resolved on the launch azimuth. The same credit is used in the target (`v_circular_rotating`), the
  conversion (`ecef_to_eci_velocity`), the budget gain (`losses.launch_site_gain`) and the
  pseudo-forces (`earth_rotation.planar_pseudoforce_rates`). The last is the ENU terms at heading
  π/2: a plane rotating at Ω = ω cos φ₀.
- The target is therefore exact level flight, and its conversion is a circular orbit.
- The azimuth enters no equation of motion. Only apogee_check's inclination diagnostic reads it.

Before this, two credits were mixed:
- The pseudo-forces were resolved on the held 44.98° heading, at a great-circle latitude.
- The budget gain was projected onto the azimuth while the target was not.

So the target was the apoapsis of an ellipse with a ~60 km periapsis (the coast-to-target the PMP
polish exploited), and the residual sat at −112 to −138 m/s.

**Every rotation-on archive and the tracked `pmp_reference.npz` predate the change.**
- The label is in the reference cache key and in `segment_reference._ARCHIVE_MUST_MATCH`, so both
  are refused.
- Sixteen tests skip until the re-fly (`tests/_refly.py`), then re-enable themselves to be
  re-pinned.
- Any segmented, `reference_track` or `pmp_baseline` flight now rebuilds the tracked reference.
- `--smoke` refuses the reference cases.
- `make_all` and `tables` refuse rotation-on rows of two models together
  (`_data.check_one_rotation_model`), so a partly re-flown matrix cannot reach the thesis
  mixed. The two rotation-off rows, `gt_norot` and `pmp_norot`, are exempt: they re-fly
  bit-identical under the new code.

**The PMP's Stage 2 carries the pseudo-forces in its state equations, with the costate equations
kept as published** (decision 7d, 2026-09-16; `INDIRECT_PMP_STAGE2_FRAME="rotating_pseudo_forces"`).
- **What the costates omit.** The partial derivatives of the Coriolis/centrifugal terms. Measured
  along the arc at the baseline site (2026-10-05), they are at most 0.3 % of the retained partials
  component-wise and 2.4e-5 of the costate-rate vector in norm. ∂H/∂s, which would make λ_s a
  fourth costate, is identically zero, since nothing depends on downrange
  (`tests/test_pmp_stage1_pseudo_forces.py` pins both).
- **The control law** is exact, since α does not appear in the terms. Every Hamiltonian the
  transversality penalty reads is `λ·f` of the flown rates (`_hamiltonian_at`).
- **The target** is the laws' own √(μ/r) − v_rot. With the terms in the state equations it is
  level flight at any site and azimuth (`tests/test_pmp_frame.py`).
- **The legacy pseudo-force-free `"rotating"` form** turns down at −0.45°/min. Its target is the
  apoapsis of an ellipse with periapsis −890 km, and the defect is the missing terms, not the frame.
- **The `"inertial"` form** (flown 2026-09-13 → 2026-09-16) is `pallone2016` to the letter. Since
  2026-10-05 it is the exact counterpart of the default, off the equator too: both credit ω·r·cos φ₀
  along-track. Before, it credited 121 m/s ≈ 944 kg more than the azimuth-resolved terms.
- `_stage1_pseudo_forces()` refuses any pairing that mixes force models within one ascent. The swarm's transversality penalty is `INDIRECT_PMP_TRANSVERSALITY`: since 2026-09-13 the
stationarity conditions of its own burn/coast/burn durations (`H_coast_end = 0`,
`H_burn1_end = H_last_burn_start`, `H_burn_end < 0`), which need no mass costate. The older Eq. 38
form took H at Stage-2 ignition and cannot be satisfied.

**When adding a new term to the equations of motion, put it in `diff_eom_base`, not in
`rocket_dynamics`** — otherwise it silently misses every population-based architecture, which is
exactly how the pseudo-force gap arose — and mirror it in the PMP's own drag-free kernel,
`indirect_pso_solver._stage2_state_rates` (the pseudo-forces reach it through the same
`planar_pseudoforce_rates` call the coast solver makes; a cross-solver propagation test
guards that one term, nothing guards a new one).

**Never latch an event out of the ODE right-hand side.** `solve_ivp` calls `rocket_dynamics` at
speculative times well beyond the step it goes on to accept, so a flag set there records a time the
trajectory may never reach — and the instantaneous kick splits Stage 1A into *two* `solve_ivp`
calls, so a flag thrown during the first is already true when the second starts. That is how the
payload fairing came to be jettisoned at T+7.5 s at 144 m altitude for every atmospheric case,
with the vehicle flying its whole max-q phase (38–52× the threshold) without one. An event belongs
in an `interrupt_*` function as a **signed function of the state passed in**, with `direction` set
so only the intended crossing counts — `interrupt_fairing_jettison` returns `q − threshold` with
`direction = -1`, because q is below the threshold early in flight as well as late.

MECO had the same defect and was fixed the same way (`interrupt_main_engine_cutoff`, returning
`m − _stage1_burnout_mass()`): burnout masses scattered from −701 to +235 kg around the 120270.0 kg
every case should share, ~936 kg across the matrix, on a quantity no guidance law controls. Read
the crossing out of `t_events`/`y_events`, **not** `sol.t[-1]`/`sol.y[:, -1]` — with a terminal
event scipy truncates `t_eval` at the last grid point at or before the root and never appends the
root, so the grid endpoint quantises the cutoff to `TIME_STEP` (~27 kg at `mdot_1`). Stage 1 is now
flown by one shared `_fly_stage1()` for both dispatchers: burn → fairing → root-found MECO →
unpowered coast to separation, the separation being a planned interval integrated to exactly.
`event_second_engine_ignition` is **still latched** from the RHS — left deliberately, since it
compares against a planned time rather than an integrated state and is reached only on the legacy
`apogee_check` path (measured: ~49 ms late; every PSO architecture uses an explicit
`_T_IGNITION_DELAY` instead).

**The archived thrust record comes from a right-hand-side log, so it had the same disease (fixed
2026-09-26).** `rocket_dynamics` appends `F_T` (and the pseudo-force magnitudes and `t`) on every
call, trial points included, and the thrust channel is interpolated from that log. Past a cutoff
root the integrator had already probed with the engine on: every archive showed 0.2–0.9 s of phantom
Stage-1 thrust after MECO and a ramp over the samples before it, and the legacy path also at its
Stage-2 cutoff, where it additionally fed the unsorted log to `np.interp`. `dv_ideal` and the
budget `residual` were off by −23.5 to +9.7 m/s; the trajectories never were (the log is
output-only). Now `_close_logged_burn(t_cut)` trims the log at each root-found cutoff and closes
the burn as a step, and **`ra.thrust_on_grid()` is the one reader** — use it, never
`interpolate_to_time(ra.time_history, ra.thrust_history, …)`, for any new thrust channel.
`tests/test_thrust_record.py` pins it (the recorded Stage-1 thrust integrates to `M_PROP_1` on both
dispatchers). The 21 matrix archives were repaired in place from bit-identical re-flights
(`dev-notes/repair_thrust_record.py`; each manifest has a `repairs` entry; originals in
`Output/results_matrix_prerepair_20260926/`). With the record right, the residual closes to
~0 m/s in both rotation-off cases and sat at −112 to −138 m/s in every rotation-on case. That was
the two mixed rotation credits, gone with `EARTH_ROTATION_MODEL = "launch_site"` (2026-10-05).
Since then the rotation-on residual is the in-plane centrifugal work alone.

**The apogee check coasted in its own frame, and its budget recorded no drag (both fixed
2026-09-30).**
- **Frame.** The legacy path converted the SECO state with the full ω·r·cos φ
  (`ecef_to_eci_velocity`, launch latitude) and flew the half-orbit coast without pseudo-forces.
  No other architecture converts before insertion, and the rotating-frame physics they flew then
  credited only the share of that speed along the heading (before `EARTH_ROTATION_MODEL`,
  2026-10-05). Coasted their way, gt_apogee's SECO
  state peaked at 184.6 km, not 499 km, and the old row was over-credited by ~0.7 t.
- **The "rotating" coast** (`ra._finish_single_burn_rotating`): the conversion-based event
  (`interrupt_single_burn_traj`) now only brackets SECO from below. The burn continues with dense
  output, and SECO is root-found so that a coast on `pso_coast_solver`'s own coast ODE
  (`_coast_to_apoapsis`, same tolerances) reaches its apoapsis at the target. The impulsive burn
  there goes to `v_circular_rotating`, the shared target, and may be a retro-burn costing |Δv|.
  It raises if the bracket assumption fails (a launch due east could break it).
  - gt_apogee re-flown: 22 168.7 → 21 534.2 kg, circularisation 89.8 → 1.1 m/s. Its search now
    finds the same coast-to-target the swarms find.
- **Read the frame, do not infer it.** `run()` sets `ra.FINAL_STATE_INERTIAL`,
  `ra.TIME_CIRCULARISATION` and, under "rotating", `ra.STATE_INSERTION`. `main.py`, the matrix
  and `dev-notes/repair_thrust_record.py` read them. Under "rotating" the last sample is 1000 s
  past insertion, far downrange: converting it gave 712 × 256 km.
- **Drag.** The full-simulation branch set `rocket_specs.C_D = 0` after SECO and never restored
  it. The flight never saw it, because `atmosphere.drag_force` binds `C_D` as a default argument
  at import. `Auxiliary.losses` binds it the same way but is imported lazily, after the flight,
  so every apogee_check budget recorded `dv_drag = 0` (gt_apogee: 34.9 m/s). The line is gone.
- `tests/test_apogee_check_coast.py` pins both; the drag test fails with the line back.
  Originals are in `Output/results_matrix_pre_apogee_fix_20260930/`.
- **Budget window and thrust record (2026-10-07).** Under "rotating" the archived budget runs
  through the coast to the apoapsis and adds the impulse (`run_record.apogee_impulse_index`),
  like every other architecture's coast. Before, it stopped at SECO: gravity loss 1 180 against
  1 576 m/s, residual +17.4 m/s. The thrust log is now closed to zero at SECO; the coast logs
  nothing, so the record held 934 kN through it. `gt_apogee` in the Chapter 6 set is re-flown
  with both (see its README).

**Fairing jettison is a planned altitude crossing, and all five architectures share it.**
`FAIRING_JETTISON_MODE` defaults to `"altitude"` — `ALT_NO_ATMOSPHERE`, 65 km — rather than to
whatever `ATMOSPHERE_EXIT_METHOD` happens to be. Under the old q-based rule the jettison landed
12–34 s before MECO and sometimes after it (`gt_apogee` crossed 3.7 s late; `gt_direct` never
crossed), which put a 1900 kg step inside the PSO search space keyed on a variable the swarm was
optimising, and left the architectures of §6.2 differing in fairing mass as well as in optimiser.

The legacy `run()` sheds it via the `interrupt_fairing_jettison` event in Stage 1 or Stage 2. The
PSO Stage-2 propagations carry no such event — their inner loop runs thousands of trajectories, so
root-finding one on every arc would be paid tens of millions of times — and instead call
`ra.shed_fairing_if_due(t, state)` at each Stage-2 arc boundary. Both share one criterion,
`ra._fairing_margin()`, so they cannot drift. Before that helper existed a trajectory staging below
the criterion carried 1900 kg of dead mass to orbit for ever.

### Segmented (multi-law) guidance

`segmented_guidance_solver` flies an ordered `GUIDANCE_SEGMENTS` schedule of `(law, altitude)`.
Two mechanisms make it work: `ra._SEGMENTED_ALPHA_HOOK` (a callable installed into
`rocket_dynamics` so a law can steer *during Stage 1*, sub-MECO) and a **planned-deadline t_go**
(`deadline − t`, deadlines from the PMP reference) instead of the rocket-equation estimate that
collapses at the stage boundary. Non-final segments aim at indirect-PMP `(alt, v, γ)` waypoints;
the final segment inserts to orbit. **The coast is not a segment boundary**: by default the final law
aims at the orbit in *both* Stage-2 burns and the swarm cuts the first one short, so a
`gravity_turn → peg_new` schedule has the same arc-1 targeting gap as `pso_coast`.
`SEGMENTED_LAW_TERMINATED_ARCS` (2026-09-25, on for the matrix) gives peg_new the reference's coast
start as its arc-1 target and lets it end both burns on its own t_go (`run_segmented_law_terminated`,
reusing `reference_track_solver.fly_law_terminated_arc`); x becomes `[Δt_c, γ_p]`. The segmented
solver also sheds the fairing at Stage-2 start and ignition since that date; before, a kick staging
below 65 km carried it to orbit. The PMP reference is cached to
`Tese/src/Output/pmp_reference.npz`, keyed by target orbit + vehicle + reference-PSO budget
(path is resolved against the project root, so it is cwd-independent). With the reference budget
equal to the PMP swarm's own (250×1000 since 2026-09-16, same seed) the reference build IS the
`pmp_baseline` run, so the cache can be seeded from that case's archive with
`segment_reference.cache_from_archive` instead of spending ~2 h rebuilding it; the archive's
manifest must match the configuration in force or it is refused. **The swarm alone does not find
the PMP optimum** (2026-09-17: a local refinement beat both production points by 620–1020 kg).
`dev-notes/pmp_swarm_polish.py` — Levenberg-Marquardt on the orbit + duration-stationarity
conditions, then γ_p continuation — writes the best extremal as a standard archive under
`Output/pmp_polish/<case>/`, and `cache_from_archive(..., allow_other_search=True)` can seed the
reference from such an archive whatever seed it came from.

**What its first run (2026-09-17) showed, and how it was resolved (2026-10-05).** With the
pseudo-forces resolved on the azimuth, the shared target (500 km, √(μ/r) − ω·r·cos φ, γ = 0) was
the APOAPSIS of a real ellipse, 123–128 m/s short of circular. The polish reached it by coasting
with no circularisation burn: +1 701 kg (baseline) and +1 496 kg (vacuum), coasting 1 282–1 381 s
with a 0.0–0.2 s last burn. The laws were exposed too, and unevenly.

The user kept the unprojected credit on 2026-08-31 and 2026-09-17. On 2026-10-05 they chose instead
to apply it everywhere, the pseudo-forces included (`EARTH_ROTATION_MODEL`, above). The target is
now circular in the model, and that exploit is closed. The archived rows and the polished
extremals predate this and must be re-flown. The replayed `pmp_baseline` extremal misses the
target under the new model (J′ 121.8 against 0.76).

Every results-matrix archive carries the optimiser's full-precision `decision_vector` since then;
the solver's console printout is rounded and does not re-fly to the archived insertion.

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
- **Never change a constant in `rocket_specs` (or `constants`) at run time.** Several helpers take
  them as default arguments bound at import (`atmosphere.drag_force(q, C_D=r.C_D, A=r.A)`,
  `losses.loss_histories`, `losses.delta_v_budget`). A runtime assignment therefore reaches only
  the modules imported after it: half the code sees the old value and half the new. That is how
  the apogee check's `r.C_D = 0` left the flight untouched but zeroed every recorded drag loss.
  Pass the value explicitly, or gate the term with a flag the dynamics reads.
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
