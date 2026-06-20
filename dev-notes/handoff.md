# Thesis Project — Session Handoff (2026-06-14)

> This replaces the older `handoff.md`, which described an `indirect_pmp` /
> `Final_TPBVP`-branch setup that is no longer the active configuration.

## Repo State

- Working directory: `c:\Users\eduar\Desktop\Tese\Code\Thesis-Project`
- Active branch: `main`
- **Nothing from this session is committed yet.** `git status`:
  ```
   M Tese/src/Guidance/apollo_guidance.py
   M Tese/src/Input_File/simulation_parameters.py
   M Tese/src/Simulation/rocket_ascent.py
   M Tese/src/Simulation/solver.py
   D Tese/src/_direct_run.py
   M Tese/src/main.py
  ?? Tese/src/Simulation/direct_pso_solver.py
  ```
- Run command: `python Tese/src/main.py` from the repo's `src` dir (or with
  `sys.path` set up as `main.py` does).

## Current Configuration (`Tese/src/Input_File/simulation_parameters.py`)

- `GUIDANCE_MODE = "apollo"`
- `COAST_METHOD = "direct"` (continuous single Stage-2 burn, MECO at circular
  velocity; no coast/circularisation burn)
- `KICK_PROFILE_MODE = "triangular"` (default — `run()`'s Stage-1A kick is a
  ramped alpha profile; `run_stage1()`-based solvers are unaffected, see below)
- `DIRECT_OPTIMIZATION_MODE = "pso"` — **the user just switched this from
  `"brute_force"` to `"pso"`**, i.e. `main.py` will now take the new
  `direct_pso_solver` path (see Part 3 below)
- `PSO_DIRECT_N_PARTICLES = 50`, `PSO_DIRECT_MAX_GENERATIONS = 100` (reduced
  from the defaults 100/250 — looks like a quick test run; bump back up for a
  "real" result)
- `ENABLE_EARTH_ROTATION = True`, `INCLUDE_PSEUDO_FORCES = True`,
  `LAUNCH_LATITUDE = 28.5`, `TARGET_ORBITAL_ALTITUDE = 500e3`,
  `TARGET_ORBIT_INCLINATION = 51.6`
- `EVENTS_PRINT = True`, `INTERRUPTS_PRINT = False`
- `DIRECT_INSERTION_VELOCITY_TOL_MS = 10.0`, `DIRECT_INSERTION_FPA_TOL_DEG =
  0.5`, `DIRECT_INSERTION_ALTITUDE_TOL_KM = 5.0`

## ⚠️ Blocking issue for the next run: pygmo not available

`DIRECT_OPTIMIZATION_MODE = "pso"` requires `pygmo`. In this session's shell:
```
python -c "import pygmo"  → ModuleNotFoundError: No module named 'pygmo'
```
and `conda` was not on PATH at all (the old handoff.md referenced a
`pygmo-env` conda environment — that may still exist but wasn't reachable from
this session's shell). **Before running `main.py` with `DIRECT_OPTIMIZATION_MODE
= "pso"`, confirm/locate an environment that has `pygmo` installed** (it's
already a hard dependency of `pso_coast_solver`/`indirect_pso_solver`, so if
those have run successfully before, that env is the one to use).

If pygmo genuinely isn't installed anywhere: `conda install -c conda-forge pygmo`.

---

## What Was Implemented This Session

### Part 1 — apollo + apogee_check incompatibility (done, pre-existing session)
- Root cause: `apollo_guidance`'s constant-acceleration horizontal channel
  (`k1=0`) commands ~46° AoA from the first guidance step at low T/W — the
  1000-point brute-force kick sweep with `COAST_METHOD="apogee_check"` finds
  no usable kick (always sentinel).
- **Resolution (no guidance-law change):** default `COAST_METHOD` changed to
  `"direct"` — pairs `apollo` with a continuous burn cut at circular velocity
  instead of requiring an apogee match.
- Cleanup: removed leftover one-shot debug print block in
  `Guidance/apollo_guidance.py`, removed temp diagnostic scripts.
- Known result for `apollo + direct + brute_force`: e≈0.028 @ ~500.6 km, not a
  "clean" insertion (residual FPA ~1.6°) — see memory `direct-insertion-cutoff`.

### Part 2 — `KICK_PROFILE_MODE` toggle (done, verified)
New config:
```python
KICK_PROFILE_MODE = "triangular"   # "triangular" | "instantaneous"
```
- `"triangular"` (default): existing ramped alpha-kick via
  `pitch_program_linear` over `DURATION_INITIAL_KICK`; search space
  `[ALPHA_LOWEST, ALPHA_HIGHEST]` rad.
- `"instantaneous"`: discontinuous γ-jump via `_run_stage1a_with_kick` (same
  mechanism `pso_coast`/`indirect_pmp` always use); search space
  `gamma_p ∈ [1.54, 1.57]` rad, `kick_angle = gamma_p - π/2`.

Changed: `Simulation/rocket_ascent.py` (`run()`'s Stage-1A alpha dispatch +
integration-call dispatch), `Simulation/solver.py`
(`find_initial_kick_angle_coast_single_burn` — searches `gamma_p` range and
converts back to `kick_angle` when `instantaneous`).

**Verified:** both modes dispatch end-to-end for `COAST_METHOD="direct"`
without crashing.

### Part 3 — 2-variable PSO for direct insertion (done, smoke-tested)
New file `Tese/src/Simulation/direct_pso_solver.py` — mirrors
`pso_coast_solver.py`'s PyGMO pattern, optimising 2 variables:
`x = [gamma_p (rad, kick angle), t_burn_pct (% of T_MAX_2, Stage-2 burn duration)]`.

- `run_pso_direct_trajectory(gamma_p, t_burn_pct)`: Stage 1 (`ra.run_stage1`,
  instantaneous kick) → pre-ignition ballistic coast → ONE continuous Stage-2
  thrust arc of fixed duration `t_burn` (no early-MECO event — PSO finds
  `t_burn` directly).
- `compute_direct_objective(result)`:
  `box_margin = ra.interrupt_direct_insertion(0, state_final)`.
  - `box_margin <= 0` (clean insertion) → `J = W_BURN * (burn_frac - 1)` (≤0)
  - `box_margin > 0` → `J = W_BOX * box_margin` (>0)
  - → any clean insertion always beats any non-clean one; ties among clean
    insertions broken by burn time.
- `run_pso_direct_optimization()`: PyGMO `pg.pso`, raises
  `ImportError("pygmo is required ... conda install -c conda-forge pygmo")` if
  pygmo missing (confirmed working).
- `run_pso_direct_full(optimal_params)`: dense re-run for plotting (mirrors
  `run_pso_coast_full`), sets `ra.LAST_DIRECT_MECO`,
  `ra.LAST_DIRECT_INSERTION_REACHED`, `ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL`.

New `main.py` branch: `elif _direct_pso:` (between the `pso_coast` branch and
the brute-force/apogee-check `else`), gated on `COAST_METHOD=="direct" and
DIRECT_OPTIMIZATION_MODE=="pso"`. Runs the PSO, the dense re-run, prints a
results summary (objective/box-margin/burn-fraction, propellant, mission
timeline, orbital elements), and falls through to the shared plot suite.

**Verified (smoke test, without pygmo):** `run_pso_direct_trajectory`,
`compute_direct_objective`, `breakdown_direct_objective`, the `ImportError`
path, and `run_pso_direct_full` all run end-to-end correctly for
`GUIDANCE_MODE="apollo"`, `gamma_p=1.555`, `t_burn_pct=85`
(Stage-1-end h≈105.7 km / v≈2788 m/s / γ≈42.9°; Stage-2-end h≈500.4 km).
**NOT yet verified:** the actual PyGMO PSO optimisation loop and the full
`main.py` PSO-direct path (blocked on pygmo availability, see above).

### Two bugs found & fixed during verification

1. **Double-kick regression** (introduced by Part 2, would have silently
   affected `pso_coast`/`indirect_pmp`/`direct_pso` too): the ODE's per-step
   alpha dispatch checked `KICK_PROFILE_MODE` directly, but `run_stage1()`
   *always* handles the kick via `_run_stage1a_with_kick`'s instantaneous
   γ-jump — so with the default `KICK_PROFILE_MODE="triangular"`, those paths
   got **both** a triangular alpha ramp *and* the γ-jump.
   **Fix:** new module flag `_stage1_kick_handled_by_gamma_jump` in
   `rocket_ascent.py`, set/cleared (try/finally) by `_run_stage1a_with_kick`,
   overrides the `KICK_PROFILE_MODE` check in the ODE dispatch.
   → memory: `kick-profile-mode-double-kick.md`

2. **Windows UTF-8 console crash** (pre-existing): `EVENTS_PRINT=True` prints
   containing Greek letters/° (e.g. `"Δγ = -0.91°"` in
   `_run_stage1a_with_kick`) raised `UnicodeEncodeError` on cp1252, crashing
   any full-sim dense re-run (`run_pso_direct_full`, and latently
   `pso_coast`/`indirect_pmp` full re-runs too).
   **Fix:** `main.py` now reconfigures `sys.stdout`/`sys.stderr` to UTF-8 at
   startup if not already UTF-8.
   → memory: `windows-console-utf8-stdout.md`

---

## Suggested Next Steps

1. Locate/activate a Python environment with `pygmo` installed.
2. Run `python Tese/src/main.py` (with `DIRECT_OPTIMIZATION_MODE = "pso"`,
   currently 50 particles × 100 gens for a quick check) and confirm:
   - `run_pso_direct_optimization` converges to a `box_margin <= 0` (clean
     insertion) or at least an improvement over the brute-force
     `e≈0.028 @ 500.6km` baseline.
   - The dense re-run / plot suite renders without errors.
3. If it works, consider bumping `PSO_DIRECT_N_PARTICLES`/`PSO_DIRECT_MAX_GENERATIONS`
   back to 100/250 for a production run.
4. Once verified, commit the session's changes (currently all uncommitted —
   see file list above).

## Key Source Files

| File | Role |
|---|---|
| `Tese/src/main.py` | Entry point — `execute()`; 3-way dispatch: `pso_coast` / `direct`+`pso` (new) / brute-force-or-apogee-check |
| `Tese/src/Simulation/direct_pso_solver.py` | **New** — 2-var PSO for direct insertion |
| `Tese/src/Simulation/pso_coast_solver.py` | Existing 4-var PSO (thrust-coast-thrust); shares helpers with `direct_pso_solver` |
| `Tese/src/Simulation/rocket_ascent.py` | Core dynamics, `run()`, `run_stage1()`, `_run_stage1a_with_kick`, kick dispatch |
| `Tese/src/Simulation/solver.py` | Brute-force kick-angle search (`KICK_PROFILE_MODE`-aware) |
| `Tese/src/Input_File/simulation_parameters.py` | All tunable parameters |

## Memory Notes (auto-loaded by Claude in future sessions)

- `direct-insertion-cutoff` — MECO trigger semantics, apollo+direct baseline result
- `kick-profile-mode-double-kick` — the double-kick fix above
- `windows-console-utf8-stdout` — the UTF-8 fix above
- `cpr-stage1-brentq-crash` — known unfixed, unrelated to this session
- `ecef-eci-velocity-arg-refactor` — known unfixed, unrelated to this session
