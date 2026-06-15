# Guidance Mode Selection Guide

## Overview

The simulator supports **9 ascent guidance laws**, selected via `GUIDANCE_MODE`
in `Tese/src/Input_File/simulation_parameters.py`. Every law (except `cpr`)
shares the same Stage-1 sequence — vertical launch, then an initial kick
maneuver — before its guidance law takes over. Each guidance law produces a
commanded angle of attack `α` at every integration step, where:

```
α = θ_cmd - γ
```

`θ_cmd` is the commanded pitch (thrust) angle and `γ` is the current
flight-path angle.

## Configuration

Edit `Tese/src/Input_File/simulation_parameters.py`:

```python
GUIDANCE_MODE = "peg_new"  # Options: "gravity_turn", "linear_tangent", "bilinear_tangent",
                            # "apollo", "cpr", "peg", "peg_new", "exp_shooting", "indirect_pmp"
```

An invalid value raises a `ValueError` at startup (`main.py` validates
`GUIDANCE_MODE` against the 9 options above before running).

---

## Guidance Modes

### `gravity_turn` — Pure Gravity Turn

The traditional, passive method: after the initial kick maneuver the thrust
vector stays aligned with the velocity vector (`α = 0`) for the rest of the
flight. The trajectory shape is determined entirely by gravity and the kick
angle — there is no separate "guidance phase".

- **Activation:** none — `α = 0` is simply the dispatch default once the kick
  is complete.
- **Key tunables:** none beyond the kick angle itself, which the selected
  `COAST_METHOD` solver optimizes.
- Implementation: `Guidance/gravity_turn.py`.

### `linear_tangent` — Linear Tangent Steering

Classical ascent guidance: `tan(α + γ) = a·(t_f − t) + b`, with coefficients
`a, b` chosen from boundary conditions (current state, target horizontal
flight at `t_f`).

- **Activation:** after the kick, once the vehicle is past the dense
  atmosphere (per `ATMOSPHERE_EXIT_METHOD`) and Stage-2 has ignited.
- **Key tunables:**
  - `GUIDANCE_UPDATE_RATE` — how often (in seconds) the coefficients and `t_go`
    are recomputed (unless held fixed below).
  - `GUIDANCE_COEFFICIENTS_FIXED` — if `True`, `a, b` are computed once at
    guidance start and held constant; only `t_go` is updated each cycle.
  - `GUIDANCE_TGO_FIXED` — if `True`, `t_go` is also frozen at guidance start.
  - `GUIDANCE_TGO_USE_PSO_PLAN` — inside `pso_coast_solver`/`direct_pso_solver`
    only, replaces the rocket-equation `t_go` estimate with the PSO-planned
    burn-arc countdown.
- Implementation: `Guidance/linear_tangent_steering.py`.

### `bilinear_tangent` — Bilinear Tangent Steering

A more flexible variant: `tan(α + γ)` is the ratio of two linear functions of
time-to-go, `[c1·τ + c2] / [c1'·τ + c2']` (τ = t_f − t), giving 4 coefficients
that control both the value and the derivative at the boundary conditions.

- **Activation:** identical to `linear_tangent` (after kick, atmosphere exit,
  Stage-2 ignition).
- **Key tunables:** same as `linear_tangent`
  (`GUIDANCE_UPDATE_RATE`, `GUIDANCE_COEFFICIENTS_FIXED`, `GUIDANCE_TGO_FIXED`,
  `GUIDANCE_TGO_USE_PSO_PLAN`).
- Implementation: `Guidance/bilinear_tangent_steering.py`.

### `apollo` — Apollo Polynomial Guidance

Classical Apollo explicit guidance: computes linear acceleration commands
`(ax, ay)` (coefficients `k1..k4`) that satisfy terminal **position and
velocity** constraints — `vy = 0` and `y = y_target` at `t_go` — i.e. a full
orbit-insertion endpoint, not just an apogee match.

- **Activation:** after the kick, once the vehicle is past the dense
  atmosphere and Stage-2 has ignited.
- **Key tunables:**
  - `GUIDANCE_UPDATE_RATE` — coefficient recompute interval.
  - `APOLLO_FREEZE_THRESHOLD` — when `t_go` drops below this (seconds), the
    `k1..k4` coefficients are frozen to avoid the numerical blow-up that
    occurs as `t_go → 0`.
  - `APOLLO_THRUST_MAGNITUDE_CONTROL` — if `True`, Apollo also commands thrust
    *magnitude* (capped at the nominal/maximum available thrust), not just
    angle.
- **Coast-method compatibility:** because Apollo's terminal constraints target
  the full insertion endpoint, pair it with `COAST_METHOD = "direct"`.
  `COAST_METHOD = "apogee_check"` cuts the burn on an unrelated mid-flight
  condition (osculating apogee reaching the target altitude while `vy` is
  still large) and is **not** a workable pairing with `apollo` — use
  `peg_new` if you need `apogee_check`.
- Implementation: `Guidance/apollo_guidance.py`.

### `cpr` — Constant Pitch Rate

Skips the kick maneuver entirely: the vehicle flies vertically until CPR
guidance takes over (as soon as `TIME_TO_START_KICK` is reached, even during
Stage 1), then linearly ramps the commanded pitch angle `θ` from 90°
(vertical) down to 0° (horizontal) at a constant rate `θ_dot`:

```
θ_cmd(t) = max(90° − θ_dot · (t − t_start), 0°)
α_cmd    = θ_cmd − γ
```

- **Activation:** immediately at `TIME_TO_START_KICK` — no kick-angle
  optimization is involved for this mode.
- **Key tunables:**
  - `CPR_THETA_DOT_MODE`:
    - `"manual"` — use `CPR_THETA_DOT` directly (deg/s, recommended
      0.1–0.5); the guidance duration is derived as `90° / CPR_THETA_DOT`.
    - `"tgo"` — derive `θ_dot = 90° / t_go`, where `t_go` comes from the
      Apollo propellant-based time-to-go estimate at guidance start.
  - `CPR_THETA_DOT` — manual pitch rate [deg/s] (only used in `"manual"` mode).
- **Known issue:** `GUIDANCE_MODE = "cpr"` currently crashes during Stage-1
  event handling when `ENABLE_EARTH_ROTATION = True` (see project memory
  `cpr-stage1-brentq-crash`). Only the kinematic CPR law described above is
  implemented; the analytic-CPR and CFPAR variants are specified but not built
  (see `dev-notes/cpr_cfpar_guidance_implementation.md`).
- Implementation: `Guidance/cpr_guidance.py`.

### `peg` — Powered Explicit Guidance

The Saturn-V-style closed-loop guidance: maintains a linear pitch program
`sin(pitch[t]) = A + B·t`, and on each major-loop cycle re-solves for the
steering constants `A, B` and the burn-time estimate `T` to drive the
predicted burnout state to the target orbit (`r_T`, `ṛ_T = 0`,
`v_θ_T = √(μ/r_T)`).

- **Activation:** Stage 2 only — initializes right after Stage-2 ignition
  (post-kick, post-atmosphere-exit), then updates every major loop while
  `F_T > 0`.
- **Key tunables:**
  - `PEG_MAJOR_LOOP_RATE` — major-loop update period [s].
  - `PEG_CONVERGENCE_MODE` — `"damped"` (recommended; damped fixed-point
    iteration, stops when `|ΔT| < PEG_CONVERGENCE_TOL`) or `"fixed_iter"`
    (runs `PEG_CONVERGENCE_MAX_ITER` undamped iterations — may oscillate near
    Stage-2 start).
  - `PEG_CONVERGENCE_DAMPING` — damping factor in `(0, 1]` (used when
    `"damped"`; 0.5 recommended).
  - `PEG_CONVERGENCE_TOL` — convergence tolerance [s] (used when `"damped"`).
  - `PEG_CONVERGENCE_MAX_ITER` — iteration cap for both modes.
  - `APOLLO_FREEZE_THRESHOLD` — shared with Apollo; also freezes PEG when the
    burn-time estimate `T` drops below this threshold.
- Implementation: `Guidance/peg_guidance.py`.

### `peg_new` — Analytical Predictor-Corrector PEG

A from-first-principles PEG derivation combining Pontryagin's minimum
principle with Jaggers' "Coke Machine" orthogonality assumption. The primary
variable is `v_go` (the 2-D velocity-to-be-gained vector); `t_go` is obtained
directly from `v_go` via the rocket equation, and the steering law is:

```
û(t) = v_go / ‖v_go‖ + λ'_r · (t − t_λ) · r̂
```

The major loop's predictor-corrector step refines the gravity integral by
averaging radial gravity over the predicted trajectory (trapezoidal rule)
instead of using only the current-position value — this is what gives a
physically correct pitch-down direction for orbit insertion.

- **Activation:** Stage 2 only, identical gating to `peg` (post-kick,
  post-atmosphere-exit, Stage-2 ignition).
- **Key tunables:** the same major-loop parameters as `peg`
  (`PEG_MAJOR_LOOP_RATE`, `PEG_CONVERGENCE_MODE`, `PEG_CONVERGENCE_DAMPING`,
  `PEG_CONVERGENCE_TOL`, `PEG_CONVERGENCE_MAX_ITER`, `APOLLO_FREEZE_THRESHOLD`).
- This is the current default (`GUIDANCE_MODE = "peg_new"`).
- Implementation: `Guidance/peg_guidance_new.py`.

### `exp_shooting` — Exponential Pitch-Law Shooting

An open-loop pitch law `θ(t_rel) = a · exp(b · t_rel)` (so `α = θ − γ`), where
`(a, b)` are solved **once**, at guidance start, via `scipy.optimize.fsolve`
so that the (simplified, drag-free, no-Earth-rotation) forward simulation
satisfies two terminal constraints at burnout: `r(T_burnout) = r_T` and
`γ(T_burnout) = 0`. The coefficients are then held fixed for the rest of the
burn.

- **Activation:** Stage 2 only, same gating as `peg`/`peg_new`
  (post-kick, post-atmosphere-exit, Stage-2 ignition, `F_T > 0`).
- **Key tunables:** none — `(a, b)` are solved automatically from the current
  state and `TARGET_ORBITAL_ALTITUDE`.
- **Coast-method compatibility:** **not supported** with
  `COAST_METHOD = "pso_coast"` — `main.py` raises a `ValueError` at startup,
  because the single-burn-to-depletion BVP cannot honour a thrust-coast-thrust
  split. Use `COAST_METHOD = "apogee_check"` or `"direct"` instead.
- Implementation: `Guidance/exp_shooting_guidance.py`.

### `indirect_pmp` — Indirect Method via Pontryagin's Minimum Principle

The most involved mode: a PyGMO PSO optimizes 7 decision variables —
`[λ0_r, λ0_v, λ0_γ, Δt_c, Δt_r%, coast_start%, γ_p]` — while costates
`[λ_r, λ_v, λ_γ]` are propagated alongside the physical state through a
drag-free Stage-2 free-flight phase. The optimal angle of attack at every
step is:

```
α = atan2(−λ_γ/V, −λ_V)
```

- **Activation / dispatch:** this mode does **not** go through
  `rocket_ascent.run()`'s per-step guidance dispatch at all. `main.py` detects
  `GUIDANCE_MODE == "indirect_pmp"` and instead calls
  `Simulation/indirect_pso_solver.run_pso_optimization()` /
  `run_indirect_full()`, which run their own Stage-1 (gravity turn, PSO-chosen
  kick angle) + Stage-2 (thrust-coast-thrust with costates) trajectory.
- **`COAST_METHOD` is ignored** for this mode — the coast/burn split is fully
  controlled by the PSO, not by `apogee_check`/`pso_coast`/`direct`.
- **Key tunables** (all in the "INDIRECT PMP / PSO PARAMETERS" section of
  `simulation_parameters.py`):
  - `PSO_N_PARTICLES`, `PSO_MAX_GENERATIONS` — swarm size / generation cap.
  - `PSO_C1`, `PSO_C2`, `PSO_OMEGA`, `PSO_VMAX`, `PSO_SEED` — PyGMO PSO
    hyperparameters.
  - `PSO_LB`, `PSO_UB` — bounds for the 7 decision variables.
  - `PENALTY_W_J`, `PENALTY_W_ALTITUDE`, `PENALTY_W_VELOCITY`,
    `PENALTY_W_FPA`, `PENALTY_W_TRANSVERS`, `GAMMA_REF_DEG` — augmented
    objective penalty weights (Eq. 39).
- **Requires `pygmo`** — raises `ImportError` with install instructions if
  missing.
- Implementation: `Guidance/indirect_pmp_guidance.py` (control law and
  costate derivatives) and `Simulation/indirect_pso_solver.py` (PSO driver).

---

## Running the Simulation

```bash
python Tese/src/main.py
```

The program will:
1. Validate `GUIDANCE_MODE` and print which mode is active.
2. Run the kick-angle / PSO optimization appropriate for the configured
   `COAST_METHOD` (or the dedicated `indirect_pmp` PSO driver).
3. Run the full dense trajectory with the optimized parameters.
4. Print a mission summary and generate the plot suite via
   `Plots/new_plot_runner.py`.

See `Project_Description/optimization_process_explanation.md` for details on
how the kick angle / PSO variables are optimized for each `COAST_METHOD`.
