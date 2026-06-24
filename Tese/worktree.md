# Simulator Configuration — Choices and How They Tangle

This document maps **every choice a user can make** in the launch-ascent simulator and **how each
choice constrains, rewrites, or breaks the others**. It is meant to be read *before* you edit the
config and press run, so you can tell which combinations are valid, which are silently ignored, and
which crash.

All settings live in two files (paths below are relative to `Tese/src/`):

- **`Input_File/simulation_parameters.py`** — the mission/guidance/optimizer config (everything you
  normally tune).
- **`Auxiliary/rocket_specs.py`** — the launch-vehicle constants (single fixed vehicle).

Dispatch and guard logic that ties choices together lives in `main.py`,
`Simulation/rocket_ascent.py`, and the three solvers in `Simulation/` (`pso_coast_solver.py`,
`direct_pso_solver.py`, `indirect_pso_solver.py`). Body constants are fixed in
`Auxiliary/constants.py`.

> ### What this build is NOT
> This is the current (reverted/baseline) code. There is **no vehicle registry / `set_vehicle()`**
> (the vehicle is one flat Falcon-9-like spec you edit by hand), **no `NUM_STAGES` flag** (staging is
> hardwired to exactly two stages), and **no planet/body selector** (Earth-only; `constants.py` is
> fixed). If you came here expecting multi-vehicle / multi-stage / multi-planet switches, they are not
> in this build.

### Legend for the "tangle" markers used throughout

| Marker | Meaning |
|---|---|
| **requires** | Has no effect (silent no-op) unless another flag is also set. |
| **forbids / raises** | An illegal combination — the run aborts with `ValueError`/`ImportError`. |
| **changes behavior** | Legal, but the physics/result differs depending on another choice. |
| **silently ignored** | The setting is read elsewhere but this code path never honors it — no warning. |
| **known-broken / footgun** | Works in some paths, surprises you in others; read the note. |

---

## 1. Decision-tree walkthrough — pick in this order

The choices are *not* independent knobs you can set in any order. The code dispatches on them in a
definite order; setting a downstream choice without respecting an upstream one is how you get silent
no-ops. Walk them top-to-bottom.

```
┌─ STEP 1 ── GUIDANCE_MODE == "indirect_pmp" ? ───────────────────────────────┐
│  YES → dedicated branch (main.py:292). It runs its OWN 7-variable PSO        │
│        (costates + timing + kick) and IGNORES COAST_METHOD / KICK_PROFILE_   │
│        MODE / RUN_FAST / DIRECT_* entirely. Requires PyGMO (hard ImportError │
│        at indirect_pso_solver.py:687). → jump to STEP 6 (env/orbit only).    │
│  NO  → continue to STEP 2.                                                   │
│  NOTE: the shipped default IS "indirect_pmp", so out of the box COAST_METHOD │
│        below is inert until you change GUIDANCE_MODE.                        │
└─────────────────────────────────────────────────────────────────────────────┘
        │
┌─ STEP 2 ── COAST_METHOD (the top-level dispatcher, main.py:405) ────────────┐
│  "pso_coast"            → Simulation/pso_coast_solver.py  (4-var PSO,        │
│                           thrust→coast→thrust, direct insertion). PyGMO req. │
│  "direct"               → Simulation/direct_pso_solver.py (always PSO:       │
│                           2-var PSO over gamma_p + burn %). PyGMO req.       │
│  "apogee_check"         → legacy single-burn-to-apogee + impulsive          │
│                           circularisation (brute-force kick search).        │
└─────────────────────────────────────────────────────────────────────────────┘
        │
┌─ STEP 3 ── GUIDANCE_MODE × COAST_METHOD compatibility ──────────────────────┐
│  exp_shooting + pso_coast      → SUPPORTED: PSO optimises the pitch-law       │
│                                  coeffs (a, b) as 2 extra decision vars,     │
│                                  re-epoched per arc (no per-arc fsolve).      │
│  apollo + apogee_check         → now RAISES ValueError (main.py): apollo's   │
│                                  vy=0/alt-at-burnout endpoint ≠ the apogee   │
│                                  cut. Use peg_new here, or apollo+direct.    │
│  {grav_turn,lin/biln_tangent,  → ✗ SUBORBITAL under "direct": one burn, no  │
│   cpr,exp_shooting} + direct     coast can't loft to target. Only apollo/    │
│                                  peg/peg_new close it; use pso_coast/apogee. │
│  cpr + apogee_check            → kick forced to 0, no kick optimisation      │
│                                  (main.py:761). cpr flies vertical first.    │
│  cpr + pso_coast/direct        → flies the gamma_p kick like every mode;     │
│                                  PSO optimises theta_dot (1 extra var). The  │
│                                  legacy Stage-1 cpr branch is gated off here │
│                                  (_IN_PSO_STAGE1) so it no longer crashes.   │
└─────────────────────────────────────────────────────────────────────────────┘
        │
┌─ STEP 4 ── KICK_PROFILE_MODE ("triangular" | "instantaneous") ──────────────┐
│  Honored ONLY in the legacy run() path (rocket_ascent.py:1062, 1887),       │
│  i.e. ONLY under apogee_check now. All three PSO solvers call run_stage1(),  │
│  which ALWAYS uses the instantaneous gamma-jump (rocket_ascent.py:2514) →   │
│  "triangular" is a SILENT NO-OP under pso_coast / direct / indirect_pmp.     │
│  Convention also switches: triangular searches kick over                    │
│  [ALPHA_LOWEST, ALPHA_HIGHEST]; instantaneous searches gamma_p in           │
│  [1.54, 1.57] rad with kick_angle = gamma_p − pi/2.                          │
└─────────────────────────────────────────────────────────────────────────────┘
        │
┌─ STEP 5 ── ATMOSPHERE_EXIT_METHOD ──────────────────────────────────────────┐
│  Sets WHEN guidance can switch on. Gates Stage-2 activation for 5 of 7       │
│  modes (linear/bilinear tangent, apollo, peg, peg_new). cpr and             │
│  exp_shooting do not gate their initial trigger on atmosphere exit.         │
│  Only ONE threshold matters, chosen by the method:                          │
│    "altitude" → ALT_NO_ATMOSPHERE ; "dynamic_pressure" →                     │
│    DYNAMIC_PRESSURE_THRESHOLD ; "aerothermal_flux" → AEROTHERMAL_FLUX_THRESH.│
└─────────────────────────────────────────────────────────────────────────────┘
        │
┌─ STEP 6 ── Environment / orbit (applies in every path) ─────────────────────┐
│  ENABLE_EARTH_ROTATION gates the whole pseudo-force/azimuth family:         │
│    COMPUTE_CROSS_HEADING_COUNTER_FORCE requires INCLUDE_PSEUDO_FORCES        │
│    AZIMUTH_INCLINATION_MODE=="iterative" is force-overwritten to            │
│                                        "formula_compare" under pso_coast     │
│                                        (main.py:424).                        │
│  Plus the always-on targets: TARGET_ORBITAL_ALTITUDE,                        │
│  TARGET_ORBIT_INCLINATION, LAUNCH_LATITUDE.                                  │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Shortcut intuition.** `GUIDANCE_MODE="indirect_pmp"` is a world of its own (ignore everything in
steps 2–4). Otherwise `COAST_METHOD` decides which solver runs, and *that* decides whether
`KICK_PROFILE_MODE` / `RUN_FAST` / the `DIRECT_*` tolerances mean anything at all.

---

## 2. Reference catalog — every configurable parameter

Tables grouped by area. Columns: **Variable · Allowed values · Default · Controls · Tangles with**.
Unless noted, line numbers are in `Input_File/simulation_parameters.py`.

### 2.1 Guidance law

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `GUIDANCE_MODE` (L119) | `gravity_turn`, `linear_tangent`, `bilinear_tangent`, `apollo`, `cpr`, `peg`, `peg_new`, `exp_shooting`, `indirect_pmp` | `indirect_pmp` | The ascent steering law (post-kick / Stage-2). | Drives **everything**: `indirect_pmp` overrides `COAST_METHOD`; `cpr`/`exp_shooting` add extra PSO vars under `pso_coast`; `apollo` **raises** under `apogee_check` (use `peg_new`); under `direct` only `apollo`/`peg`/`peg_new` reach orbit (others → suborbital); `cpr` skips the kick under apogee_check. Invalid value **raises** (`main.py:201`). |
| `GUIDANCE_UPDATE_RATE` (L124) | float s | `2` | Recompute interval for apollo/linear/bilinear coefficients. | Only matters if `GUIDANCE_COEFFICIENTS_FIXED=False`. |
| `APOLLO_FREEZE_THRESHOLD` (L125) | float s | `10.0` | t_go below which apollo/peg coefficients freeze (stability). | apollo, peg, peg_new only. |
| `APOLLO_THRUST_MAGNITUDE_CONTROL` (L127) | `True`/`False` | `False` | If True, apollo also commands thrust magnitude. | apollo only. |
| `GUIDANCE_COEFFICIENTS_FIXED` (L132) | `True`/`False` | `True` | Compute linear/bilinear coeffs once vs. every update; `t_go` always recomputed each step. | linear/bilinear tangent only; gates `GUIDANCE_UPDATE_RATE`. |
| `GUIDANCE_TGO_USE_PSO_PLAN` (L140) | `True`/`False` | `False` | Use PSO-planned burn countdown for t_go instead of rocket-equation estimate. | **silently ignored** outside `pso_coast`/`direct(pso)`; excludes `peg_new`; affects apollo/linear/bilinear/cpr/peg. |
| `TGO_ESTIMATOR` | `rocket_equation`, `peg_new` | `rocket_equation` | t_go estimator for the scalar-t_go modes: gravity-blind rocket-equation vs. peg_new's gravity-aware estimate. | affects apollo/linear/bilinear/cpr(`"tgo"`); **excludes peg** (own T solver) and peg_new (source); cpr under `pso_coast` unaffected (PSO θ_dot). |
| `CPR_THETA_DOT_MODE` (L150) | `tgo`, `manual` | `manual` | How CPR's constant pitch rate is set. | cpr + **`apogee_check` only**; `manual` activates `CPR_THETA_DOT`. Under `pso_coast` the rate is the PSO var `PSO_COAST_CPR_THETA_DOT_*`. |
| `CPR_THETA_DOT` (L154) | float deg/s (rec. 0.1–0.5) | `0.4` | Manual CPR pitch rate (duration = 90°/rate). | cpr + `apogee_check` + `manual` only. |
| `PEG_MAJOR_LOOP_RATE` (L159) | float s | `2.0` | PEG major-loop A,B,T recompute period. | peg only. |
| `PEG_CONVERGENCE_MODE` (L161) | `damped`, `fixed_iter` | `damped` | PEG Guide+Estimate convergence method. | peg only; `damped` activates damping/tol. |
| `PEG_CONVERGENCE_DAMPING` (L167) | float ∈ (0,1] | `0.5` | Damping factor. | peg + `damped` only. |
| `PEG_CONVERGENCE_TOL` (L169) | float s | `0.5` | Convergence tolerance. | peg + `damped` only. |
| `PEG_CONVERGENCE_MAX_ITER` (L170) | int | `30` | Iteration cap (exact count for `fixed_iter`). | peg only. |

### 2.2 Kick maneuver / initial pitch-over

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `KICK_PROFILE_MODE` (L20) | `triangular`, `instantaneous` | `triangular` | Kick shape: ramped alpha-kick vs. discontinuous gamma jump. | honored **only under `apogee_check`**; **silently ignored** by all PSO solvers (always instantaneous, `rocket_ascent.py:2514`); switches kick-angle convention (alpha vs. gamma_p). |
| `TIME_TO_START_KICK` (L8) | float s | `7.5` | When the kick begins after liftoff. | all kick paths. |
| `DURATION_INITIAL_KICK` (L9) | float s | `45.` | Triangular ramp duration. | `KICK_PROFILE_MODE="triangular"` only. |
| `ALPHA_LOWEST` / `ALPHA_HIGHEST` (L210–211) | float rad | `-deg2rad(5.5)` / `-deg2rad(2.5)` | Kick-angle search bounds (triangular convention). | brute-force search + triangular only; **not** linked to the `[1.54,1.57]` gamma_p PSO bounds. |
| `MAX_ACCEPTED_BURN_TIME` (L212) | float s | `100.` | Max accepted delta-v burn time during search. | apogee_check/brute-force search. |
| `APOGEE_MATCH_TOL_FRAC` (L216) | float (fraction of r_target) | `0.0002` | Apogee-match acceptance tolerance. | `apogee_check` only. |
| `RUN_FAST` (L220) | `True`/`False` | `False` | Skip kick optimisation, use `OPTIMAL_KICK_ANGLES`. | `apogee_check` only; **silently ignored** under PSO paths; needs an entry in `OPTIMAL_KICK_ANGLES`. |
| `OPTIMAL_KICK_ANGLES` (L224) | dict {mode: rad} | per-mode (e.g. gravity_turn −3°, apollo −4.5°) | Pre-computed kick angles for fast mode. | `RUN_FAST=True`; **no entry for `cpr`/`indirect_pmp`** → falls back to `INITIAL_KICK_ANGLE`. |
| `INITIAL_KICK_ANGLE` (L237) | float rad | `-deg2rad(3.0)` | Manual single-run kick angle / fast-mode fallback. | single-run + `RUN_FAST` fallback. |

### 2.3 Coast / burn-arc structure

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `COAST_METHOD` (L329) | `apogee_check`, `pso_coast`, `direct` | `direct` | Top-level dispatcher for Stage-2 insertion structure. `direct` is always PSO (2-var, needs PyGMO). | **silently ignored** when `GUIDANCE_MODE="indirect_pmp"`; selects solver; gates `DIRECT_*`, `RUN_FAST`, `KICK_PROFILE_MODE` relevance; `pso_coast` adds extra PSO vars for `cpr`/`exp_shooting`; `direct` reaches orbit **only** for `apollo`/`peg`/`peg_new` (others → suborbital — solver warns). |
| `DIRECT_INSERTION_VELOCITY_TOL_MS` (L337) | float m/s | `10.0` | "Clean insertion" velocity tolerance. | `COAST_METHOD="direct"` only (else unused). |
| `DIRECT_INSERTION_FPA_TOL_DEG` (L338) | float deg | `0.5` | "Clean insertion" FPA tolerance. | `COAST_METHOD="direct"` only. |
| `DIRECT_INSERTION_ALTITUDE_TOL_KM` (L339) | float km | `5.0` | "Clean insertion" altitude tolerance. | `COAST_METHOD="direct"` only. |

### 2.4 Atmosphere-exit / guidance-start trigger

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `ATMOSPHERE_EXIT_METHOD` (L199) | `altitude`, `dynamic_pressure`, `aerothermal_flux` | `dynamic_pressure` | Criterion to detect atmosphere exit / guidance start. | Gates activation time for 5 of 7 guidance modes (not cpr/exp_shooting). Selects which one threshold below applies. |
| `ALT_NO_ATMOSPHERE` (L200) | float m | `65e3` | Altitude threshold. | `altitude` method only. |
| `DYNAMIC_PRESSURE_THRESHOLD` (L202) | float Pa | `1000.0` | Dynamic-pressure threshold. | `dynamic_pressure` method only. |
| `AEROTHERMAL_FLUX_THRESHOLD` (L205) | float W/m² | `1135.0` | Aerothermal-flux threshold. | `aerothermal_flux` method only. |

### 2.5 Stage-1 engine performance

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `ISP_1_MODE` (L181) | `sea_level`, `vacuum`, `average`, `linear` | `sea_level` | Which Isp Stage 1 uses. | `linear` activates `ISP_1_LINEAR_UPDATE_RATE`; reads `ISP_1_SL`/`ISP_1_VAC` from `rocket_specs.py`. |
| `ISP_1_LINEAR_UPDATE_RATE` (L182) | float s | `5.0` | Isp ramp step interval. | `ISP_1_MODE="linear"` only. |
| `THRUST_1_MODE` (L191) | `sea_level`, `vacuum`, `average`, `linear` | `sea_level` | Which thrust Stage 1 uses. | `linear` activates `THRUST_1_LINEAR_UPDATE_RATE`; reads `F_THRUST_1_SL`/`_VAC`. |
| `THRUST_1_LINEAR_UPDATE_RATE` (L192) | float s | `5.0` | Thrust ramp step interval. | `THRUST_1_MODE="linear"` only. |

### 2.6 Aerodynamics / physics

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `INCLUDE_DRAG` (L51) | `True`/`False` | `True` | Include aerodynamic drag (F_D = q·C_D·A) in the EOM. **Master no-atmosphere switch.** | `False` ⇒ no-atmosphere mode: lift also forced off, fairing **not carried** (launched without it, `M_FAIRING` dropped from launch mass), atmosphere exit forced to the **altitude** method. No guidance depends on atmosphere exit, so nothing else changes. |
| `INCLUDE_LIFT` (L55) | `True`/`False` | `True` | Include aerodynamic lift (F_L = q·C_L·A) in the EOM. | Reads `C_L` from `rocket_specs.py` (else `C_L` is inert). Only effective while `INCLUDE_DRAG=True`. |

### 2.7 Earth rotation / azimuth

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `ENABLE_EARTH_ROTATION` (L29) | `True`/`False` | `True` | Include Earth-rotation effects in azimuth/ECI. | Master gate for the entire pseudo-force/azimuth family below; changes reported frame and state-vector length. |
| `LAUNCH_LATITUDE` (L30) | float deg | `28.5` | Launch site latitude. | Feeds azimuth formula `sin(β)=cos(i)/cos(φ)`. |
| `LAUNCH_LONGITUDE` (L31) | float deg | `-80.5` | Launch site longitude (reserved; not yet used). | none currently. |
| `TARGET_ORBIT_INCLINATION` (L32) | float deg | `51.6` | Desired orbit inclination. | azimuth derivation + `AZIMUTH_INCLINATION_MODE`. |
| `INCLUDE_PSEUDO_FORCES` (L61) | `True`/`False` | `True` | Coriolis/centrifugal in rotating-frame EOM. | **requires** `ENABLE_EARTH_ROTATION`; required by the counter-force flag below. |
| `COMPUTE_CROSS_HEADING_COUNTER_FORCE` (L68) | `True`/`False` | `False` | Cross-heading actuator counter-force: heading held at the launch azimuth (assumed actuator-counteracted), so **no trajectory effect**; computes/stores/plots the per-step force `m·|a_cross|` [N]. | **requires** `ENABLE_EARTH_ROTATION` **and** `INCLUDE_PSEUDO_FORCES`. Single flag for the whole feature (former `INCLUDE_CROSS_HEADING_PSEUDO_FORCE` merged in; `TRACK_HEADING_STATE` removed). |
| `AZIMUTH_INCLINATION_MODE` (L55) | `formula_compare`, `formula_back_compare`, `iterative` | `formula_compare` | How launch azimuth is derived/analyzed. | `iterative` **force-overwritten** to `formula_compare` under `pso_coast` (`main.py:424`); only exercised in the legacy path otherwise. |
| `AZIMUTH_ITER_STEP_DEG` (L56) | float deg | `0.1` | Azimuth sweep step. | `iterative` only. |
| `AZIMUTH_ITER_RANGE_DEG` (L57) | float deg | `10.0` | Azimuth sweep half-width. | `iterative` only. |
| `AZIMUTH_ITER_TOL_DEG` (L58) | float deg | `0.05` | Inclination tolerance for the sweep. | `iterative` only. |

### 2.8 Target orbit / mission

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `TARGET_ORBITAL_ALTITUDE` (L26) | float m | `500e3` | Desired circular orbit altitude. | Sets r_target used by every guidance law, the `direct` MECO trigger, and apogee_check acceptance. |

### 2.9 Optimizer — `indirect_pmp` PSO (only when `GUIDANCE_MODE="indirect_pmp"`)

> Requires PyGMO — `indirect_pso_solver.py:687` raises `ImportError` if absent (no scipy fallback).

| Variable | Allowed values | Default | Controls |
|---|---|---|---|
| `PSO_N_PARTICLES` (L261) | int | `250` | Swarm size. |
| `PSO_MAX_GENERATIONS` (L262) | int | `500` | Max generations. |
| `PSO_C1` (L263) | float | `2.05` | Cognitive parameter. |
| `PSO_C2` (L264) | float | `2.05` | Social parameter. |
| `PSO_OMEGA` (L265) | float | `0.7298` | Inertia weight. |
| `PSO_VMAX` (L266) | float | `0.5` | Max normalized particle velocity. |
| `PSO_SEED` (L267) | int | `42` | RNG seed. |
| `PSO_LB` / `PSO_UB` (L271–272) | list[7] floats | `[-1,-1,-1,0,0,0,1.54]` / `[1,1,1,2000,100,100,1.57]` | Bounds for `[λ0_r, λ0_v, λ0_γ, Δt_c, Δt_r%, coast_start%, γ_p]`. |
| `PENALTY_W_J` (L286) | float | `1.0` | Burn-time term weight. |
| `PENALTY_W_ALTITUDE` (L287) | float | `100.0` | Altitude-error penalty. |
| `PENALTY_W_VELOCITY` (L288) | float | `100.0` | Velocity-error penalty. |
| `PENALTY_W_FPA` (L289) | float | `10.0` | FPA-error penalty. |
| `PENALTY_W_TRANSVERS` (L290) | float | `10.0` | Transversality penalty (needs ‖λ₀‖=1). |
| `GAMMA_REF_DEG` (L291) | float deg | `1.0` | FPA non-dimensionalization reference. |

### 2.10 Optimizer — `direct` PSO (only when `COAST_METHOD="direct"`)

> Requires PyGMO — `direct_pso_solver.py:295` raises `ImportError` if absent.

| Variable | Default | Notes |
|---|---|---|
| `PSO_DIRECT_N_PARTICLES` (L351) | `50` | Swarm size. |
| `PSO_DIRECT_MAX_GENERATIONS` (L352) | `100` | Max generations. |
| `PSO_DIRECT_C1`/`C2`/`OMEGA`/`VMAX`/`SEED` (L353–357) | `2.05`/`2.05`/`0.7298`/`0.5`/`42` | Standard PSO hyperparameters. |
| `PSO_DIRECT_LB` / `PSO_DIRECT_UB` (L360–361) | `[1.54, 50.0]` / `[1.57, 100.0]` | Bounds for `[γ_p (rad), t_burn% of T_MAX_2]`. |
| `PSO_DIRECT_W_J`/`W_ALTITUDE`/`W_VELOCITY`/`W_FPA` (L365–368) | `1.0`/`100.0`/`100.0`/`10.0` | Objective penalty weights (4-term, no transversality). |
| `PSO_DIRECT_GAMMA_REF_DEG` (L369) | `1.0` | FPA non-dimensionalization reference [deg]. |

### 2.11 Optimizer — `pso_coast` PSO (only when `COAST_METHOD="pso_coast"`)

> Requires PyGMO — `pso_coast_solver.py:843` raises `ImportError` if absent.

| Variable | Default | Notes |
|---|---|---|
| `PSO_COAST_N_PARTICLES` (L373) | `100` | Swarm size. |
| `PSO_COAST_MAX_GENERATIONS` (L374) | `250` | Max generations. |
| `PSO_COAST_C1`/`C2`/`OMEGA`/`VMAX`/`SEED` (L375–379) | `2.05`/`2.05`/`0.7298`/`0.5`/`42` | Standard PSO hyperparameters. |
| `PSO_COAST_LB` / `PSO_COAST_UB` (L385–386) | `[0, 50, 0, 1.54]` / `[1000, 100, 100, 1.57]` | Bounds for `[Δt_c, Δt_r%, coast_start%, γ_p]`. |
| `PSO_COAST_W_J`/`W_ALTITUDE`/`W_VELOCITY`/`W_FPA` (L394–397) | `1.0`/`100.0`/`100.0`/`10.0` | Objective penalty weights (4-term, no transversality). |
| `PSO_COAST_GAMMA_REF_DEG` (L398) | `1.0` | FPA non-dimensionalization reference [deg]. |

### 2.12 Numerical / output

| Variable | Allowed values | Default | Controls | Tangles with |
|---|---|---|---|---|
| `TIME_STEP` (L243) | float s | `0.01` | Output sampling for `t_eval` (integration itself adaptive). | none. |
| `DURATION_AFTER_SIMULATION` (L245) | float s | `1000.` | Extra propagation after reaching orbit. | none. |
| `INTERRUPTS_PRINT` (L251) | `True`/`False` | `False` | Print ODE-interrupt debug. | none. |
| `EVENTS_PRINT` (L252) | `True`/`False` | `True` | Print mission-event log lines. | none. |

### 2.13 Vehicle / staging constants (`Auxiliary/rocket_specs.py`)

A single fixed (Falcon-9-like) two-stage vehicle. These are plain constants you edit directly — there
is **no registry and no stage-count switch**. Derived ratios (`M_TOTAL_*`, `LAMBDA_*`, `EPSILON_*`,
`PI_*`, L76–98) are computed, not chosen.

| Variable | Line | Default | Controls |
|---|---|---|---|
| `M_PAYLOAD` | 24 | `0e3` | Payload mass [kg]. |
| `M_FAIRING` | 27 | `1900` | Fairing mass [kg], jettisoned at atmosphere exit. |
| `TIME_First_STAGE_SEPARATION` | 32 | `3` | Stage separation delay after MECO [s]. |
| `TIME_SECOND_ENGINE_IGNITION` | 33 | `8` | Stage-2 ignition delay after MECO [s]. |
| `A` | 36 | `10.52` | Cross-sectional area [m²]. |
| `C_D` | 37 | `0.3` | Drag coefficient. |
| `C_L` | 38 | `0.1` | Lift coefficient — used only if `INCLUDE_LIFT=True`. |
| `ISP_1_SL` / `ISP_1_VAC` | 45–46 | `283` / `311` | Stage-1 sea-level / vacuum Isp [s] — selected by `ISP_1_MODE`. |
| `F_THRUST_1_SL` / `F_THRUST_1_VAC` | 48–49 | `7607e3` / `8227e3` | Stage-1 sea-level / vacuum thrust [N] — selected by `THRUST_1_MODE`. |
| `M_STRUCTURE_1` / `M_PROP_1` | 53–54 | `25.6e3` / `395.7e3` | Stage-1 structure / propellant mass [kg]. |
| `ISP_2` | 62 | `348` | Stage-2 Isp [s]. |
| `F_THRUST_2` | 63 | `934e3` | Stage-2 thrust [N]. |
| `M_STRUCTURE_2` / `M_PROP_2` | 66–67 | `3900` / `92670` | Stage-2 structure / propellant mass [kg]. |

**Body constants (NOT user choices)** — `Auxiliary/constants.py` fixes `G_0=9.81`, `R_EARTH=6378e3`,
`MU_EARTH=3.986004418e14`, `OMEGA_EARTH=7.2921159e-5`, `RHO_0=1.225`, `H=8500`. Earth-only; there is
no planet selector.

---

## 3. Master compatibility matrix — `GUIDANCE_MODE` × `COAST_METHOD`

`indirect_pmp` is a separate world (it ignores `COAST_METHOD`), so it occupies its own column. Cells
note the governing `file:line`.

**Verdicts below are empirical** — confirmed by a full (guidance × coast) sweep, with under-converged
cells re-run at higher PSO budget to separate "needs more budget" from "structurally can't get there."

| GUIDANCE_MODE | `apogee_check` | `pso_coast` | `direct` (always PSO) |
|---|---|---|---|
| `gravity_turn` | OK | OK | ✗ **suborbital** — see note |
| `linear_tangent` | OK | OK | ✗ **suborbital** — see note |
| `bilinear_tangent` | OK | OK | ✗ **suborbital** — see note |
| `apollo` | ✗ **raises `ValueError`** — incompatible (`main.py`, apogee_check branch); use `peg_new` here, or `apollo` under `direct`/`pso_coast` | OK | OK |
| `cpr` | OK — kick forced to 0 (`main.py:761`) | OK — gamma_p kick + PSO `θ_dot` (5th var); Stage-1 branch gated off (`_IN_PSO_STAGE1`) | ✗ **suborbital** — see note |
| `peg` | OK | OK | OK |
| `peg_new` | OK | OK | OK |
| `exp_shooting` | OK | OK — PSO optimises `a, b` (5th/6th vars), re-epoched per arc | ✗ **suborbital** — see note |

`indirect_pmp`: ✅ only via its own branch (`main.py:292`); `COAST_METHOD` has **no effect**.
Requires PyGMO. **Needs a large PSO budget** (the production default `250×500`) — a reduced-budget run
leaves it far from a closed orbit (it is convergence-limited, not broken).

> **The `direct` column "✗ suborbital" note.** `COAST_METHOD="direct"` is a *single continuous Stage-2
> burn with no coast* and only **2** PSO knobs (`gamma_p`, burn %). Reaching the target circular orbit
> that way is delta-v-marginal, so it closes **only** for the explicit terminal-constraint laws that
> fly the near-optimal lofting steering — **`apollo`, `peg`, `peg_new`**. For `gravity_turn`,
> `linear_tangent`, `bilinear_tangent`, `cpr`, `exp_shooting` the PSO converges (the result is
> **identical at 900 and 5000 evaluations** → a true optimum, not under-convergence) to a **suborbital**
> insertion (periapsis below the surface). More budget does **not** help; the fix is to use a coast —
> i.e. `pso_coast` or `apogee_check` — for those laws. `direct_pso_solver` prints a warning when paired
> with a non-`{apollo,peg,peg_new}` law.

**PyGMO requirement** applies to all three PSO paths: `indirect_pmp`, `pso_coast`, and `direct` each
raise `ImportError` without it (`indirect_pso_solver.py:687`, `pso_coast_solver.py:843`,
`direct_pso_solver.py:295`). Only `apogee_check` does not need PyGMO.

---

## 4. Gotchas — tangles that bite (silent no-ops & known issues)

Each is legal to set but does something other than what you'd expect. With `file:line`.

- **`KICK_PROFILE_MODE="triangular"` is a silent no-op under any PSO path.** `run_stage1()` always
  calls the instantaneous γ-jump path (`Simulation/rocket_ascent.py:2514`); only the legacy `run()`
  honors the flag (`rocket_ascent.py:1062`, `:1887`). So under `pso_coast` / `direct` /
  `indirect_pmp` the triangular ramp never happens — it only matters for `apogee_check`.

- **`cpr` is physically different depending on `COAST_METHOD`.** The initial pitch angle θ₀ is
  hardcoded to π/2 (vertical) in the legacy `apogee_check` path (`rocket_ascent.py:1076`) but set to
  the *current* flight-path angle γ at guidance start in the PSO path (`pso_coast_solver.py`). Under
  `pso_coast`, θ_dot is a PSO decision variable (not `CPR_THETA_DOT`), and the legacy Stage-1 cpr
  branch is gated off (`_IN_PSO_STAGE1`) so cpr flies the normal gamma_p kick — this fixed the former
  `brentq` Stage-1 crash. Same `GUIDANCE_MODE="cpr"`, different θ₀ → different trajectory per path.

- **`GUIDANCE_TGO_USE_PSO_PLAN` only affects the PSO solvers and skips `peg_new`.** It has no effect
  in the legacy `run()` (always uses the rocket-equation t_go estimate); inside the PSO solvers it
  affects apollo/linear/bilinear/cpr/peg but explicitly not `peg_new`.

- **`indirect_pmp` (and every PSO path) hard-requires PyGMO.** No scipy fallback despite docstring
  wording; missing PyGMO raises `ImportError` (`indirect_pso_solver.py:687`,
  `pso_coast_solver.py:843`, `direct_pso_solver.py:295`).

- **`DIRECT_INSERTION_*` tolerances are meaningless outside `COAST_METHOD="direct"`** — they only
  grade the "clean insertion" check inside the direct path.

- **`RUN_FAST` is inert under PSO paths and has no `cpr`/`indirect_pmp` entry.** It only short-circuits
  the `apogee_check` branch (`main.py:767`); for an unlisted mode it silently falls back to
  the generic `INITIAL_KICK_ANGLE`.

- **`AZIMUTH_INCLINATION_MODE="iterative"` is force-overwritten to `"formula_compare"` under
  `pso_coast`** (re-running the full PSO per azimuth is too costly) — the config object is mutated at
  runtime (`main.py:424`). Under other PSO paths it is simply never exercised.

- **Cross-heading counter-force is a pure diagnostic.** With the heading held at the launch azimuth
  (the actuator is assumed to cancel the lateral cross-heading pseudo-force), it has **no effect on the
  trajectory**. `COMPUTE_CROSS_HEADING_COUNTER_FORCE` is the single flag governing it: when True the
  per-step counter-force `m·|a_cross|` [N] is computed, stored and plotted; when False nothing is
  computed. (The former `INCLUDE_CROSS_HEADING_PSEUDO_FORCE` and `TRACK_HEADING_STATE` flags were
  removed — heading is no longer propagated as an ODE state.)

- **The default config (`indirect_pmp`) makes most of §2.3/§2.2 inert.** Out of the box,
  `COAST_METHOD="direct"`, `KICK_PROFILE_MODE`, `RUN_FAST`, and the `DIRECT_*` settings are ignored
  until you change `GUIDANCE_MODE` away from `indirect_pmp`.
