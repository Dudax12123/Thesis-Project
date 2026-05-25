# Handoff — Thesis Project (Rocket Trajectory Optimization)

## Project at a glance

Python aerospace simulator that compares 10 ascent guidance laws for a Falcon-9-class two-stage rocket targeting a **500 km circular orbit at 51.6° inclination** from Cape Canaveral (28.5° N). The repository contains the full ODE integrator, drag/atmosphere/gravity models, fairing-jettison logic, Earth-rotation handling, and a metric-plot suite.

Recent work has focused exclusively on `GUIDANCE_MODE = "pso_paper"` — an indirect-method (Pontryagin Minimum Principle) trajectory optimizer driven by Particle Swarm Optimization, faithful to Morgado, Marta & Gil (SMO 2022).

## Where to start

| What you want to do | File / command |
|---|---|
| Configure the run | [Tese/src/Input_File/simulation_parameters.py](Tese/src/Input_File/simulation_parameters.py) |
| Run the simulation | `cd Tese/src && python main.py` |
| Run unit tests | `cd Tese/src && pytest tests/test_pso_paper.py -v` |
| Understand the EOM | [Tese/src/Simulation/rocket_ascent.py](Tese/src/Simulation/rocket_ascent.py) `diff_eom_base` (~line 1488) |
| PSO outer loop | [Tese/src/Simulation/pso_paper_solver.py](Tese/src/Simulation/pso_paper_solver.py) |
| PMP math (costate ODE, steering, Hamiltonian) | [Tese/src/Guidance/pso_paper_guidance.py](Tese/src/Guidance/pso_paper_guidance.py) |
| The reference paper | Morgado, Marta, Gil — SMO 65:192 (2022) — `https://doi.org/10.1007/s00158-022-03285-y` |

---

## `pso_paper` mode — design overview

### 7-dim design vector (PSO searches over)

| index | name | bounds | meaning |
|---|---|---|---|
| 0 | `λ_h0` | [-1, 1] | initial costate (altitude) |
| 1 | `λ_V0` | [-1, 1] | initial costate (velocity) |
| 2 | `λ_γ0` | [-1, 1] | initial costate (flight-path angle) |
| 3 | `γ_p` | [1.30, 1.57] rad | initial pitch after instantaneous pitch-over |
| 4 | `Δt_c` | [0, 1500] s | mid-burn coast duration |
| 5 | `coast_pct` | [0, 1] | fraction of Stage-2 thrust time *before* coast |
| 6 | `burn_pct` | [0.70, 0.95] | fraction of `M_PROP_2 / ṁ` to actually burn |

### Trajectory timeline

| t (s) | event |
|---|---|
| 0 – 7.5 | vertical liftoff (γ frozen at 90° via `time_kick_start = None` guard) |
| 7.5 | **instantaneous γ → γ_p**; `pitch_program_linear` fires; gravity turn unfreezes |
| 7.5 – ~144 | gravity-turn Stage 1 (`α = 0`) |
| ~144 | MECO |
| ~152 | Stage 2 ignition → costate ODE starts integrating; PMP steering active |
| ~152 + `coast_pct · Δt_T` | engine cut → mid-burn coast begins |
| coast end | engine re-ignites → terminal impulse |
| `pso_paper_seco_t` | commanded SECO; PSO objective sampled here |

Total Stage-2 active thrust time `Δt_T = burn_pct · M_PROP_2 / ṁ ≈ burn_pct · 338.6 s`.

### Objective (J')

```
J' = j_impulse_frac
     + s_alt   · |h_f − h_T| / h_T              alt fractional error
     + s_vel   · |v_f − V_T| / V_T              vel fractional error
     + s_sma   · |a_f − r_T| / r_T              semi-major-axis fractional error
     + s_ecc   · e_f                             eccentricity (target 0)
     + s_gamma · |γ_f| / (π/2)                  γ fractional error
     + s_ham   · |H_residual|                   transversality residual (paper eq. 38)
     + 1e6  if (crash | NaN | escape | Stage-2 never ignited)
```

All five state-quality terms are normalised to [0, 1] range and weighted at `s = 1e3` each. `j_impulse_frac` is the terminal-burn duration normalised by `t_max_S2`. `V_T` is the **rotating-frame** circular orbital velocity at 500 km (≈ 7 204 m/s = inertial 7 612 − surface rotation 408).

### Key implementation rules
- **Pseudo-forces are auto-disabled** in `pso_paper` mode regardless of the `INCLUDE_PSEUDO_FORCES` flag — gated in `diff_eom`.
- **State vector** is extended with `[λ_h, λ_V, λ_γ]` at the end, located via `_paper_costate_offset(state_len)`.
- **Costates are frozen** before Stage-2 ignition (`dλ/dt = 0`). They evolve via paper eq. (30) only when `guidance_phase_active`.
- **Coast arc**: `F_T = 0` is passed to `costate_derivatives` so the thrust term drops out; gravity-only costate evolution.
- **Earth rotation**: rocket starts with `v = 0` (rotating frame). The `V_T` target subtracts `LAUNCH_ROTATION_SPEED` so the PSO is not penalised for the free ΔV. No frame conversions performed in the objective.

---

## Recent change log (summary of what was done)

1. Costate ODE + state-vector extension wired into the simulator.
2. PSO (pyswarms `GlobalBestPSO`) wraps `ra.run()` with `SINGLE_BURN_FULL_SIMULATION = False` per particle.
3. PSO target velocity adjusted: `V_T = √(μ/r_T) − LAUNCH_ROTATION_SPEED` (rotating-frame consistency).
4. Pitch-over moved from 3 s to 7.5 s — eliminates the `1/V`-singularity crash attractor at low velocity.
5. Fairing-jettison guard: requires `v > 200 m/s` in paper mode (prevents low-altitude false trigger at q ≈ 980 Pa).
6. Pseudo-forces auto-disabled in `diff_eom` when paper mode is active; pseudo-force plots suppressed in `new_plot_runner.py`.
7. PROPELLANT USAGE / FINAL ORBITAL ELEMENTS print blocks in `main.py` now print paper-mode–specific text (no more 9999999 sentinels, no "kick angle = 0", no "Cross-heading: ON").
8. PSO hyperparameters tuned for more exploration: `w = 0.80` (was 0.7298), `c2 = 1.50` (was 2.05).
9. Hard penalty softened: `PENALTY_HARD = 1e6` (was 1e20) — gives usable gradients near the crash boundary.
10. Coast-duration upper bound tightened: `(0, 1500) s` (was `(0, 3000)`).
11. Post-PSO diagnostic `_diagnose_best` + `_print_diagnostic` added — prints SECO state, semi-major axis, eccentricity, and per-term penalty contributions.
12. Orbital-element terms `s_sma · a_err_frac` + `s_ecc · e_f` added to J' (kept alongside the existing alt/vel/γ terms, on user request).
13. Bug fixes from final audit:
    - `time_guidance_start` added to `global` declarations in `run()` reset block.
    - Hamiltonian `H_0_last` (re-ignition sample) now uses `F_THRUST_2`, not 0.
    - `idx_eval = -1` silent fallback replaced by explicit `PENALTY_HARD` return when SECO time is missing.
    - Unused `R_earth` parameter dropped from `hamiltonian()` signature.

---

## Known open issue

**The PSO still converges to a sub-optimal trajectory.** Latest best (250 particles × 1000 iterations):
- Cost ≈ 246 (with old objective; new orbital-elements objective will push this higher to ~950)
- SECO altitude 504 km (≈ target) but velocity 8 371 m/s vs target 7 204 m/s
- Real orbit: a = 9 622 km (target 6 871), e = 0.297, apogee 6 102 km, periapsis 387 km

The trajectory hits 500 km altitude on the *ascending* leg of a highly elliptical orbit rather than at apogee. The newly-added orbital-element penalties should give the swarm a much stronger gradient away from this local minimum on the next long run. If it still stagnates, candidates for further work:
- Warm-start the PSO with a hand-tuned costate vector (currently every particle starts uniformly random in [-1, 1]³).
- Replace `GlobalBestPSO` with `LocalBestPSO` (ring topology) to maintain swarm diversity longer.
- Multi-start: run several short PSOs from different random seeds and keep the best.
- Tighten `coast_pct` lower bound away from 0 to force a meaningful pre-coast burn.

---

## Files of interest

```
Tese/src/
├── main.py                         entry point; dispatches by GUIDANCE_MODE
├── Input_File/
│   └── simulation_parameters.py    ALL user knobs (PSO_PAPER_* block at ~line 230)
├── Simulation/
│   ├── rocket_ascent.py            ODE system, schedule setup, interrupts
│   └── pso_paper_solver.py         PSO outer loop + objective + diagnostic
├── Guidance/
│   └── pso_paper_guidance.py       costate ODE, steering law, Hamiltonian (paper-faithful)
├── Plots/
│   ├── new_plot_runner.py          plot dispatcher (skips pseudo-force plots in paper mode)
│   └── new_metrics/                26 per-metric plot scripts
├── Auxiliary/
│   ├── constants.py                MU_EARTH, R_EARTH, G_0, OMEGA_EARTH
│   ├── rocket_specs.py             Falcon-9 numbers (mass, Isp, thrust)
│   ├── atmosphere.py               exponential density model
│   ├── gravity.py                  inverse-square
│   └── earth_rotation.py           ECEF↔ECI, surface rotation, launch azimuth
└── tests/
    ├── test_pso_paper.py           PMP math unit tests (11 tests, all passing at 1e-12)
    └── test_apollo_tgo.py          legacy Apollo-mode tests
```

---

## How to reproduce the most recent PSO run

In `simulation_parameters.py`:
- `GUIDANCE_MODE = "pso_paper"`
- `TARGET_ORBITAL_ALTITUDE = 500e3`
- `PSO_PAPER_POPULATION = 250`
- `PSO_PAPER_ITERATIONS = 1000`
- `ENABLE_EARTH_ROTATION = True`
- `INCLUDE_PSEUDO_FORCES = False` (auto-overridden anyway, but set for clarity)

Then `cd Tese/src && python main.py`. Expect ~2 hours wall time on a modern desktop. Watch the BEST PARTICLE DIAGNOSTIC block that prints after PSO completes — that is the most informative single output for understanding why the swarm settled where it did.

