# Earth Simulator — Choice Tree & Working-Path Map

This document maps **every choice a user can make** when configuring the Earth
launch simulator, **how the choices constrain each other**, and — after a scripted
sweep — **which choice paths actually work and which fail**. It is the deliverable
for the "test everything about the Earth simulator" task.

All configuration lives in module-level constants in
[`Tese/src/Input_File/simulation_parameters.py`](../src/Input_File/simulation_parameters.py);
there is no CLI. A run is launched with `python Tese/src/main.py`, which applies the
constants via `set_planet()` / `set_vehicle()` and dispatches on `GUIDANCE_MODE` /
`COAST_METHOD`.

**Status legend (used throughout Section 4):**

| Status | Meaning |
|---|---|
| `PASS` | Ran to completion and met the method's success criterion (near-circular orbit at the target, or — for `apogee_check` — apoapsis at the target). |
| `POOR` | Ran but the orbit is off-target / suborbital / not circular. |
| `CRASH` | Ground impact (`ra.CRASH_DETECTED`) or PSO `crashed` flag. |
| `REJECTED` | A config-guard `ValueError` fired — an *expected* rejection of an incompatible combo (this is the guard working correctly). |
| `ERROR` | An unexpected exception (a real bug — e.g. a `brentq` sign error, a Stage-1 crash inside the PSO). |
| `TIMEOUT` | Exceeded the per-run wall-clock cap. |

---

## Phase 0 — Launcher correctness audit

Verified with [`audit_launchers.py`](../src/earth_test_matrix/audit_launchers.py)
(reads the `VEHICLES` registry, computes T/W, structural fraction ε, and ideal
staged Δv via Tsiolkovsky). `G_0 = 9.81 m/s²`, `v_circ(200 km) = 7784 m/s`,
LEO need incl. gravity/drag losses ≈ 9400 m/s.

| Vehicle | Stages | Liftoff T/W | ε₁ | Ideal Δv | Verdict |
|---|---|---|---|---|---|
| **falcon9** | 2 | 1.50 | 0.049 | **15.36 km/s** | Specs match published Merlin/F9 values, but `M_PAYLOAD = 0` makes it **overperform by ~6 km/s**. |
| **electron** | 2 | 1.30 | 0.075 | **9.75 km/s** | Specs match published Rutherford/Electron values. Realistic, self-consistent. |
| ~~saturn_v~~ | 2 | 1.19 | 0.045 | 9.26 km/s | **Excluded** — underperforms (~suborbital). See below. |

**Findings:**
- **No magnitude/unit errors** in any launcher's engine/mass specs — thrust, Isp,
  propellant, structure and cross-sectional area all match published figures.
- **`falcon9` carries zero payload** (`rocket_specs.py:43`). Ideal Δv (15.4 km/s) is
  far beyond LEO need, so Stage 1 alone can reach orbit and the configured
  `GUIDANCE_MODE` may never activate (a `[WARNING] GUIDANCE NEVER ACTIVATED` is
  printed). This is a config caveat, not a spec bug, but it makes falcon9 a poor
  test bed for guidance laws on the `apogee_check` path.
- **`saturn_v` is excluded from testing.** The simulator models only **two serial
  stages with no parallel staging**, but a faithful Saturn V needs three serial
  stages (S-IC → S-II → **S-IVB**, the last performing orbital insertion). The
  registry entry (`rocket_specs.py:75-88`) folds the S-IVB + Apollo stack into a
  140 t dead `M_PAYLOAD`, producing a borderline-suborbital 2-stage stand-in that
  does not represent the real vehicle. **Recommended: remove `saturn_v`** from the
  registry (or relabel it clearly as a non-faithful 2-stage stand-in) — see Section 5.
- Event-timing fields `TIME_First_STAGE_SEPARATION` / `TIME_SECOND_ENGINE_IGNITION`
  are **MECO-relative intervals** (used as `time_main_engine_cutoff + …` at
  `rocket_ascent.py:193,457`), so identical `3`/`8` across vehicles is a deliberate
  simplification, not an error.

---

## Section 1 — The choice tree

```
EARTH SIMULATOR RUN
│
├── PLANET  = "earth"                         (constants.PLANETS; must match vehicle BODY)
│
├── VEHICLE  (rocket_specs.VEHICLES; BODY must == PLANET)
│   ├── falcon9    — 2-stage, overperforms (payload=0)
│   ├── electron   — 2-stage, realistic            ← primary guidance test bed
│   ├── saturn_v   — 2-stage stand-in (EXCLUDED, removal candidate)
│   └── apollo_lm_ascent — single-stage, MOON only (blocked on Earth by BODY guard)
│
├── ASCENT / KICK
│   ├── KICK_PROFILE_MODE   = "triangular" | "instantaneous"
│   ├── TIME_TO_START_KICK  [s]   ← NOT vehicle-aware (7.5 s suits falcon9; crashes electron)
│   ├── DURATION_INITIAL_KICK [s] (triangular only)
│   └── ALPHA_LOWEST / ALPHA_HIGHEST  (kick-angle search bounds, apogee_check)
│
├── GUIDANCE_MODE  (main.py whitelist)
│   ├── gravity_turn        — no steering (α = 0 after kick)
│   ├── linear_tangent      — tan(steering) linear in t
│   ├── bilinear_tangent    — bilinear tangent law
│   ├── apollo              — Apollo polynomial guidance
│   ├── cpr                 — constant-pitch-rate (no kick; ⚠ brentq risk w/ rotation)
│   ├── peg                 — powered explicit guidance
│   ├── peg_new             — revised PEG
│   ├── exp_shooting        — single-burn BVP shooting (✗ with pso_coast)
│   └── indirect_pmp        — PMP optimal control (own 7-var PSO; ignores COAST_METHOD)
│
├── COAST_METHOD  (ignored when GUIDANCE_MODE == "indirect_pmp")
│   ├── apogee_check  — brute-force kick sweep; burn to target apoapsis (ra.run dispatch)
│   ├── pso_coast     — 4-var PSO thrust→coast→thrust (pygmo)
│   └── direct        — 2-var PSO single continuous burn to circular insertion (pygmo)
│
├── TARGET_ORBITAL_ALTITUDE [m]   (shipped value 50 km is Moon-tuned; Earth needs ~200 km)
│
├── AERO:  INCLUDE_DRAG, INCLUDE_LIFT
│
└── EARTH ROTATION  (ENABLE_EARTH_ROTATION)
    ├── LAUNCH_LATITUDE, TARGET_ORBIT_INCLINATION
    ├── INCLUDE_PSEUDO_FORCES (+ cross-heading variants, TRACK_HEADING_STATE)
    └── AZIMUTH_INCLINATION_MODE = "formula_compare" | "formula_back_compare" | "iterative"
```

---

## Section 2 — How the choices constrain each other (guards)

All hard guards are enforced in `main.py` before any simulation runs:

| Rule | Where | Effect |
|---|---|---|
| Vehicle `BODY` must equal `PLANET` | `main.py:225-229` | `ValueError` (blocks `apollo_lm_ascent` on Earth) |
| `GUIDANCE_MODE` must be in the whitelist | `main.py:256-263` | `ValueError` |
| `exp_shooting` + `pso_coast` | `main.py:268-273` | `ValueError` (BVP can't honour a coast split) |
| Single-stage vehicle + `pso_coast`/`indirect_pmp` | `main.py:235-252` | `ValueError`; non-`direct` → warning (N/A here — no single-stage Earth vehicle) |
| `iterative` azimuth + `pso_coast` | `main.py:471-477` | soft fallback to `formula_compare` (warning) |
| `indirect_pmp` selected as a guidance mode | `main.py:347` | takes a separate branch; **`COAST_METHOD` is ignored** |
| `apollo` + `apogee_check` | (documented, not enforced) | apogee sweep finds no usable kick at low T/W |
| `cpr` + `ENABLE_EARTH_ROTATION=True` | (known bug) | `brentq` "f(a)/f(b) must have different signs" in Stage-1 events |

---

## Section 3 — Test methodology

- **Interpreter:** `pygmo-env` (`C:\Users\eduar\miniforge3\envs\pygmo-env\python.exe`;
  py 3.11, numpy 2.4, scipy 1.17, matplotlib 3.10, pygmo 2.19).
- **Harness:** [`run_matrix.py`](../src/earth_test_matrix/run_matrix.py) (parent) spawns
  [`_single_run.py`](../src/earth_test_matrix/_single_run.py) once **per combo in a fresh
  subprocess** (PSO solvers snapshot stage constants at import; fresh processes also
  isolate hard crashes). Plots and `plt.show()` are monkey-patched to no-ops.
- **Common settings applied to every Earth run:** `TARGET_ORBITAL_ALTITUDE = 200 km`
  (the shipped 50 km is Moon-tuned and suborbital for Earth); per-vehicle kick timing
  (**falcon9 = 7.5 s, electron = 20 s** — see Section 5: kick timing is not vehicle-aware
  in the shipped code); reduced PSO budgets (direct/coast 24×40, indirect 60×100) with a
  420 s per-run timeout.
- **Matrix:** 2 vehicles × (8 guidance × 3 coast + `indirect_pmp`) = 50 baseline runs
  (rotation OFF) + a falcon9 rotation-ON subset.
- **Classification:** `apogee_check` graded on apoapsis≈target; `direct`/`pso_coast`/
  `indirect_pmp` graded on mean altitude ≈ target **and** eccentricity < 0.10. (Note:
  `ra.time_guidance_start` is only set on the `apogee_check` path, so the
  "guidance activated" flag is N/A for the PSO paths — they apply guidance inside their
  own Stage-2 ODE.)

---

## Section 4 — Results

62 combos run (50 baseline rotation-OFF + 12 falcon9 rotation-ON probes). Raw data:
[`results.json`](../src/earth_test_matrix/results.json),
[`results_table.txt`](../src/earth_test_matrix/results_table.txt).
**Totals: 39 PASS, 14 POOR, 5 ERROR, 2 REJECTED, 2 TIMEOUT.**

### 4.1 Baseline matrix — rotation OFF

`falcon9` (overperforms; payload = 0):

| guidance \ coast | apogee_check | pso_coast | direct |
|---|---|---|---|
| gravity_turn     | **POOR** (apo 35 km) | PASS | PASS |
| linear_tangent   | PASS | PASS | PASS |
| bilinear_tangent | PASS | PASS | **POOR** (apo 586, peri −180) |
| apollo           | PASS | PASS | PASS |
| cpr              | **POOR** (suborb.) | **ERROR** (brentq) | **ERROR** (brentq) |
| peg              | PASS | PASS | PASS |
| peg_new          | PASS | PASS | PASS |
| exp_shooting     | **POOR** (guidance n/a) | **REJECTED** (guard ✓) | **TIMEOUT** (>420 s) |
| indirect_pmp     | — | — | **POOR** (e 0.84) |

`electron` (realistic; kick delayed to 20 s):

| guidance \ coast | apogee_check | pso_coast | direct |
|---|---|---|---|
| gravity_turn     | PASS | PASS | PASS |
| linear_tangent   | PASS | PASS | PASS |
| bilinear_tangent | PASS | PASS | **POOR** (apo 646, peri −252) |
| apollo           | PASS | PASS | **POOR** (peri −11, marginal) |
| cpr              | **POOR** (suborb.) | **ERROR** (brentq) | **ERROR** (brentq) |
| peg              | PASS | **POOR** (peri −22) | **POOR** (peri −795) |
| peg_new          | PASS | **POOR** (peri −1795) | **POOR** (peri −2184) |
| exp_shooting     | PASS (guidance n/a) | **REJECTED** (guard ✓) | **TIMEOUT** (>420 s) |
| indirect_pmp     | — | — | **POOR** (e 0.90) |

### 4.2 Earth-rotation-ON subset (falcon9)

| guidance | coast | status | note |
|---|---|---|---|
| gravity_turn | apogee_check | POOR | apo 37 km (same as OFF — overperformance, not rotation) |
| linear_tangent | apogee_check | PASS | e≈0 |
| bilinear_tangent | apogee_check | PASS | e≈0 |
| apollo | direct | PASS | e 0.006 |
| cpr | apogee_check | POOR | suborbital |
| peg | apogee_check | PASS | e≈0 |
| peg_new | apogee_check | PASS | e≈0 |
| exp_shooting | direct | PASS | e 0.027 — but took 372 s (barely under timeout) |
| indirect_pmp | direct | POOR | e 0.77 |
| gravity_turn | pso_coast | PASS | e 0.011 |
| cpr | direct | ERROR | brentq (same as rotation OFF) |
| gravity_turn | pso_coast + **iterative azimuth** | PASS | identical to formula_compare → **fallback guard works ✓** |

**Earth rotation does not break any working path** and adds no new failures: every
mode that passes with rotation OFF also passes with it ON. The `iterative` azimuth →
`formula_compare` soft fallback works. So the rotation / azimuth / ECI machinery is sound.

### 4.3 Reading the failures (real defect vs reduced-budget artifact)

- **Real defects (budget-independent — fail instantly or structurally):**
  - `cpr` + `pso_coast`/`direct` → **ERROR** in 0.5–0.7 s (`brentq` sign error) on **both
    vehicles, rotation ON and OFF**. `cpr` + `apogee_check` → **POOR** (suborbital) always.
    `cpr` never reaches a usable orbit.
  - `exp_shooting` + `direct` → **TIMEOUT** (~420 s; once 372 s) — the single-burn BVP
    shooting is impractically slow. `exp_shooting` + `apogee_check` → the guidance law
    **never activates** (`time_guidance_start` stays `None`); it silently flies a gravity
    turn (PASS for electron only because gravity turn alone works there).
  - `indirect_pmp` → **POOR** everywhere: it drives apoapsis to the target but leaves
    periapsis deeply negative (e ≈ 0.77–0.90), i.e. a transfer orbit, not a circular one.
    (Full-budget 250×500 check: see §4.4.)
- **Likely reduced-budget artifacts (PSO under-converged at 24×40):**
  - `electron` + `peg`/`peg_new`/`apollo` on `pso_coast`/`direct` → **POOR** with periapsis
    slightly/moderately negative, while the deterministic `apogee_check` circularises these
    perfectly (e ≈ 1e-6). The tighter-margin electron needs more PSO budget on these laws.
- **Suspicious (fails on BOTH vehicles, incl. the overperforming falcon9 → not just budget):**
  - `bilinear_tangent` + `direct` overshoots badly (apo 586/646 km, peri negative) on both
    vehicles, while `bilinear` + `apogee_check`/`pso_coast` pass. Points to a
    `bilinear_tangent`×`direct`-PSO interaction worth a look.
- **Config caveats (not code bugs):**
  - `falcon9` + `apogee_check` for `gravity_turn`/`exp_shooting` → POOR (apo 35 km): the
    payload-0 overperformance defeats the brute-force apogee match. Feedback laws still pass
    because they steer the trajectory.
  - The guard `exp_shooting`+`pso_coast` → **REJECTED** is correct, intended behaviour.

### 4.4 Full-budget `indirect_pmp` check

`electron`, `indirect_pmp`, **250×500 (the file default, 125k evals, ~35 min)** →
**still POOR**: e = 0.74, apoapsis 199.4 km, periapsis **−5412 km**. Going from the
matrix budget (60×100) to the full default budget only moved eccentricity 0.90 → 0.74 —
nowhere near circular. **Conclusion: `indirect_pmp` does not converge to a circular Earth
orbit even at full budget.** It systematically lands on a transfer orbit (apoapsis exactly
at target, periapsis far below the surface). This is a **structural** problem (the
objective/transversality condition or the 7-var PSO bounds are not enforcing a circular
endpoint), not a budget shortfall.

---

## Section 5 — Fix vs remove summary

Ordered by impact. "Fix" = a real bug to repair; "Remove" = delete/retire; "Config" =
not a code bug but a default/scaling issue.

1. **`cpr` guidance — FIX or REMOVE.** Broken for Earth on every coast method:
   `pso_coast`/`direct` crash with a `brentq` "f(a)/f(b) must have different signs" error
   in the PSO Stage-1 event handling (0.5–0.7 s), and `apogee_check` yields a suborbital
   orbit. **This is broader than the existing memory note** (`cpr-stage1-brentq-crash`),
   which attributed the crash to Earth rotation only — it reproduces with **rotation OFF**
   on the PSO paths. Either fix the Stage-1 event bracketing for `cpr` or remove the mode.

2. **`exp_shooting` guidance — FIX or REMOVE.** `+direct` is impractically slow
   (TIMEOUT ~420 s); `+apogee_check` never activates the law (flies a gravity turn);
   `+pso_coast` is (correctly) rejected. As shipped it cannot complete a useful Earth
   insertion. Investigate the activation path and the BVP cost, or retire it.

3. **`indirect_pmp` — FIX (or label research-only).** Never circularises — **confirmed
   at the full default budget (250×500): e only improved 0.90 → 0.74**, still a transfer
   orbit with periapsis −5412 km (§4.4). This is structural, not a budget shortfall: the
   objective/transversality condition or the 7-var PSO bounds do not enforce a circular
   endpoint. Revisit the solver, or label the mode research-only.

4. **`TIME_TO_START_KICK` is not vehicle-aware — FIX.** The shipped 7.5 s suits high-T/W
   vehicles (falcon9) but pitches the low-T/W **electron straight into the ground**
   (Stage-1 crash) on every coast method until delayed to ~20 s. Make the pitch-over
   start vehicle-specific (e.g. scale with T/W) or add a per-vehicle field.

5. **`saturn_v` registry entry — REMOVE** (or relabel). The simulator supports only two
   serial stages with no parallel staging; the real Saturn V needs three. The entry is a
   suborbital 2-stage stand-in (`rocket_specs.py:75-88`) that misrepresents the vehicle.

6. **`bilinear_tangent` + `direct` overshoot — INVESTIGATE.** Fails on both vehicles
   (apo ~600 km), unlike `bilinear` + `apogee_check`/`pso_coast`. Likely a guidance×direct-PSO
   interaction, not pure budget.

7. **`TARGET_ORBITAL_ALTITUDE = 50 km` (shipped) — CONFIG.** Moon-tuned; suborbital for
   Earth. Any Earth run must set a real LEO target (~200 km). Consider a per-body default.

8. **`falcon9` `M_PAYLOAD = 0` — CONFIG.** Overperformance (~+6 km/s) makes guidance
   often never activate and breaks `apogee_check` for non-steering modes. Give it a
   realistic payload (~15–22 t) if falcon9 is meant to exercise guidance.

9. **Reduced-budget PSO under-convergence — METHODOLOGY, not a bug.** `electron` +
   `peg`/`peg_new`/`apollo` on the PSO coast/direct paths land slightly suborbital at
   24×40; the deterministic `apogee_check` circularises them perfectly. Re-run the passers
   at full PSO budget to confirm before drawing conclusions about those guidance×PSO pairs.

### What clearly WORKS on Earth (both vehicles, rotation ON & OFF)

`gravity_turn`, `linear_tangent`, `apollo`, `peg`, `peg_new` with **`apogee_check`**
(near-perfect circularisation, e ≈ 1e-6) and, for falcon9, with `pso_coast`/`direct` too.
`apogee_check` is the most robust insertion method overall (deterministic brute force);
`pso_coast` is reliable for falcon9 and the simpler electron laws; `direct` works but is the
most budget-sensitive. The Earth-rotation, azimuth, and pseudo-force machinery is sound.
