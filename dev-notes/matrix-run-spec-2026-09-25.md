# Results-matrix production run — configuration spec (2026-09-25)

Hand this file to a new session to review, commit and launch the Chapter 6 batch. It records every
setting of the run and every decision taken with the user on 2026-09-24/25, each with an ID so it
can be kept or dropped.

- **Repository:** `C:\Users\eduar\Desktop\Tese\Code\Thesis-Project`.
- **Commands:** run from the repository root with
  `C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe` (abbreviated `PY` below).
- **Authoritative docs:** `Tese/worktree.md` §4 (the refresh item, and "The results-matrix
  configuration for the production batch") and `CLAUDE.md`.

---

## 0. State at the time of writing

- **The user kept every item** (2026-09-25), and later that day added L (with M and N, found
  while doing it).
- **Commits (pushed):**
  - `1ff794c`: items L, M, N, with their tests and docs.
  - `bee52ed`: items C, D, E, G, with their tests and docs.
  - `fa9db0e` (R2) and `1fb975d` (R1), from earlier.
- **Tests:** 232 pass (`PY -m pytest Tese/src/tests/ -q`).
- **Nothing has been launched.** The stale archives have not been moved yet. Step 0 of section 4
  is done; start at step 1.

---

## 1. Decisions and changes — keep or drop each

| ID | Change | Where | State | Dropping it means |
|---|---|---|---|---|
| **R1** | `GUIDANCE_REFRESH_MODE` switch: `"cycle"` refreshes guidance coefficients once per 2 s cycle on the accepted state, not inside the ODE RHS. Config default `"in_rhs"` is byte-identical to the old code. | `pso_coast_solver.solve_guided_arc`, `GuidanceState.refresh_force/hold`, 7 call sites; `simulation_parameters.py` L300 | pushed `1fb975d` | Reverting the commit; nothing flown depends on it yet. |
| **R2** | The matrix flies `"cycle"`. | `run_results_matrix.BASELINE` | pushed `fa9db0e` | Set `BASELINE["GUIDANCE_REFRESH_MODE"] = "in_rhs"`; the 7 apollo/peg_new cases then fly the trial-point refresh. |
| **A** | Swarm budget **250×1000** for every re-flown swarm. | run command `--budget 250,1000` (no code) | decision | Pick another `--budget`; the kept archives stay at 250×1000, so budgets would differ between sections. |
| **B** | Keep the six §6.2 gravity-turn archives. Five replay bit-identical at HEAD; `gt_apogee` not replayed. | no code | decision | Re-fly them too (6 more cases, gt_apogee ~48 s, the rest ~10+ h each). |
| **C** | `peg_direct`: peg_new ends its own burn (`DIRECT_LAW_TERMINATED_CUTOFF=True`), and the kick, the only variable left, is found by **grid + Brent** (`DIRECT_OPTIMIZER="grid_brent"`, ~520 flights). | `build_matrix()` peg_direct overrides | pushed `bee52ed` | Remove the two overrides. The swarm then picks `[kick, burn %]` as for `gt_direct`, at 250×1000 (~16 h). |
| **D** | §6.4 rows are **polished extremals re-flown from stored decision vectors**, not swarmed. `pmp_baseline` = the tracked `pmp_reference.npz` extremal (seed 3, 750×1500 + half-step polish, **22 261.2 kg**). Reverses the 2026-09-17 "raw swarm only" decision; Chapter 5/6 must then describe `dev-notes/pmp_swarm_polish.py`. | `build_matrix()` `extremal=`; `PMP_BASELINE_EXTREMAL`/`PMP_VACUUM_EXTREMAL`; `_dispatch(sim_params, case)` replay branch; `run_case` passes the case | pushed `bee52ed` | Remove the `extremal=` keys and keep the existing raw seed-42 archives (20 515.3 / 22 032.4 kg). show_ref_track then follows a different extremal from the §6.4 row (1 746 kg apart). |
| **E** | `pmp_vacuum` = the best polished vacuum (seed 3, 250×1000 + the same polish, **23 952.1 kg**). No 750×1500 vacuum swarm exists, so the two §6.4 rows started from different budgets. | same as D | pushed `bee52ed` | Keep the raw seed-42 vacuum archive (22 032.4 kg); §6.4 then sets a polished row against a raw one. |
| **F** | §6.7 showcase laws and the two reference-tracking cases kept as defined. | no code | decision | — |
| **G** | **Segmented re-run fix.** After the altitude switch, `_thrust_phase` restarts from the event root (`t_events[1]`/`y_events[1]`), not the last 0.5 s grid point. The archived flight now equals the one the swarm scored (J identical under `"cycle"`). The swarm path is unchanged. | `segmented_guidance_solver._thrust_phase` | pushed `bee52ed` | Revert the hunk; archived segmented rows then differ from their optimum by J 1e-3–3e-2. |
| **L** | **Segmented: peg_new ends both burns** (`SEGMENTED_LAW_TERMINATED_ARCS=True` on #20, #21). Arc 1 aims at the reference's **coast start** (164.7 km, 7590.5 m/s, 2.81°) and ends on peg_new's own t_go (10 s freeze); the swarm's coast follows; arc 3 aims at the orbit and ends on peg_new's t_go. The swarm picks `[Δt_c, γ_p]` (+ the hand-off, now capped at 0.98 × 164.7 = 161.4 km). Before, peg_new aimed at the orbit in both burns and the swarm cut arc 1 (the pso_coast arc-1 gap). On the reference's kick and coast: arc-1 miss +0.03 km / −0.13 m/s / +0.045°, arc 3 1.4 s, 25 490.3 kg; 0.096 s per flight (swarm-timed 0.129 s). | `segmented_guidance_solver` (`run_segmented_law_terminated`, `reference_coast_start`, `_Segments.coast_start`, `_altitude_bounds`); `simulation_parameters.py` §8a-bis; `build_matrix()` both segmented cases; `main.py` report; archive extras `arc1_target`/`arc1_achieved`/arc times | pushed `1ff794c` | Remove the override from #20/#21: they then fly the swarm-timed form with `[Δt_c, burn %, coast start %, γ_p]` and aim both burns at the orbit. |
| **M** | **Segmented fairing jettison.** The segmented solver never made the `shed_fairing_if_due` checks at Stage-2 start and ignition that every other PSO architecture makes; a kick staging below 65 km carried 1900 kg to orbit (the reference's own kick does: 1443 kg of extra propellant at it). Now made in both segmented forms. | `run_segmented_trajectory`, `run_segmented_law_terminated` | pushed `1ff794c` | A defect fix; dropping it leaves #20/#21 penalised for low-staging kicks that no other architecture pays for. |
| **N** | **Smoke flies a copy of the tracked reference.** The 8×4 token reference ended its first burn at 31.7 km, below the 120 km hand-off, so L refuses it. `SMOKE_BUDGET` no longer lowers `PMP_REFERENCE_PSO_*`; `_prepare_smoke_reference` copies `pmp_reference.npz` over `pmp_reference_smoke.npz` and exits if it does not match the case's cache key. The tracked file still cannot be written by a smoke run. | `run_results_matrix.SMOKE_BUDGET`, `_prepare_smoke_reference`, `run_case` | pushed `1ff794c` | Needed by L for step 2. |
| **H** | `show_seg_opt_alt` keeps its 10 km altitude floor. If its optimum switches below Stage-2 ignition (~69 km), report that its Stage-1 peg_new part refreshed in the RHS; the cycle refresh does not reach the Stage-1 hook. | no code | decision | Raise the floor, or extend the refresh fix to Stage 1 (code). |
| **I1** | Run the 15 cases **in parallel**, one process per case. | run procedure | decision | Sequential: one driver invocation, ~5–6 days. |
| **I2** | Move the 6 superseded folders aside before the run. | run procedure | decision | The batch overwrites them in place. |
| **I3** | Smoke check first, **into its own folder**. | run procedure | decision | Skip it. |
| **T** | Tests: new `test_results_matrix_config.py` (6 tests); `test_direct_grid` and `test_guidance_refresh_mode` pinned to the paths their archived rows used. | tests | pushed `bee52ed` | Go with C/D/E/G. |
| **K** | Docs: worktree §4 production-configuration item and segmented-fix note; CLAUDE.md matrix paragraph; test count. | docs | pushed `bee52ed` | Go with the items above. |

**Not changed, by user decision:**
- `TGO_ESTIMATOR` (apollo keeps the rocket-equation t_go).
- Arc-1 targeting under `pso_coast` (still aims at the final orbit; the waypoint version stays
  reverted). The segmented cases get it through L; the `pso_coast` cases do not.
- The λ′-drop / turn-angle cap (on hold).
- The unprojected Earth-rotation credit.

---

## 2. Global configuration (every case)

### 2a. `run_results_matrix.BASELINE` (applied before each case's overrides)

| Setting | Value |
|---|---|
| `MULTI_GUIDANCE_ENABLED` | `False` |
| `COAST_METHOD` | `"pso_coast"` |
| `INCLUDE_DRAG` / `INCLUDE_LIFT` | `True` / `True` |
| `ENABLE_EARTH_ROTATION` / `INCLUDE_PSEUDO_FORCES` / `COMPUTE_CROSS_HEADING_COUNTER_FORCE` | `True` / `True` / `True` |
| `TARGET_ORBITAL_ALTITUDE` | `500e3` m |
| `TARGET_ORBIT_INCLINATION` | `51.6` ° |
| `LAUNCH_LATITUDE` | `28.5` ° |
| `ATMOSPHERE_EXIT_METHOD` / `DYNAMIC_PRESSURE_THRESHOLD` | `"dynamic_pressure"` / `1000.0` Pa |
| `KICK_PROFILE_MODE` | `"instantaneous"` |
| `TGO_ESTIMATOR` | `"rocket_equation"` (the config file's default is `"peg_new"`; the baseline pins this) |
| `GUIDANCE_TGO_USE_PSO_PLAN` | `False` |
| `GUIDANCE_REFRESH_MODE` | `"cycle"` (R2) |
| `ISP_1_MODE` / `THRUST_1_MODE` | `"pressure"` / `"pressure"` |
| `SAVE_PLOTS` | `False` |

### 2b. Config-file values the run relies on (`Tese/src/Input_File/simulation_parameters.py`, unchanged)

| Setting | Value | Used by |
|---|---|---|
| `PSO_SEED` | 42 | every swarm |
| `APOLLO_FREEZE_THRESHOLD` | 10.0 s | apollo, peg_new, peg freeze |
| `PEG_MAJOR_LOOP_RATE` / `GUIDANCE_UPDATE_RATE` | 2.0 s / 2 s | peg_new / apollo refresh cycle |
| `GUIDANCE_COEFFICIENTS_FIXED` | `True` | closed-loop tangent fallback only (not flown) |
| `FAIRING_JETTISON_MODE` | `"altitude"` (65 km) | every architecture |
| `PSO_COAST_LB` / `UB` | `[0, 0, 0, 1.50]` / `[2000, 100, 100, 1.57]` | `[Δt_c s, burn %, coast start %, γ_p rad]` |
| `PSO_COAST_CPR_THETA_DOT_*_DEG` | 0.02 – 1.0 °/s | show_cpr |
| `PSO_COAST_TAN_THETA0_*_DEG` / `THETAF` | −20…70° / −40…30° | tangent laws |
| `PSO_COAST_BTS_MID_*` | 0.1 – 0.9 | show_bilinear_tangent |
| `PSO_COAST_EXP_A_*` / `EXP_B_*` | 0 – 1.6 / −0.05 – 0.005 | show_exp_shooting |
| `PSO_DIRECT_LB` / `UB` | `[1.50, 50]` / `[1.57, 100]` | gt_direct (kept) |
| `DIRECT_GRID_GAMMA_P_BOUNDS` / `_POINTS` / `DIRECT_BRENT_XATOL_GAMMA_P` | (1.50, 1.57) rad / 500 / 1e-7 rad | peg_direct (C) |
| `PSO_MG_LB` / `UB` | `[0, 0, 0, 1.50]` / `[2000, 100, 100, 1.57]`; under L only entries 0 and 3 are read: `[Δt_c, γ_p]` (+ altitude fraction 0–1) | segmented |
| `MULTI_GUIDANCE_ALT_LB` / `ALT_UB` | 10 km / 500 km; effective upper bound `min(500 km, 0.98 × reference apogee, 0.98 × reference coast start)` = **161.4 km** under L | show_seg_opt_alt |
| `APOLLO_FREEZE_THRESHOLD` for the segmented arc-1 target | 10 s (not `SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD` = 2 s, at which peg_new's burn never ends) | #20, #21 (L) |
| `INDIRECT_PMP_STAGE2_FRAME` / `INDIRECT_PMP_TRANSVERSALITY` | `"rotating_pseudo_forces"` / `"duration_stationarity"` | PMP replay |
| `PMP_REFERENCE_PSO_PARTICLES` × `_GENERATIONS` | 250 × 1000 | reference cache key: **do not change** (it would rebuild the tracked `pmp_reference.npz`) |
| `PMP_REFERENCE_CACHE` | `Tese/src/Output/pmp_reference.npz` (tracked) | segmented, reference_track |

### 2c. What `--budget 250,1000` sets (in memory, per case)

`PSO_N_PARTICLES`/`PSO_MAX_GENERATIONS`, `PSO_COAST_*`, `PSO_DIRECT_*` and `PSO_MG_*` are all set
to 250 / 1000. It deliberately does not touch `PMP_REFERENCE_PSO_*`. With C and D in place, only
the `pso_coast` and segmented cases actually swarm.

### 2d. The reference cache

- `Tese/src/Output/pmp_reference.npz` is tracked in git (`bceab71`).
- **Source:** `Output/pmp_polish_750x1500/pmp_baseline/b750half_start0_20260921_162253/pmp_baseline.npz`.
- Its `decision_vector` equals `run_results_matrix.PMP_BASELINE_EXTREMAL`
  (`test_results_matrix_config.py` pins this).

---

## 3. The 21 cases

Budget column: 250×1000 = swarm at `--budget 250,1000`. "Refresh" is where `"cycle"` changes the
flight.

| # | Case | § | Law | Architecture / optimiser | Overrides on top of BASELINE | Decision vector | Refresh | Plan | Est. time |
|---|---|---|---|---|---|---|---|---|---|
| 1 | gt_baseline | 6.2 | gravity_turn | pso_coast PSO | `GUIDANCE_MODE=gravity_turn` | 4 | — | **keep** (8203d94) | — |
| 2 | gt_apogee | 6.2 | gravity_turn | apogee_check grid (Ns=1000) | + `COAST_METHOD=apogee_check` | kick | — | **keep** | — |
| 3 | gt_direct | 6.2 | gravity_turn | direct PSO | + `COAST_METHOD=direct` | `[γ_p, burn %]` | — | **keep** (suborbital by design) | — |
| 4 | gt_vacuum | 6.2 | gravity_turn | pso_coast PSO | + `INCLUDE_DRAG=False` | 4 | — | **keep** | — |
| 5 | gt_norot | 6.2 | gravity_turn | pso_coast PSO | + rotation, pseudo-forces, cross-heading off | 4 | — | **keep** | — |
| 6 | gt_sea_level_engine | 6.2 | gravity_turn | pso_coast PSO | + `ISP_1_MODE`/`THRUST_1_MODE="sea_level"` | 4 | — | **keep** | — |
| 7 | peg_baseline | 6.3 | peg_new | pso_coast PSO 250×1000 | `GUIDANCE_MODE=peg_new` | 4 | yes | **fly** | ~11–12 h (12.3 h in-RHS) |
| 8 | peg_direct | 6.3 | peg_new | direct, law-terminated, **grid + Brent** (C) | + `COAST_METHOD=direct`, `DIRECT_LAW_TERMINATED_CUTOFF=True`, `DIRECT_OPTIMIZER="grid_brent"` | `[γ_p]` | n/a (burn refreshed outside the ODE already) | **fly** | minutes |
| 9 | peg_vacuum | 6.3 | peg_new | pso_coast PSO 250×1000 | + `INCLUDE_DRAG=False` | 4 | yes | **fly** | ~11 h (11.7 h in-RHS) |
| 10 | peg_vacuum_norot | 6.3 | peg_new | pso_coast PSO 250×1000 | + no drag, no rotation/pseudo-forces/cross-heading | 4 | yes | **fly** | ~6 h (6.6 h in-RHS) |
| 11 | pmp_baseline | 6.4 | indirect_pmp | **stored extremal re-flown** (D) | `GUIDANCE_MODE=indirect_pmp` | 7 (fixed) | — | **fly** (replay) | ~2 s |
| 12 | pmp_vacuum | 6.4 | indirect_pmp | **stored extremal re-flown** (E) | + `INCLUDE_DRAG=False` | 7 (fixed) | — | **fly** (replay) | ~1 s |
| 13 | show_cpr | 6.7 | cpr | pso_coast PSO 250×1000 | `GUIDANCE_MODE=cpr` | 4 + θ̇ | — | **fly** | ~10–16 h (not measured at this budget) |
| 14 | show_linear_tangent | 6.7 | linear_tangent (open-loop) | pso_coast PSO 250×1000 | `GUIDANCE_MODE=linear_tangent` | 4 + θ₀, θ_f | — | **fly** | ~10–16 h |
| 15 | show_bilinear_tangent | 6.7 | bilinear_tangent (open-loop) | pso_coast PSO 250×1000 | `GUIDANCE_MODE=bilinear_tangent` | 4 + θ₀, θ_f, μ | — | **fly** | ~10–16 h |
| 16 | show_apollo | 6.7 | apollo | pso_coast PSO 250×1000 | `GUIDANCE_MODE=apollo` | 4 | yes | **fly** | ~10–16 h |
| 17 | show_exp_shooting | 6.7 | exp_shooting | pso_coast PSO 250×1000 | `GUIDANCE_MODE=exp_shooting` | 4 + a, b | — | **fly** | ~10–16 h |
| 18 | show_ref_track | 6.7 | peg_new | reference_track (no search) | + `COAST_METHOD=reference_track` | reference's 7-vector | already outside the ODE | **fly** | ~1 s |
| 19 | show_ref_track_apollo | 6.7 | apollo | reference_track (no search) | + `GUIDANCE_MODE=apollo`, `COAST_METHOD=reference_track` | reference's 7-vector | already outside the ODE | **fly** | ~1 s |
| 20 | show_seg_fixed_alt | 6.7 | gravity_turn → peg_new @ 120 km; peg_new ends both burns (L) | segmented PSO 250×1000 | `MULTI_GUIDANCE_ENABLED=True`, `..._OPTIMIZE_ALTITUDES=False`, `SEGMENTED_LAW_TERMINATED_ARCS=True`, `GUIDANCE_SEGMENTS=[("gravity_turn",0),("peg_new",120e3)]` | `[Δt_c, γ_p]` | peg_new burns outside the ODE already (L) | **fly** | ~7–10 h (0.096 s per flight) |
| 21 | show_seg_opt_alt | 6.7 | gravity_turn → peg_new @ swarm altitude ≤ 161.4 km (L) | segmented PSO 250×1000 | same, `..._OPTIMIZE_ALTITUDES=True` | `[Δt_c, γ_p]` + altitude fraction | Stage 2 as #20; Stage 1 in-RHS if the switch lands there (H) | **fly** | ~7–10 h |

**Totals.**
- **Keep 6:** #1–6.
- **Fly 15:**
  - 10 swarms at 250×1000: #7, 9, 10, 13–17, 20, 21.
  - One grid search: #8.
  - Two replays: #11, 12.
  - Two no-search flights: #18, 19.
- **Wall time in parallel:** about 12–16 h, set by the slowest swarm. Contention between 10
  concurrent processes on 32 threads may stretch it.

---

## 4. Run procedure

### Step 0: review, commit, test

1. Decide on items C, D, E, G, L, M, N, T, K (section 1).
2. Run `PY -m pytest Tese/src/tests/ -q` (expect 232 passed, or fewer tests if items are dropped).
3. Commit and push, so every manifest names a committed state:
   - end the commit message with the repository's attribution line;
   - mask the git token in any output with `sed -E 's#//[^@]*@#//***@#'`.

### Step 1: move the superseded archives aside (I2)

The six folders are `peg_baseline`, `peg_direct`, `peg_vacuum`, `peg_vacuum_norot`,
`pmp_baseline` and `pmp_vacuum`. They go from `Tese/src/Output/results_matrix/` to
`Tese/src/Output/results_matrix_stale_pre_20260925/`. Leave the six `gt_*` folders in place.

### Step 2: smoke check into its own folder (I3)

```
PY Tese/src/run_results_matrix.py --smoke --out Output/results_matrix_smoke
```

- **Always pass `--out` with `--smoke`:** without it the token-budget results overwrite the real
  `Output/results_matrix/`.
- The smoke run redirects its reference cache to `pmp_reference_smoke.npz`, a copy of the tracked
  one (N), so the tracked cache is safe. A case that exits with "does not match this case's
  configuration" has found a reference the production run would rebuild: stop and investigate.
- Checked on 2026-09-25 for #18–21 (`--smoke --only show_seg_,show_ref_track` into the scratchpad):
  all four dispatch, and the tracked cache stays unmodified.
- **Expect all 21 cases to dispatch.** `gt_direct` finishes suborbital by design, and the token
  budget means nothing numerically.

### Step 3: launch in parallel (I1)

- **Use `--case`, never `--only`, for single cases.** `--only` is a substring filter, so
  `--only peg_vacuum` also runs `peg_vacuum_norot`.
- **One background process per case,** each with its own log (e.g.
  `Tese/src/Output/results_matrix_logs/<case>.log`).
- **Suggested extra flags** (not decided): add `--set EVENTS_PRINT=False --set
  INTERRUPTS_PRINT=False`. These are print-only settings, and a 250×1000 run otherwise logs
  several lines per trajectory (~10⁶ lines per case).

```
PY Tese/src/run_results_matrix.py --case peg_baseline         --budget 250,1000
PY Tese/src/run_results_matrix.py --case peg_vacuum           --budget 250,1000
PY Tese/src/run_results_matrix.py --case peg_vacuum_norot     --budget 250,1000
PY Tese/src/run_results_matrix.py --case show_cpr             --budget 250,1000
PY Tese/src/run_results_matrix.py --case show_linear_tangent  --budget 250,1000
PY Tese/src/run_results_matrix.py --case show_bilinear_tangent --budget 250,1000
PY Tese/src/run_results_matrix.py --case show_apollo          --budget 250,1000
PY Tese/src/run_results_matrix.py --case show_exp_shooting    --budget 250,1000
PY Tese/src/run_results_matrix.py --case show_seg_fixed_alt   --budget 250,1000
PY Tese/src/run_results_matrix.py --case show_seg_opt_alt     --budget 250,1000
PY Tese/src/run_results_matrix.py --only peg_direct,pmp_,show_ref_track --budget 250,1000
```

- **The last line** runs the five quick cases (#8, 11, 12, 18, 19). Its substrings match exactly
  those five. `--budget` is inert for them but keeps their manifests uniform.
- **`--case` writes each case's archive** to `Output/results_matrix/<case>/`. It does not write
  `results_matrix.csv`.

### Step 4: rebuild the summary CSV once everything has finished

```
PY Tese/src/run_results_matrix.py --only show_ref_track --budget 250,1000
```

This re-flies the two ~1 s cases, and the driver writes `results_matrix.csv` from every archived
row in the folder. Expect 21 rows.

### Step 5: checks after the run

- **All 21 folders** are present in `Output/results_matrix/`, and every re-flown manifest shows
  `GUIDANCE_REFRESH_MODE = "cycle"` and a committed git hash.
- **The §6.4 rows** reproduce the polish archives: `pmp_baseline` 22 261.2 kg, J′
  0.7598833804841622; `pmp_vacuum` 23 952.1 kg.
- **`show_seg_opt_alt`:** read `optimized_altitudes` from the npz. If it is below the Stage-2
  ignition altitude (~69 km), report H.
- **Both segmented cases (L):** compare `arc1_achieved` with `arc1_target` in the npz. A miss much
  larger than the ~0.03 km / 0.1 m/s measured on the reference's plan means peg_new did not
  reach the coast start at the swarm's kick. Read `t_arc3_end − t_arc3_start` as well: arc 3 is
  expected to be a second or two, as in `show_ref_track`.
- **`peg_direct`:** read `direct_grid_gamma_p_on_bound`. If it is True, the optimum sits on the
  edge of the kick box.
- **Figures:** `PY Tese/src/Plots/results_figures/make_all.py`.

---

## 5. Disclosures Chapter 6 needs with this run

- **Refresh.** Every matrix case flown from 2026-09-25 uses the cycle refresh. Measured with
  `dev-notes/refresh_ab.py`: 96–99 % of in-RHS refreshes were on trial points. J's noise fell from
  1e-8..3 to ~1e-12, and the cycle refresh takes 28–54 % fewer RHS evaluations. The six kept gt
  archives are unaffected; the gravity turn has no refresh.
- **§6.4.**
  - The rows are polished extremals (swarm + Levenberg–Marquardt polish), and the two started
    from different swarm budgets.
  - `pmp_baseline` is the reference the tracking and segmented cases follow.
- **peg_direct** differs from gt_direct in the cutoff rule as well as the law (C).
- **Reference-tracking arc 3 is a single frozen cycle:** 1.43 s for peg_new and 0.19 s for
  apollo. For apollo's 2 km altitude miss, see worktree §4 (causes and proposed fixes).
- **Earth-rotation credit.** Because the credit is unprojected, the insertion state is ~125 m/s
  below circular, and part of the PMP margin is the coast-to-target this allows. The archive's
  eccentricity column cannot show it.
- **Arc 1 under `pso_coast`** aims at the final orbit (documented, not fixed).
- **Segmented (L).**
  - peg_new aims arc 1 at the PMP reference's coast start and ends both burns on its own t_go,
    so the segmented rows use the same targets as `show_ref_track`. They differ from it in the
    gravity-turn prefix and in the kick and coast the swarm picks.
  - Their difference from `peg_baseline` is not only the schedule: the arc-1 target and the
    cutoff rule differ too.
  - The optimised hand-off is capped at 161.4 km.
  - Every segmented archive flown before 2026-09-25 is the swarm-timed form, and those with a
    low-staging kick also carried the fairing (M).
- **show_seg_opt_alt:** if its switch lands in Stage 1, that part refreshed in the RHS (H).
