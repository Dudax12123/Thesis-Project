# Arc-1 waypoint targeting under `pso_coast` — PLAN (2026-09-22)

**Status: plan only. Nothing implemented, nothing run beyond read-only offline checks.**
*(Later on 2026-09-22 this design was implemented, measured and reverted by the user; the no-optimiser alternative became `COAST_METHOD="reference_track"`, 7482e25. Kept for history.)*
Step 4 of the ordered plan of 2026-09-21. Tree at `6a1a29a`, clean. Each numbered step in §5
waits for its own authorisation.

Scope, as decided: `apollo` and `peg_new` only. Affected cases: `show_apollo` (§6.7),
`peg_baseline`, `peg_vacuum`, `peg_vacuum_norot` (§6.3). The design itself (PSO keeps
`coast_start_pct`; the reference's (h, v, γ) at the same instant `t_arc1_end` becomes the arc-1
`SegmentTarget`; the coast runs as planned; arc 3 targets the objective orbit as today) is taken
as given. This plan only says how to realise it.

---

## 0. Recommendations at a glance

| # | Question | Recommendation |
|---|---|---|
| 1 | Arriving at the PSO instant | **apollo**: `force_planned_tgo` with deadline `t_arc1_end` during arc 1 only. **peg_new**: a new *prescribed-t_go* major loop (one new function in `peg_guidance_new.py`, existing functions untouched). The native t_go disagrees with the planned one by −5 to +40 s, so it would either be cut short or arrive early and keep burning. **Both laws give up horizontal speed** and hold altitude and vertical velocity (apollo through its P12 split; peg_new's tangential component takes the Δv the fixed time leaves). The miss passes to arc 3. |
| 2 | Switching at reignition | One helper for each boundary, called from **both** builders: `_arm_arc1_waypoint` before arc 1 and `_begin_arc3` at the arc-3 block. `_begin_arc3` sets `gs.target=None`, `force_planned_tgo=False`, `peg_new_fixed_tgo=False` and deadline `t_arc3_end`. Under the switch it always calls `restart_for_new_burn()`, including when there is no coast. `restart_for_new_burn` itself stays unchanged, because the segmented solver relies on it. |
| 3 | PSO instants after the reference's cutoff | **Read the reference's coasting state literally.** No bound on the swarm and no clamp on the lookup. The target stays continuous. The over-thrust (~+37 m/s of horizontal speed per second past the cutoff) is priced by the objective as it stands. Flag it as bound-hugging if the optimum sits at the cutoff. |
| 4 | By-time lookup | `segment_reference.waypoint_at_time()`: linear `np.interp` on the reference's time grid, returning the same dict as `waypoint_at_altitude`. It costs 2.8 µs per trajectory, is computed once per trajectory (never in the right-hand side), and is loaded once per process. **It changes neither the reference cache key nor `_ARCHIVE_MUST_MATCH`.** It reads a PMP *archive* checked by the existing `_archive_mismatches(check_search=False)` and never calls `get_pmp_reference`, so it cannot trigger a rebuild. |
| 5 | Vacuum cases | MECO agrees to the millisecond (146.742 s in all three vacuum runs). **peg_vacuum → the refined vacuum extremal** (`s3half`, 23 952.1 kg). **peg_vacuum_norot has no reference of its own physics**: nothing rotation-off was ever flown by the PMP. The rotating vacuum reference is the wrong problem (441 m/s apart on the target), and the guard refuses it anyway. Recommended: build a rotation-off vacuum reference (§3.5). This is decision D1. |
| 6 | Byte-identical old paths | Two new settings: `PSO_COAST_ARC1_WAYPOINT = False` and `PSO_COAST_ARC1_REFERENCE = None`. With the switch on, any law other than apollo/peg_new **raises** rather than silently ignoring it. One new `GuidanceState` field, `peg_new_fixed_tgo`. The harness sets both settings for the four cases only. worktree §2.11, §3 and §4 and the tests are listed in §3.6. |
| 7 | Validation ladder | Unit tests → offline arc-1 flights (seconds) → smoke (minutes) → two 50×100 rehearsals (~15 min each) → comparison against the archived cases with `run_archive.py compare`. Production: ~42 h sequential or ~15 h four-wide at 250×1000, plus ~2.5 h if D1 = build. |
| 8 | Chapter 6 | Decision 18 is answered by the design, not by a searched shape parameter. The three archived §6.3 coast rows become the *untargeted* variant. The laws' arc 1 now depends on the **polished** extremal, which reverses the 09-17 "polish out of the thesis" decision. The laws are likely to inherit its depressed trajectory and its loads. The full disclosure list is in §6. |

---

## 1. Facts verified this session

All from archives on disk or pure-function calls. Scratch scripts are in the session scratchpad
(`arc1_facts.py`, `arc1_offline.py`, `arc1_offline_refign.py`). Nothing was integrated.

**The reference is as briefed.** `pmp_reference.npz` is bitwise the `b750half` archive (time and
state rows). Arc 1 runs 154.003 → **411.330 s**. At its end the state is **164.70 km,
7 590.51 m/s, γ 2.812°, 26 168.0 kg**. Then comes a **1 446.83 s** coast and a **0.025 s** last
burn (1858.163 → 1858.188). The refined vacuum extremal cuts at **405.874 s** (167.64 km,
7 586.33 m/s, 2.827°, 27 863.0 kg), coasts 1 432.2 s, and burns 0.040 s last. Both have 0.5 s
grids with 5 duplicated stamps at the arc boundaries, are monotone, and end at SECO. Their speed
channel is ground-relative throughout (no post-insertion tail).

**Same clock, with one refinement.**

| run | t_MECO [s] | Stage-2 ignition [s] | ignition mass [kg] |
|---|---|---|---|
| reference, baseline | 146.003 | 154.003 | 96 569.9 |
| peg_baseline / gt_baseline | 146.019 / 146.020 | +8 s | 96 570.0 |
| reference, vacuum; peg_vacuum; peg_vacuum_norot | 146.742 (all three) | 154.742 | 96 570.0 |

- The baseline MECO differs by 16–17 ms between runs. The cause is the trajectory dependence of
  the pressure engine model (published thrust and Isp ratios disagree by 1.6 %). In the vacuum
  runs MECO is identical.
- At the same absolute time a baseline law is therefore 4.4 kg heavier than the reference, worth
  ≤ 0.5 m/s. Absolute time, as designed, is adequate.
- The baseline reference stages at 64.87 km with the fairing still on and sheds it at ignition,
  when the pre-ignition coast crosses 65 km. So the arc-1 Δv budget at any given t is the same for
  the law and the reference. That is the precondition for the design being well-posed.

**Where arc 1 ends today versus the reference.**

| case | t_arc1_end [s] | law at t_arc1_end | reference at the same t |
|---|---|---|---|
| peg_baseline | 354.87 | 318.6 km, 5 570.8 m/s, 12.98° | 149.8 km, 6 041.0 m/s, 1.95° |
| peg_vacuum | 350.08 | 351.4 km, 5 506.6 m/s, 11.40° | 152.2 km, 6 127.6 m/s, 2.10° |
| peg_vacuum_norot | 371.62 | 349.1 km, 6 055.1 m/s, 11.33° | (no rotation-off reference) |
| show_apollo (stale, `pf_on_v3`, 100×500) | 412.07 | — | reference already coasting |

Masses agree to 4 kg. The waypoint is therefore **not** a small correction: it moves arc 1 from a
lofted ~320–350 km, 11–13° state to a depressed ~150–165 km, 2–3° one.

**The ignition states are far apart too.** They come from the kicks: the laws' γ_p is
1.528–1.544, the reference's 1.537139.

| run | h [km] | v [m/s] | γ [°] |
|---|---|---|---|
| reference, baseline | 68.84 | 3 381.7 | 13.25 |
| peg_baseline | 92.80 | 3 257.0 | 23.56 |
| peg_vacuum | 112.29 | 3 325.3 | 29.25 |
| show_apollo (stale) | 134.71 | 2 997.7 | 49.72 |

Stage 1 is shared through `ra.run_stage1()`. A law at γ_p = 1.537139 should therefore ignite in
exactly the reference's state (checked in §4, step V1a).

**peg_new, native versus planned t_go**, at the archived ignition state, targeting the reference
at t_end. "Tangential miss" is the prescribed-t_go prototype's first-order prediction at ignition:

| ignition state | t_end = 300 | 355 | 400 | 411.33 | 430 (past the cutoff) |
|---|---|---|---|---|---|
| peg_baseline (γ_p 1.5437): native − planned | +12.5 s | +8.1 | +5.3 | +4.8 | −14.1 |
| tangential miss | −208 m/s | −181 | −154 | −144 | +611 |
| peg_vacuum (γ_p 1.5284): native − planned | +30.3 s | +22.2 | +15.3 | +9.0 | −9.9 |
| tangential miss | −533 m/s | −477 | −438 | −243 | +496 |
| reference's own ignition (γ_p 1.5371): native − planned | −4.4 s | −5.1 | −5.0 | −4.8 | −23.8 |
| tangential miss | +42 m/s | +65 | +104 | +120 | +897 |

- In the peg_vacuum row, t_end = 411.33 s is already 5.5 s past the vacuum reference's cutoff
  (405.87 s), so that column is a coasting target there.
- From the laws' current kicks the native law would still be burning when the swarm cuts arc 1.
- From the reference's kick it would arrive about 5 s early and keep burning with frozen
  coefficients, while `λ_r(t − t_λ)` rotates the thrust.
- The sign change of the miss between the two kicks means a γ_p exists near the reference's where
  the fixed-time miss is zero. **The swarm is therefore expected to move γ_p towards 1.537.**

**apollo at ignition** (forced t_go = t_end − t_ign, waypoint target, P12 split):

- From peg_baseline's ignition state the vertical channel demands 0.75–3.6 × the available
  9.67 m/s² (downwards, to kill ~1 300 m/s of climb). For t_end ≤ 380 s it saturates and P12
  thrusts **straight down** (α₀ = −113.6°).
- From the stale apollo state (γ 49.7°) the vertical demand is 2.5–10.6 × a_T, α₀ = −139.7°.
- From the reference's ignition state the demand is 0.17–0.30 × a_T and α₀ is −3.4° to +4.4°:
  benign.

**Code facts** — all confirmed as briefed:

- apollo reads a deadline only through `force_planned_tgo` or `GUIDANCE_TGO_USE_PSO_PLAN`
  (`pso_coast_solver.py:335–338`). Without it, apollo's t_go is the rocket-equation estimate **to
  the final orbit** (`_compute_tgo_stage2`, `:287–321`), even when `gs.target` is set. So
  forcing is mandatory, not optional.
- peg_new never reads `tgo_deadline` (`:475–482`, `:608–628`).
- `restart_for_new_burn` leaves `target`, `tgo_deadline` and `force_planned_tgo` alone (`:222–238`).
- The arc-1 deadline is `t_ignition + T_burn_total` (`:778`, `:1183`).
- The arc-3 deadline is reset only when `delta_tc > 0.01` (`:858–859`, `:1237–1238`).

**No rotation-off PMP archive exists** anywhere under `Output/`: I searched every manifest for
`indirect_pmp` with `ENABLE_EARTH_ROTATION=False`. `_stage1_pseudo_forces()` accepts that pairing
(the terms are inert with rotation off), so one could be flown.

---

## 2. What changes, in one paragraph

With `PSO_COAST_ARC1_WAYPOINT=True` and `GUIDANCE_MODE ∈ {apollo, peg_new}`, each `pso_coast`
trajectory works as follows:

- Before arc 1 it reads the reference state at `t_arc1_end` and sets it as `gs.target`
  (a `SegmentTarget`). It sets `gs.tgo_deadline = t_arc1_end` and arms the prescribed-time
  behaviour: `force_planned_tgo` for apollo, `peg_new_fixed_tgo` for peg_new.
- At the arc-3 block it disarms all of that: target `None`, flags `False`, deadline `t_arc3_end`,
  and `restart_for_new_burn()`.
- Arc 3 is then exactly today's arc 3.

The decision vector, bounds, objective, Stage 1, the coast and every other law are untouched.

---

## 3. The eight questions

### 3.1 Arriving at the PSO instant, and what each law gives up

With full thrust from a fixed ignition time to a fixed `t_arc1_end`, the Δv of arc 1 is fixed,
`ve·ln(m_ign/m_end)`. Hitting three conditions (h, v, γ) at a fixed time with a fixed Δv is
over-determined by one. Each law gives up one component; the miss passes to the coast and arc 3.

**apollo — recommended: planned countdown, vertical channel held, horizontal speed given up.**

- Set `gs.force_planned_tgo = True` and `gs.tgo_deadline = t_arc1_end` for arc 1.
  `_tgo_for_guidance` then returns `t_arc1_end − t` (already implemented, used by the segmented
  solver).
- `compute_apollo_coefficients` with a target solves the vertical polynomial (k3, k4) for
  `(y_T, vy_T)` at the deadline. The downrange position constraint is off.
- The P12 split flies the vertical demand first, and the horizontal takes `sqrt(a_T² − a_y²)`.
  **Given up: horizontal speed** v_x. Hence speed mostly; γ = atan(v_y/v_x) moves by roughly
  −(v_y/v²)·Δv_x, ≈ −0.04° per 100 m/s at the waypoint.
- **Failure mode:** when the vertical demand exceeds a_T, the vertical channel saturates and misses
  too. The offline check shows this happens from today's lofted kicks and not from the
  reference's kick.
- No guidance-module change.

**peg_new — recommended: a prescribed-t_go major loop.**

- New function `peg_new_major_loop_fixed_tgo(state, r_T, mu, ve, F_T, t_go, v_r_T, n_pred_iter=3)`
  in `Guidance/peg_guidance_new.py`. It returns the same 6-tuple as `peg_new_major_loop`.
- The prescribed `t_go` fixes `L0 = −ve·ln(1 − t_go/τ)`.
- The radial channel is solved exactly as today: `vgo_r = (v_r_T − v_r) − ḡ_r·t_go`, with the
  same trapezoidal predictor-corrector for ḡ_r.
- **The tangential component takes what is left: `vgo_θ = sqrt(L0² − vgo_r²)`.** The thrust
  integrals and `λ'_r` are then unchanged.
- **Given up: tangential speed.** Held: radial velocity (hard) and radius (through λ'_r, soft, as
  always in PEG). This is the direct analogue of apollo's P12, so the thesis can state one rule for
  both laws.
- If `|vgo_r| > L0` (the radial demand exceeds the whole Δv), clip to pure radial thrust and flag
  it; the radius then misses too.
- **Consistency property, to be tested:** at `t_go` equal to the native t_go the prototype
  reproduces `peg_new_major_loop`'s 6-tuple (same L0, same vgo_r, therefore the same vgo_θ).
- Measured cost: `peg_new_major_loop` is 20 µs per call, and the new function is the same order.
  That is ~2.6 ms per trajectory at a 2 s major-loop rate over ~260 s, against ~180 ms per
  trajectory today.
- Dispatcher change, in the peg_new init and major-loop branches:
  `if gs.peg_new_fixed_tgo and gs.tgo_deadline is not None:` call the new function with
  `t_go = max(gs.tgo_deadline − t, 0.1)`. The existing freeze logic then freezes at
  `deadline − freeze_thr` with no further change.

*Considered, not recommended:*

1. **Native peg_new cut at `t_arc1_end`.** It is cut short from today's kicks (native − planned
   is +5 to +40 s). From the reference's kick it arrives ~5 s early and then flies frozen
   coefficients past its own t_go, which is ~+180 m/s of uncontrolled thrust at ~36 m/s².
2. **A root-find on `v_θT` in the dispatcher** (choose the v_θT whose native t_go equals the
   planned one). It is mathematically the same law with zero guidance-module lines, but costs ~10
   major-loop evaluations per cycle (~+15 % per trajectory) and hides the "tangential gives up"
   choice inside a solver.
3. **A law-terminated arc 1.** Ruled out by design point 1.

**Freeze threshold for the arc-1 target:** reuse the segmented precedent,
`SegmentTarget(freeze_threshold=SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD)`, which is 2.0 s. Arc 3
keeps `APOLLO_FREEZE_THRESHOLD` (10 s). Step V1 reports the arc-1 miss at 2 s and at 10 s so the
choice rests on a number.

**The major loop inside the right-hand side.** peg_new already runs its major loop inside the ODE
right-hand side, and the archived peg cases reconcile swarm → replay exactly. The prescribed-t_go
variant has the same structure. If the V3 rehearsal shows the swarm's J′ and the replay's J′
disagreeing, the fallback is the `peg_new_external` mechanism that `DIRECT_LAW_TERMINATED_CUTOFF`
already uses (major loop on accepted states at cycle boundaries). **Backlog item 23 (a
law-terminated cutoff for `pso_coast`) is not a prerequisite under this design**, since the law no
longer ends arc 1. It stays on hold.

### 3.2 Switching the target at reignition

Both builders repeat the arc sequence: `run_pso_coast_trajectory` (swarm, `:776–859`) and
`run_pso_coast_full` (replay, `:1181–1238`). Recommendation: two small module helpers, called from
both builders at the same points, so the swarm and the replay cannot drift.

- `_arm_arc1_waypoint(gs, t_arc1_end)`:
  - no-op unless the switch is on;
  - is called just before the arc-1 `solve_ivp`, inside the `t_coast_start > 0.01` branch, so a
    zero-length arc 1 never arms it;
  - looks up the waypoint and sets `gs.target`, `gs.tgo_deadline = t_arc1_end`, and
    `force_planned_tgo` (apollo) or `peg_new_fixed_tgo` (peg_new);
  - returns the target, so the replay can archive it.
- `_begin_arc3(gs, t_arc3_end, delta_tc)` replaces the two `if delta_tc > 0.01:` blocks:
  - with the switch off, it does exactly what they do today, with no change of order;
  - with the switch on, it always calls `restart_for_new_burn()` and then sets `gs.target = None`,
    `force_planned_tgo = False`, `peg_new_fixed_tgo = False` and `gs.tgo_deadline = t_arc3_end`.

Why "always" under the switch: with `delta_tc ≤ 0.01` today's code carries arc 1's guidance state
into arc 3 unrestarted. Arc 1 now had a *different target*, and possibly frozen coefficients, so
arc 3 must start clean. With a zero coast this is the one place arc 3 differs from today, and
deliberately so: it still flies to the objective orbit.

**The arc-3 deadline logic still holds.** Arc 3's deadline is `t_arc3_end`, as now. apollo does not
read it in arc 3 (`force_planned_tgo` is back to False and `GUIDANCE_TGO_USE_PSO_PLAN` is False in
BASELINE), so it uses its own estimate as today. peg_new does not read it. `restart_for_new_burn`
is **not** changed to clear `target`: the segmented solver calls it and then `apply()` in that
order, and its byte-identity depends on the method staying as it is.

### 3.3 PSO instants after the reference's cutoff (411.33 s baseline, 405.87 s vacuum)

- **The range can occur.** Arc 1 can end anywhere in (t_ign, t_ign + T_MAX_2] = (154, 493] s.
  The stale apollo archive ended arc 1 at 412 s.
- Past the cutoff the reference is coasting. Its speed falls slowly (7 590 → 7 586 m/s at 420 s)
  and its mass is frozen, while the law keeps thrusting at ~273.6 kg/s. At 420 s the law has
  ~2.4 t less mass, ≈ +325 m/s of Δv the target does not need.

**Recommendation: literal lookup — no swarm bound, no clamp.**

- **Continuity.** The target is continuous in `t_arc1_end` (only its derivative jumps at the
  cutoff), so the landscape gets no step.
- **Design point 2 as written.** It honours "the reference's state at that same instant". If a law
  matches that state exactly, its coast *is* the rest of the reference's coast.
- **The over-thrust is priced already.** Both laws put it into horizontal speed: +611 m/s
  predicted at 430 s from peg_baseline's kick, +897 m/s from the reference's. Arc 3 cannot remove
  that cheaply, so the existing objective penalises those particles. No new term is needed.
- **Why not a bound.** A bound `t_arc1_end ≤ t_cutoff` is a nonlinear constraint in
  (`delta_tr_pct`, `coast_start_pct`). It would change the search space and tie the swarm's box to
  the reference, contradicting design point 1.
- **Why not a clamp.** A clamp to the burnout state asks the law to be at 411.33 s's state at a
  later time. That point is not on the reference, and the law's coast would start time-shifted
  from the reference's coast.

**Diagnostic, not a constraint:** archive `t_arc1_end − t_ref_cutoff`. If a production optimum
sits within a couple of seconds of the cutoff, report it as bound-hugging in the same way as
γ_p bounds.

### 3.4 The by-time reference lookup

- **New pure function** `segment_reference.waypoint_at_time(time_full, data_full, t)`:
  - linear `np.interp` of r, v, γ, m over the reference's time grid, after dropping duplicated
    stamps (keep the last);
  - returns the same dict as `waypoint_at_altitude`, `{r, alt, v, gamma, m, t}`, with
    `alt = r − R_E` interpolated (not imposed), so `SegmentTarget.from_waypoint` works unchanged;
  - `v` and γ are ground-relative, as the reference's channels are up to its SECO. No frame
    conversion; the unprojected credit convention is untouched.
  - **Interpolation error** at the 0.5 s grid: about ≤ 0.01 m/s in v (h²/8 · a²/ve with
    a ≈ 36 m/s²) and negligible in h and γ. No spline is needed.
- **New loader** `segment_reference.load_arc1_reference(npz_path)`:
  - reads a standard trajectory archive (`time`, `data[:5]`);
  - requires its manifest and checks it with the existing
    `_archive_mismatches(config, check_search=False)`. Any physics disagreement raises;
    seed/budget differences are accepted, as for the seeding of 09-21. That also admits the smoke
    and `--budget` configurations, which a full-key comparison would refuse because smoke changes
    `PMP_REFERENCE_PSO_*`;
  - checks once that the grid covers [t_MECO + 8, t_MECO + 8 + T_MAX_2];
  - memoises by path in a module dict, so a swarm loads it once per process.
  - **It never calls `get_pmp_reference`, so a law case cannot trigger a PMP build or overwrite
    the tracked `pmp_reference.npz`.** That is the trap the smoke redirect exists for, and a
    vacuum case computing the baseline key would otherwise walk into it.
- **Cost:** 3 × `np.interp` on 18 327 points = 2.8 µs, once per trajectory, outside the
  right-hand side. Negligible against ~180 ms per trajectory.
- **Cache key and `_ARCHIVE_MUST_MATCH`: unchanged.**
  - The lookup reads and never writes.
  - The segmented cache and its key are not involved.
  - `_ARCHIVE_MUST_MATCH` is reused as it stands. It is conservative: a later change to the PMP
    swarm's `PSO_LB`/`PSO_UB` (the sphere or λ_r items) would make the loader refuse until a
    reference is re-chosen. That is the right failure.
- **What does change is the law case's provenance.** The law archive must record which reference
  fed it: path, sha256 of the npz, and the manifest's source and label. Per trajectory it must
  also record the target (t, h, v, γ), the achieved arc-1 end state and the miss. `main.py`'s
  archive call and the harness's `_dispatch` extra both receive these from a module-level
  `LAST_ARC1_WAYPOINT` set by `run_pso_coast_full` (the pattern of `LAST_PSO_COAST_HISTORY`).
- **Git reproducibility** (the archives live in the gitignored `Output/`): force-add the two (or
  three) reference archive triplets under `Tese/src/Output/arc1_reference/<case>/`, the way
  `pmp_reference.npz` is tracked. Copy them, don't move them. The `b750half` npz is bitwise the
  tracked cache's trajectory, so the baseline one duplicates known content.

### 3.5 Vacuum cases

- **Clock:** MECO 146.742 s in `s3half` vacuum, `peg_vacuum` and `peg_vacuum_norot` alike, and
  the same ignition mass (96 570 kg; `INCLUDE_DRAG=False` drops the fairing entirely). The
  vacuum clock is exact.
- **peg_vacuum → `Output/pmp_polish/pmp_vacuum/s3half_start0_20260921_163026/`** (23 952.1 kg,
  γ_p 1.508899, cutoff 405.874 s). Its manifest matches the `peg_vacuum` configuration on every
  problem setting (`INCLUDE_DRAG=False`, rotation on). The harness sets
  `PSO_COAST_ARC1_REFERENCE` to it for that case.
- **peg_vacuum_norot has no valid reference.**
  - The vacuum extremal was flown with rotation and the pseudo-forces on. The no-rotation target
    is the inertial 7 612.7 m/s, against 7 171.9 m/s rotating: a different problem, and the
    loader's guard refuses it (`ENABLE_EARTH_ROTATION`, `INCLUDE_PSEUDO_FORCES` are in
    `_ARCHIVE_MUST_MATCH`).
  - **D1, recommended (a): build a rotation-off vacuum reference.** One PMP swarm under the
    `peg_vacuum_norot` settings at 250×1000, seed 3 to mirror the vacuum reference (measured
    `pmp_vacuum` wall time 7 477 s ≈ 2.1 h). Then the half-step polish recipe
    (`--step 0.00025 --span 0.04 --max-nfev 1000`, ~20–60 min).
  - If the polish fails, as the 250×1000 baseline one did, use the raw swarm and say so.
  - Expect it to differ in kind: with the rotation off the target is a *genuinely* circular
    speed, so the coast-to-apoapsis allowed by the unprojected credit is not available and a real
    last burn should remain.
  - That makes a third PMP artefact that Chapter 6 must describe. Both steps need authorisation.
  - (b) Zero extra runs: target `peg_baseline` and `peg_vacuum` only. Keep the existing
    untargeted `peg_vacuum` / `peg_vacuum_norot` archives as the rotation comparison; they stay
    valid, because the switch is off-by-default and byte-identical. §6.3 then reports targeting
    as its own factor. This narrows your scope, so it is your call.
  - (c) Feed the rotating reference to norot: rejected, wrong problem.

### 3.6 Keeping old paths byte-identical

- **Config, new block in §2.11 of `simulation_parameters.py`:**
  - `PSO_COAST_ARC1_WAYPOINT = False`: when True, arc 1 of apollo/peg_new aims at the PMP
    reference's state at `t_arc1_end`.
  - `PSO_COAST_ARC1_REFERENCE = None`: path to an `indirect_pmp` trajectory archive of the same
    physics. Required when the switch is on; a missing file or a physics mismatch raises.
- **Guarded:** `run_pso_coast_optimization` and `run_pso_coast_full` raise if the switch is on and
  `GUIDANCE_MODE ∉ {apollo, peg_new}`. The precedent `DIRECT_LAW_TERMINATED_CUTOFF` is silently
  inert for other laws. Here a silent no-op would archive a gt run whose manifest claims targeting,
  which is exactly what the harness's `_apply` refuses for unknown names. Under `direct`,
  segmented, `apogee_check` and `indirect_pmp` the setting is never read. Document that as a no-op
  in worktree §4, like every other `PSO_COAST_*` setting.
- **`GuidanceState`:** add `peg_new_fixed_tgo: bool = False`. Reusing `force_planned_tgo` for
  peg_new is not possible: the segmented solver sets it True for every segment, so its peg_new
  segments would change.
- **Harness:** add the two settings to the overrides of `peg_baseline` and `show_apollo`
  (reference = the tracked baseline triplet), `peg_vacuum` (vacuum triplet) and `peg_vacuum_norot`
  (per D1). Leave `SMOKE_BUDGET` alone: the loader is read-only and physics-checked, so smoke
  needs no redirect for it. Then collect `LAST_ARC1_WAYPOINT` into `extra`, and update the module
  docstring and the §6.3/§6.7 comments.
- **Code comments:** replace the "KNOWN SOURCE OF ERROR, documented rather than fixed" block
  (`pso_coast_solver.py:788–803`) with a description that is true under both switch settings.
  Update the CLAUDE.md "Documented, deliberately not fixed" paragraph the same way (a doc edit,
  not a thesis edit).
- **worktree.md:**
  - §2.11: the two settings.
  - §3: the compatibility matrix gains the apollo/peg_new × `pso_coast` × switch cells.
  - §4: gotchas — others raise; the switch is inert off `pso_coast`; the reference must be of the
    same physics; the smoke redirect is not needed; arc 3 is always re-initialised under the
    switch.
  - §2.12b: new archive fields.
- **Tests** (new file `tests/test_arc1_waypoint.py`, plus one each in
  `test_segment_reference_cache.py` and `test_peg_apollo.py`):
  1. `waypoint_at_time` returns archived states exactly at grid stamps, handles duplicated stamps,
     and raises outside coverage.
  2. `peg_new_major_loop_fixed_tgo` at the native t_go reproduces `peg_new_major_loop` (6-tuple,
     tolerance about 1e-6 relative); `vgo_θ` is monotone in `t_go`; the saturation branch returns
     pure radial.
  3. `load_arc1_reference` accepts another seed/budget, refuses rotation-off against rotation-on
     and drag against vacuum, and never writes or builds (monkeypatch `_run_pmp_reference` to
     raise).
  4. **Switch off, byte-identical:** re-fly the archived `peg_baseline` decision vector through
     `run_pso_coast_trajectory` and get the archived J′ (0.791215…) to the last digit (≈ 0.3 s).
  5. **Switch on:** `gs.target is None` and both flags are False throughout arc 3, whether the
     coast is non-zero or zero; `run_pso_coast_trajectory` and `run_pso_coast_full` give the same
     insertion state to about 1e-9 for a fixed x.
  6. Switch on with `gravity_turn` raises.

  The full suite (182 tests) must pass.

### 3.7 Validation ladder (after implementation, each rung authorised separately)

| rung | what | cost |
|---|---|---|
| **V0** | the tests above + full suite | ~2 min |
| **V1a** | one `ra.run_stage1(γ_p = 1.537139)` + pre-ignition coast: confirm the ignition state equals the reference's (68.84 km, 3 381.7 m/s, 13.25°) | seconds |
| **V1b** | **offline arc-1 flights** (single trajectories, no swarm) for each law × {γ_p of the reference, of the case's archive} × t_arc1_end ∈ {300, 355, 380, 400, cutoff, 430} × freeze {2 s, 10 s}. Report the (h, v, γ) miss at `t_arc1_end`, the ignition α and the fraction of the arc spent saturated (apollo) or clipped (peg_new) | ~100 arcs × ~0.1 s |
| **V2** | re-fly the archived x of `peg_baseline` / `peg_vacuum` with the switch on (one trajectory each): how far today's optimum lands from feasible (expected: far, per §1). **Also fly the reference's own timing vector** through each targeted law. The PMP's x[3:7] is already in `pso_coast` layout: baseline `[1446.83, 75.978, 99.990, 1.537139]` gives arc 1 ending at 411.33 s and a 0.025 s arc 3; vacuum `[1432.22, 74.153, 99.984, 1.508899]`. Propellant at that point minus the reference's is the law's **pure tracking loss**, measured before any swarm | seconds |
| **V3** | `run_results_matrix.py --smoke` (all 19 dispatch; the four load their reference), then `--case peg_baseline --budget 50,100 --out Output/arc1_rehearsal` and the same for `show_apollo`. Check: swarm J′ = replay J′; `t_arc1_end` vs cutoff; γ_p vs 1.537; the arc-1 miss; coast length vs 1 447 s | smoke ~2–5 min; each rehearsal 5 000 evals × ~0.18 s ≈ **15 min** |
| **V4** | `run_archive.py compare` of rehearsal vs untargeted archive vs reference (`<dir>::<stem>` for the reference and rehearsal) — figures through the project library only; table of propellant, coast, t_arc1_end, γ_p, peak q | minutes |
| **V5** | production, four cases at 250×1000: peg_baseline ≈ 12.3 h, peg_vacuum ≈ 11.7 h, peg_vacuum_norot ≈ 6.6 h (measured untargeted wall times; the targeted cost per evaluation is within a few %), show_apollo ≈ 11.4 h (extrapolated from 0.164 s/eval at 100×500). **~42 h sequential, ~15 h four-wide** (1.2× slowdown measured for five-wide PMP runs). show_apollo's budget is also frozen decision 13 | 15–42 h |
| (D1a) | rotation-off vacuum reference: swarm + polish, before V3 for norot | ~2.5–3 h |

Stop rules:

- If V1b shows saturation or clipping over most of arc 1 even at the reference's γ_p, stop and
  report. The waypoint would then be out of the laws' reach, not merely hard.
- If V3's J′ does not reconcile, move the peg_new major loop out of the right-hand side
  (`peg_new_external`) before V5.

### 3.8 Chapter 6 consequences (thesis edits remain on hold — this is the list for later)

- **Open decision 18** asked two things: does §6.3 get arc-1 targeting, and is a searched shape
  parameter acceptable. The design answers the first with yes, for peg_new and apollo. It replaces
  the second: nothing is searched; the arc-1 aim is read from the reference at the swarm's own
  coast start.
- **Must be disclosed:**
  1. Under `pso_coast` the arc 1 of peg_new/apollo is **reference-guided**. The law tracks a state
     of the indirect-PMP solution, so these laws are no longer self-contained. Their margin over
     the gravity turn is partly the reference's, and their comparison against the PMP is partly
     circular.
  2. The reference is the **polished** extremal (swarm → Levenberg–Marquardt → γ_p continuation).
     This reverses the 2026-09-17 decision to leave the polish out of the thesis: the method now
     has to be described and cited (`dev-notes/refinement-bibliography-2026-09-22.md`), and the
     reference's single seed (3) and budget (750×1500 start) stated.
  3. **The reference's properties are likely to transfer.** The swarm is expected to move γ_p to
     ≈ 1.537 (§1). The targeted laws would then stage at ~65 km and inherit the Stage-1 max-q of
     52.5 kPa (+20 % over gt) and the Stage-2 peak q ≈ 4.2 kPa at staging. If `t_arc1_end`
     settles near 411 s with a ~1 450 s coast, they inherit the **coast-to-apoapsis with a ~0 s
     last burn** too. That is the geometry the unprojected credit admits (insertion ~125 m/s
     below circular, which the eccentricity column cannot show; already a 6.4 disclosure). It
     then becomes a 6.3/6.7 disclosure.
  4. Both laws hold altitude and vertical velocity and give up horizontal speed at the waypoint.
     State this, and the prescribed-t_go peg_new variant, as a modification of the law, with the
     P12 analogy.
  5. For vacuum/norot, whichever of D1 (a)/(b) is taken.
- **Archived cases that become stale as §6.3 rows:**
  - `peg_baseline`, `peg_vacuum`, `peg_vacuum_norot` (all three are pso_coast). They remain valid
    as the *untargeted* variant; under D1 (b) the two vacuum ones stay as the rotation pair.
  - `show_apollo` has no valid archive yet (pending anyway).
- **Not affected:** the six `gt_*`, `peg_direct` (no coast), `pmp_*`, the other showcases (not
  yet flown) and the segmented cases (on hold, own targets).
- **Downstream:** `results_matrix.csv` rows for the three peg cases; every Chapter 6 figure that
  draws them (§6.3, the §6.5 losses and the §6.6 comparison sets). Regenerating them is frozen
  item 21.

---

## 4. Implementation steps (each waits for its own go-ahead)

1. `peg_guidance_new.peg_new_major_loop_fixed_tgo` + its tests (pure; no behaviour change
   anywhere).
2. `segment_reference.waypoint_at_time` + `load_arc1_reference` + tests (read-only).
3. `pso_coast_solver`:
   - `GuidanceState.peg_new_fixed_tgo`;
   - the peg_new branch reads it;
   - `_arm_arc1_waypoint` and `_begin_arc3` wired into both builders;
   - the guard;
   - `LAST_ARC1_WAYPOINT`;
   - the rewritten arc-1 comment.
   Then the byte-identity test and the builder-agreement test.
4. Config settings (default off) + worktree §2.11/§3/§4/§2.12b + CLAUDE.md paragraph.
5. Archive plumbing: `main.py` pso_coast branch and harness `_dispatch` extra; worktree §2.12b.
6. Reference triplets copied under `Output/arc1_reference/` and force-added (a commit, on your
   word).
7. V0 → V1 → V2 (offline).
8. Harness overrides for the four cases (+ D1 outcome), smoke, rehearsals V3/V4.
9. V5 production, on your authorisation.

Estimated code size: ~40 lines in the guidance module, ~40 in `segment_reference`, ~60 in
`pso_coast_solver`, ~20 in the harness/main, plus ~150 lines of tests.

## 5. Decisions needed from you

- **D1** — `peg_vacuum_norot`: (a) build a rotation-off vacuum reference (recommended; ~2.5–3 h
  plus authorisation of the swarm and the polish) or (b) narrow the targeting to peg_baseline and
  peg_vacuum and keep the untargeted vacuum pair for the rotation factor.
  **User, 2026-09-22: test on the rotation cases only (peg_baseline, peg_vacuum, show_apollo);
  the rotation-off case (peg_vacuum_norot) and its reference are ON HOLD.**
- **D2** — accept the prescribed-t_go peg_new variant (§3.1) as the way peg_new honours
  `t_arc1_end`. It is a modification of the law that Chapter 4/6 will have to state.
- **D3** — accept the literal lookup past the reference's cutoff (§3.3), with no bound and no clamp.
- **D4** — arc-1 freeze threshold: 2 s (segmented precedent, recommended) unless V1b says 10 s.
- **D5** — track the reference triplets in git (§3.4), as `pmp_reference.npz` is.

## 5b. Follow-up answers (2026-09-22, after the user's questions)

**How peg_new's in-loop t_go behaves when a t_go is imposed.** In the prescribed variant the
native t_go is **not computed at all** during arc 1. Every major cycle (2 s) sets
`t_go = t_arc1_end − t`, and the rest follows from it:

- `‖v_go‖` comes from the rocket equation at that t_go;
- the radial channel is solved as today;
- the tangential channel takes the remainder.

There is one t_go, so nothing can conflict or oscillate. It freezes at `t_arc1_end − freeze_thr`.
Arc 3 uses the native t_go exactly as today. The native value is worth **logging as a diagnostic**
in arc 1 (one extra 20 µs call per cycle, in the replay only) so the archive shows the
disagreement.

**The "swarm learns PEG's own t_go" finding does not transfer to arc 1 automatically.** It was
measured on the final burn:

- arc 3 of the three peg coast cases within 0.02 s;
- peg_direct 311.2 s against 311.0 s.

There, the target was fixed (the objective orbit) and a miss at cutoff was penalised directly (J ×4
at ±5 s). Arc 1 differs on both counts:

- **The target moves with the cutoff time.** Consistency needs a fixed point,
  native_tgo(T) = T − t_ign.
- **Nothing penalises a waypoint miss.** The miss is only felt indirectly, through arc 3.

The ignition-time predictions of §1 locate that fixed point:

- **peg_baseline's kick:** at ~416 s, 5 s past the reference's cutoff, where the target is
  already coasting.
- **peg_vacuum's kick:** at ~420 s, 14 s past its cutoff.
- **The reference's own kick:** nowhere in 250–430 s. The native t_go stays 3.4–5.2 s short, so
  the law would arrive early and fly frozen coefficients, ≈ +180 m/s of unsteered thrust.

So a native peg_new would push the swarm either past the cutoff or onto a kick chosen for
consistency rather than performance. These are first-order predictions made at ignition; the
closed loop could differ.

**Proposed test of the hypothesis (needs the user's OK):**

- A sub-setting `PSO_COAST_ARC1_PEG_NEW_TGO = "prescribed"` (default when the switch is on) or
  `"native"`.
- `"native"` simply leaves `peg_new_fixed_tgo` False, so it is the existing code path with no
  extra guidance code.
- apollo has no native option, because its own estimate ignores `gs.target` and aims at the final
  orbit.
- Compare the two in V1b (fixed `t_arc1_end`, closed loop) and, if V1b is ambiguous, in one
  50×100 rehearsal each.

**Switch and comparison workflow.**

- `PSO_COAST_ARC1_WAYPOINT = False` is the previous behaviour, byte-identical and pinned by the
  re-fly test in §3.6. The existing `Output/results_matrix/` archives therefore **are** the
  "previous results"; nothing needs re-flying for the comparison.
- The switch-on runs go to their own root.
- Proposed harness flag `--arc1-waypoint`:
  - switches targeting on for exactly the eligible cases (peg_baseline, peg_vacuum, show_apollo;
    norot on hold);
  - defaults `--out` to `Output/results_matrix_arc1_waypoint/`, so it can never overwrite the
    untargeted set.
  - It is preferred to `--set … --only peg_vacuum`, because `--only` matches substrings and would
    catch `peg_vacuum_norot`.
- Compare with `run_archive.py compare Tese/src/Output/results_matrix/peg_baseline::peg_baseline
  Tese/src/Output/results_matrix_arc1_waypoint/peg_baseline::peg_baseline`. Each case sits in
  its own folder, and `<dir>` resolves from the working directory. That overlays the two
  trajectories and tables the manifest settings that differ, where the switch shows as the one
  factor.

## 6. Still on hold, untouched by this plan

Thesis edits; the segmented-case re-fly; the Stage-2 ignition attitude-jump treatment; every
frozen backlog item (including 13 — the showcase budget — which gates show_apollo's production run
— and 23, which this design makes unnecessary for arc 1). The unprojected rotation credit is
unchanged throughout: targets are read ground-relative from the reference and never converted.
