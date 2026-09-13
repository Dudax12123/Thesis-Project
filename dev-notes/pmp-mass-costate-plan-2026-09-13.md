# Adding the mass costate to `indirect_pmp` — plan (2026-09-13)

Status: **plan only, nothing implemented — and optional:** §1 shows the free-time conditions can be
written correctly, and satisfied, without `λ_m`. Prerequisite done the same day: the Stage-2 arc is
flown in the inertial frame (`INDIRECT_PMP_STAGE2_FRAME = "inertial"`), which is what makes
`λ_s ≡ 0` exact and the terminal target a circular orbit of the equations being integrated.
Evidence for everything below: `dev-notes/pmp_local_refine.py`, logs in
`Tese/src/Output/pmp_refine/`.

## 1. Why the formulation has no mass costate

**It follows its source.** `indirect_pmp` was implemented on 2026-05-27 (`91ae821`) from
Pontani, *Particle Swarm Optimization of Ascent Trajectories of Multistage Launch Vehicles*,
Acta Astronautica 94(2):852–864, 2014 (`pontani2014particle`), Eqs. 28–39: costates `λ_r, λ_v,
λ_γ`, `λ_s = 0`, and mass entering only through `T/m`. In that formulation every burn is at
constant thrust and the arc durations (coast, last burn) are decision variables of the swarm, so
`m(t)` is a known function of time on each arc. The steering law depends on `λ_v, λ_γ` only, and
`λ_m` never feeds back into the states or the control — so for *generating* candidate extremals it
is genuinely not needed. That is a standard and legitimate simplification.

**What the simplification does — and does not — cost.** Without `λ_m` the reduced Hamiltonian
`H = λ·f` is not conserved on the burns (−5.93 at ignition → −26.67 at cutoff on the production
solution). That does **not** make the free-time conditions unwritable. With the arc durations as
parameters, stationarity of `J′ = λ0·(D1 + D3) + ν·Ψ(x_f)` in each duration can be written with the
reduced H alone: control variations drop out because the PMP steering makes `∂H/∂α = 0`, and the
shift of the mass profile in the last burn integrates to `H_f^last − H_0^last` because
`dH/dt = −ṁ λ·∂f/∂m` there.

| duration | `λ(t_f)·∂x_f/∂D` | condition |
|---|---|---|
| coast `Dc` | `H_coast_end` (engine off) | `H_coast_end = 0` |
| last burn `D3` | `H_burn_end` | `H_burn_end = −λ0 < 0` |
| first burn `D1` | `H_burn1_end − H_last_burn_start + H_burn_end` | `H_burn1_end = H_last_burn_start` (both engine on) |

**Verified 2026-09-13** (`dev-notes/pmp_duration_conditions.py`, log
`Tese/src/Output/pmp_refine/pmp_duration_conditions.log`): central differences at the inertial-frame
refinement point B match the closed forms to 1e-7–1e-6 for all three durations. The conditions are
**satisfiable**: solving orbit + the two equalities at fixed `γ_p` from B reached
`H_coast_end = −0.018`, `H_burn1_end − H_last_burn_start = −0.004` (from −0.62 and +5.98 at B),
orbit errors 0.24 m / 7 mm/s, `H_burn_end = −1.54 < 0`, and **20 187.9 kg** left — 158 kg more than
B (20 030.3 kg), which SLSQP had returned from four starts. B is a local optimum of the direct
problem, not an extremal.

**What is wrong is the equation in the code.** `H_burn_end + H_coast_end − H_burn_start = 0` takes H
at Stage-2 ignition, a fixed instant that no condition involves. Pontani's written form
`H_f^last + H_f^coast − H_0^last = 0` equals `H_f^coast + (H_f^last − H_0^last)`: at an extremal the
first term vanishes, so it holds only if H is conserved across the last burn — true for a Hamiltonian
that includes `λ_m·ṁ`, not for the reduced one (which is why variant A2, `H_0^last` at the last-burn
start, was also infeasible). That is the one place a mass costate is needed: to use the paper's
combined equation as written. How the paper itself defines H is not yet checked.

**So `λ_m` is not required for correct optimality conditions.** Both formulations are square at
fixed `γ_p`. Today's has 5 effective unknowns (costate direction 2 + three durations) against 3 orbit
+ 2 duration conditions — the system solved above. The `λ_m` form has 4 (direction of
`(λ_r, λ_v, λ_γ, λ_m)` 3 + `t_f`) against 3 orbit + `H(t_f) = 0`, with `λ_m(t_f) < 0` as a sign
check. What `λ_m` adds is one fewer unknown (two switch times traded for one costate), a burn/coast
structure that comes out of `S` instead of being imposed, and a conserved `H` as an integrity check.
Both run under PSO. The rest of this plan is that upgrade — worth having, no longer a prerequisite.

**In this project a `λ_m` existed once, and was removed as collateral.** `476b474` (2026-07-01)
added it inside the *full-ascent* extension, as a passive diagnostic to confirm H conservation on
the Stage-1 powered arc (`λ_m(t_f) = 0` applied as a shift, objective unchanged). On 2026-07-08/09
(`d20ac94`) the whole full-ascent extension was wound back to `005b5c1` because its Stage-1 arc did
not reproduce `run_stage1`'s gravity turn (memory: `full-ascent-indirect-pmp`). `λ_m` went with it;
the Stage-2-only path never had it, and the decision was not revisited.

**Why it went unnoticed.** The penalty never reached zero (0.016–0.063 of J′ across budgets). On
2026-09-10 that was read as under-convergence because it shrank with budget. Only a local
refinement, which can satisfy constraints exactly, showed it is not locally reachable — because it
is the wrong combination of H values, not because a costate is missing.

## 2. Target formulation (Stage 2, inertial, planar)

- States `x = [r, v, γ, m]`; `s` is cyclic, so `λ_s ≡ 0` (exact only without latitude-dependent
  terms — hence the inertial frame first).
- Controls: steering `α`; engine on/off `δ ∈ {0, 1}` at fixed thrust `T`, `ṁ = −δT/c`, `c = Isp·g₀`.
- Cost (Mayer form): `J = −m(t_f)` — equivalent to minimum burn time.

```
H = λ_r v sinγ + λ_v (δT cosα/m − g sinγ) + λ_γ [δT sinα/(m v) − (g/v − v/r) cosγ] − λ_m δT/c,   g = μ/r²
```

- `λ̇_r, λ̇_v, λ̇_γ`: unchanged (`costate_derivatives`, with the thrust term multiplied by `δ`).
- **New:** `λ̇_m = −∂H/∂m = δ (T/m²)(λ_v cosα + λ_γ sinα / v)`. With the optimal steering this is
  `−δ T ‖p‖ / m² ≤ 0`, where `p = (λ_v, λ_γ/v)`.
- Steering: unchanged, `(cosα, sinα) ∝ −p` (`pmp_control_law`).
- **Switching function:** `S = −(T/m)‖p‖ − λ_m T/c`. Engine on where `S < 0`, off where `S > 0`;
  switches at `S = 0`. This is the primer-vector burn condition, and it replaces the swarm's
  `coast_start_pct` and `delta_tc`.
- Boundary conditions: `r(t_f) = r_T`, `v(t_f) = √(μ/r_T)`, `γ(t_f) = 0`; `m(t_f)` free with
  cost `−m` ⇒ **`λ_m(t_f) = −1`** (λ₀ = 1 now fixes the costate scale, replacing the ‖λ‖ = 1
  gauge); `t_f` free ⇒ **`H(t_f) = 0`**, and since the problem is autonomous, **`H ≡ 0` on the
  whole Stage-2 arc** — a checkable invariant, and better imposed at ignition than at `t_f`.
- Corners: costates and H continuous at switches; `λ_m` continuous across a fairing jettison inside
  Stage 2 (the mass jump does not depend on the state).
- **Square shooting system:** unknowns `λ_r0, λ_v0, λ_γ0, λ_m0, t_f` (5); conditions 3 terminal +
  `λ_m(t_f) = −1` + `H = 0` (5). `γ_p` stays an outer parameter — Stage 1 carries no costates.

## 3. Steps

0. **Fix the conditions in the current objective (independent of `λ_m`, do first).** Replace the
   transversality penalty with the verified pair `|H_coast_end| + |H_burn1_end − H_last_burn_start|`
   plus the sign check `H_burn_end < 0` (both diagnostics already returned by the solver). Optionally
   confirm against Pontani (2014) how the paper defines H.
1. **Derivation check.** A sympy script derives `λ̇` from H and compares against
   `costate_derivatives` at random points (the existing three and `λ̇_m`) and derives `S`.
   `tests/test_pmp_costates.py`: finite-difference `−∂H/∂x` against the implemented ODEs, and H
   conservation along a propagated extremal — the test that would have exposed this.
2. **Guidance module.** New `mass_costate_rate`, `switching_function`, `hamiltonian_full`; existing
   functions untouched.
3. **Solver.** `run_indirect_trajectory_mc(λ_r0, λ_v0, λ_γ0, λ_m0, t_f, γ_p)` on the 9-state
   `[s, r, v, γ, m, λ_r, λ_v, λ_γ, λ_m]`; engine state from `sign(S)`, with a `solve_ivp` event on
   `S` (both directions, integration restarted at each root), propellant depletion as a terminal
   event, a cap on the number of switches; ignition delay, fairing handling, frame conversion and
   ground-relative reporting exactly as the current solver.
4. **Config and docs.** `INDIRECT_PMP_FORMULATION = "pontani2014" | "mass_costate"`, default
   `"pontani2014"` until the gates in step 6 pass; `worktree.md` §2.9; `segment_reference` cache
   key.
5. **Solve strategy.**
   a. Swarm over `[λ_r0, λ_v0, λ_γ0, λ_m0, t_f, γ_p]` with penalties on the five residuals. Costate
      bounds are no longer `[−1, 1]` — take their scale from a converged solution.
   b. Newton / least-squares shooting at fixed `γ_p` from the swarm point, arc structure frozen.
   c. Outer 1-D minimisation over `γ_p`.
   Warm start from the best direct refinement: along its burns `tan α = λ_γ/(v λ_v)` fixes
   `λ_v, λ_γ` up to scale, their ODEs fix `λ_r`, and `S = 0` at its switch fixes `λ_m`.
6. **Validation gates** — all must pass before a Chapter 6 row uses the formulation:
   - `H ≡ 0` along every arc to integrator tolerance;
   - `S = 0` at every switch, `S < 0` on burns and `> 0` on coasts;
   - terminal errors ≤ 1 m / 1 mm/s / 1e-5°, and `λ_m(t_f) = −1`;
   - propellant **≥ the best direct refinement of the same problem** (same frame, same coast bound):
     an extremal that loses to a direct solution is not the optimum;
   - the same answer from at least three starts (swarm seed, warm start, perturbation);
   - a clean archive row (fix the NaN eccentricity clamp at `rocket_ascent.py:841` first).
7. **Downstream.** Re-fly `pmp_baseline` and `pmp_vacuum`; rebuild `pmp_reference.npz`;
   Chapter 3/5 text (report edits on hold) presents it as an extension of Pontani (2014).

## 4. Risks

- Costate scaling for the swarm once the unit-norm gauge is gone.
- A change in the number of switches makes the shooting function non-smooth — freeze the structure
  inside Newton, verify it afterwards.
- Singular arcs (`S ≡ 0` over an interval): unlikely for this problem, but must be detected rather
  than assumed away.
- The direct refinement in the inertial frame pushes the coast towards the 2000 s bound, so `t_f` may
  sit near the time and propellant limits; the laws' own coast bound is 1000 s, which is a fairness
  question for Chapter 6 independent of `λ_m`.
- Stage 1 of `indirect_pmp` is still flown pseudo-force-free in the rotating frame — a separate
  remaining difference from every other architecture.

Effort: derivation and tests ~½ day; solver ~1 day; tuning and validation 1–2 days; production runs
~1.5 h each.
