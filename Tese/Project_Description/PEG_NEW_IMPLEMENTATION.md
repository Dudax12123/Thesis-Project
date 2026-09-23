# `peg_new` — implementation notes

Powered Explicit Guidance in its velocity-to-be-gained form, as implemented in
[`Tese/src/Guidance/peg_guidance_new.py`](../src/Guidance/peg_guidance_new.py).

**Status (2026-09-23).** The implementation was re-aligned with its source on this date
(Section 6). Every `peg_new` result archived before then was flown with the old law and is stale.

**Source.** Mahajan & Condon [1], section *"PEG Derivation from First Principles"*: Algorithm 1
and eqs (1)–(76). The paper re-derives the Space Shuttle's PEG [2–4] from Pontryagin's principle.
Until 2026-09-23 the module's docstring credited *"Sagliano, Mooij & Theil — PEG Derivation from
First Principles"*. That was a misattribution: the section title, Algorithm 1 and the equation
numbers are Mahajan & Condon's.

---

## 1. What the law computes

`peg_new` steers the second stage from ignition to a target state `(r_T, v_θT, v_rT)` at burnout.
Its single output is the angle of attack `α = β − γ`, where `β` is the thrust pitch from the local
horizontal and `γ` the flight-path angle.

It is split in two, as in every PEG mechanisation:

- **Major loop** (`peg_new_major_loop`, every `PEG_MAJOR_LOOP_RATE` = 2 s).
  - Solves for the velocity-to-be-gained `v_go`, the time-to-go `t_go` and the steering
    constants `(t_λ, λ'_r)`.
  - Freezes once `t_go` falls below the freeze threshold.
- **Minor loop** (`peg_new_alpha`, every integration step). Evaluates the linear-tangent steering
  from the frozen constants at the time elapsed since the last major-loop update.

| function | role | paper |
|---|---|---|
| `compute_vgo_with_gr` | initial `v_go`, `t_go` for a given constant radial gravity | Alg. 1 step 3 |
| `compute_thrust_integrals` | `S0, L1, S1, t_λ` for constant thrust | eqs (64), (66)–(69) |
| `compute_lambda_r` | radial steering rate `λ'_r` | eq (71), step 10 |
| `_steering_constants` | steps 4, 6, 10, 13 for one `v_go` | — |
| `_predict_burnout` | fly the predicted burn; gravity integrals by quadrature | steps 15–16 |
| `peg_new_major_loop` | predictor–corrector until the velocity miss vanishes | steps 3–20 |
| `peg_new_alpha` | steering angle from the constants | eq (72) |
| `peg_new_tgo` | `t_go` only, for other laws' `TGO_ESTIMATOR="peg_new"` | — |

---

## 2. Optimal-control basis (paper eqs 1–24, 62–65)

The vehicle obeys

$$\dot{\mathbf r}=\mathbf v,\qquad \dot{\mathbf v}=\mathbf g+\frac{T}{m}\hat{\mathbf u},\qquad \dot m=-\frac{T}{c},$$

where `g` is the sum of all natural forces and `c` the exhaust speed.

- **The cost.** Minimising propellant, `J = −m(t_f)`, with the Hamiltonian of eq (5) gives the
  primer-vector steering `û = −λ_v/‖λ_v‖` (eq 6).
- **The flat-planet assumption.** Conventional PEG assumes a constant gravity field in the
  *costate* dynamics. Then `λ_r` is constant and `λ_v(t) = λ_v(t₀) − λ_r(t − t₀)`, so the thrust
  direction is a linear function of time, normalised (eqs 16, 24).
- **Free downrange.** With the downrange position free, `λ_r` has no downrange component
  (eq 15 → 60). The in-plane law reduces to the **linear tangent law** (eq 22):
  `tan β` is linear in `t`.
- **Jaggers' "Coke Machine" assumption** [2, 5]: the position and velocity costates stay
  orthogonal through the burn, and the thrust turns through a small angle (eqs 62–63). The thrust
  integrals then no longer depend on the costates. They are fixed by `t_go` and the thrust
  profile alone.
- **The reference time.** With `t_λ = L1/L0` (eq 64), the velocity costate at `t_λ` points along
  `v_go` (eqs 65, 70).

The steering is therefore (eq 72, exact normalised form):

$$\hat{\mathbf u}(t)\;\propto\;\frac{\mathbf v_{go}}{L_0}+\lambda'_r\,(t-t_\lambda)\,\hat{\mathbf r},
\qquad L_0=\lVert\mathbf v_{go}\rVert .$$

---

## 3. The predictor–corrector (paper eqs 44–61, 66–71; Algorithm 1)

Integrating the equations of motion over the remaining burn `t_go = t_f − t₀` (eqs 44–49):

$$\mathbf v(t_f)-\mathbf v(t_0)=\mathbf v_G+\mathbf v_T,\qquad
\mathbf r(t_f)-\mathbf r(t_0)=\mathbf v(t_0)\,t_{go}+\mathbf r_G+\mathbf r_T,$$

$$\mathbf v_G=\int_{t_0}^{t_f}\mathbf g\,dt,\qquad
\mathbf r_G=\int_{t_0}^{t_f}\!\!\int_{t_0}^{t}\mathbf g\,ds\,dt ,$$

where the subscript `T` terms are the thrust contributions.
- **Velocity to be gained** (eq 61): `v_go = v_d − v(t₀) − v_G`, the velocity the thrust must
  still supply.
- **Constant-thrust integrals** (eqs 66–69), with `τ = m(t₀)·c/T`:

$$S_0=-L_0(\tau-t_{go})+c\,t_{go},\qquad L_1=L_0t_{go}-S_0,\qquad
S_1=S_0\,\tau-\tfrac12 c\,t_{go}^2,\qquad t_\lambda=L_1/L_0 .$$

  `S1` is the closed form of `∫∫(T/m)·s ds dt` (eq 55).
- **The steering rate** follows from the position still to be gained, `r_go` (eq 71):

$$\lambda'_r=\frac{L_0\,r_{go}-S_0\,v_{go,r}}{L_0\,(S_1-t_\lambda S_0)},\qquad
r_{go}=r_T-r-v_r\,t_{go}-r_{G,r}.$$

**Algorithm 1**, summarised:
- Initialise `v_go`, `r_G` and `v_G` (step 3).
- Then repeat:
  - take `t_go` from the rocket equation (step 4);
  - evaluate the thrust integrals (step 6);
  - form `v_go` and `r_go` from the gravity integrals (steps 9–10);
  - update the costates (step 13);
  - **predict** the cutoff state (step 15), and **update `v_G` and `r_G` by numerical quadrature
    along that predicted trajectory** (step 16);
  - **correct** `v_go` by the predicted velocity miss `v_d − v_P` (steps 18–19).
- Stop when `v_go` has converged (step 20).

The paper notes that the fully analytical Shuttle variant gets the gravity integrals from a coast
arc kept close to the powered one [3], or from Jaggers' approximate integration [2, 4] (p. 13).

---

## 4. How the code realises it

### 4.1 Frame, state and force model

- **The state.** The simulator's state is `[s, r, v, γ, m]`, with speed and flight-path angle
  **ground-relative** (rotating frame). The law uses `v_r = v·sin γ`, `v_θ = v·cos γ` and radius
  `r` (never altitude).
- **Planar and polar.** The law works in the vertical plane, in the local `(r̂, θ̂)` frame.
  Downrange is free, so only the radial part of `λ'_r` exists. The out-of-plane projection of
  step 17 is trivial.
- **The force model.** In polar components the natural-force term `g` of the paper carries the
  frame kinematics. These two components are what the predictor integrates as "gravity":

  $$g_r=-\frac{\mu}{r^2}+\frac{v_\theta^2}{r},\qquad g_\theta=-\frac{v_r v_\theta}{r}.$$

- **What the law does not model.** The Earth-rotation Coriolis and centrifugal terms, which the
  simulator applies in the Stage-2 equations of motion (`_stage2_ode_guidance`), are **not** in
  the law's model. The closed loop absorbs them by re-solving every 2 s. Stage 2 is flown without
  drag, so there is none to model.

### 4.2 Targets

The target is `(r_T, v_θT, v_rT)`. The callers supply:

- **Final orbit:** `r_T = R_E + TARGET_ORBITAL_ALTITUDE`, `v_rT = 0`, and
  `v_θT = √(μ/r_T) − ω·r_T·cos φ_L` (`earth_rotation.v_circular_rotating`). That is the
  unprojected rotation-credit convention recorded in `CLAUDE.md`.
- **Intermediate waypoint** (`SegmentTarget`, the segmented mode, `COAST_METHOD="reference_track"`):
  `v_θT = v·cos γ`, `v_rT = v·sin γ` of the waypoint.
- `v_theta_T=None` falls back to the inertial `√(μ/r_T)`.

### 4.3 Major loop, step by step

| Alg. 1 | code | what it does |
|---|---|---|
| 3 | `compute_vgo_with_gr` with `g_r` at the current position; `r_G,r = ½ g_r t_go²` | initial guess; it only sets the starting point |
| 4 | `_steering_constants` | `L0 = max(‖v_go‖, 1 m/s)`, `t_go = τ(1 − e^{−L0/c})` |
| 6 | `compute_thrust_integrals` | eqs (66)–(69) including **eq (69)** |
| 10, 13 | `compute_lambda_r` | `r_go` from the quadrature `r_G,r`, then eq (71) |
| 15–16 | `_predict_burnout` | see below |
| 18–19 | `peg_new_major_loop` | `v_miss = (v_rT − v_rP, v_θT − v_θP)`; `v_go += ω·v_miss` |
| 20 | `peg_new_major_loop` | stop when `‖v_miss‖ < CORRECTOR_TOL` (0.05 m/s) or after `CORRECTOR_MAX_ITER` (20) passes |

After the loop, `t_go`, `t_λ` and `λ'_r` are recomputed from the final `v_go` and `r_G`, and the
6-tuple `(v_go,r, v_go,θ, L0, t_go, t_λ, λ'_r)` is returned.

**The predictor (`_predict_burnout`).**
- It flies the planar model `ṙ = v_r`, `v̇_r = a·u_r + g_r`, `v̇_θ = a·u_θ + g_θ`, with
  `a = T/(m − ṁt)` and `û` from eq (72) under the current constants.
- It runs for `t_go`, by classical RK4 with `n = max(2, ⌈t_go/PREDICTOR_STEP⌉)` steps
  (`PREDICTOR_STEP` = 20 s).
- On the same steps it integrates `v_G,r = ∫g_r dt` and `r_G,r = ∫v_G,r dt = ∫∫g_r`. The gravity
  integrals are therefore quadratures along the predicted *powered* trajectory, as step 16
  prescribes. They are not a two-point average.
- The tangential term `g_θ` acts on the predicted `v_θ`, so it enters the corrector through the
  velocity miss.
- It returns the predicted `v_rP`, `v_θP` and `r_G,r`.
- If the burn would consume the vehicle's whole mass (`t → τ`) there is nothing to predict: it
  returns NaN.

**The corrector step length.** Step 19 adds the full miss (`ω = 1`). Here
`ω = 1/σ ∈ [0.25, 2]`, where

$$\sigma=-\frac{(\mathbf m_k-\mathbf m_{k-1})\cdot\mathbf s_{k-1}}{\lVert\mathbf s_{k-1}\rVert^2}$$

is the fraction of the previous step `s_{k−1}` that the predicted burnout velocity took up. This is
a two-point (Barzilai–Borwein) step length [6]. The fixed point `v_miss = 0` is unchanged.

The reason is measured behaviour:
- When arc 1 is aimed at the final orbit from a low ignition, the steering turns through ~150°,
  far outside the small-angle assumption of eqs (62)–(63). There the unit step overshoots, the
  miss alternates in sign, and it shrinks by only ~0.65 per pass (24 passes).
- The secant step converges in 14.
- Where the unit step already converged quickly (5–7 passes), the secant step takes about the
  same.

**Targets that cannot be reached.** Take a large radius error with little `v_go` left, for example
a vehicle falling at 84 km with 400 kg of propellant, asked for a 500 km orbit. There is no fixed
point.
- The corrector stops after 20 passes.
- Or, if a pass would demand a burn longer than `τ`, it steps back to the last `v_go` whose burn
  could be predicted.

### 4.4 Minor loop (`peg_new_alpha`, eq 72)

$$u_r=\frac{v_{go,r}}{L_0}+\lambda'_r\,(t_s-t_\lambda),\quad u_\theta=\frac{v_{go,\theta}}{L_0},\quad
\beta=\operatorname{atan2}(u_r,u_\theta),\quad \alpha=\beta-\gamma,$$

with `t_s` the time since the last major-loop update and `(u_r, u_θ)` normalised.

### 4.5 Where it runs, and the freeze

| caller | how the major loop is driven |
|---|---|
| `pso_coast_solver._compute_alpha_stage2` | inside the ODE right-hand side, every `PEG_MAJOR_LOOP_RATE` of integrator time |
| `direct_pso_solver._fly_law_terminated_burn`, `reference_track_solver.fly_law_terminated_arc` | outside the ODE, on accepted states; the burn ends when the law's own `t_go` expires |
| `rocket_ascent.rocket_dynamics` (legacy `apogee_check`) | inside the right-hand side, module globals |
| `peg_new_tgo` | `t_go` for `apollo` (and the closed-loop tangent laws) when `TGO_ESTIMATOR="peg_new"`, on **every** right-hand-side evaluation |

The constants freeze once `t_go` falls below the freeze threshold: `APOLLO_FREEZE_THRESHOLD`
= 10 s, or a `SegmentTarget.freeze_threshold`. After that the minor loop flies them open-loop to
cutoff.

**The freeze must stay near 10 s.**
- For near-constant acceleration, `S1 − t_λ S0 ≈ −a·t_go³/12`. So any radius error left near
  cutoff drives `λ'_r` like `r_go/t_go³`.
- Below a few seconds the corrector has no fixed point, and `t_go` stops decreasing.
- Measured on the reference-tracking run: with a 2 s freeze, `t_go` wandered between 1.5 and 10 s
  for the last 90 s, and the burn ran to propellant exhaustion.
- With a 10 s freeze the endgame was clean.
- The segmented mode's `SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD` (2 s) is affected (`worktree.md`).

---

## 5. Departures from the paper, all deliberate

| # | paper | `peg_new` | why |
|---|---|---|---|
| 1 | 3-D inertial vectors | planar polar components in the Earth-fixed frame | the simulator is planar and ground-relative; the frame terms go into `g_r`, `g_θ` |
| 2 | "any arbitrary force model" | two-body gravity + polar frame terms; no Earth-rotation pseudo-forces | keeps the law self-contained (no latitude/azimuth input), like the other laws; the closed loop absorbs the rest |
| 3 | analytical cutoff prediction (eqs 74–75) with analytical gravity integrals, *or* the generalised Algorithm 1 | analytical constant-thrust integrals for the steering (eqs 66–71), numerical RK4 prediction and gravity quadrature (step 16) | the step-16 quadrature is what the old two-point average got wrong |
| 4 | step 19: `v_go += v_miss` | `v_go += ω·v_miss`, secant `ω` | convergence when the turn angle is large (§4.3) |
| 5 | step 20: until converged | `‖v_miss‖ < 0.05 m/s`, at most 20 passes, stop at a burn longer than `τ` | a bounded cost inside a swarm; unreachable targets |
| 6 | — | `L0 ≥ 1 m/s` | keeps `t_go > 0` at cutoff |
| 7 | no freeze: steps 4–19 repeat every guidance cycle until cutoff | the constants freeze once `t_go` falls below `APOLLO_FREEZE_THRESHOLD` (10 s; `SegmentTarget.freeze_threshold` for an intermediate segment) and are flown open-loop to cutoff; a law-terminated burn ends at the frozen `t_go` | near cutoff `λ'_r` grows like `r_go/t_go³` and the corrector loses its fixed point (§4.5). Teren [7] stops the major loop about 10 s before cutoff for the same reason and flies the last coefficients, rate term kept. A burn that starts inside the window gets one frozen cycle (§9) |

---

## 6. History: what was wrong before 2026-09-23

The pre-realignment code, as flown in every archived `peg_new` run:

| defect | effect at the PMP reference's Stage-2 ignition |
|---|---|
| `S1 = S0·t_go − c·t_go²/2` (τ replaced by `t_go`) | `S1 − t_λS0` 2.8× too large, so `λ'_r` 2.8× too small |
| `v_G`: each "trapezoid" pass averaged the *previous average* with burnout gravity. After 3 fixed passes it sat 7/8 of the way to `g_end`, and the convergence test never fired | mean radial gravity −1.48 m/s² against the true −5.42 m/s²; `v_go,r` −29 m/s against the +876 m/s actually needed |
| `r_G = ½·ḡ·t_go²` | gravity drop over arc 1 modelled as 49 km against the true 216 km |
| burnout radius predicted without thrust; no tangential term | minor |

Together these commanded **α = −20.2°** at ignition, where the indirect-PMP optimum flies +2.8°.
The law thought it must thrust downward to lose 53 km that gravity would in fact take away, and
it recovered later with a ~19° pitch-up. Tracking the PMP's arc 1, it lost 3.7 t (2 s freeze) or
missed the waypoint by 24 km (10 s freeze).

---

## 7. Verification

**Unit tests** — [`tests/test_peg_new_predictor.py`](../src/tests/test_peg_new_predictor.py):

- the thrust integrals equal numerical quadratures of their definitions (eqs 52–55) to 1e-10;
- `_predict_burnout` matches a tight `solve_ivp` of the same model: velocities within 0.05 m/s,
  `r_G` within 50 m of −216 km;
- the constants the loop returns, flown open-loop, reach the target velocity within 0.1 m/s,
  both for the PMP waypoint and for the 500 km orbit from the same ignition;
- the ignition pitch climbs like the optimum (15°–19°, `λ'_r < 0`) and `t_go` is within 1 s
  of the PMP's arc-1 burn;
- an unreachable target and a nearly finished burn return finite values.

Full suite: 189 passed. `test_direct_grid.py`'s `peg_direct` pin was re-set to the realigned
law's value at the archived decision vector.

**Closed loop.** `dev-notes/arc1_reference_track.py` flies `peg_baseline`'s configuration with no
optimiser. It takes the PMP reference's kick and coast length, and uses its coast-start state as
the arc-1 target. Since 2026-09-23 that flight is `COAST_METHOD="reference_track"`
(`Simulation/reference_track_solver.py`) and the results-matrix case `show_ref_track` (§6.7), which
reproduces the "after" column below bit for bit; the script imports it and adds the controls. Archives are in `Tese/src/Output/arc1_reference_track_pegfix/`; the pre-fix
ones are in `…/arc1_reference_track/`.

| | before | after | PMP reference |
|---|---|---|---|
| ignition α | −20.2° | **+3.77°** | +2.84° |
| arc-1 pitch programme | −7.0° → 4.0° | 17.0° → 10.1° | 16.1° → 9.4° |
| predicted cutoff `t + t_go` over arc 1 | drifts 406.5 → 423.9 s | **411.4–411.7 s** | 411.33 s |
| arc-1 burn (freeze 10 s) | 259.64 s | 257.39 s | 257.33 s |
| waypoint miss (freeze 10 s) | Δh −24.1 km, Δv −20.9 m/s | **Δh +0.03 km, Δv −0.11 m/s, Δγ +0.045°** | — |
| propellant vs reference (freeze 10 s) | −1 255 kg | −402 kg | 0 |

Most of the remaining −402 kg is the 1.4 s final burn. That burn gets a single frozen cycle and
cannot correct the 4.8 km apoapsis error left by the +0.045° (≈ 6 m/s radial) at arc-1 cutoff.
The likely source of that residual is the Earth-rotation terms the law does not model (§5, #2).
This is inferred, not measured.

Reproduce:

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe -m pytest Tese/src/tests/test_peg_new_predictor.py -q
```

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/arc1_reference_track.py --runs ref_track --freeze 10 --out Tese/src/Output/arc1_reference_track_pegfix
```

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe Tese/src/run_results_matrix.py --case show_ref_track
```

---

## 8. Cost

| | before | after |
|---|---|---|
| one major loop along the PMP reference's arc 1 (ignition / 250 s / 330 s / 400 s) | 20 µs | 450 / 240 / 74 / 23 µs |
| `peg_new` trajectory under `pso_coast` | 0.091 s | 0.125 s |
| `apollo` trajectory, `TGO_ESTIMATOR="peg_new"` | 0.119 s | 0.339 s |
| `apollo` trajectory, `TGO_ESTIMATOR="rocket_equation"` (results matrix) | 0.075 s | 0.078 s |

Most passes are spent at ignition (5–7) and fall to 1–3 late in the burn. Warm-starting `v_go`
from the previous cycle, the Shuttle practice [3], would cut this further. It is not implemented.

---

## 9. Open items

- Earth-rotation pseudo-forces are not in the prediction (§5, #2). This is the likely source of
  the ~6 m/s radial residual at a frozen cutoff.
- Every archived `peg_new` case is stale: `peg_baseline`, `peg_vacuum`, `peg_vacuum_norot`,
  `peg_direct`, and the segmented `peg_new` segments. So is anything flown with
  `TGO_ESTIMATOR="peg_new"`.
- Chapter 4, §"Vector Predictor–Corrector Variant" (`Thesis_Guidance.tex`), still describes the
  old trapezoid. It will be rewritten when thesis edits resume.
- A burn that starts inside the freeze window (§5, #7) is one frozen cycle. `show_ref_track`'s
  1.43 s arc 3 is the case in point. Its `λ'_r` is large enough that α swings −90° → +90°, and it
  ends 6.5 m/s short of the target speed. It still inserts into a 492 × 505 km orbit, 402 kg behind
  the reference. Two remedies were measured on 2026-09-23 with the law patched in memory:
  - Setting `λ'_r = 0` for `t_go ≤ 10 s`, in the corrector's prediction as well as the steering,
    makes arc 3 exact and saves 341 kg. Normal burns give up 30–46 m of altitude.
  - A turn-angle cap in the form of von der Porten et al. 2018 (eq. 4), with θ_max between 20° and
    45°, gives the same arc-3 result. It binds on `pso_coast` flights, where the turn angle reaches 83°.
  - No published value for θ_max was found.
  - **Both are on hold** by decision: the orbit is acceptable as it is.

---

## References

1. B. Mahajan and G. L. Condon, "Enhancements to Space Shuttle Powered Explicit Guidance for
   Planetary Ascent and Descent," AAS 25-844 (preprint), 2025. Section "PEG Derivation from First
   Principles", eqs (1)–(76) and Algorithm 1. Local copy:
   `Desktop/Tese/References/PEG_ASC25_Mahajan - PEG_recent.pdf`.
2. R. F. Jaggers, "An explicit solution to the exoatmospheric powered flight guidance and
   trajectory optimization problem for rocket propelled vehicles," AIAA Guidance and Control
   Conference, Hollywood, FL, Aug. 1977, AIAA Paper 77-1051, doi:10.2514/6.1977-1051.
3. R. L. McHenry, T. J. Brand, A. D. Long, B. F. Cockrell and J. R. Thibodeau III, "Space Shuttle
   Ascent Guidance, Navigation, and Control," *The Journal of the Astronautical Sciences*,
   Vol. XXVII, No. 1, 1979, pp. 1–38.
4. R. F. Jaggers, "Shuttle Powered Explicit Guidance (PEG) Algorithm," NASA Johnson Space Center,
   Houston, TX, Nov. 1992, JSC-26122 (as cited in [1]).
5. J. Goodman, "Roland Jaggers and the Development of Space Shuttle Powered Explicit Guidance
   (PEG)," AIAA SciTech 2021 Forum, Jan. 2021, doi:10.2514/6.2021-2021.
6. J. Barzilai and J. M. Borwein, "Two-point step size gradient methods," *IMA Journal of
   Numerical Analysis*, Vol. 8, No. 1, 1988, pp. 141–148.
7. F. Teren, "Explicit Guidance Equations for Multistage Boost Trajectories," NASA TN D-3189,
   Lewis Research Center, 1966, section "Cutoff Logic" (p. 16 of the PDF). Local copy:
   `Desktop/Tese/References/19660006073-PEG.pdf`.
