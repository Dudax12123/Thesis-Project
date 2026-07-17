# Restructured Thesis Outline (v2)

**Thesis title:** *Concurrent Simulation of Rocket Ascent Trajectories*
**Author:** Eduardo De Almeida Helena

> **Purpose of this document.** This is a restructured, annotated outline that brings the thesis
> plan up to date with the current code. Since the original outline (`thesis_outline.md`) was
> written, the project grew from 5 to **9 guidance modes**, gained **3 mission-architecture / coast
> strategies**, **4 optimization back-ends** (1 brute-force + 3 PSO), an **indirect optimal-control
> (PMP)** trajectory mode, and a reorganized configuration layer. This outline reorganizes
> Chapter 2, rebuilds Chapter 3 as a clean methodology chapter (with a flowchart and a dependency
> tree, no source code), and reorganizes Chapter 4 (Results) as a hybrid build-up. Chapter 1 is
> left untouched. References harvested from the code are mapped to the sections that need them
> (see the Reference List at the end).
>
> Notation: **[NEW]** = material to add; **[EXTEND]** = existing material to expand; **[RECONCILE]**
> = thesis text currently contradicts the code and must be fixed.

---

# Chapter 1 — Introduction  *(UNCHANGED)*

Keep the existing structure and prose:

- **1.1 Objective and Motivation**
- **1.2 Launch Vehicle and Ascent Fundamentals**
- **1.3 Literature Review**
- **1.4 Thesis Originality and Overview**

*Optional single edit:* update the framing sentence in 1.4 so the stated scope reflects the current
work — a common framework comparing **nine guidance laws**, **three mission architectures**, and an
**optimal-control reference trajectory**, rather than five guidance laws.

---

# Chapter 2 — Theoretical Background  *(REORGANIZED + EXTENDED)*

Reorganized into seven thematic blocks so that every method used in Chapter 3 has its theory
established here first. The largest gap in the current outline — numerical optimization (PSO,
brute force) and numerical methods generally — is filled by the new **§2.7**.

## 2.1 Ascent Flight Dynamics
- **2.1.1 Planar 3-DOF point-mass model.** State variables (downrange `s`, geocentric radius `r`,
  speed `v`, flight-path angle `γ`, mass `m`); assumptions (rigid vehicle, prescribed attitude
  tracking, no full rotational dynamics, motion confined to a vertical plane).
- **2.1.2 Steering kinematics and force decomposition.** Angle of attack `α = θ − γ`; decomposition
  of thrust, drag, lift, and gravity into tangential/normal components; flight-path-angle dynamics
  and the centripetal `v²/r` term.

## 2.2 Reference Frames and Rotating-Earth Effects
- **2.2.1 Frames.** Rotating (ECEF-like) frame for atmospheric-relative motion vs. inertial (ECI)
  frame for orbital mechanics; the local East–North–Up (ENU) basis.
- **2.2.2 ECEF↔ECI velocity conversion.** Adding Earth's eastward surface speed to obtain inertial
  velocity, speed, and flight-path angle.
- **2.2.3 Rotating-frame pseudo-forces** *[EXTEND]*. Coriolis and centrifugal accelerations in the
  ENU basis and their projection onto the flight-dynamics directions. → *cite Vallado / Curtis.*
- **2.2.4 Launch-azimuth and orbital-inclination geometry** *[NEW]*. Spherical-trigonometry azimuth
  from target inclination and launch latitude; rotating-frame (surface-speed) correction; achieved
  inclination from the final inertial velocity. → *cite Vallado / Curtis.*

## 2.3 Environment and External Forces
- **2.3.1 Gravity.** Central inverse-square field; spherical-Earth assumption (no `J₂` / oblateness).
- **2.3.2 Atmospheric model** *[NEW — fills the placeholder in the current §2.3]*. Exponential
  density profile, scale height, speed of sound, dynamic pressure, Mach number. → *cite US Standard
  Atmosphere / Vallado.*
- **2.3.3 Aerodynamic forces.** Drag (and optional lift) from dynamic pressure, reference area, and
  coefficients; the max-q condition.
- **2.3.4 Propulsion.** Thrust, mass-flow rate, specific impulse, pressure correction; Tsiolkovsky
  rocket equation and mass depletion.

## 2.4 Ascent Performance: ΔV Budget, Losses and Gains
- Ideal rocket-equation ΔV; the loss/gain budget — **gravity loss**, **drag loss**, **steering
  loss**, and the **Earth-rotation gain**; the drag-loss vs. gravity-loss trade-off that shapes the
  trajectory.

## 2.5 Guidance and Steering Laws  *(taxonomy + per-law theory)*
- **2.5.1 Taxonomy.** Open-loop vs. closed-loop guidance; atmospheric-arc (load-management) vs.
  exo-atmospheric (orbital-insertion) guidance.
- **2.5.2 Atmospheric-arc steering.** Initial pitchover kick; gravity turn; **constant pitch rate
  (CPR)** *[NEW]*; constant flight-path-angle rate.
- **2.5.3 Polynomial guidance** (linear-in-time acceleration / Apollo form). → **Battin (1987)**.
- **2.5.4 Linear tangent steering (LTS).** `tan(α+γ)` linear in time-to-go. → **Etkin (1972);
  Hull (1997)**.
- **2.5.5 Bilinear tangent steering (BTS)** *[NEW]*. Ratio of two linear functions of time-to-go;
  controls both value and rate at the terminal point. → **Hull (1997); Lu (1993)**.
- **2.5.6 Iterative Guidance Mode (IGM).** Historical Saturn context; linear thrust-angle assumption.
- **2.5.7 Powered Explicit Guidance (PEG).** Linear-tangent steering, predictor-corrector structure,
  time-to-go iteration, Guide+Estimate major loop. → **McHenry et al. (1979); Brand, Gans & Laue
  (1993)**; supplementary **orbiterwiki**.
- **2.5.8 Velocity-to-be-gained / analytical predictor-corrector PEG** *[NEW]*. The `v_go`
  formulation, Jaggers' "Coke-Machine" orthogonality assumption, and the gravity-integral
  predictor-corrector refinement. → **Jaggers (1977); Sagliano, Mooij & Theil**.
- **2.5.9 Open-loop exponential pitch law** *[NEW]*. Pitch `θ(t)=a·exp(b·t)` with coefficients
  fixed by a single-shooting boundary-value solve (links to §2.7.3).

## 2.6 Optimal Control Theory  *[EXTEND]*
- **2.6.1 Problem formulation.** Bolza-form cost; dynamic, boundary, and path constraints; direct
  vs. indirect solution methods.
- **2.6.2 Pontryagin's Minimum/Maximum Principle.** Hamiltonian, costate (adjoint) equations,
  stationarity and transversality conditions, the two-point boundary value problem (TPBVP).
  → **Pontryagin (1962); Bryson & Ho (1975)**.
- **2.6.3 Indirect shooting and the costate-direction gauge** *[NEW]*. Why only the *direction* of the
  costate vector matters (scale gauge) and how the TPBVP can be recast as a penalized optimization —
  the theoretical bridge to the PMP guidance mode in Chapter 3.

## 2.7 Numerical Methods  *[ALL NEW — principal gap flagged by the author]*
- **2.7.1 ODE integration.** Explicit Runge–Kutta methods; the embedded Dormand–Prince RK45 scheme;
  adaptive step-size control and tolerances. → **Dormand & Prince (1980)**.
- **2.7.2 Event detection.** Zero-crossing event functions and Brent's-method root-finding to locate
  exact phase transitions (staging, cutoff, atmosphere exit). → **Burden & Faires (2016)**.
- **2.7.3 Root-finding for guidance.** Fixed-point iteration and successive under/over-relaxation
  (SUR) — the convergence mechanism behind PEG's major loop; Newton-type shooting (`fsolve`) for the
  exponential pitch law. → **Burden & Faires (2016)**.
- **2.7.4 Brute-force / exhaustive grid search.** A global, gradient-free search; its suitability for
  low-dimensional, non-smooth, event-driven objectives, and why gradient methods struggle there.
- **2.7.5 Particle Swarm Optimization (PSO).** Swarm dynamics; inertia `ω`, cognitive/social
  coefficients `c₁`/`c₂`, velocity clamping; convergence behavior; the PyGMO implementation.
  → **Kennedy & Eberhart (1995); Biscani & Izzo (PyGMO/pagmo)**.
- **2.7.6 Constrained optimization via penalty functions.** Soft constraints and large-penalty
  handling, used uniformly across both the brute-force and PSO objectives.

---

# Chapter 3 — Methodology  *(theory → implementation; flowchart + dependency tree; NO source code)*

This chapter explains *where and how* the Chapter 2 theory is realized in the simulator, at a
surface level — architecture, data flow, and configuration — without printing source code.

## 3.1 Software Architecture Overview
Describe the layered architecture and present **two figures**.

**Layering (bottom → top):** physical constants & vehicle specs (leaf data) → environment models
(gravity, atmosphere, Earth rotation) → configuration layer → guidance laws (leaf, pure functions) →
the physics/EOM hub → solvers/optimizers → plotting → top-level driver.

**Figure 3.1 — Dependency tree (surface level):**

```
main.py
  ├─ Simulation/solver.py ────────────────┐
  ├─ Simulation/pso_coast_solver.py        │   all import →  Simulation/rocket_ascent.py
  ├─ Simulation/direct_pso_solver.py       │       (the hub: imports ALL Guidance/* + Auxiliary/*)
  │      └─ imports pso_coast_solver        │
  ├─ Simulation/indirect_pso_solver.py ────┘   └─ imports Guidance/indirect_pmp_guidance.py
  ├─ Auxiliary/{constants, gravity, atmosphere, earth_rotation, rocket_specs}.py
  ├─ Input_File/simulation_parameters.py        (imported almost everywhere)
  └─ Plots/new_plot_runner.py → Plots/new_metrics/*   (decoupled from the solvers)
```

Narrative points for the text: `rocket_ascent.py` is the central **hub**; the guidance modules are
**leaves** (dependencies run Simulation → Guidance, never the reverse), which keeps each law
unit-testable in isolation (cf. `tests/test_apollo_tgo.py`); the plotting layer consumes plain
result arrays and is decoupled from the solvers.

**Figure 3.2 — Execution flowchart:** an outer **optimizer** feeds a candidate design vector to the
**simulation engine**, which at every integrator step evaluates the **environment models**, the
active **guidance law**, and the **equations of motion**; **event detection** triggers phase
transitions and cutoff; the resulting **trajectory** is scored by the **objective function** and
returned to the optimizer.
*[RECONCILE]* The existing TikZ figure (`simulator_methodology.tex`, ~line 95) shows a single
brute-force loop — redraw it to show the **four back-ends** branching on `COAST_METHOD` /
`GUIDANCE_MODE`.

## 3.2 Mission Decomposition into Phases
Liftoff / vertical rise → pitchover kick → Stage-1 powered ascent (gravity turn) → fairing jettison
→ stage separation and coast → Stage-2 ignition → guided ascent → SECO / cutoff →
architecture-dependent coast and orbit insertion. Map each phase to where it lives in code
(`rocket_ascent.run`, `rocket_ascent.run_stage1`, and the thrust→coast→thrust arc structure of the
PSO solvers).

## 3.3 State Vector and Numerical Integration
- Base state `[s, r, v, γ, m]`; latitude `φ` appended when Earth rotation is enabled.
  *[RECONCILE]* The heading state `ψ` was **removed** ("drop heading tracking") — document the
  current state vector and note ψ-tracking as deprecated / future work (the `.tex` still shows a
  7-state vector).
- `scipy.integrate.solve_ivp` with RK45 (Dormand–Prince), adaptive step, tight tolerance, dense
  output; event functions for MECO, stage separation, fairing jettison, apogee-match, circular-
  velocity cutoff, and ground-collision safeguards.

## 3.4 Reference-Frame Strategy (as implemented)
Mixed-frame approach: rotating-frame ascent dynamics with ENU pseudo-force corrections; switch to
pure inertial two-body propagation after SECO; orbital elements always evaluated in the inertial
frame.

## 3.5 Implemented Equations of Motion
Base spherical EOM (`ṡ, ṙ, v̇, γ̇, ṁ`); rotating-frame Coriolis/centrifugal augmentation; latitude
reconstructed from great-circle geometry to avoid drift; numerical safeguards (small-velocity guard,
vertical-hold during initial rise).

## 3.6 Environmental and Force Models (as implemented)
Inverse-square gravity; exponential atmosphere with altitude- or dynamic-pressure-triggered exit;
drag (constant `C_D`, fixed reference area) and lift (`F_L = q·C_L·A`, constant `C_L`, active only
while `INCLUDE_DRAG` is on); stage-dependent thrust/`Iₛₚ`; **Stage-1 engine modes** (`ISP_1_MODE` /
`THRUST_1_MODE`: sea_level / vacuum / average / linear); the **`INCLUDE_DRAG` no-atmosphere mode**
(also drops the fairing and forces altitude-based exit). *[RECONCILE]* Lift is **modeled**, not
neglected: `INCLUDE_LIFT = True` by default (small constant `C_L`, independent of angle of attack).

## 3.7 Guidance Laws Implemented  *(all nine — one short subsection each)*
The nine current `GUIDANCE_MODE` options: `gravity_turn` · `linear_tangent` · `bilinear_tangent` ·
`apollo` (k1–k4 coefficients, coefficient freezing near small `t_go`, optional thrust-magnitude
control) · `cpr` (constant pitch rate) · `peg` (classical, SUR-damped major loop) · `peg_new`
(predictor-corrector `v_go`) · `exp_shooting` (one-shot `fsolve`) · `indirect_pmp` (costate-driven
control `α = atan2(−λ_γ/V, −λ_V)`). Note the **`TGO_ESTIMATOR`** option (`rocket_equation` vs.
`peg_new`) shared by the apollo/lts/bts/cpr laws.
*[RECONCILE]* The legacy `simple_poly` mode (still described in `simulator_methodology.tex`) was
**removed** from the code (`main.py:200`) and is no longer a valid `GUIDANCE_MODE`.

## 3.8 Mission Architectures and Optimization Strategies  *(maps §2.6–2.7 theory to code)*
- **3.8.1 `apogee_check`.** Brute-force kick-angle grid search (`scipy.optimize.brute`, 1000 points)
  + physics-based apogee-match event cutoff + impulsive circularization (Tsiolkovsky). Objective:
  minimize total propellant (ascent + circularization).
- **3.8.2 `pso_coast`.** 4-variable PyGMO PSO over `[delta_tc, delta_tr_pct, coast_start_pct,
  gamma_p]`; thrust→coast→thrust with direct insertion; non-dimensional 4-term penalized objective
  (burn-time + altitude/velocity/FPA errors + crash penalty).
- **3.8.3 `direct`.** Single continuous Stage-2 burn cut at circular velocity; sub-modes
  `DIRECT_OPTIMIZATION_MODE = "brute_force"` (reuses the 1000-point grid against a direct-insertion
  box-margin objective) or `"pso"` (2-variable PSO over `[gamma_p, t_burn_pct]`).
- **3.8.4 `indirect_pmp`.** 7-variable PSO over initial costates `[λ0_r, λ0_v, λ0_g]` plus timing/kick
  variables; the augmented state `[s, r, v, γ, m, λ_r, λ_v, λ_γ]` is propagated jointly; the
  objective adds a **transversality-condition residual** penalty. This is the optimal-control
  *reference* trajectory. *[RECONCILE]* `indirect_pmp` is **Stage-2-only**: Stage 1 still flies the
  fixed gravity turn up to MECO; a full-ascent (costate-driven-from-liftoff) extension was explored
  and then reverted after its Stage-1 arc lofted well past the standard gravity-turn profile.
- **3.8.5 Segmented multi-law guidance (`MULTI_GUIDANCE_ENABLED`)** *[NEW]*. An architecture
  orthogonal to `COAST_METHOD`: instead of one fixed `GUIDANCE_MODE`, the rocket flies an ordered
  `GUIDANCE_SEGMENTS` schedule of `(law, activation altitude)` pairs. The FIRST law takes over right
  after the kick — a gravity turn is now just one selectable choice there, no longer hard-wired to
  fly first. Every non-final segment steers toward the cached indirect-PMP optimal `(h, v, γ)`
  waypoint at the NEXT segment's activation altitude; the FINAL segment inserts to orbit by reusing
  the `pso_coast` thrust→coast→thrust engine. Because `t_go` is a planned-deadline countdown sourced
  from the PMP reference (deadline − t) rather than the rocket-equation estimate, a law can fly
  during Stage 1 (below MECO) without its `t_go` collapsing at the stage boundary. The non-first
  activation altitudes can themselves be PSO-optimized (`MULTI_GUIDANCE_OPTIMIZE_ALTITUDES`) — on
  top of the usual 4 coast variables — to minimize Stage-2 burn time.
- **3.8.6 Separation of concerns.** The optimizer is guidance-mode-agnostic: the outer loop supplies
  a design vector and receives a scalar cost; the inner simulation selects and runs whichever
  guidance law is active. This is what makes the comparative study fair.

## 3.9 Configuration System
Catalog the user-facing switches in `Input_File/simulation_parameters.py`, grouped by function:
mission geometry & target orbit; Earth-rotation/pseudo-force flags (`ENABLE_EARTH_ROTATION`,
`INCLUDE_PSEUDO_FORCES`, `AZIMUTH_INCLINATION_MODE`, `COMPUTE_CROSS_HEADING_COUNTER_FORCE`);
atmosphere/drag (`INCLUDE_DRAG`, `INCLUDE_LIFT`, `ATMOSPHERE_EXIT_METHOD`); kick profile
(`KICK_PROFILE_MODE`); guidance selection & tuning (`GUIDANCE_MODE`, `TGO_ESTIMATOR`,
`CPR_THETA_DOT_MODE`, `PEG_CONVERGENCE_MODE`); engine modes (`ISP_1_MODE`/`THRUST_1_MODE`); mission
architecture (`COAST_METHOD`, `DIRECT_OPTIMIZATION_MODE`); segmented multi-law guidance
(`MULTI_GUIDANCE_ENABLED`, `GUIDANCE_SEGMENTS`, `MULTI_GUIDANCE_OPTIMIZE_ALTITUDES`); optimizer
settings (`RUN_FAST`, PSO tuning blocks incl. `PSO_MG_*` for the segmented solver); integration/output.
Present as tables (refresh the tables already drafted in `simulator_methodology.tex`).

## 3.10 Vehicle Configuration and Baseline Mission
Falcon-9-class two-stage parameters (thrust, `Iₛₚ`, structural/propellant masses, reference area,
staging timing) in a table; baseline mission: 500 km circular orbit at 51.6°, launched from latitude
28.5° (Kennedy Space Center / ISS-like inclination).

## 3.11 Software Usage  *[fills the placeholder]*
How to configure and run a single mission (`main.py`), the batch comparison runners
(`all_guidance_plotting/run_all_guidance_methods.py`, `guidance_comparison/compare_guidance_methods.py`),
and the plot suite (`Plots/new_plot_runner.py`).

---

# Chapter 4 — Results and Discussion  *(HYBRID BUILD-UP — new organization)*

The chapter builds up from a verified baseline, adds physics fidelity incrementally, then turns to
the core comparison (guidance laws), then the architecture/optimizer comparison and the
optimal-control benchmark, and closes with losses and limitations. Each section ends with a short
discussion.

## 4.1 Baseline Case and Verification
Describe the reference mission and run sanity checks: the ΔV budget should reconcile (orbital speed +
gravity loss ≈ 1000 m/s + drag loss ≈ 100 m/s − Earth-rotation gain ≈ 410 m/s), and the ascent
should qualitatively match a known Falcon-9-class profile (max-q, staging times, SECO).

## 4.2 Modeling-Fidelity Ladder  *(gravity-turn reference, physics added incrementally)*
- **4.2.1 No-atmosphere** (`INCLUDE_DRAG = False`) — clean reference with no drag loss.
- **4.2.2 + Atmosphere** — drag, max-q, dynamic-pressure-triggered fairing jettison / guidance start.
- **4.2.3 + Spherical and rotating Earth** — Coriolis/centrifugal pseudo-forces; launch azimuth and
  achieved-inclination drift `Δi`.

## 4.3 Guidance-Law Comparison  *(core of the thesis)*
All feasible laws under common conditions: propellant to orbit, ΔV losses, insertion accuracy
(`Δh`, `ΔV`, `Δγ`), steering-angle and pitch-angle profiles, and time-to-go behavior (including
Apollo coefficient freezing). Use the existing plot suite — FPA, steering angle, thrust, altitude,
trajectory x–y, losses, `apollo_tgo`, etc.

## 4.4 Mission-Architecture / Optimizer Comparison
`apogee_check` vs. `pso_coast` vs. `direct` vs. `indirect_pmp`: propellant/payload, PSO convergence
behavior (`pso_convergence` plot), runtime, and a **feasibility matrix** showing which
guidance×architecture combinations actually reach orbit. Report the known negative results: several
laws go suborbital under `direct`; only `{apollo, peg, peg_new}` reliably close a single continuous
burn; Apollo × `apogee_check` raises an infeasibility; CPR + Earth rotation under `apogee_check`
crashes.

## 4.5 Optimal-Control Reference
Use the `indirect_pmp` (PMP) trajectory as a near-optimal benchmark: how closely the explicit laws
(especially `peg_new` and `apollo`) approach it, and an interpretation of the costate histories and
the transversality residual at convergence.

## 4.6 Loss Breakdown
Gravity vs. drag vs. steering losses across laws and architectures (from `trajectory_losses_over_time`),
tied back to the propellant ranking in §4.4.

## 4.7 Sensitivity, Limitations and Negative Results
Effect of `KICK_PROFILE_MODE` (triangular vs. instantaneous — and the gotcha that triangular is a
no-op under the PSO paths); engine-mode (vacuum-thrust) feasibility; known incompatibilities/crashes;
modeling limitations (exponential atmosphere, no `J₂`, planar model, constant/AoA-independent lift
coefficient). *[RECONCILE]* Earlier drafts noted lift = 0 (neglected); the code models lift
(`INCLUDE_LIFT = True` default, `F_L = q·C_L·A`) — the residual limitation is the fixed, small `C_L`,
not its absence.

---

# Chapter 5 — Conclusions

- **5.1 Achievements.** A single framework comparing 9 guidance laws, 4 optimizers, and 3 mission
  architectures, with a quantified comparison against an optimal-control reference.
- **5.2 Future Work.** Analytic-CPR and CFPAR variants (specified in `dev-notes/` but not
  implemented); `J₂`/oblateness; full 3-D out-of-plane motion; higher-fidelity (angle-of-attack-
  dependent) lift modeling; multi-revolution insertion (`Ideas.md`); reinstating heading-state
  tracking.

---

# Front / Back Matter — placeholders to complete
- English Abstract and Portuguese Resumo.
- Convert remaining internal/Portuguese notes into final thesis text.

---

# Reference List (mapped to sections)

### Already cited in the code (use as-is)
| Reference | Section(s) |
|---|---|
| Etkin, B. (1972). *Dynamics of Atmospheric Flight* | §2.5.4 (LTS) |
| Hull, D. G. (1997). *Optimal Control Theory for Applications* | §2.5.4–2.5.5 (LTS/BTS) |
| Lu, P. (1993). *Inverse Dynamics Approach to Trajectory Optimization* | §2.5.5 (BTS) |
| Battin, R. H. (1987). *An Introduction to the Mathematics and Methods of Astrodynamics* | §2.5.3 (Apollo) |
| McHenry, R. L., et al. (1979). *Space Shuttle Ascent Guidance, Navigation and Control*, J. Astronautical Sci. 27(1), 1–38 | §2.5.7 (PEG) |
| Brand, T. J., Gans, N. R., & Laue, G. H. (1993). *Powered Explicit Guidance Improvements and Comparison with PEG4*, NASA JSC | §2.5.7 (PEG convergence) |
| Jaggers, R. F. (1977). *An explicit solution to the exoatmospheric powered flight guidance…*, AIAA Paper 77-1051 | §2.5.8 |
| Sagliano, M., Mooij, E., & Theil, S. *PEG Derivation from First Principles* | §2.5.8 (predictor-corrector PEG) |
| Burden, R. L., & Faires, J. D. (2016). *Numerical Analysis* (10th ed.), Cengage | §2.7.2–2.7.3 (root-finding, fixed-point/SUR) |
| orbiterwiki, *Powered Explicit Guidance* (URL) | §2.5.7 (supplementary — replace with a peer-reviewed source where possible) |

### Recommended additions (standard for these topics; currently uncited in the repo)
| Reference | Section(s) |
|---|---|
| Pontryagin, L. S. (1962). *The Mathematical Theory of Optimal Processes* | §2.6.2 (PMP) |
| Bryson, A. E., & Ho, Y.-C. (1975). *Applied Optimal Control* | §2.6 (optimal control / TPBVP) |
| Kennedy, J., & Eberhart, R. (1995). *Particle Swarm Optimization*, Proc. IEEE ICNN | §2.7.5 (PSO) |
| Biscani, F., & Izzo, D. *A parallel global multiobjective framework for optimization: pagmo* (PyGMO) | §2.7.5 (PSO implementation) |
| Dormand, J. R., & Prince, P. J. (1980). *A family of embedded Runge–Kutta formulae*, J. Comp. Appl. Math. 6(1) | §2.7.1 |
| Vallado, D. A. *Fundamentals of Astrodynamics and Applications* (or Curtis, *Orbital Mechanics for Engineering Students*) | §2.2.3–2.2.4, §2.3.2 (azimuth/inclination, ECEF↔ECI, atmosphere) — **currently uncited in the repo; needs a source** |

### Dangling reference to resolve
- `dev-notes/cpr_cfpar_guidance_implementation.md` uses an unnamed "reference material" for the
  analytic-CPR / CFPAR formulas — track down and cite before using these (future-work item).

---

# Reconciliation Notes (stale content to fix in the thesis body)
1. `simulator_methodology.tex` documents **5 guidance modes** (including `simple_poly`, since
   **removed**) and a **7-state heading vector (ψ)** — both stale. The current code has **9 modes**
   (none named `simple_poly`) and **no heading state**. The new outline reflects this.
2. `simulator_methodology.tex` describes only the **brute-force** optimizer — add the **3 PSO
   back-ends + indirect PMP**.
3. The TikZ architecture figure (`simulator_methodology.tex`, ~line 95) shows a single optimization
   loop — redraw to show the four back-ends branching on `COAST_METHOD` / `GUIDANCE_MODE`.
