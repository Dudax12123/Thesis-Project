# Trajectory Optimization Process

## Overview

The trajectory optimization process depends on `COAST_METHOD`
(`Tese/src/Input_File/simulation_parameters.py`):

- **`COAST_METHOD = "apogee_check"`** (and `COAST_METHOD = "direct"` with
  `DIRECT_OPTIMIZATION_MODE = "brute_force"`) employ the hybrid approach
  described in Sections 1–2 below: explicit brute-force optimization of the
  initial kick angle, combined with implicit, physics-based determination of
  the engine cutoff timing. Section 3 explains why this optimization
  framework is independent of the active `GUIDANCE_MODE`.
- **`COAST_METHOD = "pso_coast"`**, **`COAST_METHOD = "direct"` with
  `DIRECT_OPTIMIZATION_MODE = "pso"`**, and **`GUIDANCE_MODE = "indirect_pmp"`**
  instead use PyGMO Particle Swarm Optimization over several decision
  variables at once. These PSO-based paths are described in Section 4.

## 1. Initial Kick Angle Optimization

### 1.1 Problem Formulation

The primary optimization problem seeks to determine the optimal initial kick angle (α) for the gravity turn maneuver that minimizes the total propellant consumption required to achieve the target orbit. The optimization is formulated as:

**Minimize:** J(α) = m_prop,total(α)

**Subject to:**
- α_min ≤ α ≤ α_max
- r_apo = r_target (apogee altitude constraint)
- m_prop,available ≥ m_prop,required (propellant availability constraint)
- Δt_burn ≤ Δt_max (maximum burn time constraint)

Where:
- J(α) is the objective function (total propellant consumption)
- m_prop,total is the sum of ascent propellant and circularization propellant
- α is the initial kick angle (typically between -5° and -2°)
- r_apo is the achieved apogee radius
- r_target is the target orbital altitude (500 km)

### 1.2 Objective Function

The objective function is implemented as:

```
J(α) = m_prop,ascent(α) + m_prop,circ(α)
```

Where:
- **m_prop,ascent** is the propellant consumed during the powered ascent phase
- **m_prop,circ** is the propellant required for circularization at apogee

The circularization propellant is computed using the Tsiolkovsky rocket equation:

```
m_prop,circ = m_apo × (1 - exp(-Δv_circ / (I_sp × g_0)))
```

Where:
- m_apo is the vehicle mass at apogee
- Δv_circ is the velocity change needed to circularize the orbit
- I_sp is the specific impulse of the second stage engine
- g_0 is standard gravitational acceleration (9.80665 m/s²)

The circularization Δv is calculated from the orbital velocity at apogee on the transfer trajectory and the circular orbital velocity:

```
Δv_circ = |v_circular - v_apo|
v_circular = √(μ_Earth / r_target)
v_apo = √(μ_Earth × a × (1 - e²)) / r_apo
```

Where:
- μ_Earth is Earth's gravitational parameter (3.986004418 × 10¹⁴ m³/s²)
- a is the semi-major axis of the transfer orbit
- e is the eccentricity of the transfer orbit

### 1.3 Optimization Algorithm

The optimization employs a brute force grid search algorithm implemented using `scipy.optimize.brute`:

**Algorithm Parameters:**
- Search method: Exhaustive grid search
- Grid resolution: 1000 uniformly distributed sample points
- Refinement: None (finish=None)
- Bounds: [α_min, α_max] = [-5°, -2°] (configurable)

**Algorithm Steps:**
1. Initialize search grid with N=1000 points uniformly spanning [α_min, α_max]
2. For each grid point α_i:
   - Run complete trajectory simulation with kick angle α_i
   - Compute total propellant consumption J(α_i)
   - Store result
3. Return α_optimal = arg min_i J(α_i)

**Computational Complexity:** O(N × T_sim), where T_sim is the time to simulate one trajectory. For N=1000 evaluations with typical simulation times of 1-3 seconds per trajectory, the optimization completes in approximately 20-50 minutes.

**Rationale for Brute Force Approach:**
- The objective function J(α) is non-smooth due to discrete events (stage separation, engine cutoffs)
- Gradient-based methods would require finite difference approximations with high computational cost
- The one-dimensional search space with N=1000 points provides sufficient resolution
- The global optimum is guaranteed to be found within grid resolution

### 1.4 Feasibility Constraints

The optimization includes penalty mechanisms to handle infeasible solutions:

**Constraint 1: Propellant Availability**
If m_prop,required > m_prop,available, the objective function returns J(α) = 999,999,999 kg (essentially infinite), eliminating this solution from consideration.

**Constraint 2: Maximum Burn Time**
If the circularization burn time exceeds the maximum acceptable duration (configurable parameter), the solution is similarly penalized with an infinite cost.

These soft constraints ensure that the optimizer only considers physically realizable trajectories.

## 2. Coasting Time Determination

### 2.1 Automatic Detection via Event Function

Unlike the initial kick angle, the coasting time (equivalently, the engine cutoff time) is **not explicitly optimized**. Instead, it is determined automatically through a physics-based event detection mechanism during trajectory integration.

### 2.2 Event Function Formulation

The engine cutoff is triggered by a zero-crossing event function:

```
g(t, y) = r_apo(t, y) - r_target
```

Where:
- t is the current simulation time
- y is the current state vector [s, r, v, γ, m]
- r_apo(t, y) is the apogee radius of the current trajectory

The apogee radius is computed from the instantaneous orbital elements:

```
a = -μ_Earth / (2ε)
e = √(1 - h² / (μ_Earth × a))
r_apo = a(1 + e)
```

Where:
- ε = v²/2 - μ_Earth/r is the specific orbital energy
- h = r × v × cos(γ) is the specific angular momentum

### 2.3 Event Detection Implementation

The event function is implemented as an interrupt in the `scipy.integrate.solve_ivp` integrator:

**Activation Condition:** Altitude > 65 km (above dense atmosphere)

**Event Trigger:** Zero crossing of g(t, y)

**Termination Action:** Stop integration, record final state

The integrator continuously monitors g(t, y) during the powered ascent phase (only above 65 km altitude). When g(t, y) changes sign (crosses zero), the exact time of crossing is determined through root-finding, and the engines are shut off.

### 2.4 Physical Interpretation

This automatic detection mechanism reflects the operational logic of a real launch vehicle:

"Continue burning until the current trajectory, if left unpowered, would coast to the desired apogee altitude."

This is optimal because:
- Burning beyond this point would overshoot the target apogee
- Stopping earlier would require more circularization propellant at a lower apogee
- The detection is based on instantaneous orbital mechanics, not pre-computed trajectories

The coasting time therefore emerges naturally from the interaction between:
- The thrust profile (determined by rocket specifications)
- The steering commands (determined by guidance mode)
- The gravitational and aerodynamic forces
- The target orbital altitude

## 3. Guidance Mode Independence

### 3.1 Optimization Framework

A critical feature of this optimization architecture is that the **optimization algorithm is completely independent of the guidance mode**. The nine guidance modes — `gravity_turn`, `linear_tangent`, `bilinear_tangent`, `apollo`, `cpr`, `peg`, `peg_new`, `exp_shooting`, and `indirect_pmp` (see `GUIDANCE_MODE_README.md`) — affect the trajectory evolution but, for the `apogee_check` and brute-force `direct` paths, do not alter the Section 1/2 optimization methodology. (`indirect_pmp` and the PSO-based paths of Section 4 follow the same separation-of-concerns principle but with a different optimization methodology.)

### 3.2 Separation of Concerns

The software architecture implements a clear separation:

**Outer Loop (Optimization Layer):**
- Varies the kick angle α
- Calls the simulation engine
- Evaluates the objective function J(α)
- Selects the optimal α
- **Does not know which guidance mode is active**

**Inner Loop (Simulation Layer):**
- Integrates equations of motion
- Implements guidance law (mode-dependent)
- Detects engine cutoff event
- Returns propellant consumption
- **Receives kick angle as input parameter**

### 3.3 Guidance Mode Effects

While the optimization process is identical, the guidance mode affects the optimization results:

**Trajectory Shape:**
- `gravity_turn`: passive guidance, α = 0° after the initial kick
- `linear_tangent`: tan(α + γ) varies linearly with time-to-go
- `bilinear_tangent`: tan(α + γ) = ratio of two linear functions of time-to-go
- `apollo`: acceleration-command polynomial enforcing full position and velocity terminal constraints
- `cpr`: no kick — pitch angle ramps linearly from 90° to 0° at a constant rate
- `peg`: linear pitch program sin(pitch) = A + B·t, re-solved every major loop
- `peg_new`: analytical predictor-corrector PEG driven by the velocity-to-be-gained vector `v_go`
- `exp_shooting`: exponential pitch law θ(t) = a·exp(b·t), solved once via `fsolve` at guidance start
- `indirect_pmp`: PMP costate-driven control α = atan2(−λ_γ/V, −λ_V) (uses its own PSO — see Section 4.3)

**Propellant Efficiency:**
Different guidance laws produce different trajectory shapes with varying propellant consumption for the same kick angle.

**Optimal Kick Angle:**
The optimal α varies between modes because each guidance law responds differently to the initial conditions set by the kick maneuver.

**Computational Cost:**
For the `apogee_check` and brute-force `direct` paths, all modes use the same number of evaluations (1000), so optimization time is similar across modes, though individual trajectory simulations may have slightly different durations. The PSO-based paths (Section 4) instead scale with swarm size × generations, independent of `GUIDANCE_MODE`.

### 3.4 Universality of Event Detection (apogee_check)

The coasting time determination is also guidance-independent for `COAST_METHOD = "apogee_check"`:

All guidance modes paired with `apogee_check` use the **identical event function** g(t, y) = r_apo - r_target. The guidance mode affects:
- How quickly the vehicle reaches the cutoff condition
- The state (position, velocity, flight path angle) at cutoff
- The amount of propellant consumed before cutoff

But the **mechanism** for detecting the optimal cutoff time remains the same.

### 3.5 Practical Implications

This design enables:

**Modularity:** New guidance modes can be added without modifying optimization code

**Comparison:** Fair comparison between guidance modes using identical optimization procedures

**Flexibility:** Users can switch guidance modes via configuration file without code changes

**Maintainability:** Changes to optimization algorithm benefit all guidance modes simultaneously

## 4. PSO-Based Optimization Paths

Three configurations replace the Section 1/2 brute-force-plus-event-detection
approach with a PyGMO Particle Swarm Optimization (PSO) over several decision
variables simultaneously. All three require the `pygmo` package and raise an
`ImportError` with installation instructions if it is missing.

### 4.1 `COAST_METHOD = "pso_coast"` — 4-variable PSO (thrust → coast → thrust)

Implemented in `Simulation/pso_coast_solver.py`. Optimises:

```
x = [delta_tc,        coast phase duration [s]
     delta_tr_pct,    Stage-2 burn as % of T_MAX_2 [%]
     coast_start_pct, coast start as % of Stage-2 burn time [%]
     gamma_p]         pitch maneuver (kick) angle [rad]
```

The trajectory structure is thrust → coast → thrust with **direct orbit
insertion** (no separate circularisation burn). During both thrust arcs, the
steering angle α is computed by the `GUIDANCE_MODE` selected in
`simulation_parameters.py` — this path is compatible with every mode except
`exp_shooting` (see `GUIDANCE_MODE_README.md`) and `indirect_pmp` (which has
its own PSO, Section 4.3).

Objective (4 terms, no transversality):

```
J' = w_J · J_nd + w_alt · |Δh_nd| + w_vel · |ΔV_nd| + w_fpa · |Δγ_nd| + CRASH_PENALTY
```

### 4.2 `COAST_METHOD = "direct"` with `DIRECT_OPTIMIZATION_MODE = "pso"` — 2-variable PSO

Implemented in `Simulation/direct_pso_solver.py`. Optimises:

```
x = [gamma_p,     pitch maneuver (kick) angle [rad], in [1.54, 1.57]
     t_burn_pct]  Stage-2 continuous burn duration as % of T_MAX_2 [%]
```

The trajectory structure is: Stage 1 (instantaneous kick via
`rocket_ascent.run_stage1`) → pre-ignition ballistic coast → **one continuous
Stage-2 thrust arc** of duration `t_burn` → direct orbit insertion (no
coast-to-apogee, no circularisation burn). The selected `GUIDANCE_MODE` steers
the single thrust arc.

Objective (same 4-term structure as 4.1, no coast split):

```
J = w_J · J_nd + w_alt · |Δh_nd| + w_vel · |ΔV_nd| + w_fpa · |Δγ_nd| + CRASH_PENALTY
```

where `J_nd = t_burn / T_MAX_2` (burn-time fraction) and `Δh`/`ΔV`/`Δγ` are
the altitude/velocity/flight-path-angle errors of the final state versus the
rotating-frame circular-orbit target at `TARGET_ORBITAL_ALTITUDE`.

### 4.3 `GUIDANCE_MODE = "indirect_pmp"` — 7-variable PSO with PMP costates

Implemented in `Simulation/indirect_pso_solver.py` (PSO driver) and
`Guidance/indirect_pmp_guidance.py` (control law and costate derivatives).
Optimises:

```
x = [lambda0_r, lambda0_v, lambda0_g,  initial costate values, each in [-1, 1]
     delta_tc,                          coast duration [s]
     delta_tr_pct,                      Stage-2 burn as % of T_max [%]
     coast_start_pct,                   coast start as % of burn time [%]
     gamma_p]                           pitch maneuver (kick) angle [rad]
```

Each PSO evaluation runs Stage 1 (gravity turn, PSO-chosen kick angle) then a
Stage-2 thrust → coast → thrust sequence in which the augmented state
`[s, r, v, γ, m, λ_r, λ_v, λ_γ]` is propagated together by `solve_ivp`, using
the drag-free PMP control law α = atan2(−λ_γ/V, −λ_V) (Eq. 34) during thrust
arcs. The objective (Eq. 39) penalises altitude, velocity, and flight-path-
angle terminal errors, the transversality condition (Eq. 38), and trajectories
that crash or deplete propellant before reaching orbit. `COAST_METHOD` has no
effect on this mode — the coast/burn split is fully controlled by the PSO.

### 4.4 Direct-insertion cutoff and `DIRECT_OPTIMIZATION_MODE = "brute_force"`

When `COAST_METHOD = "direct"`, the Stage-2 burn is cut (MECO) the instant the
inertial velocity reaches circular velocity `√(μ/r_target)` — a different
cutoff condition from the apogee-match event of Section 2. Whether the
flight-path angle and altitude *also* land within
`DIRECT_INSERTION_FPA_TOL_DEG` / `DIRECT_INSERTION_ALTITUDE_TOL_KM` of the
target (together with `DIRECT_INSERTION_VELOCITY_TOL_MS`) determines whether
the insertion is graded "clean".

`DIRECT_OPTIMIZATION_MODE = "brute_force"` reuses the **same 1000-point grid
search over the kick angle** described in Section 1.3, but with a different
objective: instead of minimising propellant mass against an apogee-match
event, it minimises the combined (tolerance-normalised) "box margin" against
the direct-insertion cutoff above. `DIRECT_OPTIMIZATION_MODE = "pso"` replaces
this 1-D grid search with the 2-variable PSO of Section 4.2.

## 5. Summary

The trajectory optimization employs one of three approaches, selected via
`COAST_METHOD` (and, for `COAST_METHOD = "direct"`,
`DIRECT_OPTIMIZATION_MODE`):

**1. `apogee_check` (two-level, Sections 1–2):**

- **Level 1 (Explicit):** Brute force grid search optimizes the initial kick angle to minimize total propellant consumption. The search evaluates 1000 uniformly distributed angles and selects the global optimum.
- **Level 2 (Implicit):** Physics-based event detection automatically determines the optimal engine cutoff time by monitoring when the current trajectory's apogee matches the target altitude.

**2. `direct` (Section 4.4):** Either the same 1000-point kick-angle grid
search (`brute_force`, against a direct-insertion box-margin objective and a
circular-velocity MECO event) or the 2-variable PSO of Section 4.2 (`pso`).

**3. `pso_coast` / `indirect_pmp` (Sections 4.1 and 4.3):** PyGMO PSO jointly
optimises the kick angle together with the coast/burn timing (and, for
`indirect_pmp`, the initial PMP costates).

Across all three approaches:
- **Robust:** brute-force search is guaranteed to find the global optimum within grid resolution; PSO is a well-established global optimizer.
- **Physical:** cutoff conditions (apogee match, circular-velocity MECO, or PSO-planned burn end) are based on orbital mechanics principles.
- **Universal:** Sections 1–2 and 4.1–4.3 are applicable to every compatible `GUIDANCE_MODE` without modification (see `GUIDANCE_MODE_README.md` for the few mode/`COAST_METHOD` incompatibilities).

The separation between optimization framework and guidance implementation provides a flexible, maintainable architecture suitable for comparative studies of different guidance strategies.
