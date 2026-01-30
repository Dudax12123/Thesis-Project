# Trajectory Optimization Process

## Overview

The trajectory optimization for the coasting single burn strategy employs a hybrid approach that combines explicit optimization of the initial kick angle with implicit, physics-based determination of the engine cutoff timing. This section describes the mathematical formulation, algorithmic implementation, and the relationship between the optimization process and the guidance modes.

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

A critical feature of this optimization architecture is that the **optimization algorithm is completely independent of the guidance mode**. The three guidance modes (gravity turn, simple polynomial, and Apollo) affect the trajectory evolution but do not alter the optimization methodology.

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
- Gravity turn: Passive guidance, α = 0° after initial kick
- Simple polynomial: Linear transition of flight path angle to horizontal
- Apollo: Acceleration command profiles enforcing terminal constraints

**Propellant Efficiency:**
Different guidance laws produce different trajectory shapes with varying propellant consumption for the same kick angle.

**Optimal Kick Angle:**
The optimal α varies between modes because each guidance law responds differently to the initial conditions set by the kick maneuver.

**Computational Cost:**
All modes use the same number of evaluations (1000), so optimization time is similar across modes, though individual trajectory simulations may have slightly different durations.

### 3.4 Universality of Event Detection

The coasting time determination is also guidance-independent:

All three guidance modes use the **identical event function** g(t, y) = r_apo - r_target. The guidance mode affects:
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

## 4. Summary

The trajectory optimization employs a two-level approach:

**Level 1 (Explicit):** Brute force grid search optimizes the initial kick angle to minimize total propellant consumption. The search evaluates 1000 uniformly distributed angles and selects the global optimum.

**Level 2 (Implicit):** Physics-based event detection automatically determines the optimal engine cutoff time by monitoring when the current trajectory's apogee matches the target altitude.

This hybrid approach is:
- **Robust:** No gradient information required, guaranteed to find global optimum
- **Efficient:** Coasting time determined automatically without additional optimization
- **Universal:** Applicable to all guidance modes without modification
- **Physical:** Cutoff condition based on orbital mechanics principles

The separation between optimization framework and guidance implementation provides a flexible, maintainable architecture suitable for comparative studies of different guidance strategies.
