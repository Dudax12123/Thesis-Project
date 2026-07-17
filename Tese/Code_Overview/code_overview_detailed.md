# Simulator — Detailed Overview
### *Concurrent Simulation of Rocket Ascent Trajectories*

> Companion to the one-page summary (`code_overview.md`). This document explains the simulator in
> more depth while staying conceptual — no source code, and only a couple of illustrative formulas.

---

## 1. Purpose and scope

The tool is a configurable Python simulator that flies a two-stage (Falcon-9-class) launch vehicle
from lift-off to a target circular orbit. Its goal is not to design one trajectory, but to provide a
**single, common framework** in which many different *guidance laws* (how the rocket steers) and
*trajectory-optimization methods* (how the best trajectory is found) can be run on the same vehicle,
the same mission, and the same physics — and therefore compared **fairly**. For each run it reports
the achieved orbit, the propellant used (equivalently, the payload capability), the breakdown of
velocity losses, and a set of diagnostic plots.

At the highest level, every run is one pass through the same loop: an optimizer proposes a candidate,
the simulator flies and scores it, and the loop repeats until convergence.

```
Optimizer ─→ Simulator (environment + guidance + equations of motion, evaluated step by step)
          ─→ event detection / engine cut-off ─→ trajectory ─→ score ─┐
          └───────────────────────── repeat ────────────────────────┘
```

## 2. The physical model

The vehicle is treated as a **point mass moving in a vertical plane** (a 3-degree-of-freedom model).
At every instant its state is described by five quantities — downrange distance, altitude (geocentric
radius), speed, flight-path angle, and mass — with latitude added as a sixth when Earth rotation is
enabled. The forces acting on it are:

- **Thrust**, whose direction is steered by the *angle of attack* `α = θ − γ` (the difference between
  the vehicle's pitch angle `θ` and its flight-path angle `γ`); this `α` is the single command that
  every guidance law produces.
- **Aerodynamic drag and lift**, from an exponential atmosphere model. Lift is included in the force
  model by default (`F_L = q · C_L · A`, evaluated alongside the drag force), not neglected.
- **Gravity**, an inverse-square central field around a spherical Earth.
- Optionally, **Coriolis and centrifugal pseudo-forces** when the rotating Earth is modelled.

Mass is depleted according to the rocket equation via the engine's specific impulse. The simulator
uses a **mixed-frame strategy**: ascent dynamics and atmospheric effects are handled in a frame that
rotates with the Earth, while orbital quantities (semi-major axis, apogee, inclination) are evaluated
in an inertial frame; after engine cut-off the motion is propagated as a pure two-body orbit.

**Stated assumptions / limitations:** planar 3-DOF motion, spherical non-oblate Earth (no J₂
perturbation), exponential atmosphere with drag and lift both included, and no attitude (rotational)
dynamics.

## 3. The mission, phase by phase

A simulation reproduces the natural sequence of an orbital launch:

1. **Lift-off and vertical rise** — the vehicle ascends vertically for a few seconds.
2. **Pitchover "kick"** — a small, brief tilt that starts the trajectory leaning downrange. The size
   of this kick is the key initial-condition the optimizer tunes.
3. **First-stage powered ascent** — the vehicle coasts through a gravity turn shaped by the kick and
   gravity, flying through the dense atmosphere (this is where drag and max-q occur).
4. **Stage separation and short coast** — the spent first stage is dropped; a brief unpowered gap
   follows before the upper stage lights.
5. **Second-stage guided ascent** — the selected guidance law actively steers the vehicle toward the
   target orbit.
6. **Engine cut-off (SECO)** and **orbit insertion** — the burn ends and the vehicle is placed on (or
   circularized into) the target orbit.

Each phase boundary — staging, atmosphere exit, engine cut-off — is found automatically by an
**adaptive numerical integrator with event detection**, which detects the exact instant a physical
condition is met rather than relying on fixed timing.

## 4. The three axes the tool compares

The heart of the work is that three independent choices can be varied and compared.

### 4.1 Guidance law — *how the vehicle steers*

Nine guidance modes are implemented, spanning a spectrum from passive to optimal:

| Mode | Family | One-line description |
|---|---|---|
| `gravity_turn` | Passive | No active steering after the kick; gravity shapes the path. |
| `cpr` | Passive (kinematic) | Constant pitch rate: pitch ramps steadily from vertical to horizontal. |
| `linear_tangent` | Tangent-steering | `tan(α+γ)` varies linearly with time-to-go. |
| `bilinear_tangent` | Tangent-steering | Ratio of two linear functions — controls both value and rate at burnout. |
| `apollo` | Explicit closed-loop | Linear-in-time acceleration commands that meet terminal position **and** velocity targets. |
| `peg` | Explicit closed-loop | Classical Powered Explicit Guidance (Space-Shuttle-heritage) with an iterative major loop. |
| `peg_new` | Explicit closed-loop | Analytical predictor-corrector PEG built around the velocity-to-be-gained vector. |
| `exp_shooting` | Open-loop, solved | Exponential pitch law fixed once by a boundary-value (shooting) solve. |
| `indirect_pmp` | Optimal-control reference | Pontryagin Minimum Principle: steering follows the optimal-control law from costates. |

`indirect_pmp` is **Stage-2-only by design**: the costate-optimal control steers only the
exo-atmospheric second-stage arc, while Stage 1 always flies the fixed gravity turn (a full-ascent
extension was explored and reverted).

This mix lets the thesis place simple, robust laws and sophisticated closed-loop laws against a
near-optimal benchmark.

### 4.2 Mission architecture — *how the vehicle reaches the orbit*

- **`apogee_check`** — burn, then coast up to apogee, then perform an impulsive **circularization
  burn**. The engine cut-off is triggered by physics: the burn stops exactly when the coasting
  trajectory's apogee would reach the target altitude.
- **`pso_coast`** — a **thrust → coast → thrust** profile that inserts directly into the target orbit
  (no separate circularization burn).
- **`direct`** — a **single continuous burn** that ends the instant the vehicle reaches circular
  orbital velocity.
- **`segmented` (multi-guidance)** — instead of one guidance law for the whole ascent, the vehicle
  flies an **ordered schedule of guidance laws**, each one activated at a chosen altitude.

When segmented mode is enabled (`MULTI_GUIDANCE_ENABLED`), the first scheduled law takes over right
after the pitch-over kick — a gravity turn is only one selectable option here, it is no longer forced
to fly first. Every non-final segment steers toward the optimal *(altitude, velocity, flight-path
angle)* waypoint taken from a cached indirect-PMP reference trajectory, evaluated at the next
segment's activation altitude; the final segment performs the orbit insertion itself, reusing the
`pso_coast` thrust–coast–thrust engine. Because a segment can be set to activate below MECO, a
closed-loop law can fly *during* the first stage — something none of the other architectures allow.
The activation altitudes can be fixed by the user or, when `MULTI_GUIDANCE_OPTIMIZE_ALTITUDES` is
set, chosen by the PSO itself so as to minimize Stage-2 burn time. The segmented solver has its own
dedicated PSO configuration block (`PSO_MG_*`), separate from the `pso_coast` PSO settings it used to
reuse.

### 4.3 Optimization method — *how the best trajectory is found*

- **Brute-force grid search** — exhaustively evaluates ~1000 candidate kick angles (one variable) and
  picks the best. Robust and simple, well suited to this low-dimensional, non-smooth problem.
- **Particle Swarm Optimization (PSO)** — a population-based global optimizer (via the PyGMO library)
  used when several variables are tuned at once, e.g. kick angle plus coast/burn timing (2–4
  variables).
- **Indirect optimal control** — solves the optimal-control problem by searching over the initial
  *costate* values (7 variables) so that the optimality (transversality) conditions are met; this
  produces the near-optimal reference trajectory used by the `indirect_pmp` mode.

A recurring design point: where possible the cut-off is determined by **physics** (apogee match or
circular-velocity), so the optimizer only has to tune a few meaningful variables.

## 5. Modeling-fidelity options

The same mission can be re-run at different levels of physical fidelity, which is what makes the
"effect of each phenomenon" studies possible:

- **Atmosphere on/off** — including a clean no-atmosphere reference case.
- **Earth rotation on/off** — including or excluding Coriolis/centrifugal pseudo-forces and the
  launch-azimuth correction for the rotating Earth.
- **Engine-performance model** — first-stage thrust and specific impulse as sea-level, vacuum,
  averaged, or altitude-varying.
- **Kick-manoeuvre profile** — a smooth triangular ramp or an instantaneous tilt.
- **Atmosphere-exit criterion** — by altitude or by dynamic pressure.

## 6. User choices (configuration)

Everything is set in a single parameter file; the choices group naturally into three categories.

**(a) Mission & vehicle**

| Setting | Meaning |
|---|---|
| Target orbit | Altitude and inclination of the desired circular orbit (baseline: 500 km, 51.6°). |
| Launch site | Launch latitude (baseline: 28.5°, Kennedy Space Center). |
| Vehicle specification | Stage masses, thrust, specific impulse, and aerodynamic data (Falcon-9-class baseline). |

**(b) Method selection**

| Setting | Meaning |
|---|---|
| Guidance law | One of the nine steering modes (§4.1). |
| Mission architecture | Coast strategy: `apogee_check`, `pso_coast`, `direct`, or `segmented` (multi-guidance) (§4.2). |
| Optimization method | Brute-force, PSO, or indirect optimal control (§4.3). |

**(c) Modeling & run settings**

| Setting | Meaning |
|---|---|
| Fidelity toggles | Atmosphere, Earth rotation/pseudo-forces, engine model, kick profile, atmosphere-exit criterion (§5). |
| Run mode | Full optimization, or a fast mode that reuses a previously found optimum. |

## 7. Outputs and diagnostics

Each run produces:

- **Final orbit** — semi-major axis, eccentricity, apogee/perigee altitude, achieved inclination, and
  period.
- **Propellant used / payload capability**, plus a **velocity-loss breakdown** into gravity, drag, and
  steering losses (and the Earth-rotation gain).
- A **trajectory plot suite** — altitude, flight-path angle, steering and pitch angle, thrust,
  dynamic pressure (max-q), Mach number, the ground-track/trajectory profile, the loss history, and
  (for PSO runs) the optimizer convergence curve.

## 8. Software design

The code is organized in **modular layers**: physical constants and vehicle data at the bottom;
environment models (gravity, atmosphere, Earth rotation) above them; the equations-of-motion engine
in the centre; the guidance laws, the optimizers, and the plotting as separate, interchangeable
components. The defining principle is **separation of concerns** — the optimizer does not know which
guidance law is active; it simply supplies a candidate and receives a score. That is precisely what
makes the comparison across guidance laws and architectures fair. The numerical machinery rests on
two well-established libraries: `scipy.integrate.solve_ivp` for adaptive integration with event
detection, and `pygmo` for Particle Swarm Optimization.

## 9. Assumptions and limitations (summary)

Planar 3-DOF point-mass motion; spherical, non-oblate Earth (no J₂); exponential atmosphere with drag
and lift both included (`F_L = q · C_L · A`); no attitude/rotational dynamics. These keep the model
fast and transparent — appropriate for a comparative, preliminary-design study — while leaving clear
avenues for higher-fidelity extensions.
