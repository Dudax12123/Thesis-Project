# Simulator Overview — *Concurrent Simulation of Rocket Ascent Trajectories*

**What it is.** A configurable Python tool that simulates the powered ascent of a two-stage
(Falcon-9-class) launch vehicle from lift-off to a target circular orbit. Its purpose is to compare
different **guidance laws** (how the rocket steers) and **trajectory-optimization methods** (how the
best trajectory is found) inside one common framework, and to quantify the mission outcomes —
propellant used / payload capability, ΔV losses, and orbit-insertion accuracy.

**How it works.** The tool numerically integrates the rocket's equations of motion through the
natural mission phases — lift-off and vertical rise → pitchover *"kick"* → first-stage powered ascent
→ stage separation and short coast → second-stage guided ascent → engine cut-off → orbit insertion.
An adaptive integrator with *event detection* automatically locates the phase transitions (staging,
atmosphere exit, engine cut-off). The software is modular — the environment models (gravity,
atmosphere, Earth rotation), the active guidance law, the optimizer, and the plotting are separate
components — so any steering law can be swapped in and compared on equal terms.

**Processing flow.**

```
Optimizer ─→ Simulator (environment + guidance + equations of motion, evaluated step by step)
          ─→ event detection / engine cut-off ─→ trajectory ─→ score ─┐
          └───────────────────────── repeat ────────────────────────┘
```

## User choices

Six groups of decisions the user controls through a single parameter file:

| Choice | What it controls | Options (summarized) |
|---|---|---|
| **Guidance law** | How the vehicle steers during the second-stage burn | 9 modes, from a passive *gravity turn* and simple tangent-steering laws, up to closed-loop *Powered Explicit Guidance (PEG)*, *Apollo* polynomial guidance, and an *optimal-control (PMP)* reference |
| **Mission architecture** | How the vehicle reaches the final orbit | *coast + circularization burn* · *thrust–coast–thrust direct insertion* · *single continuous burn* |
| **Optimization method** | How the best trajectory is found | *brute-force grid search* (1-variable), *Particle Swarm Optimization* (multi-variable), or *indirect optimal control* |
| **Physics fidelity** | Which effects are modelled | atmosphere on/off · Earth rotation & Coriolis/centrifugal forces on/off · engine-performance model (sea-level / vacuum / …) · kick-manoeuvre profile |
| **Mission & vehicle** | The scenario being flown | target orbit altitude & inclination · launch-site latitude · vehicle masses, thrust, specific impulse, aerodynamics |
| **Run mode** | Speed vs. thoroughness | full optimization, or a fast mode that reuses a stored optimum |

**Outputs.** The final orbit (semi-major axis, eccentricity, apogee/perigee altitudes, achieved
inclination), propellant used and payload capability, a gravity/drag/steering **loss breakdown**, and
a suite of trajectory diagnostic plots.

**Key design idea.** *Separation of concerns:* the optimizer does not know which guidance law is
active, so every guidance law and mission architecture is compared under an identical, fair
procedure.
