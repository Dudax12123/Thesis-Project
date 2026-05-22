# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Python-based aerospace simulation and optimization framework for comparing rocket trajectory guidance algorithms. The target vehicle is a SpaceX Falcon 9–class two-stage rocket. The codebase simulates ascent trajectories using 9 different guidance laws and optimizes for minimum propellant consumption.

## Environment Setup

```bash
python -m venv .venv
.venv\Scripts\activate        # Windows
pip install numpy scipy matplotlib pytest pillow
```

## Commands

**Run the main simulation (from repo root):**
```bash
cd Tese/src
python main.py
```

**Run tests:**
```bash
cd Tese/src
pytest tests/
```

**Run a single test file:**
```bash
cd Tese/src
pytest tests/test_apollo_tgo.py -v
```

## Configuration

All simulation parameters are in [Tese/src/Input_File/simulation_parameters.py](Tese/src/Input_File/simulation_parameters.py). Key settings:

- `GUIDANCE_MODE` — selects which guidance law to use (string: `"gravity_turn"`, `"apollo"`, `"peg_new"`, etc.)
- `TARGET_ALTITUDE` — target circular orbit altitude (m)
- `LAUNCH_LATITUDE`, `TARGET_INCLINATION` — orbital geometry
- `EARTH_ROTATION_ENABLED`, `AZIMUTH_MODE` — Earth rotation effects
- `ALPHA_LOWEST` / `ALPHA_HIGHEST` — brute-force optimizer search bounds for initial kick angle
- `AERO_DRAG_ENABLED`, `AERO_LIFT_ENABLED` — aerodynamic forces

## Architecture

```
Tese/src/
├── main.py                    # Entry point: runs optimizer then full simulation + plots
├── Input_File/
│   └── simulation_parameters.py   # All user-facing config
├── Simulation/
│   ├── rocket_ascent.py       # ODE system (scipy.integrate.solve_ivp) — the physics engine
│   └── solver.py              # Brute-force grid search optimizer (scipy.optimize.brute)
├── Guidance/                  # 9 guidance law implementations
├── Auxiliary/                 # Physics models (atmosphere, gravity, Earth rotation, constants)
├── Plots/                     # 26 metric plots + orchestrator
│   ├── new_plot_runner.py     # Plotting pipeline entry point
│   ├── plot_state_utils.py    # Shared plotting utilities
│   └── new_metrics/           # One file per metric (altitude, Mach, drag, etc.)
└── tests/
    └── test_apollo_tgo.py
```

### ODE State Vector

`rocket_ascent.py` integrates: `[s, r, v, gamma, m, heading]`
- `s` — downrange distance
- `r` — geocentric radius
- `v` — velocity magnitude
- `gamma` — flight-path angle
- `m` — vehicle mass
- `heading` — azimuth (active only when Earth rotation is enabled)

### Guidance Interface

All guidance modules in `Tese/src/Guidance/` share a common calling convention: they receive the current state + time, and return the steering angle `alpha` (angle of attack = `theta - gamma`). Guidance activates after an initial kick maneuver and can be configured to start either `"after_kick"` or `"after_atmosphere_exit"`.

### Optimization Loop

`solver.py` runs a brute-force grid search (1000 points) over the initial kick angle. Each candidate runs a full trajectory simulation; the objective is minimum total propellant (ascent ΔV + circularization burn at apogee).

### Guidance Modes

| Mode | Key Characteristic |
|---|---|
| `gravity_turn` | Ballistic after kick, no active steering |
| `simple_polynomial` | Linear flight-path angle transition |
| `linear_tangent_steering` | Classical linear tangent law |
| `bilinear_tangent_steering` | Ratio of two linear tangent functions |
| `apollo_guidance` | Polynomial acceleration profiles with terminal constraints |
| `cpr_guidance` | Constant pitch rate ramp |
| `peg_guidance` | Powered Explicit Guidance via damped fixed-point iteration |
| `peg_guidance_new` | Analytical predictor-corrector PEG (Pontryagin minimum principle) |
| `exp_shooting_guidance` | Exponential pitch law with single-shot optimization |

PEG variants (`peg_guidance`, `peg_guidance_new`) activate after atmosphere exit (~65 km); all others activate immediately after the kick.

## Key Physics Details

- Two-stage rocket: stage separation timing and specs in [Tese/src/Auxiliary/rocket_specs.py](Tese/src/Auxiliary/rocket_specs.py)
- Atmosphere: exponential density model with configurable scale height
- Fairing jettison triggered at atmosphere exit (65 km)
- Back-pressure thrust loss modeled via ambient pressure
- Earth rotation adds Coriolis and centrifugal pseudo-forces to the ODE when enabled
