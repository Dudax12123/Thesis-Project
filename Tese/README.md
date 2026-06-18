# Thesis-Project: Concurrent Launch-Vehicle Trajectory Simulator

A two-stage launch-vehicle ascent simulator and trajectory optimizer. It
integrates the equations of motion from launch to orbital insertion, supports
nine different ascent guidance laws, and can optimize the kick angle and/or
burn/coast timing to reach a target circular orbit with minimum propellant.

## Running the simulator

From the repository root:

```bash
python Tese/src/main.py
```

`main.py` adds `Tese/src` to `sys.path` itself, so it can also be run from
inside `Tese/src` as `python main.py`. On completion it prints a mission
summary and writes a suite of plots (trajectory, steering angles, dynamic
pressure, etc.) via `Plots/new_plot_runner.py`.

### Dependencies

- `numpy`, `scipy`, `matplotlib` — required for every configuration.
- `pygmo` — required only when `COAST_METHOD = "pso_coast"`,
  `COAST_METHOD = "direct"`, or `GUIDANCE_MODE = "indirect_pmp"`. These
  configurations raise an `ImportError` with installation instructions
  (`conda install -c conda-forge pygmo`) if `pygmo` is missing.

## Configuration

All tunable parameters live in
[`src/Input_File/simulation_parameters.py`](src/Input_File/simulation_parameters.py).
The two most important switches are:

- **`GUIDANCE_MODE`** — selects the ascent guidance law (9 options):
  `gravity_turn`, `linear_tangent`, `bilinear_tangent`, `apollo`, `cpr`,
  `peg`, `peg_new`, `exp_shooting`, `indirect_pmp`.
  See [`Project_Description/GUIDANCE_MODE_README.md`](Project_Description/GUIDANCE_MODE_README.md)
  for a description of each mode.

- **`COAST_METHOD`** — selects how the Stage-2 burn/coast/insertion timing is
  determined (3 options):
  - `"apogee_check"` — brute-force kick-angle search, burn cut when the
    osculating apogee matches the target altitude, followed by a
    circularization burn.
  - `"pso_coast"` — 4-variable PyGMO PSO optimizes a
    thrust → coast → thrust profile for direct orbit insertion (requires
    `pygmo`).
  - `"direct"` — a single continuous burn straight to orbit insertion,
    optimized by a 2-variable PyGMO PSO (`gamma_p` + burn duration; requires
    `pygmo`). The only `COAST_METHOD` that supports single-stage vehicles.

  See [`Project_Description/optimization_process_explanation.md`](Project_Description/optimization_process_explanation.md)
  for details on all three optimization paths.

## Documentation

- [`Project_Description/`](Project_Description/) — design notes, guidance-mode
  reference, optimization process, and Earth-rotation handling.
- [`simulator_methodology.tex`](simulator_methodology.tex) and
  [`Project_Description/simulator_eom_dynamics_kinematics.tex`](Project_Description/simulator_eom_dynamics_kinematics.tex) —
  the mathematical methodology and equations of motion/kinematics.
- [`../dev-notes/`](../dev-notes/) — session handoffs and exploratory scripts
  (not part of the simulator itself).
