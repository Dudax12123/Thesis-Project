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
summary and draws a suite of plots (trajectory, steering angles, dynamic
pressure, etc.) via `Plots/new_plot_runner.py`.

Figures go to `Tese/src/Output_Plots/<run_id>/`, one folder per run. Data never
mixes with them.

It also **archives the run** to `Tese/src/Output/runs/`: the trajectory, every
diagnostic channel, the optimizer's convergence curve, and a manifest recording
the full configuration, the vehicle constants and the git commit. This happens
whatever `PLOT_SUITE` and `SAVE_PLOTS` are set to, and runs accumulate rather
than overwrite, so a solve never has to be flown twice to be looked at again:

```bash
python Tese/src/run_archive.py list
python Tese/src/run_archive.py compare <run_id> <run_id>
```

`compare` overlays any number of archived runs and prints the settings on which
they differ. `replay <run_id>` redraws the full plot suite from disk. Turn
archiving off with `ARCHIVE_RUNS = False`.

### Dependencies

- `numpy`, `scipy`, `matplotlib` — required for every configuration.
- `pygmo` — required only when `COAST_METHOD = "pso_coast"`,
  `COAST_METHOD = "direct"` with `DIRECT_OPTIMIZATION_MODE = "pso"`, or
  `GUIDANCE_MODE = "indirect_pmp"`. These configurations raise an `ImportError`
  with installation instructions (`conda install -c conda-forge pygmo`) if
  `pygmo` is missing.

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
  - `"direct"` — a single continuous Stage-2 burn cut at orbital insertion,
    either via a 1-D brute-force kick-angle sweep
    (`DIRECT_OPTIMIZATION_MODE = "brute_force"`) or a 2-variable PyGMO PSO
    (`DIRECT_OPTIMIZATION_MODE = "pso"`, requires `pygmo`).

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
