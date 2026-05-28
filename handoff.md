# Thesis Project — Session Handoff

## Project Overview

Rocket ascent trajectory optimization using the **Indirect Method via Pontryagin's Minimum Principle (PMP)** with **PyGMO PSO**.

- Working directory: `C:\Users\eduar\Desktop\Tese\Thesis-Project`
- Active branch: `Final_TPBVP`
- Python environment: `conda activate pygmo-env` (Miniforge at `C:\Users\eduar\miniforge3\envs\pygmo-env`)
- Run command: `conda run -n pygmo-env python Tese/src/main.py`

---

## Current Guidance Mode

`GUIDANCE_MODE = "indirect_pmp"` in [Tese/src/Input_File/simulation_parameters.py](Tese/src/Input_File/simulation_parameters.py)

---

## Key Architecture

### Stage 1 — Gravity Turn with Instantaneous Kick
- Kick is an **instantaneous γ jump** applied at `TIME_TO_START_KICK = 7.5 s`
- The kick angle is one of the 7 PSO decision variables (`gamma_p` maps to kick via `kick_angle = gamma_p - π/2`)
- Implementation: `_run_stage1a_with_kick()` in [Tese/src/Simulation/rocket_ascent.py](Tese/src/Simulation/rocket_ascent.py)
  - Splits `solve_ivp` at `TIME_TO_START_KICK`: runs segment 1, applies `state[3] += kick_angle`, runs segment 2
  - Uses `_MergedSol` class to concatenate both segments transparently

### Stage 2 — PMP Guidance
- Three sub-arcs: **Arc 1 (thrust)** → **Arc 2 (coast)** → **Arc 3 (thrust)**
- Costates `[λ_r, λ_v, λ_γ]` propagated alongside physical state `[s, r, v, γ, m]`
- Control law: `α = atan2(−λ_γ/V, −λ_V)` (Eq. 34 of paper)
- Drag-free EOM used in Stage 2 (consistent with costate equations)
- **Apogee trigger is NOT used**; coast timing fully controlled by PSO

### Objective Function (Eq. 39)
```
J'  = J  +  s1|Δh|  +  s2|ΔV|  +  s3|Δγ|  +  s4|transversality|  +  C
J   = t_f − t_cf = T_burn_total       (total Stage-2 powered time, minimized)
t_f = T_burn_total + delta_tc         (total Stage-2 flight time)
t_cf = delta_tc                        (coast duration)
```

---

## PSO Decision Variables (7D)

| Variable | Description | Bounds |
|---|---|---|
| `lambda0_r` | Initial costate for r | [-1, 1] |
| `lambda0_v` | Initial costate for v | [-1, 1] |
| `lambda0_g` | Initial costate for γ | [-1, 1] |
| `delta_tc` | Coast duration [s] | [0, 2000] |
| `delta_tr_pct` | Stage-2 burn as % of T_max | [0, 100] |
| `coast_start_pct` | Coast start as % of burn time | [0, 100] |
| `gamma_p` | Post-kick flight-path angle [rad] | [1.54, 1.57] |

Current PSO settings (in `simulation_parameters.py`):
- `PSO_N_PARTICLES = 100`, `PSO_MAX_GENERATIONS = 50`
- `PSO_OMEGA = 0.7298`, `PSO_C1 = PSO_C2 = 2.05`, `PSO_VMAX = 0.5`

Penalty weights:
- `PENALTY_W_ALTITUDE = 1e-3`, `PENALTY_W_VELOCITY = 1e-1`, `PENALTY_W_FPA = 1e2`, `PENALTY_W_TRANSVERS = 1e1`

---

## Key Source Files

| File | Role |
|---|---|
| [Tese/src/main.py](Tese/src/main.py) | Entry point — `execute()` calls PSO, full run, plots |
| [Tese/src/Simulation/indirect_pso_solver.py](Tese/src/Simulation/indirect_pso_solver.py) | PSO outer loop, trajectory evaluator, objective function |
| [Tese/src/Guidance/indirect_pmp_guidance.py](Tese/src/Guidance/indirect_pmp_guidance.py) | PMP control law, costate derivatives, Hamiltonian |
| [Tese/src/Simulation/rocket_ascent.py](Tese/src/Simulation/rocket_ascent.py) | Stage 1 simulation, instantaneous kick logic |
| [Tese/src/Input_File/simulation_parameters.py](Tese/src/Input_File/simulation_parameters.py) | All tunable parameters |
| [Tese/src/Plots/new_plot_runner.py](Tese/src/Plots/new_plot_runner.py) | Plot suite runner (`run_new_plot_suite`) |
| [check_pygmo.py](check_pygmo.py) | Environment + unit test readiness check |

---

## Recent Changes (last two sessions)

### Non-blocking plots fix (main.py)
All `run_new_plot_suite(...)` calls inside `execute()` now use `show=False, close_after=False`.
The single `plt.show()` (blocking) lives at the very bottom of `__main__`, **after** `execute()` returns.
This means: all terminal output prints first, then all plots appear simultaneously.

```python
# __main__ block (Tese/src/main.py ~line 640)
if __name__ == "__main__":
    execute()
    plt.show()   # single blocking call after all terminal output is printed
```

`input()` was intentionally removed — `conda run` does not attach interactive stdin and it caused `EOFError`.

### Instantaneous kick (rocket_ascent.py)
Replaced the old triangular pitch-over program with an event-driven `solve_ivp` split at `TIME_TO_START_KICK = 7.5 s`. The kick is a discontinuous γ jump: `state[3] += kick_angle` applied between the two segments.

### Correct J definition (indirect_pso_solver.py)
```python
# J = total burn time = T_burn_total
t_f_result  = T_burn_total + delta_tc   # total Stage-2 flight time (planned)
t_cf_result = delta_tc                   # coast duration (planned)
# J = t_f_result - t_cf_result = T_burn_total  ← correct per paper Eq. 27
```
This was previously broken (J ≈ 0), causing PSO to trivially minimize by collapsing arc 3 and coast.

### PyGMO-only solver
The scipy fallback was removed. `run_pso_optimization()` raises `ImportError` if `pygmo` is not installed (no silent fallback).

---

## Known Issues / Outstanding Work

- **PSO convergence**: With 100 particles × 50 generations the optimizer has not been observed to fully converge before `max_gen`. Paper used 250 × 1000 for the full run. Consider increasing `PSO_N_PARTICLES` and `PSO_MAX_GENERATIONS` once the setup is verified correct.
- **Near-zero coast**: PSO tends to produce solutions with very short coast phases. This may be physical for the current vehicle/target-orbit combination, or it may indicate that `PENALTY_W_TRANSVERS` needs tuning to better enforce the Weierstrass–Erdmann condition.
- **Penalty weight tuning**: The weights `s1…s4` in `simulation_parameters.py` are the main levers for convergence quality. If terminal constraint errors are large, raise the corresponding weight.

---

## Environment Notes

- The `.venv` folder in the project root activates automatically in VS Code. This overrides the conda env and causes `ModuleNotFoundError: No module named 'pygmo'`. Always `deactivate` the venv or use `conda run -n pygmo-env python ...` from a clean shell.
- The readiness check script `check_pygmo.py` runs a mini-PSO (5 particles × 5 gen) to verify the full pipeline end-to-end before a full run.

---

## Quick-start Commands

```powershell
# Verify environment (run once per session)
conda run -n pygmo-env python check_pygmo.py

# Full optimisation run
conda run -n pygmo-env python Tese/src/main.py
```
