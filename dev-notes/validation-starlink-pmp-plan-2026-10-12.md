# Validation run: PMP reference with the Starlink (Nov 2019) mission (2026-10-12)

Follow-up to thesis §6.7 "Results Validation" (`sec:validation`). User decisions, 2026-10-12:
seed 3 only, swarm at 250×1000, refinement at the coast bounds 2000, 2500 and 3000 s, first pass
only.

## Mission

| Setting | Reference | Starlink run |
|---|---|---|
| `TARGET_ORBITAL_ALTITUDE` | 500 km | 280 km, circular (press kit's deployment altitude) |
| `TARGET_ORBIT_INCLINATION` | 51.6° | 53° (a label: enters no equation of motion) |
| `M_PAYLOAD` | 0 | 15 600 kg (NSF) |
| `LAUNCH_LATITUDE` | 28.5° | 28.5° |

The booster recovery is not modelled: Stage 1 burns to depletion, so MECO stays near 146 s.

## Why a branch and a worktree

- `M_PAYLOAD` is bound at import (`M_TOTAL_1`, `pso_coast_solver._DRY_MASS_2`), and constants
  must not be changed at run time, so it is edited in the file.
- Branch `validation-starlink` (one commit editing `rocket_specs.py` and
  `simulation_parameters.py`), never merged into `main`. Worktree
  `C:\Users\eduar\Desktop\Tese\Code\Thesis-Project-starlink-20261012`.
- Any `pmp_baseline` flight there may rebuild the worktree's `pmp_reference.npz`, not main's.
- Output: `Tese/src/Output/validation_starlink_20261012/` in the main repo (untracked).

## Steps

1. Smoke: `--case pmp_baseline --swarm-extremal --smoke`.
2. Swarm: `--case pmp_baseline --swarm-extremal --budget 250,1000 --set PSO_SEED=3`
   → `swarm/seed_3/`. About 2–4 h on one P-core.
3. Refinement: `pmp_swarm_polish.py --no-swarm --start <swarm npz>` through a coast-bound
   wrapper (as `launch_20261006/polish_coast_ub.py`), bounds 2000, 2500, 3000 s, first pass only
   → `polish/c2000|c2500|c3000/`.
4. Gate (show, then wait): propellant left, γ_p, coast, final burn, lowest point after MECO, orbit
   error, event timeline against the cited Starlink times (74, 153, 164, 204, 529 s; coast
   2 161 s; burn 2 s). Stop if γ_p is within 0.005 rad of [1.50, 1.57] or the coast is on its
   bound.
5. Thesis changes only after the user's go; wording shown first.
