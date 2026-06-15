**Title:** Concurrent Simulation of Launcher Trajectories

**Description:**
This project compares several algorithms for simulating launch
vehicle trajectories — including gravity‑turn, polynomial,
tangent and bi‑tangent methods — with the goal of identifying
which approaches perform best for preliminary launcher design.

The work will produce a tool that can simulate and optimize
trajectories under different control laws, run parametric
studies concurrently to speed up analysis, and provide
launch-site-specific guidance (for example, the recommended
initial azimuth and payload capability) while accounting for
launch latitude and planetary rotation.

**Objectives:**
- Implement and compare multiple trajectory algorithms
- Build a flexible simulation tool that supports different
	control laws and optimization objectives
- Enable concurrent execution to accelerate parametric studies
- Produce decision guidance for initial azimuth and payload
	capability given launch latitude and target orbit

**Methodology:**
- Implement the selected algorithms (gravity‑turn, polynomial,
	tangent, bi‑tangent) in a common simulation framework
- Support trajectory optimization and constraint handling
- Run comparative simulations across a range of vehicle and
	launch-site parameters, using concurrent runs where
	appropriate

**Expected results:**
- For a given vehicle and mission constraints, identify which
	algorithm yields the best payload to orbit
- Quantify sensitivity to launch latitude and final orbit
	altitude
- Provide clear recommendations for a first-pass design tool
	that practitioners can use during early-stage launcher design

**Deliverables:**
- The simulation and optimization tool (codebase)
- A set of comparative results and plots showing algorithm
	performance across representative cases
- A short report summarizing findings and recommended
	algorithms for preliminary design use

**Last updated:** 2025-11-10

---

## Implementation status (2026-06-15 addendum)

The implementation has grown beyond the four original families (gravity-turn,
polynomial, tangent, bi-tangent) named above. The simulator now supports
**9 guidance modes**, selected via `GUIDANCE_MODE` in
`Tese/src/Input_File/simulation_parameters.py`:

- `gravity_turn`, `linear_tangent`, `bilinear_tangent` — the original three
  families
- `apollo` — Apollo-style polynomial acceleration-command guidance
- `cpr` — constant pitch-rate guidance
- `peg` and `peg_new` — Powered Explicit Guidance (classical and an
  analytical predictor-corrector derivation from first principles)
- `exp_shooting` — exponential pitch-law guidance via single-shot shooting
- `indirect_pmp` — indirect optimization via Pontryagin's Minimum Principle
  with PSO-optimised initial costates

Trajectory optimization now offers **three strategies**, selected via
`COAST_METHOD`:

- `apogee_check` — the original brute-force kick-angle search with
  apogee-match engine cutoff (Sections 1–2 of
  `optimization_process_explanation.md`)
- `pso_coast` — a 4-variable PyGMO PSO jointly optimising kick angle and
  coast/burn timing for direct orbit insertion
- `direct` — a single continuous Stage-2 burn cut at orbital insertion, with
  the kick angle found either by brute-force grid search or a 2-variable PSO

See `GUIDANCE_MODE_README.md` and `optimization_process_explanation.md` for
details on each mode and optimization strategy.