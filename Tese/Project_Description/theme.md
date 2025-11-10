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