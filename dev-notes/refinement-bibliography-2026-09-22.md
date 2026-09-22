# The bibliography behind the PMP refinement: organisation and meaning — 2026-09-22

This document explains which published work supports each part of the refinement in
`dev-notes/pmp_swarm_polish.py`, what each reference actually says, how strongly it supports us, and what it must
**not** be cited for. It supersedes the reference tables of `dev-notes/refinement-references-2026-09-21.md`; the
BibTeX has moved to Appendix B here.

Standing constraints:
- **Thesis edits are on hold**, so nothing here has been added to `Thesis_Bibliography_DB.bib`. The suggested wording
  in §7 is a draft for when edits resume.
- **Every entry's publication details were checked against Crossref or the publisher's metadata.** The *content* each
  one is cited for was checked at the level given by its "Checked" tag (§1.3). Read the full text before quoting any
  statement more specific than its tag covers.

---

## 1. How to read this document

### 1.1 What the refinement does

The refinement takes a swarm result (the 7-number `decision_vector` of an `indirect_pmp` archive) and turns it into an
**extremal**: a trajectory that satisfies the first-order necessary conditions of the Stage-2 optimal control problem to
tight tolerances. It does so in two stages.

**Stage A: the extremal at fixed γ_p** (`newton()`, [pmp_swarm_polish.py:182](pmp_swarm_polish.py:182))
- Stage 2 flies burn 1 (`D1`), a coast (`Dc`) and a last burn (`D3`).
- **Unknowns:** `u = [a, b, D1, Dc, D3]`, where `a` and `b` are the costate direction written as two angles on the unit
  sphere (`x_from`/`u_from_x`, [:83](pmp_swarm_polish.py:83), [:90](pmp_swarm_polish.py:90)).
- **Residuals**, five of them:
  - the insertion altitude, speed and flight-path angle errors;
  - `H_coast_end`, the Hamiltonian at the end of the coast;
  - `H_burn1_end − H_last_burn_start`, the difference of the Hamiltonian across the coast.
- **Solver:** Levenberg–Marquardt (`scipy.optimize.least_squares(method="lm")`,
  [:211](pmp_swarm_polish.py:211)).
  - The Jacobian is taken by central finite differences ([:203](pmp_swarm_polish.py:203)).
  - Unknowns and residuals are scaled (`S`, `RES_SCALE`, [:78](pmp_swarm_polish.py:78), [:169](pmp_swarm_polish.py:169)).
  - A solution counts as converged when every residual is within `TOL_PHYS` ([:170](pmp_swarm_polish.py:170)).
- **Coast bounds:** if the coast leaves [0, 2 000] s it is pinned at the bound. `H_coast_end = 0` is dropped and replaced
  by a one-sided sign condition ([:218](pmp_swarm_polish.py:218), [:289](pmp_swarm_polish.py:289)).

**Stage B: continuation in γ_p** (`polish()`, [:296](pmp_swarm_polish.py:296))
- γ_p is the Stage-1 kick parameter, and no costate condition governs it.
- It is stepped in both directions. Each re-solve starts from the previous extremal, with a fixed step and no
  extrapolation ([:323](pmp_swarm_polish.py:323)).
- A sweep stops at a γ_p bound, at the first failed solve ([:344](pmp_swarm_polish.py:344)), or once propellant drops
  50 kg below the best found ([:350](pmp_swarm_polish.py:350)).
- The best extremal found is kept. If it is the last point of a sweep it is flagged ([:359](pmp_swarm_polish.py:359)).

**What it found** (2026-09-21, seed 3, 750×1500): the γ_p sweep ends where `D3` reaches zero. That is the end of the
two-burn family of extremals. Best point: 22 261.2 kg at γ_p 1.537139.

### 1.2 The eight claims the bibliography has to carry

| # | Claim | Refinement step |
|---|---|---|
| **K1** | The problem is posed through Pontryagin's necessary conditions, with costates. | whole method |
| **K2** | A global heuristic supplies the starting guess, and a local solver on the necessary conditions produces the extremal. | swarm → Stage A |
| **K3** | The unknowns (normalised costate direction, arc durations) and the conditions (terminal state, Hamiltonian conditions for free burn and coast durations) are the right ones for a burn–coast–burn stage. | Stage A system |
| **K4** | Levenberg–Marquardt with a finite-difference Jacobian is a sound solver for that system. | Stage A solver |
| **K5** | A duration held at a bound gets a one-sided (KKT) condition. | coast pinning |
| **K6** | A parameter outside the costate problem (γ_p) can be handled by continuation, warm-starting each solve. | Stage B |
| **K7** | Treating a Stage-1 parameter separately from the upper-stage costate problem is established practice for launch vehicles. | architecture |
| **K8** | The limits: shooting is sensitive, continuation fails when the arc structure changes, and an extremal is only first-order. | Chapter 6 caveats |

### 1.3 Tags

**In bib** means the entry is already in `Thesis_Bibliography_DB.bib`; **New** means it would be added (BibTeX in
Appendix B).

**Checked** says how far its content was confirmed:

| Tag | Meaning |
|---|---|
| **full text** | I read the paper; the stated content is confirmed. |
| **abstract** | The abstract, or the publisher's summary. The content stated is at the abstract's level of detail. |
| **secondary** | Described by other sources or search summaries; the paper itself not read. Read it before quoting anything. |
| **metadata** | Only the bibliographic details were confirmed. The content attributed is standard textbook material, still to be confirmed against the chapter used. |

---

## 2. How the bibliography is organised

The references form seven layers. Each layer carries some of the claims, and each one leans on the layer above it.

```
L1  Foundations of optimal control ............ K1, K3, K5   (Pontryagin; Bryson & Ho; Longuski et al.)
      │
L2  Indirect methods and shooting ............. K1, K4, K8   (Betts; Colasurdo & Casalino; Stoer & Bulirsch; Pesch; Trélat)
      │
L3  Heuristic + indirect hybrids .............. K2, K3       (Conway; Pontani & Conway ×2; Jiang et al.; Hecht & Botta)
      │
L4  Launch-vehicle ascent practice ............ K3, K6, K7, K8 (Lu et al.; Gath & Calise; Brown et al.; Pallone et al.;
      │                                                        Pontani & Teofilatto; Pontani ×3; Calise et al.; Bonalli et al.)
L5  The numerical solver ...................... K4, K5       (Levenberg; Marquardt; Moré; Nocedal & Wright; SciPy)
      │
L6  Continuation .............................. K6, K8       (Allgower & Georg; Keller; Trélat; Pan et al. ×2)
      │
L7  Limits: structure changes, second order ... K8           (Bonnans & Hermant; Pesch; Pontani 2013)
```

The layers read as one argument:
- **L1–L2** say what an extremal is and how indirect methods compute one.
- **L3** says why a swarm followed by a local solve is a recognised way to start that computation.
- **L4** shows the same problem structure in launch vehicles: a burn–coast–burn upper stage, a Stage-1 parameter kept
  outside, and continuation in ascent work.
- **L5–L6** justify the specific algorithms.
- **L7** bounds what the result may be claimed to be.

**Claims × layers** (● main support, ○ secondary support):

| | L1 | L2 | L3 | L4 | L5 | L6 | L7 |
|---|---|---|---|---|---|---|---|
| K1 necessary conditions | ● | ● | | ○ | | | |
| K2 swarm → local | | | ● | ○ | | | |
| K3 unknowns & conditions | ● | | ○ | ● | | | |
| K4 LM + FD Jacobian | | ○ | | ○ | ● | | |
| K5 one-sided bound | ● | | | | ● | | |
| K6 γ_p continuation | | ○ | | ○ | | ● | |
| K7 Stage-1 parameter outside | | | | ● | | | |
| K8 limits | | ● | ○ | ● | | ○ | ● |

---

## 3. The references, layer by layer

For each reference: what the work is; what it supports here; what not to use it for.

### L1 — Foundations of optimal control

**Pontryagin et al. 1987, *The Mathematical Theory of Optimal Processes*** · `pontryagin1987mathematical` · In bib · metadata
- **What it is:** the original statement of the maximum principle: optimal controls maximise, or minimise by
  convention, the Hamiltonian along costate trajectories.
- **Supports:** K1. Every condition the refinement solves is a consequence of the principle.
- **Not for:** any specific numerical method.

**Bryson & Ho 1975, *Applied Optimal Control*** · New `bryson1975applied` · metadata (standard textbook content)
- **What it is:** the engineering textbook on optimal control. It covers the Euler–Lagrange and costate equations,
  free-final-time transversality (H = 0 at a free terminal time without a time-dependent terminal cost), problems with
  parameters, and interior-point or junction conditions for multi-arc problems. It also covers the adjoint-sensitivity
  identity: the costate times a state perturbation gives the change of the terminal quantities.
- **Supports:** K1, and K3 in particular. Our duration conditions are *derived* from these results. The adjoint
  sensitivity of the terminal state to each arc duration is written with the reduced Hamiltonian at the arc junctions,
  which gives:
  - `λ(t_f)·∂x_f/∂Dc = H_coast_end`
  - `λ(t_f)·∂x_f/∂D3 = H_burn_end`
  - `λ(t_f)·∂x_f/∂D1 = H_burn1_end − H_last_burn_start + H_burn_end`

  These are the forms pinned in `tests/test_pmp_transversality.py`. Also supports K5: parameters with bounds.
- **Not for:** the specific no-mass-costate form. No textbook writes our conditions; cite Bryson & Ho as the theory and
  present the formulas as this thesis's derivation.

**Longuski, Guzmán & Prussing 2014, *Optimal Control with Aerospace Applications*** · New `longuski2014optimal` · metadata (standard textbook content)
- **What it is:** a modern aerospace optimal-control textbook: multi-arc problems, corner conditions, bang–off–bang
  thrusting with coast arcs, primer-vector theory.
- **Supports:** K1 and K3, as an aerospace-oriented alternative to Bryson & Ho for burn and coast arcs.
- **Not for:** anything numerical.

### L2 — Indirect methods and shooting

**Betts 1998, "Survey of Numerical Methods for Trajectory Optimization", *JGCD* 21(2)** · New `betts1998survey` · metadata + secondary
- **What it is:** the standard survey dividing trajectory optimisation into direct methods (transcription to nonlinear
  programming) and indirect methods (the necessary conditions solved as a boundary-value problem, usually by shooting
  with Newton's method).
- **Supports:** K1, placing the refinement in the indirect class. Also K8: the survey is the usual citation for
  indirect shooting needing good costate guesses and having a small region of convergence.
- **Not for:** quoting the sensitivity wording. It is confirmed only from secondary sources, so read §-level text
  before quoting.

**Colasurdo & Casalino 2012, "Indirect Methods for the Optimization of Spacecraft Trajectories"** (Springer, *Modeling and Optimization in Space Engineering*, pp. 141–158) · New `colasurdo2012indirect` · secondary
- **What it is:** a tutorial chapter from the Turin school. Optimal control theory turns a trajectory problem into a
  multi-point boundary-value problem, solved by Newton iteration, with homotopy used to find tentative solutions.
- **Supports:** K1 and K4 (Newton-type solution of the boundary-value problem), and K6 (homotopy for starting guesses).
- **Not for:** Levenberg–Marquardt specifically.

**Stoer & Bulirsch 2002, *Introduction to Numerical Analysis*, 3rd ed.** · New `stoer2002introduction` · metadata (standard textbook content)
- **What it is:** the numerical-analysis text whose chapter on boundary-value problems treats single and multiple
  shooting and their conditioning.
- **Supports:** K8. Single shooting over a long arc is ill-conditioned, and multiple shooting is the standard remedy.
  That explains the refinement's 10⁷–10¹⁰ condition numbers across a coast of up to 1 447 s.
- **Not for:** anything specific to aerospace.

**Pesch 1994, "A Practical Guide to the Solution of Real-Life Optimal Control Problems", *Control and Cybernetics* 23(1/2)** · New `pesch1994practical` · metadata + secondary
- **What it is:** a practitioner's guide to multiple shooting combined with homotopy on realistic problems, including
  how the switching structure is found and changed.
- **Supports:** K8, and K6 as general practice for continuation in indirect methods. Continuation that meets a change
  of switching structure must be re-posed with the new structure.
- **Not for:** a statement about our specific end of family (the last burn → 0); that is our observation.

**Trélat 2012, "Optimal Control and Applications to Aerospace: Some Results and Challenges", *JOTA* 154(3)** · New `trelat2012optimal` · abstract
- **What it is:** a review of the maximum principle, conjugate-point theory and their numerical implementation for
  aerospace problems. It presents continuation (homotopy) as the tool that makes shooting practical by overcoming its
  initialisation difficulty.
- **Supports:** K6 (continuation around shooting) and K8 (conjugate points: a first-order extremal is not yet shown to
  be optimal).
- **Not for:** a γ_p-type parameter sweep specifically. Its continuation deforms problems; ours sweeps a design
  parameter (see §4).

### L3 — Heuristic + indirect hybrids

**Conway 2012, "A Survey of Methods Available for the Numerical Optimization of Continuous Dynamic Systems", *JOTA* 152(2)** · `Conway2012SurveyMethodsContinuousDynamicSystems` · In bib · abstract
- **What it is:** a survey of discretisation and nonlinear-programming methods, and of the more recent use of
  evolutionary and metaheuristic algorithms on the same problems.
- **Supports:** K2, as the general framing that metaheuristics are an accepted component of trajectory optimisation
  and are combined with other methods.
- **Not for:** the specific swarm → Levenberg–Marquardt pipeline.

**Pontani & Conway 2014, "Optimal Low-Thrust Orbital Maneuvers via Indirect Swarming Method", *JOTA* 162(1)** · `pontani2014indirectswarming` · In bib · abstract
- **What it is:** the "indirect swarming method". The analytical necessary conditions (costate equations and control
  law) are combined with a particle swarm that searches for the unknown costates, with no starting guess needed.
- **Supports:** K2, the swarm half. Our `indirect_pmp` swarm is exactly this kind of method: PSO over the initial
  costates, with the maximum-principle steering.
- **Not for:** a local refinement after the swarm. The abstract describes PSO enforcing the conditions by itself. This
  also answers the 19b handoff's §4.4 flag (§6).

**Pontani & Conway 2015, "Minimum-Fuel Finite-Thrust Relative Orbit Maneuvers via Indirect Heuristic Method", *JGCD* 38(5)** · `pontani2015indirectheuristic` · In bib · abstract
- **What it is:** the same indirect-heuristic approach applied to minimum-fuel manoeuvres. A switching function derived
  from the Hamiltonian decides the sequence and durations of the thrust and coast arcs.
- **Supports:** K3. Hamiltonian-based conditions decide burn and coast durations in a finite-thrust problem.
- **Not for:** launch vehicles or local refinement.

**Jiang, Baoyin & Li 2012, "Practical Techniques for Low-Thrust Trajectory Optimization with Homotopic Approach", *JGCD* 35(1)** · New `jiang2012practical` · secondary
- **What it is:** practical techniques for indirect low-thrust optimisation:
  - normalising the initial costates (with the cost multiplier) onto a **unit hypersphere**;
  - switching detection during integration;
  - energy-to-fuel homotopy;
  - searching for the normalised initial values with PSO before the shooting solve.
- **Supports:** K2 (PSO search, then shooting) and K3 (costate normalisation). The swarm's `‖λ‖ = 1` normalisation and
  the refinement's two-angle parameterisation are a use of that normalisation.
- **Not for:** the angle coordinates themselves; I have not confirmed that they parameterise the sphere by angles (see
  §6). Also not for launch vehicles.

**Hecht & Botta 2023, "Particle Swarm Optimization-Based Co-State Initialization for Low-Thrust Minimum-Fuel Trajectory Optimization", *Acta Astronautica* 211** · `hecht2023pso` · In bib · **full text** (arXiv 2302.04128)
- **What it is:**
  - The swarm minimises a weighted sum of squares of the final boundary-condition residuals of the shooting problem.
  - The swarm's costates then **seed single shooting with homotopy continuation, solved by a trust-region nonlinear
    solver**.
  - They report that when the shooting Jacobian is **rank-deficient**, the Newton steps inside the trust-region solver
    make little progress and fail.
  - The application is low-thrust transfers in the restricted three-body problem.
- **Supports:**
  - K2: **the closest precedent** for "swarm, then a local solve on the necessary conditions".
  - K4: a trust-region Newton-type solver, the same class as MINPACK's Levenberg–Marquardt.
  - K8: rank deficiency stalls the solver, which is the mechanism of our step-5 slide, measured at condition number
    1.1×10⁹ with two near-null directions.
- **Not for:** launch ascent. Cite it as the methodological precedent from another trajectory problem.

### L4 — Launch-vehicle ascent practice

**Lu, Griffin, Dukeman & Chavez 2008, "Rapid Optimal Multiburn Ascent Planning and Guidance", *JGCD* 31(6)** · `lu2008rapid` · In bib · abstract
- **What it is:** an analytical multiple-shooting method for the optimal **exoatmospheric ascent of two burns separated
  by an optimal coast arc**. The authors call the problem highly sensitive, and solve it with a **dogleg trust-region
  method, more robust than Newton–Raphson**.
- **Supports:**
  - K3: the same burn–coast–burn structure with an optimal coast, solved on the necessary conditions.
  - K4: a trust-region Newton-type solver, preferred over plain Newton for exactly this problem.
  - K8: the sensitivity.

  It is **the closest launch-vehicle precedent**, and it carries three claims at once.
- **Not for:** our exact junction-condition form (not confirmed at abstract level; read the paper), or the
  atmospheric Stage 1.

**Gath & Calise 2001, "Optimization of Launch Vehicle Ascent Trajectories with Path Constraints and Coast Arcs", *JGCD* 24(2)** · `gath2001optimization` · In bib · metadata (title)
- **What it is:** ascent optimisation that includes coast arcs **and path constraints**.
- **Supports:**
  - K3: coast arcs in ascent optimisation.
  - K8, **against** the refined trajectory. Real ascent optimisations impose path constraints; ours imposes none on
    dynamic pressure or heating, and the refined point exploits that.
- **Not for:** support of the refined trajectory's realism.

**Brown, Harrold & Johnson 1969, "Rapid Optimization of Multiple-Burn Rocket Flights", NASA CR-1430** · `brown1969rapid` · In bib · metadata
- **What it is:** classical NASA work on optimising rocket flights with several burns.
- **Supports:** K3, the historical lineage of multiple-burn optimisation with coast arcs.
- **Not for:** anything numerical in our pipeline.

**Pallone, Pontani & Teofilatto 2016, "Modeling and Performance Evaluation of Multistage Launch Vehicles through Firework Algorithm", ICATT 2016** · `pallone2016` · In bib · **full text**
- **What it is:**
  - The upper stage, which includes a coast arc, is optimised with the Euler–Lagrange equations and the minimum
    principle.
  - The unknown parameters are found by a firework algorithm, a swarm-type heuristic.
  - It writes the free-final-time transversality condition H(t_f) + ∂Φ/∂t_f = 0 (their Eq. 45).
  - It cites Lu et al. 2008 for the burn–coast–burn case.
- **Supports:**
  - K1 and K3: the costate equations and transversality condition for an upper stage with a coast.
  - K2: the heuristic half, as in Pontani & Conway.
  - K7: the lower stages are treated separately from the costate problem.
- **Not for:** local refinement (none is used).

**Pontani & Teofilatto 2014, "Simple Method for Performance Evaluation of Multistage Rockets", *Acta Astronautica* 94(1)** · New `pontani2014simple` · abstract
- **What it is:** a three-step performance method.
  1. The flight-path angle at each stage separation is **guessed**.
  2. The velocity at the first and second separations is maximised.
  3. The last stage's thrust direction comes from PSO combined with the Euler–Lagrange equations and the minimum
     principle.
- **Supports:** **K7, the closest structural precedent** for keeping a Stage-1 quantity (for us, γ_p) outside the
  upper-stage costate problem and treating it as an outer parameter.
- **Not for:** continuation. Their outer quantity is guessed; ours is swept.

**Pontani 2014, "Particle Swarm Optimization of Ascent Trajectories of Multistage Launch Vehicles", *Acta Astronautica*** · `pontani2014particle` · In bib · secondary
- **What it is:** the indirect-heuristic approach for the whole ascent of a multistage launch vehicle, with thrust
  phases and coast arcs. It handles a **maximum-dynamic-pressure path constraint** in the atmospheric phase.
- **Supports:** K2 and K7, as an ascent example of PSO enforcing the necessary conditions. It also supports K8 in the
  same way as Gath & Calise: the dynamic-pressure constraint that we lack.
- **Not for:** local refinement.

**Pontani 2013, "Ascent Trajectories of Multistage Launch Vehicles: Numerical Optimization with Second-Order Conditions Verification", *ISRN Aerospace Engineering*** · `pontani2013ascent` · In bib · metadata (title)
- **What it is:** ascent optimisation in which the second-order optimality conditions are checked, not just the
  first-order ones.
- **Supports:** K8. It shows what a full optimality claim requires. Our extremals satisfy first-order conditions only,
  and no second-order check was done.
- **Not for:** support of our result's optimality; it is cited to state the gap.

**Pontani & Teofilatto 2019, "Ascent Trajectory Optimization and Neighboring Optimal Guidance of Multistage Launch Vehicles"** (Springer chapter) · `pontani2019ascent` · In bib · metadata (title)
- **What it is:** ascent optimisation together with neighbouring-optimal guidance.
- **Supports:** background for L4 only.
- **Not for:** any specific refinement claim.

**Calise, Melamed & Lee 1998, "Design and Evaluation of a Three-Dimensional Optimal Ascent Guidance Algorithm", *JGCD* 21(6)** · New `calise1998design` · secondary
- **What it is:** an optimal ascent guidance algorithm, hybrid analytic/numerical and solved by collocation. It reaches
  the atmospheric solution by **homotopy starting from the optimal vacuum ascent**.
- **Supports:** K6. Continuation is standard practice in *ascent* optimisation.
- **Not for:** shooting or Levenberg–Marquardt; they use collocation.

**Bonalli, Hérissé & Trélat 2020, "Optimal Control of Endoatmospheric Launch Vehicle Systems: Geometric and Computational Issues", *IEEE TAC* 65(6)** · New `bonalli2020optimal` · abstract
- **What it is:** indirect optimal guidance of endo-atmospheric launch vehicles under mixed control–state constraints.
  A **shooting method is combined with continuation**: an analytical guidance law for simplified dynamics gives the
  initial guess, and continuation brings it to the full dynamics.
- **Supports:**
  - K6: shooting plus continuation for launch vehicles, the most recent ascent precedent.
  - K8: they include constraints, which we don't.
- **Not for:** a γ_p sweep; their continuation deforms the dynamics.

### L5 — The numerical solver

**Levenberg 1944, "A Method for the Solution of Certain Non-Linear Problems in Least Squares", *Q. Appl. Math.* 2(2)** · New `levenberg1944method` · metadata + abstract
- **What it is:** the origin of the damped least-squares step. The linearised (Taylor) correction can overshoot and
  increase the residuals when it is large, so the correction is damped.
- **Supports:** K4, the origin of the damping.
- **Not for:** the modern implementation; cite Moré for that.

**Marquardt 1963, "An Algorithm for Least-Squares Estimation of Nonlinear Parameters", *J. SIAM* 11(2)** · New `marquardt1963algorithm` · metadata + abstract
- **What it is:** the algorithm that interpolates between the Gauss–Newton step (fast near a solution, but can diverge)
  and steepest descent (safe but slow), using a single damping parameter.
- **Supports:** K4, the algorithm itself.

**Moré 1978, "The Levenberg–Marquardt Algorithm: Implementation and Theory", LNM 630** · New `more1978levenberg` · metadata + abstract; **named by SciPy's documentation**
- **What it is:** a robust and efficient implementation of Levenberg–Marquardt with strong convergence properties. It
  is the basis of MINPACK's routines. SciPy's `least_squares` documents `method="lm"` as the MINPACK implementation
  and cites this paper. The installed SciPy (1.17.1) calls MINPACK's `lmder` when a Jacobian function is supplied, as
  ours is.
- **Supports:** K4, **the implementation actually run**. It is also why Levenberg–Marquardt belongs to the same
  trust-region family as the dogleg method of Lu et al. 2008 and the trust-region solver of Hecht & Botta 2023.
- **Not for:** the finite-difference Jacobian; cite Nocedal & Wright for that.

**Nocedal & Wright 2006, *Numerical Optimization*, 2nd ed.** · `nocedal2006numerical` · In bib · metadata (standard textbook content)
- **What it is:** the standard optimisation textbook, with three chapters used here:
  - Ch. 8: finite-difference derivatives, including central versus one-sided differences and choosing the step;
  - Ch. 10: nonlinear least squares, with Levenberg–Marquardt as a trust-region method;
  - Ch. 12: the KKT conditions for constrained optimisation.
- **Supports:** K4 (the Jacobian by central differences, h = 10⁻² in scaled units) and **K5 (the one-sided bound
  condition)**.

**Virtanen et al. 2020, "SciPy 1.0", *Nature Methods*** · `virtanen2020scipy` · In bib
- **Supports:** K4, the software used (`scipy.optimize.least_squares`).

### L6 — Continuation

**Allgower & Georg 2003, *Introduction to Numerical Continuation Methods*** (SIAM) · New `allgower2003introduction` · metadata (standard textbook content)
- **What it is:** the reference text on numerical continuation. It covers predictor–corrector methods, the difference
  between natural-parameter continuation (step the parameter, re-solve) and pseudo-arclength continuation (step along
  the solution curve, which passes turning points), and step-size control.
- **Supports:** K6. Stage B is natural-parameter continuation with a zeroth-order predictor (the previous solution)
  and a fixed step. The text also names the known weaknesses of that simplest form and their standard fixes: a secant
  or tangent predictor, and adaptive steps (K8).

**Keller 1977, "Numerical Solution of Bifurcation and Nonlinear Eigenvalue Problems"** (in Rabinowitz ed., *Applications of Bifurcation Theory*) · New `keller1977numerical` · secondary
- **What it is:** the origin of pseudo-arclength continuation.
- **Supports:** K8. It is the fix that would be needed if the γ_p family had a fold (a turning point).
- **Note:** it is cited to say that it was **not** needed. The half-step test of 2026-09-21 showed the family ends where
  `D3` reaches zero, not at a fold.

**Pan, Lu, Pan & Ma 2016, "Double-Homotopy Method for Solving Optimal Control Problems", *JGCD* 39(8)** · New `pan2016double` · metadata
- **What it is:** a homotopy method for optimal control problems, from the ascent-guidance group of Ping Lu.
- **Supports:** K6, background only.
- **Not for:** anything specific until the paper has been read.

**Pan, Ran, Zhao & Qing 2026, "Review of Homotopy Methods for Aerospace Trajectory Optimization", *Astrodynamics* 10(4)** · New `pan2026review` · abstract
- **What it is:** a recent review of homotopy methods for trajectory optimisation, within both the indirect and direct
  frameworks.
- **Supports:** K6, the current state of the art. It is a good single citation for "continuation is standard practice
  in aerospace trajectory optimisation".

(Trélat 2012, in L2, is also a main K6 reference.)

### L7 — Limits: structure changes and second-order conditions

**Bonnans & Hermant 2008, "Stability and Sensitivity Analysis for Optimal Control Problems with a First-Order State Constraint and Application to Continuation Methods", *ESAIM: COCV* 14(4)** · New `bonnans2008stability` · secondary
- **What it is:** the theory of how the arc structure of a solution changes along a continuation (arcs appearing,
  vanishing or merging), proved for state-constrained problems, and a continuation algorithm that handles the changes.
- **Supports:** K8, **by analogy**. Our two-burn family ends when the last burn's duration reaches zero: an arc
  vanishes. Continuing past that point needs a re-posed problem with one burn fewer, which the refinement does not
  attempt.
- **Not for:** the same situation. Theirs is a state constraint; ours is a thrust arc shrinking to zero.

(Pesch 1994 in L2 and Pontani 2013 in L4 complete this layer.)

---

## 4. The claims no reference carries: internal evidence

These parts of the refinement are this thesis's own choices. They are supported by derivation, tests or measurement,
not by a citation. Present each one as such.

| Item | Evidence | Where |
|---|---|---|
| Duration conditions **without a mass costate** (`H_coast_end = 0`, `H_burn1_end = H_last_burn_start`, `H_burn_end < 0`) | Derived from Bryson & Ho's adjoint sensitivity. The closed forms are **checked against finite differences** by a test. | `Tese/src/tests/test_pmp_transversality.py:104` (`test_closed_form_duration_sensitivities_match_finite_differences`); one-sided coast tests at `:64` and `:71` |
| Costate equations **without the pseudo-force terms** | Measured: the omission is 3×10⁻⁵ of the costate rate in norm, and ∂H/∂s < 5×10⁻⁶ over a 2 000 s coast; tests pin both. The 2026-09-16 search found no source that puts pseudo-forces in the costates. | `Tese/src/tests/test_pmp_stage1_pseudo_forces.py`; CLAUDE.md, decision 7d |
| **γ_p optimised by a sweep** | No optimality condition in γ_p is solved. The result is the best point of the two-burn family, at its end (`D3` → 0), not a stationary point in γ_p. The one-burn family beyond is unexplored. | log `Output/pmp_refine/pmp_swarm_polish_b750half.log` |
| Tolerances (1 m, 1 cm/s, 10⁻⁴°, 0.05 in H) and stopping rules (stop at the first failure; stop 50 kg past the best) | Engineering choices. Re-flying the archive gives 0.14 m and 0.5 mm/s of insertion error. Near the start of the continuation, points within tolerance differ by 36–53 kg. | `pmp_swarm_polish.py:170`, `:344`, `:350`; memory note of 2026-09-21 |
| Conditioning | Scaled-Jacobian condition numbers: 1.4×10⁷ at the starts, 1.1×10⁹ at the slid step-5 point, 1.5×10¹⁰ at the wall. | scratch script `cond_check.py` (2026-09-21) |
| Global versus local | Five seeds; seed 4 lands on a worse branch, so several extremals exist. The result is the best of five local extremals. | `Output/pmp_refine/*b750*.json` |

---

## 5. What may and may not be claimed

**May be claimed** (each with its citations):
- The refinement uses established components in a combination specific to this work: a swarm initial guess (Hecht &
  Botta; Jiang et al.; Pontani & Conway), a trust-region Levenberg–Marquardt solve of the necessary conditions
  (Levenberg; Marquardt; Moré; SciPy), and parameter continuation (Allgower & Georg; Trélat; Pan et al. 2026).
- The burn–coast–burn structure with an optimal coast, solved on the necessary conditions by a trust-region
  Newton-type method, follows Lu et al. 2008.
- Keeping a Stage-1 quantity outside the upper-stage costate problem follows Pontani & Teofilatto 2014 and Pallone et
  al. 2016.
- Continuation is standard in ascent optimisation (Calise et al. 1998; Bonalli et al. 2020).
- The results are first-order extremals of the two-burn problem as modelled, the best of five seeds, lying where the
  last burn reaches zero.

**May not be claimed:**
- "The optimal ascent", "the PMP optimum", or global optimality. There is no second-order check (Pontani 2013 shows
  what one needs) and there are several branches.
- That any cited author used this exact pipeline.
- That the refined trajectory is flyable. It stages at 63 km with 4.2 kPa of dynamic pressure in a drag-free Stage 2
  (about 12 kg of uncharged drag), has about 13 times gt's heating, and has no dynamic-pressure or heating constraint,
  which Gath & Calise, Pontani 2014 and Bonalli et al. include. It also exploits the unprojected rotation credit. These
  are Chapter 6 disclosures.

---

## 6. The two claims the 19b handoff flagged

- **§4.4, `pontani2014indirectswarming`.** From the abstract (the full text is paywalled), it is PSO combined with the
  analytical necessary conditions, needing no starting guess, and it shows **no local refinement**. Cite it for the
  swarm half only. Anything more specific attributed to it still needs the full text.
- **§5.4, attributing the spherical-angle parameterisation.** Attribute the *normalisation* of the costates onto a unit
  hypersphere to Jiang, Baoyin & Li 2012 (checked from secondary sources). I have not confirmed that they parameterise
  it with angles. Present the two angles `(a, b)` as this work's coordinates on that sphere, unless their full text
  shows otherwise.

---

## 7. Suggested wording (draft; for when thesis edits resume)

> The swarm's best point is not an extremal. It is refined by solving the first-order necessary conditions of the
> burn–coast–burn problem [bryson1975applied; lu2008rapid]: the terminal-state constraints and the stationarity of the
> burn and coast durations, which Appendix X derives without a mass costate. The unknowns are the costate direction on
> the unit sphere [jiang2012practical] and the three arc durations, and the square system is solved by the
> trust-region Levenberg–Marquardt method [levenberg1944method; marquardt1963algorithm; more1978levenberg] as
> implemented in SciPy [virtanen2020scipy]. Using a swarm to initialise a local solution of the necessary conditions
> follows [hecht2023pso; jiang2012practical]; the swarm itself is an indirect swarming method
> [pontani2014indirectswarming]. The kick parameter γ_p, which no costate condition governs, is treated as an outer
> parameter [pontani2014simple] and advanced by natural-parameter continuation, each solve warm-started from the
> previous extremal [allgower2003introduction; trelat2012optimal]. Continuation is standard in ascent optimisation
> [calise1998design; bonalli2020optimal]. The resulting points are first-order extremals of the two-burn problem as
> modelled. No second-order check was made [pontani2013ascent], and the family ends where the last burn vanishes, a
> change of arc structure [bonnans2008stability], beyond which a one-burn problem would have to be posed.

---

## Appendix A — Reading priority before writing

1. **Lu et al. 2008:** confirm how they write the coast-arc junction conditions (K3).
2. **Jiang, Baoyin & Li 2012:** confirm the normalisation and whether angles are used (§6).
3. **Pontani & Conway 2014:** confirm the §4.4 claim (§6).
4. **Betts 1998:** the exact wording on the sensitivity of shooting (K8).
5. **Bonnans & Hermant 2008:** confirm the structure-change analogy is fair (K8).
6. **Pan et al. 2016:** read before citing it for anything.

## Appendix B — BibTeX for the new entries

```bibtex
@book{bryson1975applied,
  author    = {Bryson, Arthur E., Jr. and Ho, Yu-Chi},
  title     = {Applied Optimal Control: Optimization, Estimation, and Control},
  publisher = {Hemisphere}, address = {Washington, DC}, year = {1975},
  note      = {Reprinted by Routledge, 2018, doi:10.1201/9781315137667}
}
@book{longuski2014optimal,
  author    = {Longuski, James M. and Guzm{\'a}n, Jos{\'e} J. and Prussing, John E.},
  title     = {Optimal Control with Aerospace Applications},
  publisher = {Springer}, address = {New York}, year = {2014},
  series    = {Space Technology Library},
  doi       = {10.1007/978-1-4614-8945-0}
}
@article{betts1998survey,
  author  = {Betts, John T.},
  title   = {Survey of Numerical Methods for Trajectory Optimization},
  journal = {Journal of Guidance, Control, and Dynamics},
  volume  = {21}, number = {2}, pages = {193--207}, year = {1998},
  doi     = {10.2514/2.4231}
}
@incollection{colasurdo2012indirect,
  author    = {Colasurdo, Guido and Casalino, Lorenzo},
  title     = {Indirect Methods for the Optimization of Spacecraft Trajectories},
  booktitle = {Modeling and Optimization in Space Engineering},
  series    = {Springer Optimization and Its Applications},
  publisher = {Springer}, address = {New York}, year = {2012},
  pages     = {141--158},
  doi       = {10.1007/978-1-4614-4469-5_6}
}
@book{stoer2002introduction,
  author    = {Stoer, Josef and Bulirsch, Roland},
  title     = {Introduction to Numerical Analysis},
  edition   = {3rd},
  publisher = {Springer}, address = {New York}, year = {2002},
  series    = {Texts in Applied Mathematics},
  doi       = {10.1007/978-0-387-21738-3}
}
@article{pesch1994practical,
  author  = {Pesch, Hans Josef},
  title   = {A Practical Guide to the Solution of Real-Life Optimal Control Problems},
  journal = {Control and Cybernetics},
  volume  = {23}, number = {1/2}, pages = {7--60}, year = {1994}
}
@article{trelat2012optimal,
  author  = {Tr{\'e}lat, Emmanuel},
  title   = {Optimal Control and Applications to Aerospace: Some Results and Challenges},
  journal = {Journal of Optimization Theory and Applications},
  volume  = {154}, number = {3}, pages = {713--758}, year = {2012},
  doi     = {10.1007/s10957-012-0050-5}
}
@article{jiang2012practical,
  author  = {Jiang, Fanghua and Baoyin, Hexi and Li, Junfeng},
  title   = {Practical Techniques for Low-Thrust Trajectory Optimization with Homotopic Approach},
  journal = {Journal of Guidance, Control, and Dynamics},
  volume  = {35}, number = {1}, pages = {245--258}, year = {2012},
  doi     = {10.2514/1.52476}
}
@article{pontani2014simple,
  author  = {Pontani, Mauro and Teofilatto, Paolo},
  title   = {Simple Method for Performance Evaluation of Multistage Rockets},
  journal = {Acta Astronautica},
  volume  = {94}, number = {1}, pages = {434--445}, year = {2014},
  doi     = {10.1016/j.actaastro.2013.01.013}
}
@article{calise1998design,
  author  = {Calise, Anthony J. and Melamed, Nahum and Lee, Seungjae},
  title   = {Design and Evaluation of a Three-Dimensional Optimal Ascent Guidance Algorithm},
  journal = {Journal of Guidance, Control, and Dynamics},
  volume  = {21}, number = {6}, pages = {867--875}, year = {1998},
  doi     = {10.2514/2.4350}
}
@article{bonalli2020optimal,
  author  = {Bonalli, Riccardo and H{\'e}riss{\'e}, Bruno and Tr{\'e}lat, Emmanuel},
  title   = {Optimal Control of Endoatmospheric Launch Vehicle Systems: Geometric and Computational Issues},
  journal = {IEEE Transactions on Automatic Control},
  volume  = {65}, number = {6}, pages = {2418--2433}, year = {2020},
  doi     = {10.1109/TAC.2019.2929099}
}
@article{levenberg1944method,
  author  = {Levenberg, Kenneth},
  title   = {A Method for the Solution of Certain Non-Linear Problems in Least Squares},
  journal = {Quarterly of Applied Mathematics},
  volume  = {2}, number = {2}, pages = {164--168}, year = {1944},
  doi     = {10.1090/qam/10666}
}
@article{marquardt1963algorithm,
  author  = {Marquardt, Donald W.},
  title   = {An Algorithm for Least-Squares Estimation of Nonlinear Parameters},
  journal = {Journal of the Society for Industrial and Applied Mathematics},
  volume  = {11}, number = {2}, pages = {431--441}, year = {1963},
  doi     = {10.1137/0111030}
}
@incollection{more1978levenberg,
  author    = {Mor{\'e}, Jorge J.},
  title     = {The {L}evenberg--{M}arquardt Algorithm: Implementation and Theory},
  booktitle = {Numerical Analysis},
  editor    = {Watson, G. A.},
  series    = {Lecture Notes in Mathematics}, volume = {630},
  publisher = {Springer}, address = {Berlin, Heidelberg}, year = {1978},
  pages     = {105--116},
  doi       = {10.1007/BFb0067700}
}
@book{allgower2003introduction,
  author    = {Allgower, Eugene L. and Georg, Kurt},
  title     = {Introduction to Numerical Continuation Methods},
  publisher = {Society for Industrial and Applied Mathematics}, address = {Philadelphia}, year = {2003},
  series    = {Classics in Applied Mathematics},
  doi       = {10.1137/1.9780898719154}
}
@incollection{keller1977numerical,
  author    = {Keller, Herbert B.},
  title     = {Numerical Solution of Bifurcation and Nonlinear Eigenvalue Problems},
  booktitle = {Applications of Bifurcation Theory},
  editor    = {Rabinowitz, Paul H.},
  publisher = {Academic Press}, address = {New York}, year = {1977},
  pages     = {359--384}
}
@article{pan2016double,
  author  = {Pan, Binfeng and Lu, Ping and Pan, Xun and Ma, Yangyang},
  title   = {Double-Homotopy Method for Solving Optimal Control Problems},
  journal = {Journal of Guidance, Control, and Dynamics},
  volume  = {39}, number = {8}, pages = {1706--1720}, year = {2016},
  doi     = {10.2514/1.G001553}
}
@article{pan2026review,
  author  = {Pan, Binfeng and Ran, Yunting and Zhao, Mengxin and Qing, Wenjie},
  title   = {Review of Homotopy Methods for Aerospace Trajectory Optimization},
  journal = {Astrodynamics},
  volume  = {10}, number = {4}, pages = {507--536}, year = {2026},
  doi     = {10.1007/s42064-026-0315-7}
}
@article{bonnans2008stability,
  author  = {Bonnans, J. Fr{\'e}d{\'e}ric and Hermant, Audrey},
  title   = {Stability and Sensitivity Analysis for Optimal Control Problems with a First-Order State Constraint and Application to Continuation Methods},
  journal = {ESAIM: Control, Optimisation and Calculus of Variations},
  volume  = {14}, number = {4}, pages = {825--863}, year = {2008},
  doi     = {10.1051/cocv:2008016}
}
```

## Appendix C — Sources consulted (2026-09-21/22)

- Crossref metadata API for every DOI in Appendix B.
- SciPy `least_squares` documentation; the installed SciPy 1.17.1 source (`_lsq/least_squares.py`, `call_minpack`).
- Full text:
  - arXiv 2302.04128 (Hecht & Botta);
  - the ICATT 2016 PDF on ESA Indico (Pallone et al.).
- Publisher abstracts (Springer, AIAA, ScienceDirect, IEEE): Pontani & Conway 2014/2015, Lu et al. 2008, Pontani &
  Teofilatto 2014, Trélat 2012, Bonalli et al. 2020, Pan et al. 2026, Conway 2012, Moré 1978, Levenberg 1944,
  Marquardt 1963.
- Search summaries for the entries tagged "secondary".
