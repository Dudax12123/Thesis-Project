# Guidance-law implementation audit — 2026-09-10

Scope: every law the results matrix flies, checked against Chapter 4 of the thesis
(`Thesis_Guidance.tex`), Chapter 3 §3.x for the indirect formulation, and the sources the
code itself cites (Orbiter-wiki PEG for `peg`, Shuttle-PEG integrals for `peg_new`).
Behaviour was verified on the archived (stale, pre-MECO-fix) batch in
`Tese/src/Output/results_matrix/` — the MECO fix does not change how any law steers.
Analysis script: scratchpad `alpha_audit.py` (session-local).

Stale-batch propellant remaining, for context [t]: gt 20.29 · linear 20.34 ·
bilinear 21.31 · cpr 18.52 · exp 17.32 · peg_new 18.79 · apollo 13.72 · peg 14.13 · pmp 20.12.

## A. Defects that change the flown α (and therefore propellant)

**A1 — `peg` (classical scalar PEG, `show_peg`) drops the gravity term of its own
reference.** The Orbiter-wiki algorithm the module cites solves the guide step with
gravity *neglected* in the radial channel and restores it in the steering:
`sin(pitch) = A + B·t + C`, `C = (μ/r² − ω²r)/a₀`. `peg_alpha` flies `A + B·t`.
`estimate_peg_T` also deviates: `f_r = A·(1+C)` (wiki `A + C`), `C` at `r̄` with `ω`
at the current point, Δv numerator missing the `f̈_θ v_e T²/2` term and using `T`
where the wiki has `τ`. At the archived ignition state (h 136 km, γ 49°, T/m 9.65):
`converge_peg` → A = −0.739 → pitch −47.7°, **α = −97°** (thrust has a rearward
component); with C = 0.913 restored: pitch +10°, α = −39°. Gravity left out of the
radial prediction over T = 326 s: 469 km, against a 364 km altitude gap. Archived α:
arc-1 rms 39°, |α| > 15° for 55 % of the arc; arc-3 −70…+56°. Chapter 4 §"Classical
Scalar Variant" describes the code as written (α = arcsin(A + Bτ) − γ), i.e. the
thesis also omits C.

**A2 — `apollo` subtracts a gravity vector built in the wrong frame.** Positions and
velocities are local curvilinear (x = s, y = h, vx = v cos γ, vy = v sin γ), but
`apollo_guidance` rotates gravity by the central angle s/R
(`ax_g = g sin(s/R)`, `ay_g = −g cos(s/R)`) as if x,y were launch-fixed Cartesian.
Local-frame kinematics give `ax = −vx·vy/r`, `ay = −g + vx²/r`. Along the archived
trajectory the error reaches Δax = +1.9, Δay = −7.3 m/s² at cutoff (s/R = 12.7°,
v ≈ 7.2 km/s): the law asks thrust to carry 7.3 m/s² the orbital motion already
supplies (≈ 8° of pitch). Chapter 4 says "subtracting the local gravity vector" —
the code does not do that.

**A3 — `apollo` commands an acceleration magnitude the vehicle cannot produce, and only
the direction is used.** At the archived ignition state: t_go(rocket eq) = 283 s,
k2 = 18.6 m/s² horizontal, k4 = −5.5 m/s² vertical → |a_cmd| = 18.9 m/s² vs 9.7
available (×1.95); at arc-3 start ×1.81. `APOLLO_THRUST_MAGNITUDE_CONTROL = False`
and the dispatcher discards the magnitude. The polynomial's terminal guarantees do not
hold for a fixed-thrust vehicle; the law flies α ≈ −30° for the whole first arc
(100 % of arc 1 beyond 15°) and swings −41…+70° near cutoff. Together with A2 this
accounts for most of Apollo's 6.6 t deficit against the passive gravity turn.

**A4 — `linear_tangent` is flown open-loop, contrary to the chapter — and the
chapter's closed-loop version degenerates to a gravity turn.** Manifest:
`GUIDANCE_COEFFICIENTS_FIXED = True` (config default; `BASELINE` does not override it).
Chapter 4: coefficients "re-derive[d] from the current state at every guidance
update". `compute_lts_coefficients` sets a = tan γ / t_go, b = 0, so a refresh gives
α = arctan(a·t_go) − γ = 0 identically (the bilinear module's "CRITICAL: don't just use
current gamma, that causes α = 0 throughout" comment is about exactly this). As flown
(fixed): tan θ = tan γ₀ · t_go(t)/t_go₀ — a countdown-driven pitch programme with no
altitude or velocity targeting (`target_altitude` is unused). Archived α: −4…+1°,
rms 1.6°; propellant 20.34 t vs 20.29 t for the gravity turn.

**A5 — Closed-loop laws target the *final* orbit from the *first* burn arc; the coast
then discards that plan.** `peg_new`, `peg`, `apollo` (and the t_go the tangent laws
use) aim at (500 km, v_circ, γ = 0) at their own t_go from ignition — direct
insertion — while the swarm inserts a 100–540 s coast. peg_new at ignition commands
pitch 0° (α = −34°) to bend a 34° state into a single-burn insertion; the coast then
throws that away and arc 3 re-targets. Stale batch: peg_new 18.79 t vs gravity turn
20.29 t — the "closed-loop ceiling" sits 1.5 t below the passive floor under the same
swarm. Chapter 4 says this only of exp_shooting ("assumes a single continuous burn,
which a thrust–coast–thrust split contradicts"); it applies equally to PEG/Apollo.
Not a coding error — an architecture/law mismatch that decides §6.3's headline.

## B. Modelling gaps inside the laws (small, shared)

- **B1** Internal gravity models ignore the rotating-frame terms the trajectory flies
  with (peg_new `g_r = −μ/r² + v_θ²/r` with ground-relative v_θ leaves ≈ −0.95 m/s² at
  the target that Coriolis actually supplies; Apollo/peg likewise). Closed loops absorb
  it; freeze-window error ≈ 45 m. Same blind spot in every law, so fair.
- **B2** peg_new's tangential channel `v_go_θ = v_θT − v_θ` omits
  dv_θ/dt = −v_r v_θ / r (≈ −0.7 m/s² at ignition, 100–150 m/s over the burn):
  L₀ and t_go low by 2–3 %. Closed loop absorbs.
- **B3** Rocket-equation t_go is gravity-blind (documented); laws' t_go and the swarm's
  cutoff coincide only in the limit. Documented.

## C. Fairness / consistency

- **C1** α limiting differs: bilinear clips ±15°; Apollo and linear clips are
  commented out; `peg` clips sin(pitch); peg_new normalises. Bilinear never hit its clip
  in the archive (max 11.3°).
- **C2** Update cadence differs: LT/BT once per arc, Apollo 2 s, peg/peg_new 2 s major
  loop, CPR/exp none; freeze at t_go < 10 s only for apollo/peg/peg_new. Conventional.
- **C3** Chapter: exo laws activate on the atmosphere-exit criterion; PSO dispatcher:
  at Stage-2 ignition. Identical in the matrix (MECO 99–137 km > 65 km). Doc only.
- **C4** Guidance state (coefficient refresh, freeze flags, epochs) is mutated inside the
  ODE RHS at speculative times — same hazard class as the fairing/MECO latches. Harness
  already suspects it for `peg` under the planned t_go (J′ 0.888 vs 7.679 on replay).
  Under the flown settings replay reconciles; nothing guarantees it.

## D. Thesis ↔ code mismatches

- D1 Bilinear terminal derivative: thesis "d tan/dτ|₀ = −0.02 s⁻¹"; code sets
  d/dt = −0.02, i.e. d/dτ = +0.02 (c₁ = +0.02).
- D2 Linear tangent closed-loop claim (A4).
- D3 Apollo "local gravity vector" (A2).
- D4 Classical PEG described without its C term (A1).
- D5 `pmp_control_law` multiplies by the norm where its docstring and Eq. (indirect_control)
  divide — harmless under atan2, but the code does not match its own documentation.
- D6 Thesis: ‖λ‖ = 1 gauge fixed "at the pitch-over point"; code normalises at Stage-2
  ignition.
- D7 `H_f^last < 0` (thesis) is not enforced; only |residual| is penalised.

## E. Verified correct

PMP costate equations (30b–d) re-derived from H by hand — all three match; control law
direction ✓; transversality expression and J′ normalisation match Ch. 3. peg_new thrust
integrals map exactly to Shuttle-PEG (S₀ = c₀, L₁ = b₁, S₁ = c₁), radial λ̇ channel
= (r_go − Sλ)/(Q − S t_λ) ✓, free-downrange form is the linear-tangent law Ch. 4 calls
c₁' = 0. `peg` guide step matches the wiki. Rotating-frame targets are one value across
objective / apollo / peg / peg_new / t_go (the known, kept, unprojected credit). CPR and
exp_shooting match the chapter (PSO-supplied θ̇ and (a, b), re-epoched per arc, bounds as
stated). Bilinear coefficient algebra checks (f(t_go) = tan_initial, f(0) = 0). Kick
bounds identical across all architectures. α diagnostic channel is sorted before
interpolation (`prepare_monotonic_series`), so speculative RHS logs do not corrupt it.

---

# Decisions and fixes — same day

User's decisions (2026-09-10): fix A1 (classical PEG) and A2/A3 (Apollo); check whether A4's
α ≡ 0 collapse and the large steering angles at guidance start are known in the literature;
A5 (arc-1 direct-insertion targeting) is to be **documented as a possible error source, not
fixed**.

## Fix 1 — classical `peg` (`Guidance/peg_guidance.py`, both dispatchers)

- `peg_alpha(t, A, B, gamma, C)` now flies `sin(pitch) = A + B·t + C`; `C` is a required
  argument, supplied by `compute_gravity_term(state, F_T, mu)` at the current state on every
  minor-loop call (reference: "f_r = A + C, sin(pitch) at current time").
- `estimate_peg_T` is now term-for-term the Orbiter-wiki estimate step: `C` at the current
  point, `f_r = A + C`, `C_T` at cutoff with `a[T] = v_e/(τ − T)`, and the Δv numerator's
  `f̈_θ v_e T²/2` term restored (with `τ`, not `T`, in the bracket). `f_r` and `f_{r,T}` are
  clipped to ±1 as the minor loop clips them.
- Effect at the archived show_peg ignition (h 136 km, γ 49.3°, T/m 9.65): pitch −47.7° → +17.1°,
  α₀ −97° → −32°, converged T 326 → 284 s; damped and undamped iterations now agree (4 and 9
  iterations). Also checked at the peg_baseline state (γ 34.4°): pitch +71°, α₀ +36°.
- **Validity limit found, reported not patched.** At this vehicle's Stage-2 ignition
  `C = 0.85–0.91`; for the shallower kicks the guide step returns `A + C = 1.2–1.6` (the
  stage cannot hold altitude even thrusting vertically). There the reference's small-pitch
  expansion `f_θ = 1 − f_r²/2` reads 0.5 where cos 90° = 0, the denominator can turn negative,
  and `T ↦ T_est(T)` is non-monotonic (gt_baseline state: T_est(275) = 294, T_est(284) = 266),
  so neither damping setting has a unique fixed point. This is the classical algorithm on a
  T/W ≈ 1 stage — the Orbiter wiki itself warns that when `f̂·r̂` "goes out of range" the
  linearised estimator "will come up with an answer... well off of what is needed". Comment in
  `estimate_peg_T` records the numbers.

## Fix 2 — `apollo` (`Guidance/apollo_guidance.py`, both dispatchers)

- New `local_frame_accelerations(state)` returns the non-thrust accelerations of
  `(v cos γ, v sin γ)`: `−g + v_x²/r` vertically, `−v_x v_y/r` horizontally. Tested equal to
  `diff_eom_base` with the engine off at three states (`tests/test_peg_apollo.py`).
- `apollo_guidance(..., a_thrust_available=F_T/m)` resolves the magnitude constraint the way
  Luminary P12 does: vertical channel first, `a_x = sqrt(a_T² − a_y²)` with the sign of the
  horizontal demand, `a_y` clipped to ±`a_T`. Luminary `ASCENT_GUIDANCE.agc` (Apollo 11):
  radial `ATR = (A + B(T−T0))/TBUP`, downrange `ATP = sqrt(AT² − AH²)`, and "IF ATP3 NEG, GO TO
  NO-ATP" scales the perpendicular components when they exceed AT; `TGO = TBUP·VG·(1 − KT·VG/VE)/VE`.
  The direction-only path is kept when the argument is omitted (fallback only).
- Effect at the archived show_apollo ignition: demanded 19.6 m/s² vs 9.7 available; α₀ −37°
  (old) → −30° (P12). Sanity: on the target circular orbit with zero polynomial command the
  law now asks for horizontal thrust (old: +g upward).
- Both fixes still command large |α| at ignition for shallow-kick states (Apollo +64°, PEG
  +64° at the gt_baseline state): both laws then want the thrust vertical because the direct
  insertion they plan needs more radial acceleration than a T/W ≈ 1 stage has. That is the
  laws' honest answer for those states, see the bibliography note below.

Tests: 88 pass (`tests/test_peg_apollo.py` added: 12 tests). Check runs at 60×120 in
`Output/audit_fix/{peg,apollo}_v1` — see the end of this note for the numbers.

## Bibliography check 1 — is the linear-tangent α ≡ 0 collapse known?

No. In every source the constants of `tan θ = a·t_go + b` come from the terminal boundary
conditions of the optimal-control problem, never from the current flight-path angle:

- Perkins 1966, *Derivation of linear-tangent steering laws* (AD0643209): derived by "the
  classical Lagrange technique", applicable "above the drag sensible atmosphere", "the precise
  mathematical optimum" for thrust-only position and velocity changes.
- The thesis's own Chapter 4 (Ulrich, Edberg): "the constants are determined by enforcing the
  prescribed initial and terminal boundary conditions"; "a separate numerical optimization…
  determines the optimal γ(t) and t_f; the linear or bilinear tangent law then provides the
  steering history α(t) consistent with that optimal trajectory".
- Shuttle UPFG (PEGAS docs): `pitch = atan(A·t + B)` with A, B from the guidance solution.
- Federici/Zavoli/Colasurdo (arXiv 1910.03268): `tan θ = (C₂t + C₃)/(C₁t + 1)` with the
  coefficients as optimisation variables; `C₁ = 0` gives the linear law.

Matching `tan θ = tan γ_now` at `t_go` is this implementation's own boundary condition. With
the terminal condition `θ = 0` it has no targeting content (altitude and speed never enter), and
under refresh it returns α = 0 by construction. So the collapse is not a property of the law;
it is a consequence of that choice. As flown (`GUIDANCE_COEFFICIENTS_FIXED = True`) the
coefficients are frozen at ignition and the law is a countdown pitch programme — legitimate,
but Chapter 4 says the opposite and should be corrected either way.

## Bibliography check 2 — are large steering angles at guidance start known?

Partly, and the literature treats them as something to be prevented, not flown:

- Flight guidance never hands a raw explicit-guidance command to the autopilot at initiation.
  Shuttle PEG/UPFG runs in a *prethrust* mode until converged: "when the algorithm is first
  called, the error between predicted and desired states can be very large and thus the
  resulting steering constants are not very reliable… [only after convergence] the vehicle
  can safely start following the calculated guidance" (PEGAS/UPFG docs).
- Saturn V freezes attitude around the hand-over: "At T2−11 (or chi-freeze gate), the IGM's
  chi-tilde logic is frozen briefly to allow for staging"; "the LVDC freezes the IGM's attitude
  to allow for a safe staging" (NASSP LVDC page). The IGM turning rate is also limit-tested
  (Saturn V guidance equations; number not retrieved).
- The Orbiter-wiki PEG notes the estimator is linearised and "will come up with an answer,
  but the answer will be well off" when `f̂·r̂` leaves its range — which is the `A + C > 1`
  regime above.

More important than any of that: a real first stage's pitch programme is *designed* so that
the explicit law engages near its own optimal path, with a small transient. Here Stage 1 is a
gravity turn from a kick bounded to `γ_p ∈ [85.9°, 89.95°]`, which hands Stage 2 a state at
99–137 km with γ = 26–50° and T/W ≈ 1. Every direct-insertion law then either wants the thrust
near-horizontal (excess vertical velocity, steep kicks) or straight up (altitude gap, shallow
kicks). The −30…+64° openings are the laws' correct answer to a hand-over state the bibliography
would not present them with. No source rate-limits α inside the law; simulations that care
either limit the attitude rate in the control loop or design the hand-over. Neither exists here,
and α is unlimited except in bilinear (±15°). Recommendation for the thesis: state this as a
limitation of the fixed Stage-1/kick design, and optionally add an attitude-rate limit as a
common (fair) control-loop constraint — not as part of any law.

## A5 — documented as a source of error, not fixed

Terminology first, because it matters: Stage 1 *is* a gravity turn in every non-segmented
architecture (α = 0 after the kick). The mismatch is in **Stage 2's pre-coast burn**
("Arc 1" in `run_pso_coast_trajectory`, ignition → coast start): under `pso_coast` the active
law steers that arc — the archives show peg_new opening at α = −34°, Apollo at −37°, arc-1 α
rms 14–39° — and every law aims it at the final orbit as a direct insertion, which the swarm's
coast then discards. Recorded as a comment above Arc 1 in `pso_coast_solver.py` and in
`CLAUDE.md`. Suggested thesis wording (Chapter 4, after the exp_shooting remark, or Chapter 6
limitations):

> Under the coast-parameter architecture the closed-loop laws steer the pre-coast burn as
> well as the insertion burn, and in both they aim at the final orbit: their internal
> prediction is a single continuous burn to the target, since none of them carries a notion
> of the coast that the optimiser inserts afterwards. The pre-coast command is therefore
> computed for a trajectory that is never flown, and is re-planned from scratch once the
> second burn begins. This is a known source of error in the comparison of Sections 6.3 and
> 6.7: a closed-loop law can be out-performed by the passive gravity turn under the same
> optimiser not because its steering is worse, but because the architecture asks it to plan
> the wrong trajectory for a third of the powered flight. The remark already made for the
> exponential law applies to every law of this chapter except the gravity turn; only the
> segmented mode of Section 4.x gives that arc a target of its own.

## Check runs with the fixed laws (60×120, seed 42, `Output/audit_fix/{peg,apollo}_v1`)

Not production numbers — the stale batch is 100×250 — but enough to see what the corrected
laws do. Both insert cleanly; neither becomes economical.

| case | budget | J′ | prop. left | e | h_p [km] | Stage-2 structure | α at ignition | arc-1 α rms |
|---|---|---|---|---|---|---|---|---|
| show_peg, pre-fix | 100×250 | 0.879 | 14.13 t | 3.9e-4 | 500.0 | 194 s burn, 112 s coast, 94 s burn | −97° (rearward) | 39° |
| show_peg, fixed | 60×120 | 0.896 | 12.52 t | 2.0e-4 | 496.8 | single 293 s burn, no coast | −69° (pitch −6°) | 39° |
| show_apollo, pre-fix | 100×250 | 0.971 | 13.72 t | 1.9e-4 | 497.8 | 112 s, 104 s coast, 176 s | −37° | 30° |
| show_apollo, fixed | 60×120 | 0.960 | 10.61 t | 7e-4 | 500.1 | 281 s burn, 19 s burn, no coast | +64° (pitch 90°) | 46° |

What the corrected laws do, and why it is the algorithm and not the code:

- **peg**: the swarm now pushes the kick to the steep bound (γ = 63° at ignition, h = 147 km,
  ballistic apogee already 531 km) so that the direct-insertion law can burn near-horizontal
  and even slightly *below* the horizon (pitch −6° at ignition: it must push the vehicle
  down to arrive at 500 km with ṙ = 0). α decays smoothly −69° → −6° over the burn — the
  linear-sine programme behaving exactly as the source describes, with no coast because the
  law never planned one. The cost is a lofted Stage 1 and a long gravity-loss-heavy burn.
- **apollo**: from a 26° state at 98 km the vertical polynomial demands 14.6 m/s² of total
  vertical acceleration against 9.7 available, so the P12 rule points the thrust straight up
  (α = +64°) for ~100 s, then the law swings to −35…−57° to kill the vertical velocity it
  built. That is the Luminary "NO-ATP" branch (perpendicular channels take the whole thrust)
  applied to a stage the Apollo programme never had to fly: the LM ascent stage had T/W ≈ 3
  in lunar gravity; this stage has ≈ 1.1 at ignition.
- Both are direct-insertion formulations derived for T/W comfortably above 1 (Saturn upper
  stages, LM ascent). On this vehicle `C = (g − ω²r)/a₀ ≈ 0.85–0.91` at ignition, the
  classical PEG estimate is outside its expansion (`A + C > 1`, see Fix 1), and Apollo's
  vertical channel saturates. The passive gravity turn with an optimised coast (a two-burn
  transfer) is simply the cheaper way to 500 km for this stage, which is now the result to
  report rather than an artefact to fix. Pre-fix, both laws happened to fly less absurd α
  only because their gravity handling was wrong in a direction that mimicked a shallower plan.
- The two remaining large-α openings are the hand-over problem of Bibliography check 2, not
  a residual defect: the laws are engaged on a state their sources would never hand them.

---

# Fix 3 — tangent laws open-loop under pso_coast (2026-09-11)

User decision (2026-09-11): adopt the open-loop tangent laws with swarm-chosen constants.
Report updates on hold.

**Form.** With `σ = (t − t0)/(tf − t0)`, `t0` the first Stage-2 ignition and `tf` the planned
final cutoff (coast included), and `θ = α + γ` from the local horizontal:

- linear: `tan θ = (1 − σ) tan θ0 + σ tan θf`
- bilinear: `tan θ = tan θ0 + (tan θf − tan θ0)·(1 + k)σ/(1 + kσ)`, `k = (2μ − 1)/(1 − μ)`

`μ` is the fraction of the change in `tan θ` completed at mid-span; `μ = 0.5` gives `k = 0`, the
linear law, nested at the centre of the bounds. Both are exactly Chapter 4's `tan(α+γ) = a·t_go + b`
and `(c1 t_go + c2)/(c1' t_go + c2')` with `t_go = tf − t` (`open_loop_coefficients` in each module
converts; tested). Decision vector: linear `+[θ0, θf]` (6 vars), bilinear `+[θ0, θf, μ]` (7).
Bounds `θ0 ∈ [−20°, 70°]`, `θf ∈ [−40°, 30°]`, `μ ∈ [0.1, 0.9]`, set from the archived pitch
ranges (ignition +0.2…+26.3°, steep kicks to ~63°; cutoff −14.9…+6.9°) with ~20° margin.

**Bibliographic basis.** Constants from the terminal conditions of the flat-Earth problem, never
from the current γ (Perkins 1966; Bryson & Ho; Chapter 4 via Ulrich/Edberg: "a separate numerical
optimization … determines the optimal γ(t) and t_f"); Federici, Zavoli & Colasurdo (arXiv
1910.03268) fly the bilinear law with its three constants expressed through initial/final angles
and a curvature measure, chosen by the optimiser — the same family as here.

**Two conventions, measured before adoption.** Least-squares fits of the archived steering
(`tan θ` against `σ` over the Stage-2 burn samples), rms error in degrees:

| case | frame | time | linear | bilinear |
|---|---|---|---|---|
| pmp_baseline | fixed @ ignition | absolute | 0.69 | 0.17 |
| pmp_baseline | fixed @ ignition | powered only | 8.14 | 5.09 |
| pmp_baseline | local | absolute | 0.84 | **0.03** |
| pmp_baseline | local | powered only | 4.43 | 3.01 |
| gt_baseline | local | absolute | 2.79 | 0.17 |
| gt_baseline | local | powered only | 1.56 | 1.60 |

- *Time continuous through the coast.* In the flat-Earth derivation the costates propagate
  through a coast exactly as through a burn (`λ̇_vy = −λ_y` in both), so `tan θ` is linear in
  absolute time. The indirect optimum agrees: absolute time fits it to 0.03–0.84°, re-epoching
  per arc (what `exp_shooting` does) to 3–8°. `GuidanceState.tan_t0/tan_tf` are set by the
  trajectory runner and survive `restart_for_new_burn`.
- *Frame.* The derivation measures θ from a fixed horizontal; on the PMP optimum the local and
  fixed frames fit comparably, the local one better for bilinear. Chapter 4's frame (local) kept.
- In the local frame the bilinear law with absolute time reproduces the indirect-PMP steering to
  0.03° rms: the law can represent the optimum, so any gap the swarm leaves is convergence or
  the arc-1 structure, not the law's shape.

**Scope.** `pso_coast` only, as for `cpr`/`exp_shooting`. `apogee_check`, `direct` and segmented
supply no constants and keep the closed-loop form (a test pins its α = 0-at-engagement property).
`GUIDANCE_COEFFICIENTS_FIXED`, `GUIDANCE_UPDATE_RATE`, `TGO_ESTIMATOR` and
`GUIDANCE_TGO_USE_PSO_PLAN` no longer reach the tangent laws under `pso_coast`
(`worktree.md` updated).

**Verification.** 25 new tests (`tests/test_tangent_open_loop.py`; suite 113). Smoke flight per
law: replay objective equals the swarm's fitness to 1e-13; the first command after the coast
equals the law at absolute σ (not re-epoched).

**Flagged, not done.** The same evidence argues against `exp_shooting`'s per-arc re-epoch — its
own steering fits 0.26–0.94° in absolute time. Changing it is a separate decision.

**Thesis (on hold).** Table 4.1 `t_go` column → "—" for both tangent laws; the §4.x paragraph
"Both forms are implemented, and both depart … re-derive them from the current state at every
guidance update", Eq. `lts_alpha` and the 0.7/0.3 `bts_blend` paragraph are superseded under
`pso_coast`; the continuity-through-coast convention and the `μ` parametrisation need a sentence.
